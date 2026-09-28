/*
 * esp_now_hosted.cpp
 *
 * ESP-NOW for radio-less hosts (ESP32-P4 on M5Stack Tab5).
 *
 * The P4 has no radio; Wi-Fi is provided by the on-board ESP32-C6 through
 * esp-hosted. esp-hosted proxies esp_wifi.h but not esp_now.h, so the
 * esp_now_* symbols are missing at link time. This file provides them and
 * forwards each call to the C6 over esp-hosted's CustomRpc
 * ("peer data transfer") channel.
 *
 * The C6 must run esp-hosted slave firmware built with the ESP-NOW overlay
 * from https://github.com/esphome/esp-hosted-firmware (Apache-2.0), at the
 * same esp-hosted version as the host (arduino-esp32 3.3.12: 2.12.13).
 * The wire contract is esp_now_hosted_rpc.h, a verbatim copy of that
 * repository's slave-overlay header.
 *
 * This is an independent implementation for ESPTalkie (MIT). Differences from
 * a plain synchronous proxy:
 *   - esp_now_send() is fire-and-forget: it does not wait for the RPC reply,
 *     so audio streaming never blocks on the SDIO round trip, and a lost
 *     SEND-status event cannot stall transmission.
 *   - Control calls (init/peer management) are synchronous with a timeout.
 */

#include <sdkconfig.h>

#if defined(CONFIG_ESP_HOSTED_ENABLED) && !defined(CONFIG_SOC_WIFI_SUPPORTED)

#include <string.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "esp_hosted_misc.h"
#include "esp_now_hosted.h"
#include "esp_now_hosted_rpc.h"

namespace {

const char *TAG = "esp_now_hosted";

SemaphoreHandle_t s_req_mutex = nullptr;   // serializes synchronous requests
SemaphoreHandle_t s_resp_sem = nullptr;    // given when the awaited reply arrives
portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

volatile bool s_rpc_registered = false;
volatile bool s_waiting = false;
volatile uint8_t s_wait_opcode = 0;
volatile uint8_t s_wait_seq = 0;
uint8_t s_seq = 0;

esp_err_t s_resp_status = ESP_FAIL;
uint8_t s_resp_ret[8];
uint16_t s_resp_ret_len = 0;

esp_now_recv_cb_t s_recv_cb = nullptr;
esp_now_hosted_monitor_cb_t s_monitor_cb = nullptr;
volatile bool s_monitor_suppress = false;
esp_now_send_cb_t s_send_cb = nullptr;

// Static RX metadata handed to the application callback (called from the
// single esp-hosted RX thread, so one instance is enough).
wifi_pkt_rx_ctrl_t s_rx_ctrl;
esp_now_recv_info_t s_rx_info;

uint8_t s_tx_buf[sizeof(esp_now_hosted_req_t) + sizeof(esp_now_hosted_send_req_t) + ESP_NOW_HOSTED_MAX_FRAME];
SemaphoreHandle_t s_tx_mutex = nullptr;

uint8_t next_seq()
{
    uint8_t seq;
    portENTER_CRITICAL(&s_lock);
    seq = ++s_seq;
    portEXIT_CRITICAL(&s_lock);
    return seq;
}

void on_resp(uint32_t, const uint8_t *data, size_t len, void *)
{
    if (len < sizeof(esp_now_hosted_resp_t)) {
        return;
    }
    const auto *resp = reinterpret_cast<const esp_now_hosted_resp_t *>(data);
    if (len < sizeof(esp_now_hosted_resp_t) + resp->ret_len) {
        return;
    }
    // Replies to fire-and-forget sends are simply dropped here.
    if (!s_waiting || resp->opcode != s_wait_opcode || resp->seq != s_wait_seq) {
        return;
    }
    s_resp_status = static_cast<esp_err_t>(resp->status);
    s_resp_ret_len = resp->ret_len > sizeof(s_resp_ret) ? sizeof(s_resp_ret) : resp->ret_len;
    memcpy(s_resp_ret, resp->ret, s_resp_ret_len);
    s_waiting = false;
    xSemaphoreGive(s_resp_sem);
}

void on_recv(uint32_t, const uint8_t *data, size_t len, void *)
{
    if (len < sizeof(esp_now_hosted_recv_evt_t)) {
        return;
    }
    const auto *evt = reinterpret_cast<const esp_now_hosted_recv_evt_t *>(data);
    if (len < sizeof(esp_now_hosted_recv_evt_t) + evt->data_len) {
        return;
    }
    esp_now_hosted_monitor_cb_t mon = s_monitor_cb;
    if (mon) {
        mon(evt->src_addr, evt->rssi, evt->channel, evt->data, evt->data_len);
        if (s_monitor_suppress) {
            return;
        }
    }
    esp_now_recv_cb_t cb = s_recv_cb;
    if (!cb) {
        return;
    }
    memset(&s_rx_ctrl, 0, sizeof(s_rx_ctrl));
    s_rx_ctrl.rssi = evt->rssi;
    s_rx_ctrl.channel = evt->channel;
    s_rx_info.src_addr = const_cast<uint8_t *>(evt->src_addr);
    s_rx_info.des_addr = const_cast<uint8_t *>(evt->des_addr);
    s_rx_info.rx_ctrl = &s_rx_ctrl;
    cb(&s_rx_info, evt->data, evt->data_len);
}

void on_send_status(uint32_t, const uint8_t *data, size_t len, void *)
{
    if (len < sizeof(esp_now_hosted_send_evt_t)) {
        return;
    }
    esp_now_send_cb_t cb = s_send_cb;
    if (!cb) {
        return;
    }
    const auto *evt = reinterpret_cast<const esp_now_hosted_send_evt_t *>(data);
    wifi_tx_info_t info = {};
    info.des_addr = const_cast<uint8_t *>(evt->des_addr);
    cb(&info, static_cast<esp_now_send_status_t>(evt->status));
}

esp_err_t ensure_registered()
{
    if (s_rpc_registered) {
        return ESP_OK;
    }
    if (!s_req_mutex) s_req_mutex = xSemaphoreCreateMutex();
    if (!s_resp_sem) s_resp_sem = xSemaphoreCreateBinary();
    if (!s_tx_mutex) s_tx_mutex = xSemaphoreCreateMutex();
    if (!s_req_mutex || !s_resp_sem || !s_tx_mutex) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = esp_hosted_register_custom_callback(ESP_NOW_HOSTED_MSG_RESP, on_resp, nullptr);
    if (err == ESP_OK) err = esp_hosted_register_custom_callback(ESP_NOW_HOSTED_MSG_RECV, on_recv, nullptr);
    if (err == ESP_OK) err = esp_hosted_register_custom_callback(ESP_NOW_HOSTED_MSG_SEND, on_send_status, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register custom callback failed: %s", esp_err_to_name(err));
        return err;
    }
    s_rpc_registered = true;
    return ESP_OK;
}

// Synchronous request. ret/ret_len are optional.
esp_err_t request(uint8_t opcode, const void *payload, uint16_t payload_len,
                  void *ret = nullptr, size_t ret_len = 0,
                  uint32_t timeout_ms = ESP_NOW_HOSTED_TIMEOUT_MS)
{
    esp_err_t err = ensure_registered();
    if (err != ESP_OK) {
        return err;
    }
    if (payload_len > ESP_NOW_HOSTED_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t buf[sizeof(esp_now_hosted_req_t) + 64];
    if (sizeof(esp_now_hosted_req_t) + payload_len > sizeof(buf)) {
        return ESP_ERR_INVALID_SIZE;
    }

    xSemaphoreTake(s_req_mutex, portMAX_DELAY);
    auto *req = reinterpret_cast<esp_now_hosted_req_t *>(buf);
    req->opcode = opcode;
    req->seq = next_seq();
    req->payload_len = payload_len;
    if (payload_len) {
        memcpy(req->payload, payload, payload_len);
    }

    xSemaphoreTake(s_resp_sem, 0);  // drop a stale give
    s_wait_opcode = opcode;
    s_wait_seq = req->seq;
    s_waiting = true;

    err = esp_hosted_send_custom_data(ESP_NOW_HOSTED_MSG_REQ, buf, sizeof(esp_now_hosted_req_t) + payload_len);
    if (err == ESP_OK) {
        if (xSemaphoreTake(s_resp_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            err = s_resp_status;
            if (ret && ret_len) {
                memcpy(ret, s_resp_ret, ret_len < s_resp_ret_len ? ret_len : s_resp_ret_len);
            }
        } else {
            err = ESP_ERR_TIMEOUT;
        }
    }
    s_waiting = false;
    xSemaphoreGive(s_req_mutex);
    return err;
}

void to_wire_peer(const esp_now_peer_info_t *peer, esp_now_hosted_peer_t *out)
{
    memset(out, 0, sizeof(*out));
    memcpy(out->peer_addr, peer->peer_addr, 6);
    memcpy(out->lmk, peer->lmk, sizeof(out->lmk));
    out->channel = peer->channel;
    out->ifidx = static_cast<uint8_t>(peer->ifidx);
    out->encrypt = peer->encrypt ? 1 : 0;
}

}  // namespace

// ── esp_now.h API ──────────────────────────────────────────────────────────

extern "C" esp_err_t esp_now_init(void)
{
    return request(ESP_NOW_HOSTED_OP_INIT, nullptr, 0);
}

extern "C" esp_err_t esp_now_deinit(void)
{
    return request(ESP_NOW_HOSTED_OP_DEINIT, nullptr, 0);
}

extern "C" esp_err_t esp_now_get_version(uint32_t *version)
{
    if (!version) return ESP_ERR_INVALID_ARG;
    *version = 0;
    return request(ESP_NOW_HOSTED_OP_GET_VERSION, nullptr, 0, version, sizeof(*version));
}

extern "C" esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb)
{
    s_recv_cb = cb;
    return ensure_registered();
}

extern "C" esp_err_t esp_now_unregister_recv_cb(void)
{
    s_recv_cb = nullptr;
    return ESP_OK;
}

extern "C" esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb)
{
    s_send_cb = cb;
    return ensure_registered();
}

extern "C" esp_err_t esp_now_unregister_send_cb(void)
{
    s_send_cb = nullptr;
    return ESP_OK;
}

extern "C" esp_err_t esp_now_add_peer(const esp_now_peer_info_t *peer)
{
    if (!peer) return ESP_ERR_INVALID_ARG;
    esp_now_hosted_peer_t p;
    to_wire_peer(peer, &p);
    return request(ESP_NOW_HOSTED_OP_ADD_PEER, &p, sizeof(p));
}

extern "C" esp_err_t esp_now_mod_peer(const esp_now_peer_info_t *peer)
{
    if (!peer) return ESP_ERR_INVALID_ARG;
    esp_now_hosted_peer_t p;
    to_wire_peer(peer, &p);
    return request(ESP_NOW_HOSTED_OP_MOD_PEER, &p, sizeof(p));
}

extern "C" esp_err_t esp_now_del_peer(const uint8_t *peer_addr)
{
    if (!peer_addr) return ESP_ERR_INVALID_ARG;
    return request(ESP_NOW_HOSTED_OP_DEL_PEER, peer_addr, 6);
}

extern "C" bool esp_now_is_peer_exist(const uint8_t *peer_addr)
{
    if (!peer_addr) return false;
    uint8_t exists = 0;
    if (request(ESP_NOW_HOSTED_OP_IS_PEER_EXIST, peer_addr, 6, &exists, sizeof(exists)) != ESP_OK) {
        return false;
    }
    return exists != 0;
}

extern "C" esp_err_t esp_now_set_pmk(const uint8_t *pmk)
{
    if (!pmk) return ESP_ERR_INVALID_ARG;
    return request(ESP_NOW_HOSTED_OP_SET_PMK, pmk, 16);
}

extern "C" esp_err_t esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    if (!data || len == 0 || len > ESP_NOW_HOSTED_MAX_FRAME) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ensure_registered();
    if (err != ESP_OK) {
        return err;
    }
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    auto *req = reinterpret_cast<esp_now_hosted_req_t *>(s_tx_buf);
    auto *send = reinterpret_cast<esp_now_hosted_send_req_t *>(req->payload);
    req->opcode = ESP_NOW_HOSTED_OP_SEND;
    req->seq = next_seq();
    req->payload_len = static_cast<uint16_t>(sizeof(esp_now_hosted_send_req_t) + len);
    send->has_addr = peer_addr ? 1 : 0;
    if (peer_addr) {
        memcpy(send->peer_addr, peer_addr, 6);
    } else {
        memset(send->peer_addr, 0, 6);
    }
    send->data_len = static_cast<uint16_t>(len);
    memcpy(send->data, data, len);
    err = esp_hosted_send_custom_data(ESP_NOW_HOSTED_MSG_REQ, s_tx_buf,
                                      sizeof(esp_now_hosted_req_t) + req->payload_len);
    xSemaphoreGive(s_tx_mutex);
    return err;
}

extern "C" esp_err_t esp_now_set_wake_window(uint16_t)
{
    return ESP_ERR_NOT_SUPPORTED;
}

// ── ESPTalkie helpers ──────────────────────────────────────────────────────

void esp_now_hosted_set_monitor(esp_now_hosted_monitor_cb_t cb, bool suppress_app_rx)
{
    s_monitor_suppress = cb ? suppress_app_rx : false;
    s_monitor_cb = cb;
}

bool esp_now_hosted_available(uint32_t timeout_ms)
{
    // Any reply (even an error such as "ESP-NOW not initialized") proves that
    // the co-processor firmware carries the ESP-NOW overlay.
    uint32_t v = 0;
    const esp_err_t err = request(ESP_NOW_HOSTED_OP_GET_VERSION, nullptr, 0, &v, sizeof(v), timeout_ms);
    return err != ESP_ERR_TIMEOUT && err != ESP_ERR_NO_MEM && s_rpc_registered;
}

#endif  // CONFIG_ESP_HOSTED_ENABLED && !CONFIG_SOC_WIFI_SUPPORTED
