#pragma once

#include <sdkconfig.h>
#include <stdint.h>

#if defined(CONFIG_ESP_HOSTED_ENABLED) && !defined(CONFIG_SOC_WIFI_SUPPORTED)
#define ESP_NOW_HOSTED_SHIM 1
// True when the co-processor answers ESP-NOW-over-CustomRpc requests
// (i.e. it runs the esp-hosted firmware with the ESP-NOW overlay).
// esp-hosted Wi-Fi must already be started (WiFi.mode(WIFI_STA)).
bool esp_now_hosted_available(uint32_t timeout_ms = 1000);

// Observe every received ESP-NOW frame (any sender, any payload) with its RSSI
// and channel, e.g. for a channel-activity scan. With suppress_app_rx the frame
// is not passed on to the registered esp_now recv callback. nullptr removes it.
// Called from the esp-hosted RX thread: keep it short.
typedef void (*esp_now_hosted_monitor_cb_t)(const uint8_t *src, int8_t rssi, uint8_t channel,
                                            const uint8_t *data, int len);
void esp_now_hosted_set_monitor(esp_now_hosted_monitor_cb_t cb, bool suppress_app_rx);
#else
#define ESP_NOW_HOSTED_SHIM 0
#endif
