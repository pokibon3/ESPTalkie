#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "OutputBuffer.h"
#include "EspNowTransport.h"
#include "config.h"

const int MAX_ESP_NOW_PACKET_SIZE = 250;
const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static EspNowTransport *instance = NULL;

static void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (!instance || type != WIFI_PKT_MGMT) {
        return;
    }
    const wifi_promiscuous_pkt_t *ppkt = static_cast<wifi_promiscuous_pkt_t *>(buf);
    instance->setRSSI(ppkt->rx_ctrl.rssi);
}

void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen)
{
    (void)macAddr;
    if (!instance) {
        return;
    }
    int header_size = instance->m_header_size;
    // first m_header_size bytes of m_buffer are the expected header
    if ((dataLen > header_size) && (dataLen<=MAX_ESP_NOW_PACKET_SIZE) && (memcmp(data,instance->m_buffer,header_size) == 0)) {
      uint32_t now_ms = millis();
      if (instance->m_last_rx_ms != 0) {
        uint32_t gap_ms = now_ms - instance->m_last_rx_ms;
        if (gap_ms > 30) {
          instance->m_rx_gap_events++;
        }
        if (gap_ms > instance->m_rx_max_gap_ms) {
          instance->m_rx_max_gap_ms = gap_ms;
        }
      }
      instance->m_last_rx_ms = now_ms;
      instance->m_output_buffer->add_samples(data + header_size, dataLen - header_size);
      instance->m_rx_ok_packets++;
      instance->m_rx_ok_bytes += static_cast<uint32_t>(dataLen - header_size);
    } else if (dataLen <= header_size || dataLen > MAX_ESP_NOW_PACKET_SIZE) {
      instance->m_rx_invalid_len_packets++;
    } else {
      instance->m_rx_bad_header_packets++;
    }
}

void EspNowTransport::setWifiChannel(uint16_t ch)
{
    m_wifi_channel  = ch;
    esp_wifi_set_channel(m_wifi_channel, WIFI_SECOND_CHAN_NONE);
}



bool EspNowTransport::begin()
{
    // Set Wifi channel
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(m_wifi_channel, WIFI_SECOND_CHAN_NONE);
#ifdef ESPNOW_LONG_RANGE
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#endif
    esp_err_t result = esp_now_init();
    if (result == ESP_OK) {
        Serial.println("ESPNow Init Success");
        esp_now_register_recv_cb(receiveCallback);
        esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
    } else {
        Serial.printf("ESPNow Init failed: %s\n", esp_err_to_name(result));
        return false;
    }
    // this will broadcast a message to everyone in range
    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
    if (!esp_now_is_peer_exist(broadcastAddress)) {
        result = esp_now_add_peer(&peerInfo);
        if (result != ESP_OK) {
            Serial.printf("Failed to add broadcast peer: %s\n", esp_err_to_name(result));
            return false;
        }   
    }
    return true;
}

EspNowTransport::EspNowTransport(OutputBuffer *output_buffer, uint8_t wifi_channel) : Transport(output_buffer, MAX_ESP_NOW_PACKET_SIZE)
{
  instance = this;  
  m_wifi_channel = wifi_channel;
}

int16_t EspNowTransport::getRSSI(void)
{
  return m_rssi;
}

void EspNowTransport::send()
{
  m_tx_packets++;
  esp_err_t result = esp_now_send(broadcastAddress, m_buffer, m_index + m_header_size);
//  Serial.printf("m_index : %d\n", m_index);
//  for (int i = 0; i < m_index; i++) 
//    Serial.println(m_buffer[i]);
  if (result != ESP_OK) {
    m_tx_failures++;
    Serial.printf("Failed to send: %s\n", esp_err_to_name(result));
  }
}

void EspNowTransport::snapshot_and_reset_stats(uint32_t &rx_ok,
                                               uint32_t &rx_ok_bytes,
                                               uint32_t &rx_bad_header,
                                               uint32_t &rx_invalid_len,
                                               uint32_t &rx_gap_events,
                                               uint32_t &rx_max_gap_ms,
                                               uint32_t &tx_packets,
                                               uint32_t &tx_failures)
{
  rx_ok = m_rx_ok_packets;
  rx_ok_bytes = m_rx_ok_bytes;
  rx_bad_header = m_rx_bad_header_packets;
  rx_invalid_len = m_rx_invalid_len_packets;
  rx_gap_events = m_rx_gap_events;
  rx_max_gap_ms = m_rx_max_gap_ms;
  tx_packets = m_tx_packets;
  tx_failures = m_tx_failures;
  m_rx_ok_packets = 0;
  m_rx_ok_bytes = 0;
  m_rx_bad_header_packets = 0;
  m_rx_invalid_len_packets = 0;
  m_rx_gap_events = 0;
  m_rx_max_gap_ms = 0;
  m_tx_packets = 0;
  m_tx_failures = 0;
}
