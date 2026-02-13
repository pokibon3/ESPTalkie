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

void receiveCallback(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen)
{
    if (recvInfo && recvInfo->rx_ctrl) {
      instance->setRSSI(recvInfo->rx_ctrl->rssi);
    }
    int header_size = instance->m_header_size;
    // first m_header_size bytes of m_buffer are the expected header
    if ((dataLen > header_size) && (dataLen<=MAX_ESP_NOW_PACKET_SIZE) && (memcmp(data,instance->m_buffer,header_size) == 0)) {
      instance->m_output_buffer->add_samples(data + header_size, dataLen - header_size);
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
  esp_err_t result = esp_now_send(broadcastAddress, m_buffer, m_index + m_header_size);
//  Serial.printf("m_index : %d\n", m_index);
//  for (int i = 0; i < m_index; i++) 
//    Serial.println(m_buffer[i]);
  if (result != ESP_OK) {
    Serial.printf("Failed to send: %s\n", esp_err_to_name(result));
  }
}
