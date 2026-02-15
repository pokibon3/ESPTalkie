#pragma once

#include "Transport.h"
#include <esp_now.h>

class OutputBuffer;

class EspNowTransport: public Transport {
private:
    uint8_t m_wifi_channel;
    int16_t m_rssi = -127;
    volatile uint32_t m_rx_ok_packets = 0;
    volatile uint32_t m_rx_ok_bytes = 0;
    volatile uint32_t m_rx_bad_header_packets = 0;
    volatile uint32_t m_rx_invalid_len_packets = 0;
    volatile uint32_t m_rx_gap_events = 0;
    volatile uint32_t m_rx_max_gap_ms = 0;
    volatile uint32_t m_last_rx_ms = 0;
    volatile uint32_t m_tx_packets = 0;
    volatile uint32_t m_tx_failures = 0;
protected:
    void send();
public:
    EspNowTransport(OutputBuffer *output_buffer, uint8_t wifi_channel);
    virtual bool begin() override;
    friend void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen);
    void        setRSSI(int16_t rssi) { m_rssi = rssi;}
    int16_t     getRSSI(void) override;
    uint16_t    getWifiChannel(void) { return m_wifi_channel;}
    void        setWifiChannel(uint16_t ch);
    void        snapshot_and_reset_stats(uint32_t &rx_ok,
                                         uint32_t &rx_ok_bytes,
                                         uint32_t &rx_bad_header,
                                         uint32_t &rx_invalid_len,
                                         uint32_t &rx_gap_events,
                                         uint32_t &rx_max_gap_ms,
                                         uint32_t &tx_packets,
                                         uint32_t &tx_failures);
};
