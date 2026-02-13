#pragma once

#include "Transport.h"
#include <esp_now.h>

class OutputBuffer;

class EspNowTransport: public Transport {
private:
    uint8_t m_wifi_channel;
    int16_t m_rssi = -127;
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
};
