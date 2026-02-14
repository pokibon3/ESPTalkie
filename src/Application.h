#pragma once

#include <cstdint>

class Transport;
class OutputBuffer;

class Application
{
private:
    Transport       *m_transport;
    OutputBuffer    *m_output_buffer;
    uint16_t        m_channel;
    uint8_t         m_speaker_volume;
    volatile uint8_t m_tx_pitch_mode;

public:
    enum : uint8_t {
        kTxPitchModeM1 = 1,
        kTxPitchModeM2 = 2,
        kTxPitchModeM3 = 3,
    };

    Application();
    int16_t getRSSI(void);
    void dispRSSI(int16_t);
    void dispStatus(bool transmitting);
    void dispTxPower(int16_t dbm);
    void begin();
    void loop();
    void setChannel(uint16_t ch);
    void setSpeakerVolume(uint8_t volume);
    uint8_t getSpeakerVolume() const;
    void setTxPitchMode(uint8_t mode);
    uint8_t getTxPitchMode() const;
};
