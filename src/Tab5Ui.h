#pragma once

#include <cstdint>

// M5Stack Tab5 user interface (portrait 720x1280).
//   Main screen : one image from the SD card, full screen, with the PTT button
//                 at the bottom centre and SETUP at the bottom right.
//   Setup panel : CHANNEL / VOLUME / VOICE, signal meter and a channel scan
//                 (RSSI of Wi-Fi access points on channels 1-13).
// All functions are no-ops on other targets.

enum class Tab5Action : uint8_t {
    None = 0,
    ChannelDown,
    ChannelUp,
    VolumeDown,
    VolumeUp,
    Mode1,
    Mode2,
    Mode3,
};

// restore_radio: called (from the scan task) after a channel scan ends, to put
// the radio back on the configured channel.
void tab5_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode, void (*restore_radio)());
// Call from loop() after M5.update(). Returns a settings action (edge-triggered).
Tab5Action tab5_ui_poll();
bool tab5_ui_ptt_pressed();
// True while a channel scan owns the radio.
bool tab5_ui_scanning();

void tab5_ui_set_settings(int channel, int volume_level, uint8_t tx_pitch_mode);
void tab5_ui_set_status(bool transmitting, bool continuous);
void tab5_ui_set_rssi(int16_t rssi);
void tab5_ui_set_tx_power(int16_t dbm);
// Full-width message at the top of the screen (boot / co-processor update).
void tab5_ui_message(const char *msg);
