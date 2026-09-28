#pragma once

#include <cstdint>

// M5Stack Tab5 user interface: touch controls (right column) and an SD-card
// image slideshow (left pane). All functions are no-ops on other targets.

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

void tab5_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode);
// Call from loop() after M5.update(). Returns a settings action (edge-triggered).
Tab5Action tab5_ui_poll();
bool tab5_ui_ptt_pressed();

void tab5_ui_set_settings(int channel, int volume_level, uint8_t tx_pitch_mode);
void tab5_ui_set_status(bool transmitting, bool continuous);
void tab5_ui_set_rssi(int16_t rssi);
void tab5_ui_set_tx_power(int16_t dbm);
// Full-width message on the status bar (boot / co-processor update progress).
void tab5_ui_message(const char *msg);
