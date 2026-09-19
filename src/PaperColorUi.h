#pragma once

#include <cstdint>

enum class PaperColorRadioState : uint8_t {
    Idle = 0,
    Receiving,
    Transmitting,
    Error,
};

void papercolor_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode);
void papercolor_ui_service();
void papercolor_ui_set_radio_state(PaperColorRadioState state);
void papercolor_ui_update_settings(int channel, int volume_level, uint8_t tx_pitch_mode, int16_t rssi);
void papercolor_ui_show_badge();
void papercolor_ui_show_settings();
bool papercolor_ui_settings_visible();
