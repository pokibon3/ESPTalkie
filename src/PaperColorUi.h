#pragma once

#include <cstdint>

enum class PaperColorRadioState : uint8_t {
    Idle = 0,
    Receiving,
    Transmitting,
    Error,
    TransmittingContinuous,
};

enum class PaperColorSettingItem : uint8_t {
    Channel = 0,
    Volume,
    Voice,
    Count,
};

void papercolor_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode);
void papercolor_ui_service();
void papercolor_ui_set_radio_state(PaperColorRadioState state);
void papercolor_ui_update_settings(int channel, int volume_level, uint8_t tx_pitch_mode, int16_t rssi);
void papercolor_ui_show_badge();
void papercolor_ui_show_settings();
bool papercolor_ui_settings_visible();
void papercolor_ui_set_selected_item(PaperColorSettingItem item);
bool papercolor_ui_ptt_pressed();
