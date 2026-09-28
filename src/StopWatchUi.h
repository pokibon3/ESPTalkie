#pragma once

#include <cstdint>

// M5Stack StopWatch user interface (round 466x466 AMOLED).
//   Main screen : embedded image cropped to a circle in the centre, with a
//                 status ring around it (blue=standby, green=receiving,
//                 red=TX, orange=continuous TX), RSSI / TX power on the left,
//                 battery on the right, CH / VOL / VOICE at the bottom.
//   Setup screen: CHANNEL / VOLUME / VOICE.
// Buttons: blue (BtnB, GPIO1) = PTT, yellow (BtnA, GPIO2) long press = setup.
// All functions are no-ops on other targets.

enum class StopWatchSetupItem : uint8_t {
    Channel = 0,
    Volume,
    Voice,
    Count,
};

enum class StopWatchTouch : uint8_t {
    None = 0,
    Minus,
    Plus,
    SelectChannel,
    SelectVolume,
    SelectVoice,
};

void stopwatch_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode);
// Call from loop() after M5.update(): RX indicator and vibration pattern.
void stopwatch_ui_service(uint32_t last_rx_ms);
// Touch on the setup screen (edge-triggered). Call after M5.update().
StopWatchTouch stopwatch_ui_poll_touch();

bool stopwatch_ui_ptt_pressed();
bool stopwatch_ui_setup_visible();
void stopwatch_ui_show_setup(bool show);
void stopwatch_ui_set_selected_item(StopWatchSetupItem item);
StopWatchSetupItem stopwatch_ui_selected_item();

void stopwatch_ui_set_settings(int channel, int volume_level, uint8_t tx_pitch_mode);
void stopwatch_ui_set_status(bool transmitting, bool continuous);
void stopwatch_ui_set_rssi(int16_t rssi);
void stopwatch_ui_set_tx_power(int16_t dbm);
// Start the "buzz-buzz-buzz" pattern on the vibration motor.
void stopwatch_ui_vibrate();
