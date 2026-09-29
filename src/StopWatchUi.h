#pragma once

#include <cstdint>

// M5Stack StopWatch user interface (round 466x466 AMOLED).
//   Main screen : embedded photo (circle) with an analog clock.
//                 - translucent hands over the photo (yellow click: opaque
//                   for a few seconds)
//                 - hour / minute / second markers on the dial ring around
//                   the photo
//                 - outer ring: status (top), battery (right),
//                   CH / VOL / VOICE (bottom), signal (left)
//   Setup screen: CHANNEL / VOLUME / VOICE / TIME (NTP sync) / WIFI (enter
//                 the home AP from a phone via QR code).
// Buttons: blue (BtnB, GPIO1) = PTT, yellow (BtnA, GPIO2) click = show
// hands, long press = setup.
// All functions are no-ops on other targets.

enum class StopWatchSetupItem : uint8_t {
    Channel = 0,
    Volume,
    Voice,
    Time,
    Wifi,
    Count,
};

enum class StopWatchTouch : uint8_t {
    None = 0,
    Minus,
    Plus,
    SelectChannel,
    SelectVolume,
    SelectVoice,
    SelectTime,
    SelectWifi,
};

void stopwatch_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode);
// Call from loop() after M5.update(): clock, RX indicator, vibration, redraw.
void stopwatch_ui_service(uint32_t last_rx_ms);
// Touch on the setup screen (edge-triggered). Call after M5.update().
StopWatchTouch stopwatch_ui_poll_touch();

bool stopwatch_ui_ptt_pressed();
bool stopwatch_ui_setup_visible();
void stopwatch_ui_show_setup(bool show);
void stopwatch_ui_set_selected_item(StopWatchSetupItem item);
StopWatchSetupItem stopwatch_ui_selected_item();
// Show the clock hands opaque for STOPWATCH_HANDS_CLEAR_MS.
void stopwatch_ui_show_hands();
// Text shown in the TIME row (nullptr = current time).
void stopwatch_ui_set_time_message(const char *msg);
// Full-screen one-line message (boot-time NTP sync).
void stopwatch_ui_message(const char *msg);

void stopwatch_ui_set_settings(int channel, int volume_level, uint8_t tx_pitch_mode);
void stopwatch_ui_set_status(bool transmitting, bool continuous);
void stopwatch_ui_set_rssi(int16_t rssi);
void stopwatch_ui_set_tx_power(int16_t dbm);
// Start the "buzz-buzz-buzz" pattern on the vibration motor.
void stopwatch_ui_vibrate();
