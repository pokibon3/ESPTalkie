/*
 * main.cpp
 */

#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <math.h>

#include "Application.h"
#include "DisplaySync.h"
#include "PaperColorUi.h"
#include "UiLayout.h"
#include "config.h"

namespace {

Application *application = nullptr;
int channel = 1;
int volume_level = 3;  // 1..5
uint8_t tx_pitch_mode = Application::kTxPitchModeM1;  // 1..3
enum class EditMode : uint8_t {
    None = 0,
    Volume = 1,
    Channel = 2,
    Mode = 3,
};
EditMode edit_mode = EditMode::None;
uint32_t mode_selected_at_ms = 0;
#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
constexpr uint8_t kVolumeTable[5] = { 20, 30, 45, 60, 80 };
#else
constexpr uint8_t kVolumeTable[5] = { 80, 120, 160, 208, 255 };
#endif
constexpr bool kMatchTestModeSpeakerGain = false;
constexpr uint8_t kTestLikeSpeakerGain = 255;
Preferences prefs;
#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
constexpr int kDefaultVolumeLevel = 1;
#else
constexpr int kDefaultVolumeLevel = 3;
#endif
constexpr uint32_t kModeAutoClearMs = 5000;

#if TALKIE_TARGET_M5PAPERCOLOR
constexpr int kPaperButtonAPin = 10;
constexpr int kPaperButtonBPin = 9;
constexpr int kPaperButtonCPin = 1;
#endif

enum class ShakeAction : int8_t {
    None = 0,
    Increase = 1,
    Decrease = -1,
    SwitchMode = 2,
};

ShakeAction detect_shake_action()
{
#if !SHAKE_SWITCH_ENABLED
    return ShakeAction::None;
#else
    static uint32_t last_trigger_ms = 0;
    static bool armed = true;

    if (!M5.Imu.isEnabled() || M5.BtnA.isPressed()) {
        return ShakeAction::None;
    }
    if (!M5.Imu.update()) {
        return ShakeAction::None;
    }

    const auto imu = M5.Imu.getImuData();
    const float ax = fabsf(imu.accel.x);
    const float ay = fabsf(imu.accel.y);
    const float az = fabsf(imu.accel.z);
    const uint32_t now = millis();
    // Map IMU axes to logical horizontal/vertical based on current display orientation.
    float horizontal_axis = ax;
    float vertical_axis = ay;
    if (M5.Display.height() > M5.Display.width()) {
        horizontal_axis = ay;
        vertical_axis = ax;
    }
    const bool strong_horizontal =
        (horizontal_axis >= SHAKE_X_THRESHOLD_G) &&
        (horizontal_axis > (vertical_axis + SHAKE_X_DOMINANCE_G)) &&
        (horizontal_axis > (az + SHAKE_X_DOMINANCE_G));
    const bool strong_vertical =
        (vertical_axis >= SHAKE_Y_THRESHOLD_G) &&
        (vertical_axis > (horizontal_axis + SHAKE_Y_DOMINANCE_G)) &&
        (vertical_axis > (az + SHAKE_Y_DOMINANCE_G));
    // Make mode-switch shake (depth axis) less sensitive than up/down adjustments.
    constexpr float kModeSwitchExtraThresholdG = 1.80f;
    constexpr float kModeSwitchExtraDominanceG = 0.90f;
    const bool strong_depth =
        (az >= (SHAKE_Z_THRESHOLD_G + kModeSwitchExtraThresholdG)) &&
        (az > (ax + SHAKE_Z_DOMINANCE_G + kModeSwitchExtraDominanceG)) &&
        (az > (ay + SHAKE_Z_DOMINANCE_G + kModeSwitchExtraDominanceG));

    if (ax < SHAKE_REARM_G && ay < SHAKE_REARM_G && az < SHAKE_REARM_G) {
        armed = true;
    }
#if TALKIE_TARGET_M5STICKS3
    const bool shake_triggered = (strong_horizontal || strong_vertical);
#else
    const bool shake_triggered = (strong_horizontal || strong_vertical || strong_depth);
#endif
    if (armed && shake_triggered && (now - last_trigger_ms >= SHAKE_COOLDOWN_MS)) {
        armed = false;
        last_trigger_ms = now;
#if !TALKIE_TARGET_M5STICKS3
        if (strong_depth && az >= ax && az >= ay) {
            return ShakeAction::SwitchMode;
        }
#endif
        // Horizontal=Increase, Vertical=Decrease (display orientation aware).
        if (strong_horizontal && (!strong_vertical || horizontal_axis >= vertical_axis)) {
            return ShakeAction::Increase;
        }
        return ShakeAction::Decrease;
    }
    return ShakeAction::None;
#endif
}

int wrapped_step(int value, int minv, int maxv, int delta)
{
    if (delta > 0) {
        ++value;
        if (value > maxv) value = minv;
    } else if (delta < 0) {
        --value;
        if (value < minv) value = maxv;
    }
    return value;
}

void draw_channel()
{
    display_lock();
    const uint16_t panel = M5.Display.color565(44, 52, 62);
    const uint16_t accent = TFT_BLUE;
    const uint16_t text = TFT_WHITE;
    const uint16_t active = TFT_GREEN;
    const bool channel_selected = (edit_mode == EditMode::Channel);
    const bool mode_selected = (edit_mode == EditMode::Mode);
    const uint16_t mode_color = mode_selected ? active : text;

    M5.Display.fillRoundRect(kUiLayout.channel_x, kUiLayout.channel_y, kUiLayout.channel_w, kUiLayout.channel_h, kUiLayout.channel_radius, panel);
    M5.Display.drawRoundRect(kUiLayout.channel_x, kUiLayout.channel_y, kUiLayout.channel_w, kUiLayout.channel_h, kUiLayout.channel_radius, accent);
    M5.Display.setTextColor(channel_selected ? active : text, panel);
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(top_left);
#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
    constexpr const char* kChannelLabel = "CH";
#else
    constexpr const char* kChannelLabel = "CHANNEL";
#endif
    M5.Display.drawString(kChannelLabel, kUiLayout.channel_x + kUiLayout.channel_label_x, kUiLayout.channel_y + kUiLayout.channel_label_y);

    M5.Display.setTextColor(channel_selected ? active : text, panel);
#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
    M5.Display.setFont(&fonts::Font4);
#else
    M5.Display.setFont(kUiLayout.channel_compact_font ? &fonts::Font6 : &fonts::Font7);
#endif
    M5.Display.setTextSize(1);
    char ch_text[4];
    snprintf(ch_text, sizeof(ch_text), "%02d", channel);
#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
    const int channel_value_y = kUiLayout.channel_y + kUiLayout.channel_value_y - 13;
#else
    const int channel_value_y = kUiLayout.channel_y + kUiLayout.channel_value_y - 8;
#endif
    M5.Display.setTextDatum(middle_center);
    M5.Display.drawString(ch_text, kUiLayout.channel_x + (kUiLayout.channel_w / 2), channel_value_y);
    M5.Display.setFont(&fonts::Font0);

#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
    const int mode_area_x = kUiLayout.channel_x + 4;
    const int mode_area_w = kUiLayout.channel_w - 8;
    const int inner_bottom = kUiLayout.channel_y + kUiLayout.channel_h - 2;
#else
    const int mode_area_x = kUiLayout.channel_x + 6;
    const int mode_area_w = kUiLayout.channel_w - 12;
    const int inner_bottom = kUiLayout.channel_y + kUiLayout.channel_h - 3;
#endif
    const int underline_h = 2;
    const int underline_gap = 1;
    const int text_h = M5.Display.fontHeight();
    const int mode_text_y = inner_bottom - underline_h - underline_gap - text_h;
    M5.Display.fillRect(mode_area_x, mode_text_y - 1, mode_area_w, text_h + underline_h + 4, panel);
    M5.Display.setTextColor(mode_color, panel);
    M5.Display.setTextSize(1);
    constexpr const char *kModeLabels[3] = { "M1", "M2", "M3" };
    const int text_center_y = mode_text_y + (M5.Display.fontHeight() / 2);
    int selected_center_x = mode_area_x + (mode_area_w / 6);
    int selected_w = M5.Display.textWidth("M1");
    M5.Display.setTextDatum(middle_center);
    for (int i = 0; i < 3; ++i) {
        const int center_x = mode_area_x + ((mode_area_w * (2 * i + 1)) / 6);
        const int label_w = M5.Display.textWidth(kModeLabels[i]);
        M5.Display.drawString(kModeLabels[i], center_x, text_center_y);
        if (tx_pitch_mode == static_cast<uint8_t>(i + 1)) {
            selected_center_x = center_x;
            selected_w = label_w;
        }
    }
    const int underline_y = mode_text_y + text_h + underline_gap;
    M5.Display.fillRect(selected_center_x - (selected_w / 2), underline_y, selected_w, 2, mode_color);
    M5.Display.setTextDatum(top_left);
    display_unlock();
}

void draw_volume()
{
    display_lock();
    const uint16_t panel = M5.Display.color565(44, 52, 62);
    const uint16_t accent = TFT_BLUE;
    const uint16_t text = TFT_WHITE;
    const uint16_t sub = TFT_WHITE;
    const uint16_t active = TFT_GREEN;

    M5.Display.fillRoundRect(kUiLayout.volume_x, kUiLayout.info_y, kUiLayout.info_w, kUiLayout.info_h, kUiLayout.info_radius, panel);
    M5.Display.drawRoundRect(kUiLayout.volume_x, kUiLayout.info_y, kUiLayout.info_w, kUiLayout.info_h, kUiLayout.info_radius, accent);
    const bool volume_selected = (edit_mode == EditMode::Volume);
    M5.Display.setTextColor(volume_selected ? active : sub, panel);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(kUiLayout.volume_x + kUiLayout.volume_label_x, kUiLayout.info_y + kUiLayout.volume_label_y);
    M5.Display.print("VOL");

    M5.Display.setTextColor(volume_selected ? active : text, panel);
    M5.Display.setTextSize(kUiLayout.volume_value_text_size);
    M5.Display.setTextDatum(middle_center);
    char vol_text[4];
    snprintf(vol_text, sizeof(vol_text), "%d", volume_level);
    M5.Display.drawString(vol_text, kUiLayout.volume_x + (kUiLayout.info_w / 2), kUiLayout.info_y + (kUiLayout.info_h / 2) + kUiLayout.volume_value_y);
    M5.Display.setTextDatum(top_left);
    display_unlock();
}

void draw_layout()
{
    display_lock();
    const uint16_t bg = M5.Display.color565(10, 18, 36);
    const uint16_t accent = TFT_BLUE;

    M5.Display.fillScreen(bg);

    M5.Display.drawFastHLine(0, kUiLayout.status_h, M5.Display.width(), accent);

    draw_channel();
    draw_volume();

    // RSSI value box (right side of info row)
    const uint16_t panel = M5.Display.color565(44, 52, 62);
    M5.Display.fillRoundRect(kUiLayout.rssi_x, kUiLayout.info_y, kUiLayout.info_w, kUiLayout.info_h, kUiLayout.info_radius, panel);
    M5.Display.drawRoundRect(kUiLayout.rssi_x, kUiLayout.info_y, kUiLayout.info_w, kUiLayout.info_h, kUiLayout.info_radius, accent);

    // Level bar area (bottom)
    M5.Display.fillRoundRect(kUiLayout.bar_x, kUiLayout.bar_y, kUiLayout.bar_w, kUiLayout.bar_h, kUiLayout.bar_radius, M5.Display.color565(232, 250, 255));
    M5.Display.drawRoundRect(kUiLayout.bar_x, kUiLayout.bar_y, kUiLayout.bar_w, kUiLayout.bar_h, kUiLayout.bar_radius, accent);
    display_unlock();
}

uint8_t current_speaker_gain()
{
    if (kMatchTestModeSpeakerGain) {
        return kTestLikeSpeakerGain;
    }
    return kVolumeTable[volume_level - 1];
}

#if TALKIE_TARGET_M5PAPERCOLOR
void papercolor_loop()
{
    // A short debounce is enough because two separate buttons must be held.
    // Keeping this small also makes the chord practical on the recessed keys.
    constexpr uint32_t kChordHoldMs = 100;
    constexpr uint32_t kSettingsCommitDelayMs = 2000;
    static bool chord_tracking = false;
    static bool chord_triggered = false;
    static uint32_t chord_started_ms = 0;
    static bool settings_dirty = false;
    static uint32_t settings_changed_ms = 0;

    papercolor_ui_service();

    // Use the physical labels from the Paper Color pin map. M5Unified 0.2.15
    // exposes GPIO9 as BtnA and GPIO10 as BtnB, reversing physical A/B.
    const bool a_pressed = digitalRead(kPaperButtonAPin) == LOW;
    const bool b_pressed = digitalRead(kPaperButtonBPin) == LOW;
    const bool c_pressed = digitalRead(kPaperButtonCPin) == LOW;

    // Log each electrical transition so the physical button mapping can be
    // verified on the connected unit without refreshing the E-Ink panel.
    static uint8_t last_button_bits = 0xFF;
    const uint8_t button_bits =
        (a_pressed ? 1U : 0U) |
        ((b_pressed ? 1U : 0U) << 1) |
        ((c_pressed ? 1U : 0U) << 2);
    if (button_bits != last_button_bits) {
        Serial.printf("PaperColor buttons: A=%u B=%u C=%u (GPIO10/9/1)\n",
                      button_bits & 1U,
                      (button_bits >> 1) & 1U,
                      (button_bits >> 2) & 1U);
        last_button_bits = button_bits;
    }
    if (a_pressed && b_pressed) {
        if (!chord_tracking) {
            chord_tracking = true;
            chord_triggered = false;
            chord_started_ms = millis();
        }
        if (!chord_triggered && millis() - chord_started_ms >= kChordHoldMs) {
            Serial.println("PaperColor: A+B chord accepted; toggling page");
            papercolor_ui_update_settings(channel, volume_level, tx_pitch_mode, application->getRSSI());
            if (papercolor_ui_settings_visible()) {
                papercolor_ui_show_badge();
            } else {
                papercolor_ui_show_settings();
            }
            chord_triggered = true;
            settings_dirty = false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
        return;
    }
    if (chord_tracking) {
        if (!a_pressed && !b_pressed) {
            chord_tracking = false;
            chord_triggered = false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
        return;
    }

    bool changed = false;
    if (!papercolor_ui_settings_visible()) {
        vTaskDelay(pdMS_TO_TICKS(5));
        return;
    }
    // Physical B / left-middle (GPIO9) is reported as BtnA.
    if (M5.BtnA.wasHold()) {
        channel = wrapped_step(channel, 1, 13, -1);
        application->setChannel(static_cast<uint16_t>(channel));
        prefs.putInt("channel", channel);
        changed = true;
    } else if (M5.BtnA.wasClicked()) {
        channel = wrapped_step(channel, 1, 13, +1);
        application->setChannel(static_cast<uint16_t>(channel));
        prefs.putInt("channel", channel);
        changed = true;
    // Physical A / left-upper (GPIO10) is reported as BtnB.
    } else if (M5.BtnB.wasHold()) {
        tx_pitch_mode = static_cast<uint8_t>(wrapped_step(
            static_cast<int>(tx_pitch_mode),
            static_cast<int>(Application::kTxPitchModeM1),
            static_cast<int>(Application::kTxPitchModeM3),
            +1));
        application->setTxPitchMode(tx_pitch_mode);
        prefs.putInt("txmode", tx_pitch_mode);
        changed = true;
    } else if (M5.BtnB.wasClicked()) {
        volume_level = wrapped_step(volume_level, 1, 5, +1);
        application->setSpeakerVolume(current_speaker_gain());
        prefs.putInt("volume", volume_level);
        changed = true;
    }

    if (changed) {
        papercolor_ui_update_settings(channel, volume_level, tx_pitch_mode, application->getRSSI());
        settings_dirty = true;
        settings_changed_ms = millis();
    }
    if (settings_dirty && papercolor_ui_settings_visible() &&
        millis() - settings_changed_ms >= kSettingsCommitDelayMs) {
        papercolor_ui_show_settings();
        settings_dirty = false;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}
#endif

}  // namespace

void setup()
{
    Serial.begin(115200);
    auto cfg = M5.config();
#if TALKIE_TARGET_M5STICKS3 || TALKIE_TARGET_M5PAPERCOLOR
    cfg.output_power = false;
#else
    cfg.output_power = true;
#endif
#if TALKIE_TARGET_M5PAPERCOLOR
    // Keep the retained E-Ink image visible until the background display task
    // has finished rendering the next page.
    cfg.clear_display = false;
#endif
#if M5UNIFIED_USE_ATOMIC_ECHO_BASE
    cfg.external_speaker.atomic_echo = true;
#endif
    M5.begin(cfg);
#if TALKIE_TARGET_M5PAPERCOLOR
    pinMode(kPaperButtonAPin, INPUT_PULLUP);
    pinMode(kPaperButtonBPin, INPUT_PULLUP);
    pinMode(kPaperButtonCPin, INPUT_PULLUP);
    M5.Display.setRotation(0);
    M5.Display.setEpdMode(epd_mode_t::epd_quality);
#endif

    prefs.begin("esptalkie", false);
    channel = prefs.getInt("channel", 1);
    if (channel < 1 || channel > 13) channel = 1;
    volume_level = prefs.getInt("volume", kDefaultVolumeLevel);
    if (volume_level < 1 || volume_level > 5) volume_level = 3;
#if PTT_LOCAL_PLAYBACK_TEST_MODE
    volume_level = 5;
#endif
    tx_pitch_mode = prefs.getInt("txmode", Application::kTxPitchModeM1);
    if (tx_pitch_mode < Application::kTxPitchModeM1 || tx_pitch_mode > Application::kTxPitchModeM3) {
        tx_pitch_mode = Application::kTxPitchModeM1;
    }

#if !PTT_LOCAL_PLAYBACK_TEST_MODE && !TALKIE_TARGET_M5PAPERCOLOR
    draw_layout();
#elif PTT_LOCAL_PLAYBACK_TEST_MODE
    display_lock();
    M5.Display.setRotation(1);
    M5.Display.fillScreen(TFT_BLACK);
    display_unlock();
#endif
    Serial.printf("Detected board=%d, display=%dx%d\n",
                  static_cast<int>(M5.getBoard()),
                  M5.Display.width(), M5.Display.height());

    application = new Application();
    application->setChannel(static_cast<uint16_t>(channel));
    application->setSpeakerVolume(current_speaker_gain());
    application->setTxPitchMode(tx_pitch_mode);
    Serial.printf("VOL level=%d mapped=%u applied=%u\n",
                  volume_level,
                  static_cast<unsigned>(current_speaker_gain()),
                  static_cast<unsigned>(application->getSpeakerVolume()));
    mode_selected_at_ms = millis();
    application->begin();
#if TALKIE_TARGET_M5PAPERCOLOR
    papercolor_ui_begin(channel, volume_level, tx_pitch_mode);
#endif
#if !PTT_LOCAL_PLAYBACK_TEST_MODE
    application->dispStatus(false);
#endif

    Serial.println("ESP32Talkie Application started");
}

void loop()
{
    M5.update();
#if TALKIE_TARGET_M5PAPERCOLOR
    papercolor_loop();
    return;
#endif
#if PTT_LOCAL_PLAYBACK_TEST_MODE
    vTaskDelay(pdMS_TO_TICKS(5));
    return;
#endif
    const ShakeAction shake_action = detect_shake_action();
    if (edit_mode != EditMode::None &&
        (millis() - mode_selected_at_ms >= kModeAutoClearMs)) {
        edit_mode = EditMode::None;
        draw_channel();
        draw_volume();
    }

    if (M5.BtnB.wasHold() || shake_action == ShakeAction::SwitchMode) {
        if (edit_mode == EditMode::None) {
            edit_mode = EditMode::Volume;
        } else if (edit_mode == EditMode::Volume) {
            edit_mode = EditMode::Channel;
        } else if (edit_mode == EditMode::Channel) {
            edit_mode = EditMode::Mode;
        } else {
            edit_mode = EditMode::Volume;
        }
        mode_selected_at_ms = millis();
        draw_channel();
        draw_volume();
    } else {
        int delta = 0;
        if (M5.BtnB.wasClicked()) {
            delta = +1;
        } else if (shake_action == ShakeAction::Increase) {
            delta = +1;
        } else if (shake_action == ShakeAction::Decrease) {
            delta = -1;
        }
        if (delta == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        if (edit_mode == EditMode::None) {
            vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }

        if (edit_mode == EditMode::Volume) {
            volume_level = wrapped_step(volume_level, 1, 5, delta);
            application->setSpeakerVolume(current_speaker_gain());
            prefs.putInt("volume", volume_level);
            mode_selected_at_ms = millis();
            draw_volume();
        } else if (edit_mode == EditMode::Channel) {
            channel = wrapped_step(channel, 1, 13, delta);
            application->setChannel(static_cast<uint16_t>(channel));
            prefs.putInt("channel", channel);
            mode_selected_at_ms = millis();
            draw_channel();
        } else {
            tx_pitch_mode = static_cast<uint8_t>(
                wrapped_step(static_cast<int>(tx_pitch_mode),
                             static_cast<int>(Application::kTxPitchModeM1),
                             static_cast<int>(Application::kTxPitchModeM3),
                             delta));
            application->setTxPitchMode(tx_pitch_mode);
            prefs.putInt("txmode", tx_pitch_mode);
            mode_selected_at_ms = millis();
            draw_channel();
        }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}
