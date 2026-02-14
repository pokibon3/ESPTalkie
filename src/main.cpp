/*
 * main.cpp
 */

#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>

#include "Application.h"
#include "DisplaySync.h"
#include "UiLayout.h"
#include "config.h"

namespace {

Application *application = nullptr;
int channel = 1;
int volume_level = 3;  // 1..5
bool volume_mode = true;  // true: volume adjust, false: channel adjust
constexpr uint8_t kVolumeTable[5] = { 44, 66, 88, 110, 132 };
Preferences prefs;
#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
constexpr int kDefaultVolumeLevel = 1;
#else
constexpr int kDefaultVolumeLevel = 3;
#endif

void draw_channel()
{
    display_lock();
    const uint16_t panel = M5.Display.color565(44, 52, 62);
    const uint16_t accent = TFT_BLUE;
    const uint16_t text = TFT_WHITE;
    const uint16_t active = TFT_GREEN;
    const uint16_t inactive = TFT_WHITE;

    M5.Display.fillRoundRect(kUiLayout.channel_x, kUiLayout.channel_y, kUiLayout.channel_w, kUiLayout.channel_h, kUiLayout.channel_radius, panel);
    M5.Display.drawRoundRect(kUiLayout.channel_x, kUiLayout.channel_y, kUiLayout.channel_w, kUiLayout.channel_h, kUiLayout.channel_radius, accent);
    M5.Display.setTextColor(volume_mode ? inactive : active, panel);
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(top_left);
#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
    constexpr const char* kChannelLabel = "CH";
#else
    constexpr const char* kChannelLabel = "CHANNEL";
#endif
    M5.Display.drawString(kChannelLabel, kUiLayout.channel_x + kUiLayout.channel_label_x, kUiLayout.channel_y + kUiLayout.channel_label_y);

    M5.Display.setTextColor(volume_mode ? text : active, panel);
    M5.Display.setFont(kUiLayout.channel_compact_font ? &fonts::Font6 : &fonts::Font7);
    M5.Display.setTextSize(1);
    char ch_text[4];
    snprintf(ch_text, sizeof(ch_text), "%02d", channel);
    M5.Display.setTextDatum(middle_center);
    M5.Display.drawString(ch_text, kUiLayout.channel_x + (kUiLayout.channel_w / 2), kUiLayout.channel_y + kUiLayout.channel_value_y);
    M5.Display.setFont(&fonts::Font0);
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
    M5.Display.setTextColor(volume_mode ? active : sub, panel);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(kUiLayout.volume_x + kUiLayout.volume_label_x, kUiLayout.info_y + kUiLayout.volume_label_y);
    M5.Display.print("VOL");

    M5.Display.setTextColor(volume_mode ? active : text, panel);
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

}  // namespace

void setup()
{
    Serial.begin(115200);
    auto cfg = M5.config();
#if M5UNIFIED_USE_ATOMIC_ECHO_BASE
    cfg.external_speaker.atomic_echo = true;
#endif
    M5.begin(cfg);

    prefs.begin("esptalkie", false);
    channel = prefs.getInt("channel", 1);
    if (channel < 1 || channel > 13) channel = 1;
    volume_level = prefs.getInt("volume", kDefaultVolumeLevel);
    if (volume_level < 1 || volume_level > 5) volume_level = 3;

    draw_layout();
    Serial.printf("Detected board=%d, display=%dx%d\n",
                  static_cast<int>(M5.getBoard()),
                  M5.Display.width(), M5.Display.height());

    application = new Application();
    application->setChannel(static_cast<uint16_t>(channel));
    application->setSpeakerVolume(kVolumeTable[volume_level - 1]);
    application->begin();
    application->dispStatus(false);

    Serial.println("M5StickS3 Walkie Talkie Application started");
}

void loop()
{
    M5.update();

    if (M5.BtnB.wasHold()) {
        volume_mode = !volume_mode;
        draw_channel();
        draw_volume();
    } else if (M5.BtnB.wasClicked()) {
        if (volume_mode) {
            volume_level++;
            if (volume_level > 5) {
                volume_level = 1;
            }
            application->setSpeakerVolume(kVolumeTable[volume_level - 1]);
            prefs.putInt("volume", volume_level);
            draw_volume();
        } else {
            channel++;
            if (channel > 13) {
                channel = 1;
            }
            application->setChannel(static_cast<uint16_t>(channel));
            prefs.putInt("channel", channel);
            draw_channel();
        }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}
