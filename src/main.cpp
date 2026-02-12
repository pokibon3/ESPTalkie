/*
 * main.cpp
 */

#include <Arduino.h>
#include <M5Unified.h>
#include <nvs.h>
#include <Preferences.h>

#include "Application.h"
#include "DisplaySync.h"

namespace {

Application *application = nullptr;
int channel = 1;
int volume_level = 3;  // 1..5
bool volume_mode = true;  // true: volume adjust, false: channel adjust
constexpr uint8_t kVolumeTable[5] = { 44, 66, 88, 110, 132 };
Preferences prefs;

constexpr int kChannelX = 8;
constexpr int kChannelY = 28;
constexpr int kChannelW = 119;
constexpr int kChannelH = 88;
constexpr int kInfoY = 122;
constexpr int kInfoH = 44;
constexpr int kInfoW = 56;
constexpr int kVolumeX = 8;
constexpr int kRssiX = 71;

void draw_channel()
{
    display_lock();
    auto panel = M5.Display.color565(18, 32, 62);
    auto accent = M5.Display.color565(90, 190, 255);
    auto text = M5.Display.color565(235, 245, 255);
    auto active = BLUE;
    auto inactive = M5.Display.color565(160, 205, 255);

    M5.Display.fillRoundRect(kChannelX, kChannelY, kChannelW, kChannelH, 10, panel);
    M5.Display.drawRoundRect(kChannelX, kChannelY, kChannelW, kChannelH, 10, accent);
    M5.Display.setTextColor(volume_mode ? inactive : active, panel);
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(top_center);
    M5.Display.drawString("CHANNEL", kChannelX + (kChannelW / 2), kChannelY + 6);

    M5.Display.setTextColor(volume_mode ? text : active, panel);
    M5.Display.setFont(&fonts::Font7);
    M5.Display.setTextSize(1);
    char ch_text[4];
    snprintf(ch_text, sizeof(ch_text), "%02d", channel);
    M5.Display.setTextDatum(middle_center);
    M5.Display.drawString(ch_text, kChannelX + (kChannelW / 2), kChannelY + 54);
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextDatum(top_left);
    display_unlock();
}

void draw_volume()
{
    display_lock();
    auto panel = M5.Display.color565(18, 32, 62);
    auto accent = M5.Display.color565(90, 190, 255);
    auto text = M5.Display.color565(235, 245, 255);
    auto sub = M5.Display.color565(160, 205, 255);
    auto active = BLUE;
    auto mode_color = volume_mode ? M5.Display.color565(110, 230, 150) : M5.Display.color565(255, 200, 120);

    M5.Display.fillRoundRect(kVolumeX, kInfoY, kInfoW, kInfoH, 8, panel);
    M5.Display.drawRoundRect(kVolumeX, kInfoY, kInfoW, kInfoH, 8, accent);
    M5.Display.setTextColor(volume_mode ? active : sub, panel);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(kVolumeX + 8, kInfoY + 8);
    M5.Display.print("VOL");

    M5.Display.setTextColor(volume_mode ? active : text, panel);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(kVolumeX + 14, kInfoY + 22);
    M5.Display.printf("%d", volume_level);

    M5.Display.setTextSize(1);
    M5.Display.setTextColor(mode_color, panel);
    M5.Display.setCursor(kVolumeX + 34, kInfoY + 30);
    M5.Display.print(volume_mode ? "VOL" : "CH");
    display_unlock();
}

void draw_layout()
{
    display_lock();
    auto bg = M5.Display.color565(10, 18, 36);
    auto accent = M5.Display.color565(90, 190, 255);

    M5.Display.fillScreen(bg);

    M5.Display.drawFastHLine(0, 22, M5.Display.width(), accent);

    draw_channel();
    draw_volume();

    // RSSI value box (right side of info row)
    auto panel = M5.Display.color565(18, 32, 62);
    M5.Display.fillRoundRect(kRssiX, kInfoY, kInfoW, kInfoH, 8, panel);
    M5.Display.drawRoundRect(kRssiX, kInfoY, kInfoW, kInfoH, 8, accent);

    // Level bar area (bottom)
    M5.Display.fillRoundRect(8, 170, 119, 62, 8, M5.Display.color565(232, 250, 255));
    M5.Display.drawRoundRect(8, 170, 119, 62, 8, accent);
    display_unlock();
}

}  // namespace

void setup()
{
    Serial.begin(115200);

    // Clear cached board autodetect result so M5GFX_BOARD is applied.
    nvs_handle_t nvs_handle;
    if (nvs_open("m5gfx", NVS_READWRITE, &nvs_handle) == ESP_OK) {
        nvs_erase_key(nvs_handle, "AUTODETECT");
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }

    auto cfg = M5.config();
    cfg.fallback_board = m5::board_t::board_M5StickS3;
    M5.begin(cfg);

    M5.Display.setRotation(0);

    prefs.begin("esptalkie", false);
    channel = prefs.getInt("channel", 1);
    if (channel < 1 || channel > 13) channel = 1;
    volume_level = prefs.getInt("volume", 3);
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
