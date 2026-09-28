#include "Tab5Ui.h"

#include "config.h"

#if TALKIE_TARGET_M5TAB5

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>  // must precede M5Unified so M5GFX enables fs::FS image loaders
#include <M5Unified.h>

#include <algorithm>
#include <string>
#include <vector>

#include "DisplaySync.h"

namespace {

struct Rect {
    int x, y, w, h;
    bool contains(int px, int py) const
    {
        return px >= x && px < x + w && py >= y && py < y + h;
    }
};

// ── Palette (matches the small-screen UI) ─────────────────────────────────
uint16_t c_bg, c_panel, c_accent, c_text, c_sub, c_active, c_btn, c_btn_pressed;

// ── Layout (computed for the landscape panel in tab5_ui_begin) ────────────
Rect r_image;
Rect r_status;
Rect r_ch_down, r_ch_val, r_ch_up;
Rect r_vol_down, r_vol_val, r_vol_up;
Rect r_mode[3];
Rect r_signal;
Rect r_ptt;

// ── State ─────────────────────────────────────────────────────────────────
int s_channel = 1;
int s_volume = 3;
uint8_t s_mode = 1;
bool s_tx = false;
bool s_cont = false;
int16_t s_rssi = -127;
int16_t s_tx_dbm = 0;
volatile bool s_ptt_touched = false;
bool s_ptt_drawn_pressed = false;
bool s_ready = false;

// ── Slideshow ─────────────────────────────────────────────────────────────
TaskHandle_t s_slide_task = nullptr;

void draw_button(const Rect &r, const char *label, bool pressed, const lgfx::IFont *font = &fonts::FreeSansBold18pt7b)
{
    const uint16_t fill = pressed ? c_btn_pressed : c_btn;
    M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 14, fill);
    M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 14, c_accent);
    M5.Display.setFont(font);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(c_text, fill);
    M5.Display.drawString(label, r.x + r.w / 2, r.y + r.h / 2);
}

void draw_value_box(const Rect &r, const char *label, const char *value)
{
    M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 14, c_panel);
    M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 14, c_accent);
    M5.Display.setTextDatum(top_center);
    M5.Display.setFont(&fonts::FreeSans12pt7b);
    M5.Display.setTextColor(c_sub, c_panel);
    M5.Display.drawString(label, r.x + r.w / 2, r.y + 8);
    M5.Display.setTextDatum(bottom_center);
    M5.Display.setFont(&fonts::Font7);
    M5.Display.setTextColor(c_text, c_panel);
    M5.Display.drawString(value, r.x + r.w / 2, r.y + r.h - 8);
}

void draw_channel_row()
{
    char v[4];
    snprintf(v, sizeof(v), "%02d", s_channel);
    draw_button(r_ch_down, "-", false, &fonts::FreeSansBold24pt7b);
    draw_value_box(r_ch_val, "CHANNEL", v);
    draw_button(r_ch_up, "+", false, &fonts::FreeSansBold24pt7b);
}

void draw_volume_row()
{
    char v[4];
    snprintf(v, sizeof(v), "%d", s_volume);
    draw_button(r_vol_down, "-", false, &fonts::FreeSansBold24pt7b);
    draw_value_box(r_vol_val, "VOLUME", v);
    draw_button(r_vol_up, "+", false, &fonts::FreeSansBold24pt7b);
}

void draw_mode_row()
{
    static const char *kLabels[3] = { "M1", "M2", "M3" };
    for (int i = 0; i < 3; ++i) {
        const Rect &r = r_mode[i];
        const bool sel = (s_mode == i + 1);
        const uint16_t fill = sel ? c_active : c_btn;
        M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 14, fill);
        M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 14, c_accent);
        M5.Display.setFont(&fonts::FreeSansBold18pt7b);
        M5.Display.setTextDatum(middle_center);
        M5.Display.setTextColor(sel ? TFT_BLACK : c_text, fill);
        M5.Display.drawString(kLabels[i], r.x + r.w / 2, r.y + r.h / 2);
    }
}

void draw_status()
{
    const uint16_t color = s_tx ? TFT_RED : TFT_BLUE;
    M5.Display.fillRect(r_status.x, r_status.y, r_status.w, r_status.h, color);
    M5.Display.setFont(&fonts::FreeSansBold24pt7b);
    M5.Display.setTextDatum(middle_left);
    M5.Display.setTextColor(TFT_WHITE, color);
    const char *label = s_tx ? (s_cont ? "CONT TX" : "TRANSMIT") : "RECEIVE";
    M5.Display.drawString(label, r_status.x + 16, r_status.y + r_status.h / 2);

    const int32_t batt = M5.Power.getBatteryLevel();
    if (batt >= 0) {
        char b[8];
        snprintf(b, sizeof(b), "%d%%", static_cast<int>(batt));
        M5.Display.setFont(&fonts::FreeSans12pt7b);
        M5.Display.setTextDatum(middle_right);
        M5.Display.drawString(b, r_status.x + r_status.w - 14, r_status.y + r_status.h / 2);
    }
}

void draw_signal()
{
    static const int16_t kLevels[8] = { -90, -80, -70, -60, -50, -40, -30, -20 };
    const Rect &r = r_signal;
    M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 14, c_panel);
    M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 14, c_accent);

    M5.Display.setFont(&fonts::FreeSans12pt7b);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextColor(c_sub, c_panel);
    M5.Display.drawString(s_tx ? "TX dBm" : "RSSI", r.x + 14, r.y + 8);
    char v[8];
    snprintf(v, sizeof(v), "%d", s_tx ? s_tx_dbm : s_rssi);
    M5.Display.setFont(&fonts::FreeSansBold24pt7b);
    M5.Display.setTextColor(c_text, c_panel);
    M5.Display.setTextDatum(bottom_left);
    M5.Display.drawString(v, r.x + 14, r.y + r.h - 6);

    const int bars_x = r.x + 150;
    const int bars_w = r.w - 150 - 14;
    const int gap = 6;
    const int bw = (bars_w - gap * 7) / 8;
    const int base = r.y + r.h - 10;
    const int max_h = r.h - 20;
    for (int i = 0; i < 8; ++i) {
        const int h = max_h * (i + 3) / 10;
        bool on;
        if (s_tx) {
            on = s_tx_dbm >= (i * 3);  // 0..21 dBm in 3 dB steps
        } else {
            on = s_rssi >= kLevels[i];
        }
        const uint16_t color = on ? (i < 5 ? TFT_GREEN : TFT_RED) : TFT_BLACK;
        M5.Display.fillRoundRect(bars_x + i * (bw + gap), base - h, bw, h, 3, color);
    }
}

void draw_ptt(bool pressed)
{
    const uint16_t fill = pressed ? TFT_RED : M5.Display.color565(150, 20, 20);
    M5.Display.fillRoundRect(r_ptt.x, r_ptt.y, r_ptt.w, r_ptt.h, 24, fill);
    M5.Display.drawRoundRect(r_ptt.x, r_ptt.y, r_ptt.w, r_ptt.h, 24, TFT_WHITE);
    M5.Display.setFont(&fonts::FreeSansBold24pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(TFT_WHITE, fill);
    M5.Display.drawString("PTT", r_ptt.x + r_ptt.w / 2, r_ptt.y + r_ptt.h / 2 - 12);
    M5.Display.setFont(&fonts::FreeSans9pt7b);
    M5.Display.drawString("double-tap: continuous", r_ptt.x + r_ptt.w / 2, r_ptt.y + r_ptt.h / 2 + 26);
}

void draw_all_controls()
{
    display_lock();
    M5.Display.fillRect(r_status.x, 0, M5.Display.width() - r_status.x, M5.Display.height(), c_bg);
    draw_status();
    draw_channel_row();
    draw_volume_row();
    draw_mode_row();
    draw_signal();
    draw_ptt(false);
    display_unlock();
}

void flash_button(const Rect &r, const char *label)
{
    display_lock();
    draw_button(r, label, true, &fonts::FreeSansBold24pt7b);
    display_unlock();
}

// ── Slideshow ─────────────────────────────────────────────────────────────

bool is_image_name(const std::string &name)
{
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    if (!lower.empty() && lower[0] == '.') return false;  // macOS "._" files etc.
    auto ends = [&](const char *ext) {
        const size_t n = strlen(ext);
        return lower.size() > n && lower.compare(lower.size() - n, n, ext) == 0;
    };
    return ends(".jpg") || ends(".jpeg") || ends(".png") || ends(".bmp");
}

std::vector<std::string> list_images()
{
    std::vector<std::string> files;
    File dir = SD_MMC.open(TAB5_SLIDESHOW_DIR);
    if (!dir || !dir.isDirectory()) {
        return files;
    }
    File f = dir.openNextFile();
    while (f) {
        if (!f.isDirectory()) {
            std::string name = f.name();
            const size_t slash = name.find_last_of('/');
            if (slash != std::string::npos) name = name.substr(slash + 1);
            if (is_image_name(name)) {
                files.push_back(std::string(TAB5_SLIDESHOW_DIR) + "/" + name);
            }
        }
        f.close();
        f = dir.openNextFile();
    }
    dir.close();
    std::sort(files.begin(), files.end());
    return files;
}

bool draw_image(M5Canvas &canvas, const std::string &path)
{
    std::string lower = path;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    const int w = canvas.width();
    const int h = canvas.height();
    canvas.fillScreen(TFT_BLACK);
    // scale_x = 0: fit inside w x h keeping the aspect ratio.
    if (lower.size() >= 4 && lower.compare(lower.size() - 4, 4, ".png") == 0) {
        return canvas.drawPngFile(SD_MMC, path.c_str(), 0, 0, w, h, 0, 0, 0.0f, 0.0f, middle_center);
    }
    if (lower.size() >= 4 && lower.compare(lower.size() - 4, 4, ".bmp") == 0) {
        return canvas.drawBmpFile(SD_MMC, path.c_str(), 0, 0, w, h, 0, 0, 0.0f, 0.0f, middle_center);
    }
    return canvas.drawJpgFile(SD_MMC, path.c_str(), 0, 0, w, h, 0, 0, 0.0f, 0.0f, middle_center);
}

void draw_placeholder(M5Canvas &canvas, const char *line2)
{
    canvas.fillScreen(c_bg);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(c_text);
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.drawString("ESPTalkie", canvas.width() / 2, canvas.height() / 2 - 30);
    canvas.setFont(&fonts::FreeSans12pt7b);
    canvas.setTextColor(c_sub);
    canvas.drawString(line2, canvas.width() / 2, canvas.height() / 2 + 30);
}

void slideshow_task(void *)
{
    M5Canvas canvas(&M5.Display);
    canvas.setColorDepth(16);
    canvas.setPsram(true);
    if (!canvas.createSprite(r_image.w, r_image.h)) {
        Serial.println("Tab5: failed to allocate slideshow canvas");
        vTaskDelete(nullptr);
    }

    auto push = [&]() {
        display_lock();
        canvas.pushSprite(r_image.x, r_image.y);
        display_unlock();
    };

    const bool sd_ok = SD_MMC.begin("/sdcard", false);
    if (!sd_ok) {
        Serial.println("Tab5: SD card not mounted");
        draw_placeholder(canvas, "No SD card");
        push();
        vTaskDelete(nullptr);
    }

    size_t index = 0;
    std::vector<std::string> files;
    while (true) {
        if (index == 0) {
            files = list_images();  // rescan once per cycle
        }
        if (files.empty()) {
            draw_placeholder(canvas, "Put images in /images on the SD card");
            push();
            vTaskDelay(pdMS_TO_TICKS(TAB5_SLIDESHOW_INTERVAL_MS));
            continue;
        }
        if (index >= files.size()) index = 0;
        const std::string &path = files[index];
        const uint32_t t0 = millis();
        if (draw_image(canvas, path)) {
            push();
            Serial.printf("Tab5: slide %s (%lu ms)\n", path.c_str(), static_cast<unsigned long>(millis() - t0));
        } else {
            Serial.printf("Tab5: failed to decode %s\n", path.c_str());
        }
        index = (index + 1) % files.size();
        vTaskDelay(pdMS_TO_TICKS(TAB5_SLIDESHOW_INTERVAL_MS));
    }
}

}  // namespace

void tab5_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode)
{
    s_channel = channel;
    s_volume = volume_level;
    s_mode = tx_pitch_mode;

    display_lock();
    // Tab5 panel is 720x1280 portrait; use landscape.
    M5.Display.setRotation(1);
    if (M5.Display.width() < M5.Display.height()) {
        M5.Display.setRotation(0);
    }
    c_bg = M5.Display.color565(10, 18, 36);
    c_panel = M5.Display.color565(44, 52, 62);
    c_accent = TFT_BLUE;
    c_text = TFT_WHITE;
    c_sub = M5.Display.color565(160, 205, 255);
    c_active = TFT_GREEN;
    c_btn = M5.Display.color565(30, 60, 110);
    c_btn_pressed = M5.Display.color565(80, 140, 230);
    M5.Display.fillScreen(c_bg);
    display_unlock();

    const int W = M5.Display.width();
    const int H = M5.Display.height();
    const int col_w = 420;
    const int x0 = W - col_w;
    const int pad = 12;
    const int cw = col_w - pad * 2;
    const int cx = x0 + pad;

    r_image = { 0, 0, x0, H };
    r_status = { x0, 0, col_w, 80 };
    int y = r_status.h + pad;
    const int row_h = 120;
    const int btn_w = 100;
    r_ch_down = { cx, y, btn_w, row_h };
    r_ch_val = { cx + btn_w + pad, y, cw - 2 * (btn_w + pad), row_h };
    r_ch_up = { cx + cw - btn_w, y, btn_w, row_h };
    y += row_h + pad;
    r_vol_down = { cx, y, btn_w, row_h };
    r_vol_val = { cx + btn_w + pad, y, cw - 2 * (btn_w + pad), row_h };
    r_vol_up = { cx + cw - btn_w, y, btn_w, row_h };
    y += row_h + pad;
    const int mode_h = 90;
    const int mode_w = (cw - pad * 2) / 3;
    for (int i = 0; i < 3; ++i) {
        r_mode[i] = { cx + i * (mode_w + pad), y, mode_w, mode_h };
    }
    y += mode_h + pad;
    r_signal = { cx, y, cw, 80 };
    y += r_signal.h + pad;
    r_ptt = { cx, y, cw, H - y - pad };

    draw_all_controls();
    s_ready = true;

    xTaskCreatePinnedToCore(slideshow_task, "tab5_slides", 8192, nullptr, 0, &s_slide_task, 0);
}

Tab5Action tab5_ui_poll()
{
    if (!s_ready) {
        return Tab5Action::None;
    }
    Tab5Action action = Tab5Action::None;
    bool ptt = false;

    const size_t n = M5.Touch.getCount();
    for (size_t i = 0; i < n; ++i) {
        const auto &t = M5.Touch.getDetail(i);
        // PTT follows the finger: sliding out of the button releases it.
        if (t.isPressed() && r_ptt.contains(t.x, t.y)) {
            ptt = true;
        }
        if (!t.wasPressed()) {
            continue;
        }
        const Rect *flash = nullptr;
        const char *flash_label = nullptr;
        if (r_ch_down.contains(t.x, t.y)) {
            action = Tab5Action::ChannelDown; flash = &r_ch_down; flash_label = "-";
        } else if (r_ch_up.contains(t.x, t.y)) {
            action = Tab5Action::ChannelUp; flash = &r_ch_up; flash_label = "+";
        } else if (r_vol_down.contains(t.x, t.y)) {
            action = Tab5Action::VolumeDown; flash = &r_vol_down; flash_label = "-";
        } else if (r_vol_up.contains(t.x, t.y)) {
            action = Tab5Action::VolumeUp; flash = &r_vol_up; flash_label = "+";
        } else if (r_mode[0].contains(t.x, t.y)) {
            action = Tab5Action::Mode1;
        } else if (r_mode[1].contains(t.x, t.y)) {
            action = Tab5Action::Mode2;
        } else if (r_mode[2].contains(t.x, t.y)) {
            action = Tab5Action::Mode3;
        }
        if (flash) {
            flash_button(*flash, flash_label);
        }
    }

    s_ptt_touched = ptt;
    if (ptt != s_ptt_drawn_pressed) {
        s_ptt_drawn_pressed = ptt;
        display_lock();
        draw_ptt(ptt);
        display_unlock();
    }
    return action;
}

bool tab5_ui_ptt_pressed()
{
    return s_ptt_touched;
}

void tab5_ui_set_settings(int channel, int volume_level, uint8_t tx_pitch_mode)
{
    const bool ch = channel != s_channel;
    const bool vol = volume_level != s_volume;
    const bool mode = tx_pitch_mode != s_mode;
    s_channel = channel;
    s_volume = volume_level;
    s_mode = tx_pitch_mode;
    if (!s_ready) return;
    display_lock();
    // Always redraw +/- rows: this also clears the pressed flash.
    draw_channel_row();
    draw_volume_row();
    if (mode) draw_mode_row();
    (void)ch;
    (void)vol;
    display_unlock();
}

void tab5_ui_set_status(bool transmitting, bool continuous)
{
    if (!s_ready) return;
    if (transmitting == s_tx && continuous == s_cont) {
        return;
    }
    s_tx = transmitting;
    s_cont = continuous;
    display_lock();
    draw_status();
    draw_signal();
    display_unlock();
}

void tab5_ui_set_rssi(int16_t rssi)
{
    if (!s_ready) return;
    static uint32_t last_batt_ms = 0;
    const uint32_t now = millis();
    const bool batt_due = now - last_batt_ms > 30000;
    if (rssi == s_rssi && !batt_due) {
        return;
    }
    s_rssi = rssi;
    display_lock();
    if (!s_tx) draw_signal();
    if (batt_due) {
        last_batt_ms = now;
        draw_status();
    }
    display_unlock();
}

void tab5_ui_set_tx_power(int16_t dbm)
{
    if (!s_ready) return;
    s_tx_dbm = dbm;
    display_lock();
    if (s_tx) draw_signal();
    display_unlock();
}

void tab5_ui_message(const char *msg)
{
    display_lock();
    if (M5.Display.width() < M5.Display.height()) {
        M5.Display.setRotation(1);
    }
    const uint16_t bg = M5.Display.color565(20, 20, 20);
    M5.Display.fillRect(0, 0, M5.Display.width(), 80, bg);
    M5.Display.setFont(&fonts::FreeSansBold18pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(TFT_YELLOW, bg);
    M5.Display.drawString(msg, M5.Display.width() / 2, 40);
    display_unlock();
    Serial.printf("Tab5: %s\n", msg);
}

#else  // !TALKIE_TARGET_M5TAB5

void tab5_ui_begin(int, int, uint8_t) {}
Tab5Action tab5_ui_poll() { return Tab5Action::None; }
bool tab5_ui_ptt_pressed() { return false; }
void tab5_ui_set_settings(int, int, uint8_t) {}
void tab5_ui_set_status(bool, bool) {}
void tab5_ui_set_rssi(int16_t) {}
void tab5_ui_set_tx_power(int16_t) {}
void tab5_ui_message(const char *) {}

#endif
