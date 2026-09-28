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

// Screen layout (landscape 1280x720)
//
//  Main screen : SD image full screen, overlay at the bottom:
//                [ level meter ][   PTT   ]              [SETUP]
//  Setup panel : CHANNEL -/+, VOLUME -/+, VOICE M1/M2/M3,   [CLOSE]
//                (CLOSE sits where SETUP is, so the same spot toggles)

namespace {

struct Rect {
    int x, y, w, h;
    bool contains(int px, int py) const
    {
        return px >= x && px < x + w && py >= y && py < y + h;
    }
};

// ── Palette ───────────────────────────────────────────────────────────────
uint16_t c_bg, c_panel, c_accent, c_text, c_sub, c_active, c_btn, c_btn_pressed;

// ── Layout ────────────────────────────────────────────────────────────────
int W = 1280, H = 720;
// main screen overlay
Rect r_meter, r_ptt, r_setup;
// setup panel
Rect r_title;
Rect r_ch_down, r_ch_val, r_ch_up;
Rect r_vol_down, r_vol_val, r_vol_up;
Rect r_mode[3];
Rect r_info;

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
volatile bool s_panel_open = false;

TaskHandle_t s_slide_task = nullptr;

// ── Drawing helpers ───────────────────────────────────────────────────────

void draw_button(const Rect &r, const char *label, bool pressed,
                 const lgfx::IFont *font = &fonts::FreeSansBold24pt7b,
                 uint16_t fill_normal = 0, uint16_t text_color = TFT_WHITE)
{
    const uint16_t fill = pressed ? c_btn_pressed : (fill_normal ? fill_normal : c_btn);
    M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 16, fill);
    M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 16, c_accent);
    M5.Display.setFont(font);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(text_color, fill);
    M5.Display.drawString(label, r.x + r.w / 2, r.y + r.h / 2);
}

void draw_value_box(const Rect &r, const char *label, const char *value)
{
    M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 16, c_panel);
    M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 16, c_accent);
    M5.Display.setTextDatum(top_center);
    M5.Display.setFont(&fonts::FreeSans12pt7b);
    M5.Display.setTextColor(c_sub, c_panel);
    M5.Display.drawString(label, r.x + r.w / 2, r.y + 10);
    M5.Display.setTextDatum(bottom_center);
    M5.Display.setFont(&fonts::Font7);
    M5.Display.setTextColor(c_text, c_panel);
    M5.Display.drawString(value, r.x + r.w / 2, r.y + r.h - 10);
}

// Level meter: RX = RSSI, TX = TX power. Also shows state and settings.
void draw_meter()
{
    static const int16_t kLevels[8] = { -90, -80, -70, -60, -50, -40, -30, -20 };
    const Rect &r = r_meter;
    const uint16_t border = s_tx ? TFT_RED : c_accent;
    M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 16, c_panel);
    M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 16, border);
    M5.Display.drawRoundRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2, 15, border);

    // top line: state + settings
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextColor(s_tx ? TFT_RED : TFT_GREEN, c_panel);
    M5.Display.drawString(s_tx ? (s_cont ? "CONT TX" : "TX") : "RX", r.x + 14, r.y + 10);
    char info[32];
    snprintf(info, sizeof(info), "CH%02d  VOL%d  M%u", s_channel, s_volume, static_cast<unsigned>(s_mode));
    M5.Display.setFont(&fonts::FreeSans12pt7b);
    M5.Display.setTextDatum(top_right);
    M5.Display.setTextColor(c_sub, c_panel);
    M5.Display.drawString(info, r.x + r.w - 14, r.y + 10);

    // value
    char v[12];
    const bool has_signal = s_tx || s_rssi > -127;
    if (s_tx) {
        snprintf(v, sizeof(v), "%ddBm", s_tx_dbm);
    } else if (has_signal) {
        snprintf(v, sizeof(v), "%d", s_rssi);
    } else {
        snprintf(v, sizeof(v), "---");
    }
    M5.Display.setFont(&fonts::FreeSansBold18pt7b);
    M5.Display.setTextDatum(bottom_left);
    M5.Display.setTextColor(c_text, c_panel);
    M5.Display.drawString(v, r.x + 14, r.y + r.h - 10);

    // bars
    const int bars_x = r.x + 150;
    const int bars_w = r.w - 150 - 14;
    const int gap = 6;
    const int bw = (bars_w - gap * 7) / 8;
    const int base = r.y + r.h - 12;
    const int max_h = r.h - 50;
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
    const bool active = pressed || s_tx;
    const uint16_t fill = active ? TFT_RED : M5.Display.color565(150, 20, 20);
    M5.Display.fillRoundRect(r_ptt.x, r_ptt.y, r_ptt.w, r_ptt.h, 24, fill);
    M5.Display.drawRoundRect(r_ptt.x, r_ptt.y, r_ptt.w, r_ptt.h, 24, TFT_WHITE);
    M5.Display.drawRoundRect(r_ptt.x + 1, r_ptt.y + 1, r_ptt.w - 2, r_ptt.h - 2, 23, TFT_WHITE);
    M5.Display.setFont(&fonts::FreeSansBold24pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(TFT_WHITE, fill);
    M5.Display.drawString("PTT", r_ptt.x + r_ptt.w / 2, r_ptt.y + r_ptt.h / 2 - 10);
    M5.Display.setFont(&fonts::FreeSans9pt7b);
    M5.Display.drawString(s_cont ? "double-tap: stop" : "double-tap: continuous",
                          r_ptt.x + r_ptt.w / 2, r_ptt.y + r_ptt.h / 2 + 28);
}

void draw_setup_button(bool pressed)
{
    draw_button(r_setup, s_panel_open ? "CLOSE" : "SETUP", pressed, &fonts::FreeSansBold18pt7b);
}

void draw_overlay()
{
    draw_meter();
    draw_ptt(s_ptt_drawn_pressed);
    draw_setup_button(false);
}

// ── Setup panel ───────────────────────────────────────────────────────────

void draw_panel_title()
{
    const uint16_t color = s_tx ? TFT_RED : TFT_BLUE;
    M5.Display.fillRect(r_title.x, r_title.y, r_title.w, r_title.h, color);
    M5.Display.setFont(&fonts::FreeSansBold24pt7b);
    M5.Display.setTextDatum(middle_left);
    M5.Display.setTextColor(TFT_WHITE, color);
    M5.Display.drawString("SETUP", r_title.x + 24, r_title.y + r_title.h / 2);
    M5.Display.setFont(&fonts::FreeSans18pt7b);
    M5.Display.setTextDatum(middle_right);
    const char *state = s_tx ? (s_cont ? "CONT TX" : "TRANSMIT") : "RECEIVE";
    char right[40];
    const int32_t batt = M5.Power.getBatteryLevel();
    if (batt >= 0) {
        snprintf(right, sizeof(right), "%s   BATT %d%%", state, static_cast<int>(batt));
    } else {
        snprintf(right, sizeof(right), "%s", state);
    }
    M5.Display.drawString(right, r_title.x + r_title.w - 24, r_title.y + r_title.h / 2);
}

void draw_panel_channel()
{
    char v[4];
    snprintf(v, sizeof(v), "%02d", s_channel);
    draw_button(r_ch_down, "-", false);
    draw_value_box(r_ch_val, "CHANNEL", v);
    draw_button(r_ch_up, "+", false);
}

void draw_panel_volume()
{
    char v[4];
    snprintf(v, sizeof(v), "%d", s_volume);
    draw_button(r_vol_down, "-", false);
    draw_value_box(r_vol_val, "VOLUME", v);
    draw_button(r_vol_up, "+", false);
}

void draw_panel_mode()
{
    static const char *kLabels[3] = { "M1", "M2", "M3" };
    M5.Display.setFont(&fonts::FreeSans12pt7b);
    M5.Display.setTextDatum(bottom_left);
    M5.Display.setTextColor(c_sub, c_bg);
    M5.Display.drawString("VOICE", r_mode[0].x, r_mode[0].y - 8);
    for (int i = 0; i < 3; ++i) {
        const bool sel = (s_mode == i + 1);
        draw_button(r_mode[i], kLabels[i], false, &fonts::FreeSansBold24pt7b,
                    sel ? c_active : c_btn, sel ? TFT_BLACK : TFT_WHITE);
    }
}

void draw_panel_info()
{
    M5.Display.fillRect(r_info.x, r_info.y, r_info.w, r_info.h, c_bg);
    M5.Display.setFont(&fonts::FreeSans12pt7b);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextColor(c_sub, c_bg);
    char line[64];
    if (s_rssi > -127) {
        snprintf(line, sizeof(line), "Last RSSI: %d dBm", s_rssi);
    } else {
        snprintf(line, sizeof(line), "Last RSSI: ---");
    }
    M5.Display.drawString(line, r_info.x, r_info.y);
    M5.Display.drawString("Slideshow: SD " TAB5_SLIDESHOW_DIR " (JPG/PNG/BMP)", r_info.x, r_info.y + 32);
}

void draw_panel()
{
    M5.Display.fillScreen(c_bg);
    draw_panel_title();
    draw_panel_channel();
    draw_panel_volume();
    draw_panel_mode();
    draw_panel_info();
    draw_setup_button(false);
}

void flash_button(const Rect &r, const char *label)
{
    display_lock();
    draw_button(r, label, true);
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
    canvas.drawString("ESPTalkie", canvas.width() / 2, canvas.height() / 2 - 120);
    canvas.setFont(&fonts::FreeSans12pt7b);
    canvas.setTextColor(c_sub);
    canvas.drawString(line2, canvas.width() / 2, canvas.height() / 2 - 70);
}

// Push the current image and repaint the overlay on top (main screen only).
void present(M5Canvas &canvas)
{
    display_lock();
    if (!s_panel_open) {
        canvas.pushSprite(0, 0);
        draw_overlay();
    }
    display_unlock();
}

void slideshow_task(void *)
{
    M5Canvas canvas(&M5.Display);
    canvas.setColorDepth(16);
    canvas.setPsram(true);
    if (!canvas.createSprite(W, H)) {
        Serial.println("Tab5: failed to allocate slideshow canvas");
        vTaskDelete(nullptr);
    }

    const bool sd_ok = SD_MMC.begin("/sdcard", false);
    if (!sd_ok) {
        Serial.println("Tab5: SD card not mounted");
        draw_placeholder(canvas, "No SD card");
    } else {
        draw_placeholder(canvas, "Loading images...");
    }
    present(canvas);

    size_t index = 0;
    std::vector<std::string> files;
    uint32_t next_slide_ms = millis();
    while (true) {
        // Wake for the next slide, or early when the panel closes (repaint).
        const uint32_t now = millis();
        const uint32_t wait = (int32_t)(next_slide_ms - now) > 0 ? next_slide_ms - now : 0;
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait)) > 0) {
            present(canvas);
            continue;
        }
        next_slide_ms = millis() + TAB5_SLIDESHOW_INTERVAL_MS;
        if (!sd_ok || s_panel_open) {
            continue;
        }
        if (index == 0) {
            files = list_images();  // rescan once per cycle
        }
        if (files.empty()) {
            draw_placeholder(canvas, "Put images in /images on the SD card");
            present(canvas);
            continue;
        }
        if (index >= files.size()) index = 0;
        const std::string &path = files[index];
        const uint32_t t0 = millis();
        if (draw_image(canvas, path)) {
            present(canvas);
            Serial.printf("Tab5: slide %s (%lu ms)\n", path.c_str(), static_cast<unsigned long>(millis() - t0));
        } else {
            Serial.printf("Tab5: failed to decode %s\n", path.c_str());
        }
        index = (index + 1) % files.size();
    }
}

void open_panel()
{
    s_panel_open = true;
    display_lock();
    draw_panel();
    display_unlock();
}

void close_panel()
{
    s_panel_open = false;
    if (s_slide_task) {
        xTaskNotifyGive(s_slide_task);  // repaint image + overlay
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
    c_panel = M5.Display.color565(28, 34, 44);
    c_accent = TFT_BLUE;
    c_text = TFT_WHITE;
    c_sub = M5.Display.color565(160, 205, 255);
    c_active = TFT_GREEN;
    c_btn = M5.Display.color565(30, 60, 110);
    c_btn_pressed = M5.Display.color565(80, 140, 230);
    M5.Display.fillScreen(c_bg);
    display_unlock();

    W = M5.Display.width();
    H = M5.Display.height();
    const int pad = 16;

    // ── main screen overlay (bottom) ──
    const int bar_h = 130;
    const int bar_y = H - bar_h - pad;
    const int meter_w = 380;
    const int ptt_w = 340;
    const int group_w = meter_w + pad + ptt_w;
    const int gx = (W - group_w) / 2;
    r_meter = { gx, bar_y, meter_w, bar_h };
    r_ptt = { gx + meter_w + pad, bar_y, ptt_w, bar_h };
    const int setup_w = 170;
    r_setup = { W - setup_w - pad, bar_y + 20, setup_w, bar_h - 20 };

    // ── setup panel ──
    r_title = { 0, 0, W, 90 };
    const int col_x = 80;
    const int btn_w = 140;
    const int row_h = 130;
    const int val_w = 260;
    int y = r_title.h + 30;
    r_ch_down = { col_x, y, btn_w, row_h };
    r_ch_val = { col_x + btn_w + pad, y, val_w, row_h };
    r_ch_up = { col_x + btn_w + pad + val_w + pad, y, btn_w, row_h };
    const int col2_x = r_ch_up.x + btn_w + 80;
    r_vol_down = { col2_x, y, btn_w, row_h };
    r_vol_val = { col2_x + btn_w + pad, y, val_w - 60, row_h };
    r_vol_up = { r_vol_val.x + r_vol_val.w + pad, y, btn_w, row_h };
    y += row_h + 70;
    const int mode_w = 200;
    for (int i = 0; i < 3; ++i) {
        r_mode[i] = { col_x + i * (mode_w + pad), y, mode_w, 110 };
    }
    y += 110 + 40;
    r_info = { col_x, y, W - col_x - setup_w - 3 * pad, 80 };

    s_ready = true;
    display_lock();
    draw_overlay();
    display_unlock();

    xTaskCreatePinnedToCore(slideshow_task, "tab5_slides", 8192, nullptr, 0, &s_slide_task, 0);
}

Tab5Action tab5_ui_poll()
{
    if (!s_ready) {
        return Tab5Action::None;
    }
    Tab5Action action = Tab5Action::None;
    bool ptt = false;
    bool toggle_panel = false;

    const size_t n = M5.Touch.getCount();
    for (size_t i = 0; i < n; ++i) {
        const auto &t = M5.Touch.getDetail(i);
        // PTT follows the finger: sliding out of the button releases it.
        if (!s_panel_open && t.isPressed() && r_ptt.contains(t.x, t.y)) {
            ptt = true;
        }
        if (!t.wasPressed()) {
            continue;
        }
        if (r_setup.contains(t.x, t.y)) {
            toggle_panel = true;
            continue;
        }
        if (!s_panel_open) {
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

    if (toggle_panel) {
        Serial.printf("Tab5: %s setup panel\n", s_panel_open ? "close" : "open");
        if (s_panel_open) {
            close_panel();
        } else {
            open_panel();
        }
    }

    s_ptt_touched = ptt;
    if (ptt != s_ptt_drawn_pressed) {
        s_ptt_drawn_pressed = ptt;
        if (!s_panel_open) {
            display_lock();
            draw_ptt(ptt);
            display_unlock();
        }
    }
    return action;
}

bool tab5_ui_ptt_pressed()
{
    return s_ptt_touched;
}

void tab5_ui_set_settings(int channel, int volume_level, uint8_t tx_pitch_mode)
{
    const bool mode_changed = tx_pitch_mode != s_mode;
    s_channel = channel;
    s_volume = volume_level;
    s_mode = tx_pitch_mode;
    if (!s_ready) return;
    display_lock();
    if (s_panel_open) {
        // Always redraw +/- rows: this also clears the pressed flash.
        draw_panel_channel();
        draw_panel_volume();
        if (mode_changed) draw_panel_mode();
    } else {
        draw_meter();
    }
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
    if (s_panel_open) {
        draw_panel_title();
    } else {
        draw_meter();
        draw_ptt(s_ptt_drawn_pressed);
    }
    display_unlock();
}

void tab5_ui_set_rssi(int16_t rssi)
{
    if (!s_ready) return;
    static uint32_t last_batt_ms = 0;
    const uint32_t now = millis();
    const bool batt_due = s_panel_open && now - last_batt_ms > 30000;
    if (rssi == s_rssi && !batt_due) {
        return;
    }
    s_rssi = rssi;
    display_lock();
    if (s_panel_open) {
        draw_panel_info();
        if (batt_due) {
            last_batt_ms = now;
            draw_panel_title();
        }
    } else if (!s_tx) {
        draw_meter();
    }
    display_unlock();
}

void tab5_ui_set_tx_power(int16_t dbm)
{
    if (!s_ready) return;
    s_tx_dbm = dbm;
    if (s_panel_open || !s_tx) return;
    display_lock();
    draw_meter();
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
