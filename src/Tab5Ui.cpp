#include "Tab5Ui.h"

#include "config.h"

#if TALKIE_TARGET_M5TAB5

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>  // must precede M5Unified so M5GFX enables fs::FS image loaders
#include <WiFi.h>
#include <esp_wifi.h>
#include <M5Unified.h>

#include "DisplaySync.h"
#include "esp_now_hosted.h"

// Screen layout (portrait 720x1280)
//
//  Main screen : SD image full screen
//                                  [     PTT     ]  [SETUP]
//  Setup panel : title (state / battery)
//                CHANNEL -/+
//                VOLUME  -/+
//                VOICE   M1 M2 M3
//                signal meter (RSSI / TX power)
//                channel scan graph (CH1-13)
//                [START/STOP]                       [CLOSE]
//                (CLOSE sits where SETUP is, so the same spot toggles)

namespace {

struct Rect {
    int x, y, w, h;
    bool contains(int px, int py) const
    {
        return px >= x && px < x + w && py >= y && py < y + h;
    }
};

constexpr int kChannels = 13;
constexpr int16_t kNoSignal = -127;

// ── Palette ───────────────────────────────────────────────────────────────
uint16_t c_bg, c_panel, c_accent, c_text, c_sub, c_active, c_btn, c_btn_pressed;

// ── Layout ────────────────────────────────────────────────────────────────
int W = 720, H = 1280;
Rect r_ptt, r_setup;                       // main screen
Rect r_title;                              // setup panel
Rect r_ch_down, r_ch_val, r_ch_up;
Rect r_vol_down, r_vol_val, r_vol_up;
Rect r_mode[3];
Rect r_meter;
Rect r_scan;                               // graph area
Rect r_scan_btn;

// ── State ─────────────────────────────────────────────────────────────────
int s_channel = 1;
int s_volume = 3;
uint8_t s_mode = 1;
bool s_tx = false;
bool s_cont = false;
int16_t s_rssi = kNoSignal;
int16_t s_tx_dbm = 0;
volatile bool s_ptt_touched = false;
bool s_ptt_drawn_pressed = false;
bool s_ready = false;
volatile bool s_panel_open = false;

TaskHandle_t s_image_task = nullptr;

// ── Channel scan ──────────────────────────────────────────────────────────
void (*s_restore_radio)() = nullptr;
volatile bool s_scan_running = false;
volatile bool s_scan_stop = false;
int8_t s_scan_rssi[kChannels];   // strongest AP per channel (kNoSignal = none)
uint8_t s_scan_count[kChannels]; // number of APs per channel
uint32_t s_scan_sweeps = 0;
bool s_scan_has_data = false;  // graph shows measurements
// ESP-NOW activity heard while hopping (LR and 11b/g/n)
int8_t s_now_rssi[kChannels];
uint16_t s_now_frames[kChannels];
volatile int8_t s_hop_rssi[kChannels];
volatile uint16_t s_hop_frames[kChannels];
volatile int s_hop_ch = 0;
// 0 = idle, 1 = Wi-Fi AP scan, 2 = ESP-NOW listen on s_hop_ch
volatile uint8_t s_scan_phase = 0;
constexpr uint32_t kHopDwellMs = 250;

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
    M5.Display.drawString(label, r.x + r.w / 2, r.y + 8);
    M5.Display.setTextDatum(bottom_center);
    M5.Display.setFont(&fonts::Font7);
    M5.Display.setTextColor(c_text, c_panel);
    M5.Display.drawString(value, r.x + r.w / 2, r.y + r.h - 8);
}

// ── Main screen ───────────────────────────────────────────────────────────

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
    M5.Display.drawString(s_tx ? (s_cont ? "CONT TX" : "TX") : "PTT",
                          r_ptt.x + r_ptt.w / 2, r_ptt.y + r_ptt.h / 2 - 10);
    char sub[48];
    snprintf(sub, sizeof(sub), "CH%02d   %s", s_channel,
             s_cont ? "double-tap: stop" : "double-tap: continuous");
    M5.Display.setFont(&fonts::FreeSans12pt7b);
    M5.Display.drawString(sub, r_ptt.x + r_ptt.w / 2, r_ptt.y + r_ptt.h / 2 + 32);
}

void draw_setup_button(bool pressed)
{
    draw_button(r_setup, s_panel_open ? "CLOSE" : "SETUP", pressed, &fonts::FreeSansBold12pt7b);
}

void draw_overlay()
{
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
    const char *state = s_scan_running ? "SCANNING"
                      : s_tx ? (s_cont ? "CONT TX" : "TRANSMIT") : "RECEIVE";
    char right[40];
    const int32_t batt = M5.Power.getBatteryLevel();
    if (batt >= 0) {
        snprintf(right, sizeof(right), "%s  %d%%", state, static_cast<int>(batt));
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
    M5.Display.fillRect(r_mode[0].x, r_mode[0].y - 30, W - 2 * r_mode[0].x, 26, c_bg);
    M5.Display.setFont(&fonts::FreeSans12pt7b);
    M5.Display.setTextDatum(bottom_left);
    M5.Display.setTextColor(c_sub, c_bg);
    M5.Display.drawString("VOICE", r_mode[0].x, r_mode[0].y - 6);
    for (int i = 0; i < 3; ++i) {
        const bool sel = (s_mode == i + 1);
        draw_button(r_mode[i], kLabels[i], false, &fonts::FreeSansBold24pt7b,
                    sel ? c_active : c_btn, sel ? TFT_BLACK : TFT_WHITE);
    }
}

// Signal meter: RX = RSSI of received ESPTalkie frames, TX = TX power.
void draw_meter()
{
    static const int16_t kLevels[8] = { -90, -80, -70, -60, -50, -40, -30, -20 };
    const Rect &r = r_meter;
    const uint16_t border = s_tx ? TFT_RED : c_accent;
    M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 16, c_panel);
    M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 16, border);

    M5.Display.setFont(&fonts::FreeSans12pt7b);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextColor(c_sub, c_panel);
    M5.Display.drawString(s_tx ? "TX POWER" : "SIGNAL", r.x + 16, r.y + 10);

    char v[12];
    if (s_tx) {
        snprintf(v, sizeof(v), "%ddBm", s_tx_dbm);
    } else if (s_rssi > kNoSignal) {
        snprintf(v, sizeof(v), "%d", s_rssi);
    } else {
        snprintf(v, sizeof(v), "---");
    }
    M5.Display.setFont(&fonts::FreeSansBold18pt7b);
    M5.Display.setTextDatum(bottom_left);
    M5.Display.setTextColor(c_text, c_panel);
    M5.Display.drawString(v, r.x + 16, r.y + r.h - 10);

    const int bars_x = r.x + 200;
    const int bars_w = r.w - 200 - 16;
    const int gap = 8;
    const int bw = (bars_w - gap * 7) / 8;
    const int base = r.y + r.h - 12;
    const int max_h = r.h - 24;
    for (int i = 0; i < 8; ++i) {
        const int h = max_h * (i + 3) / 10;
        const bool on = s_tx ? (s_tx_dbm >= i * 3) : (s_rssi >= kLevels[i]);
        const uint16_t color = on ? (i < 5 ? TFT_GREEN : TFT_RED) : TFT_BLACK;
        M5.Display.fillRoundRect(bars_x + i * (bw + gap), base - h, bw, h, 3, color);
    }
}

// Channel scan graph per channel (-100..-30 dBm):
//   left bar  = strongest Wi-Fi AP (count above)
//   right bar = strongest ESP-NOW frame heard (frames above)
void draw_scan()
{
    const Rect &r = r_scan;
    const uint16_t c_now = TFT_CYAN;
    M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 16, c_panel);
    M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 16, c_accent);

    M5.Display.setFont(&fonts::FreeSans12pt7b);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextColor(c_sub, c_panel);
    char head[48];
    if (!s_scan_running) {
        if (s_scan_sweeps == 0) {
            snprintf(head, sizeof(head), "CHANNEL SCAN  (press START)");
        } else {
            snprintf(head, sizeof(head), "CHANNEL SCAN  #%lu", static_cast<unsigned long>(s_scan_sweeps));
        }
        M5.Display.drawString(head, r.x + 16, r.y + 10);
    } else {
        // What the radio is doing right now.
        if (s_scan_phase == 2 && s_hop_ch) {
            snprintf(head, sizeof(head), "#%lu  ESP-NOW  RX CH%d", static_cast<unsigned long>(s_scan_sweeps + 1), s_hop_ch);
        } else {
            snprintf(head, sizeof(head), "#%lu  Wi-Fi AP scan", static_cast<unsigned long>(s_scan_sweeps + 1));
        }
        M5.Display.setTextColor(TFT_YELLOW, c_panel);
        M5.Display.drawString(head, r.x + 16, r.y + 10);
    }
    // legend
    M5.Display.setFont(&fonts::FreeSans9pt7b);
    M5.Display.setTextDatum(top_right);
    M5.Display.fillRect(r.x + r.w - 250, r.y + 16, 14, 14, TFT_GREEN);
    M5.Display.setTextColor(c_text, c_panel);
    M5.Display.drawString("Wi-Fi AP", r.x + r.w - 150, r.y + 14);
    M5.Display.fillRect(r.x + r.w - 130, r.y + 16, 14, 14, c_now);
    M5.Display.drawString("ESP-NOW", r.x + r.w - 16, r.y + 14);

    constexpr int kMin = -100, kMax = -30;
    const int gx = r.x + 60;
    const int gw = r.w - 60 - 16;
    const int gy = r.y + 70;
    const int gh = r.h - 70 - 60;
    const int base = gy + gh;

    // dBm grid
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextDatum(middle_right);
    for (int db = kMax; db >= kMin; db -= 10) {
        const int y = base - (db - kMin) * gh / (kMax - kMin);
        M5.Display.drawFastHLine(gx, y, gw, M5.Display.color565(60, 70, 85));
        char t[6];
        snprintf(t, sizeof(t), "%d", db);
        M5.Display.setTextColor(c_sub, c_panel);
        M5.Display.drawString(t, gx - 6, y);
    }

    // Quietest channel: lowest of (strongest AP, strongest ESP-NOW).
    auto level = [](int ch) {
        return s_scan_rssi[ch] > s_now_rssi[ch] ? s_scan_rssi[ch] : s_now_rssi[ch];
    };
    int best = 0;
    if (s_scan_has_data) {
        for (int ch = 1; ch < kChannels; ++ch) {
            if (level(ch) < level(best)) best = ch;
        }
    }

    auto bar_h = [&](int v) {
        const int clamped = v < kMin ? kMin : (v > kMax ? kMax : v);
        return (clamped - kMin) * gh / (kMax - kMin);
    };
    const int slot = gw / kChannels;
    const int bw = (slot - 8) / 2;
    for (int ch = 0; ch < kChannels; ++ch) {
        const int x = gx + ch * slot + 4;
        if (s_scan_has_data) {
            const int16_t ap = s_scan_rssi[ch];
            if (ap > kNoSignal) {
                const int h = bar_h(ap);
                uint16_t color = TFT_GREEN;
                if (ap >= -60) color = TFT_RED;
                else if (ap >= -75) color = TFT_YELLOW;
                M5.Display.fillRect(x, base - h, bw, h, color);
            }
            const int16_t now = s_now_rssi[ch];
            if (now > kNoSignal) {
                const int h = bar_h(now);
                M5.Display.fillRect(x + bw, base - h, bw, h, c_now);
            }
            M5.Display.setFont(&fonts::Font0);
            M5.Display.setTextDatum(bottom_center);
            char c[6];
            if (s_scan_count[ch]) {
                snprintf(c, sizeof(c), "%u", s_scan_count[ch]);
                M5.Display.setTextColor(TFT_GREEN, c_panel);
                M5.Display.drawString(c, x + bw / 2, gy - 4);
            }
            if (s_now_frames[ch]) {
                snprintf(c, sizeof(c), "%u", s_now_frames[ch] > 999 ? 999 : s_now_frames[ch]);
                M5.Display.setTextColor(c_now, c_panel);
                M5.Display.drawString(c, x + bw + bw / 2, gy - 14);
            }
        }
        // channel label: listening now = yellow, configured = boxed,
        // quietest = green number
        const bool listening = s_scan_running && s_scan_phase == 2 && (ch + 1 == s_hop_ch);
        const bool current = (ch + 1 == s_channel);
        const bool quiet = s_scan_has_data && ch == best;
        const uint16_t lab_bg = listening ? TFT_YELLOW : (current ? c_btn_pressed : c_panel);
        M5.Display.fillRoundRect(x - 2, base + 8, 2 * bw + 4, 36, 6, lab_bg);
        if (listening) {
            // marker above the bars
            M5.Display.fillTriangle(x + bw - 8, gy - 42, x + bw + 8, gy - 42, x + bw, gy - 30, TFT_YELLOW);
        }
        M5.Display.setFont(&fonts::FreeSansBold12pt7b);
        M5.Display.setTextDatum(middle_center);
        M5.Display.setTextColor(listening ? TFT_BLACK : (quiet ? TFT_GREEN : c_text), lab_bg);
        char l[4];
        snprintf(l, sizeof(l), "%d", ch + 1);
        M5.Display.drawString(l, x + bw, base + 26);
    }
}

void draw_scan_button(bool pressed)
{
    const uint16_t fill = s_scan_running ? M5.Display.color565(170, 30, 30) : M5.Display.color565(20, 120, 50);
    draw_button(r_scan_btn, s_scan_running ? "STOP" : "START", pressed, &fonts::FreeSansBold18pt7b, fill);
}

void draw_panel()
{
    M5.Display.fillScreen(c_bg);
    draw_panel_title();
    draw_panel_channel();
    draw_panel_volume();
    draw_panel_mode();
    draw_meter();
    draw_scan();
    draw_scan_button(false);
    draw_setup_button(false);
}

void flash_button(const Rect &r, const char *label)
{
    display_lock();
    draw_button(r, label, true);
    display_unlock();
}

// ── Channel scan task ─────────────────────────────────────────────────────

// Runs in the esp-hosted RX thread for every ESP-NOW frame during a scan.
void scan_monitor(const uint8_t *, int8_t rssi, uint8_t channel, const uint8_t *, int)
{
    const int ch = s_hop_ch;
    if (ch < 1 || ch > kChannels) return;
    if (channel && channel != ch) return;  // straggler from the previous channel
    if (rssi > s_hop_rssi[ch - 1]) s_hop_rssi[ch - 1] = rssi;
    if (s_hop_frames[ch - 1] < 65535) s_hop_frames[ch - 1]++;
}

void scan_task(void *)
{
    // Receive both LR and normal (11b/g/n) frames; the Wi-Fi scan also needs
    // 11b/g/n. ESPTalkie itself runs LR-only.
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    // Count every ESP-NOW frame; keep scan traffic out of the audio path.
    esp_now_hosted_set_monitor(scan_monitor, true);
    auto redraw = [] {
        if (s_panel_open) {
            display_lock();
            draw_scan();
            display_unlock();
        }
    };
    while (!s_scan_stop) {
        s_scan_phase = 1;
        redraw();
        const int n = WiFi.scanNetworks(false, true, false, 120);
        int8_t rssi[kChannels];
        uint8_t count[kChannels];
        for (int i = 0; i < kChannels; ++i) {
            rssi[i] = kNoSignal;
            count[i] = 0;
        }
        for (int i = 0; i < n; ++i) {
            const int ch = WiFi.channel(i);
            if (ch < 1 || ch > kChannels) continue;
            const int v = WiFi.RSSI(i);
            if (v > rssi[ch - 1]) rssi[ch - 1] = static_cast<int8_t>(v);
            if (count[ch - 1] < 99) count[ch - 1]++;
        }
        WiFi.scanDelete();
        if (n < 0) {
            Serial.printf("Tab5 scan: failed (%d)\n", n);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        // Show the AP result right away.
        memcpy(s_scan_rssi, rssi, sizeof(rssi));
        memcpy(s_scan_count, count, sizeof(count));
        s_scan_has_data = true;
        // Listen for ESP-NOW on each channel (mixed LR + 11b/g/n receive).
        for (int ch = 1; ch <= kChannels && !s_scan_stop; ++ch) {
            s_hop_rssi[ch - 1] = kNoSignal;
            s_hop_frames[ch - 1] = 0;
            s_hop_ch = ch;
            s_scan_phase = 2;
            esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
            redraw();
            vTaskDelay(pdMS_TO_TICKS(kHopDwellMs));
            // Update this channel's ESP-NOW bar as soon as its dwell ends.
            s_now_rssi[ch - 1] = s_hop_rssi[ch - 1];
            s_now_frames[ch - 1] = s_hop_frames[ch - 1];
        }
        s_hop_ch = 0;
        s_scan_phase = 0;
        if (s_scan_stop) {
            break;
        }
        s_scan_sweeps++;
        Serial.printf("Tab5 scan #%lu: %d APs, ESP-NOW frames:", static_cast<unsigned long>(s_scan_sweeps), n);
        for (int i = 0; i < kChannels; ++i) Serial.printf(" %u", s_now_frames[i]);
        Serial.println();
        if (s_panel_open) {
            display_lock();
            draw_scan();
            display_unlock();
        }
    }

    esp_now_hosted_set_monitor(nullptr, false);
    s_hop_ch = 0;
    s_scan_phase = 0;
#ifdef ESPNOW_LONG_RANGE
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#endif
    if (s_restore_radio) {
        s_restore_radio();  // back to the configured channel
    }
    Serial.printf("Tab5 scan: stopped, back on CH%d\n", s_channel);
    s_scan_running = false;
    if (s_panel_open) {
        display_lock();
        draw_panel_title();
        draw_scan();
        draw_scan_button(false);
        display_unlock();
    }
    vTaskDelete(nullptr);
}

void scan_start()
{
    if (s_scan_running) return;
    s_scan_stop = false;
    s_scan_running = true;
    s_scan_sweeps = 0;
    s_scan_has_data = false;
    for (int i = 0; i < kChannels; ++i) {
        s_now_rssi[i] = kNoSignal;
        s_now_frames[i] = 0;
    }
    display_lock();
    draw_panel_title();
    draw_scan();
    draw_scan_button(false);
    display_unlock();
    if (xTaskCreatePinnedToCore(scan_task, "tab5_scan", 6144, nullptr, 1, nullptr, 0) != pdPASS) {
        s_scan_running = false;
    }
}

void scan_stop()
{
    // The task finishes the running sweep, restores protocol and channel,
    // then clears s_scan_running.
    s_scan_stop = true;
}

// ── Background image ──────────────────────────────────────────────────────

const char *find_image()
{
    static const char *kCandidates[] = {
        TAB5_IMAGE_PATH,
        "/images/pokibon.jpeg",
        "/images/pokibon.jpg",
        "/pokibon.jpeg",
        "/pokibon.jpg",
        "/images/pokibon.png",
        "/pokibon.png",
    };
    for (const char *p : kCandidates) {
        if (SD_MMC.exists(p)) return p;
    }
    return nullptr;
}

bool draw_image(M5Canvas &canvas, const char *path)
{
    const int w = canvas.width();
    const int h = canvas.height();
    canvas.fillScreen(TFT_BLACK);
    const size_t len = strlen(path);
    const bool png = len >= 4 && strcasecmp(path + len - 4, ".png") == 0;
    // scale_x = 0: fit inside w x h keeping the aspect ratio.
    if (png) {
        return canvas.drawPngFile(SD_MMC, path, 0, 0, w, h, 0, 0, 0.0f, 0.0f, middle_center);
    }
    return canvas.drawJpgFile(SD_MMC, path, 0, 0, w, h, 0, 0, 0.0f, 0.0f, middle_center);
}

void draw_placeholder(M5Canvas &canvas, const char *line2)
{
    canvas.fillScreen(c_bg);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(c_text);
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.drawString("ESPTalkie", canvas.width() / 2, canvas.height() / 2 - 60);
    canvas.setFont(&fonts::FreeSans12pt7b);
    canvas.setTextColor(c_sub);
    canvas.drawString(line2, canvas.width() / 2, canvas.height() / 2);
}

// Push the image and repaint the overlay on top (main screen only).
void present(M5Canvas &canvas)
{
    display_lock();
    if (!s_panel_open) {
        canvas.pushSprite(0, 0);
        draw_overlay();
    }
    display_unlock();
}

// Loads the image once, then repaints it whenever the setup panel closes.
void image_task(void *)
{
    M5Canvas canvas(&M5.Display);
    canvas.setColorDepth(16);
    canvas.setPsram(true);
    if (!canvas.createSprite(W, H)) {
        Serial.println("Tab5: failed to allocate image canvas");
        vTaskDelete(nullptr);
    }
    if (!SD_MMC.begin("/sdcard", false)) {
        Serial.println("Tab5: SD card not mounted");
        draw_placeholder(canvas, "No SD card");
    } else {
        const char *path = find_image();
        if (!path) {
            draw_placeholder(canvas, "Put " TAB5_IMAGE_PATH " on the SD card");
        } else {
            const uint32_t t0 = millis();
            if (draw_image(canvas, path)) {
                Serial.printf("Tab5: image %s (%lu ms)\n", path, static_cast<unsigned long>(millis() - t0));
            } else {
                draw_placeholder(canvas, "Image decode failed");
            }
        }
        SD_MMC.end();
    }
    present(canvas);
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        present(canvas);
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
    scan_stop();  // restores the configured channel when the sweep ends
    s_panel_open = false;
    if (s_image_task) {
        xTaskNotifyGive(s_image_task);  // repaint image + overlay
    }
}

}  // namespace

void tab5_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode, void (*restore_radio)())
{
    s_channel = channel;
    s_volume = volume_level;
    s_mode = tx_pitch_mode;
    s_restore_radio = restore_radio;
    for (int i = 0; i < kChannels; ++i) {
        s_scan_rssi[i] = kNoSignal;
        s_scan_count[i] = 0;
        s_now_rssi[i] = kNoSignal;
        s_now_frames[i] = 0;
    }

    display_lock();
    M5.Display.setRotation(0);  // Tab5 panel is natively 720x1280 portrait
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

    // ── main screen ──
    const int ptt_h = 150;
    const int ptt_w = 420;
    r_ptt = { (W - ptt_w) / 2, H - pad - ptt_h, ptt_w, ptt_h };
    const int setup_x = r_ptt.x + r_ptt.w + 20;
    r_setup = { setup_x, r_ptt.y + 20, W - pad - setup_x, ptt_h - 20 };

    // ── setup panel ──
    r_title = { 0, 0, W, 90 };
    const int btn_w = 140;
    const int row_h = 120;
    const int val_w = W - 2 * pad - 2 * (btn_w + pad);
    int y = r_title.h + 24;
    r_ch_down = { pad, y, btn_w, row_h };
    r_ch_val = { pad + btn_w + pad, y, val_w, row_h };
    r_ch_up = { W - pad - btn_w, y, btn_w, row_h };
    y += row_h + 20;
    r_vol_down = { pad, y, btn_w, row_h };
    r_vol_val = { pad + btn_w + pad, y, val_w, row_h };
    r_vol_up = { W - pad - btn_w, y, btn_w, row_h };
    y += row_h + 50;
    const int mode_w = (W - 4 * pad) / 3;
    for (int i = 0; i < 3; ++i) {
        r_mode[i] = { pad + i * (mode_w + pad), y, mode_w, 100 };
    }
    y += 100 + 20;
    r_meter = { pad, y, W - 2 * pad, 100 };
    y += 100 + 20;
    r_scan = { pad, y, W - 2 * pad, r_setup.y - 20 - y };
    r_scan_btn = { pad, r_setup.y, 260, r_setup.h };

    s_ready = true;
    display_lock();
    draw_overlay();
    display_unlock();

    xTaskCreatePinnedToCore(image_task, "tab5_image", 8192, nullptr, 0, &s_image_task, 0);
}

Tab5Action tab5_ui_poll()
{
    if (!s_ready) {
        return Tab5Action::None;
    }
    Tab5Action action = Tab5Action::None;
    bool ptt = false;
    bool toggle_panel = false;
    bool toggle_scan = false;

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
        } else if (r_scan_btn.contains(t.x, t.y)) {
            toggle_scan = true;
        }
        if (flash) {
            flash_button(*flash, flash_label);
        }
    }

    if (toggle_scan) {
        if (s_scan_running) {
            Serial.println("Tab5 scan: STOP");
            scan_stop();
            display_lock();
            draw_scan_button(true);  // stays pressed until the sweep ends
            display_unlock();
        } else {
            Serial.println("Tab5 scan: START");
            scan_start();
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

bool tab5_ui_scanning()
{
    return s_scan_running;
}

void tab5_ui_set_settings(int channel, int volume_level, uint8_t tx_pitch_mode)
{
    const bool mode_changed = tx_pitch_mode != s_mode;
    const bool ch_changed = channel != s_channel;
    s_channel = channel;
    s_volume = volume_level;
    s_mode = tx_pitch_mode;
    if (!s_ready || !s_panel_open) return;
    display_lock();
    // Always redraw +/- rows: this also clears the pressed flash.
    draw_panel_channel();
    draw_panel_volume();
    if (mode_changed) draw_panel_mode();
    if (ch_changed) draw_scan();  // current-channel marker
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
        draw_meter();
    } else {
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
    if (!s_panel_open) return;
    display_lock();
    if (!s_tx) draw_meter();
    if (batt_due) {
        last_batt_ms = now;
        draw_panel_title();
    }
    display_unlock();
}

void tab5_ui_set_tx_power(int16_t dbm)
{
    if (!s_ready) return;
    s_tx_dbm = dbm;
    if (!s_panel_open || !s_tx) return;
    display_lock();
    draw_meter();
    display_unlock();
}

void tab5_ui_message(const char *msg)
{
    display_lock();
    const uint16_t bg = M5.Display.color565(20, 20, 20);
    M5.Display.fillRect(0, 0, M5.Display.width(), 80, bg);
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(TFT_YELLOW, bg);
    M5.Display.drawString(msg, M5.Display.width() / 2, 40);
    display_unlock();
    Serial.printf("Tab5: %s\n", msg);
}

#else  // !TALKIE_TARGET_M5TAB5

void tab5_ui_begin(int, int, uint8_t, void (*)()) {}
Tab5Action tab5_ui_poll() { return Tab5Action::None; }
bool tab5_ui_ptt_pressed() { return false; }
bool tab5_ui_scanning() { return false; }
void tab5_ui_set_settings(int, int, uint8_t) {}
void tab5_ui_set_status(bool, bool) {}
void tab5_ui_set_rssi(int16_t) {}
void tab5_ui_set_tx_power(int16_t) {}
void tab5_ui_message(const char *) {}

#endif
