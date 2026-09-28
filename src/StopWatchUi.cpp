#include "StopWatchUi.h"

#include "config.h"

#if TALKIE_TARGET_M5STOPWATCH

#include <Arduino.h>
#include <M5Unified.h>
#include <math.h>

#include "DisplaySync.h"

// Screen layout (round 466x466, all positions relative to the centre)
//
//                    RECEIVE              <- status label
//               .--------------.
//        RSSI  (  ( image    )  )  BAT     <- status ring around the image
//        -67   (  (  circle  )  )  87%
//        ||||   '--------------'   [##]
//                    CH 01
//                 VOL 3  VOICE M1
//
// Setup screen: CHANNEL / VOLUME / VOICE rows, [-] [+] touch buttons.

extern const uint8_t sw_image_start[] asm("_binary_assets_stopwatch_dog_jpg_start");
extern const uint8_t sw_image_end[] asm("_binary_assets_stopwatch_dog_jpg_end");

namespace {

constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

constexpr uint16_t kBg          = rgb(0, 0, 0);       // black: AMOLED pixels off
constexpr uint16_t kText        = rgb(240, 245, 255);
constexpr uint16_t kTextSub     = rgb(150, 190, 240);
constexpr uint16_t kTextDim     = rgb(120, 130, 145);
constexpr uint16_t kStandby     = rgb(30, 120, 255);
constexpr uint16_t kReceiving   = rgb(0, 210, 90);
constexpr uint16_t kTransmit    = rgb(240, 40, 40);
constexpr uint16_t kContTx      = rgb(255, 140, 0);
constexpr uint16_t kBarOff      = rgb(45, 50, 60);
constexpr uint16_t kBarLow      = rgb(0, 210, 90);
constexpr uint16_t kBarHigh     = rgb(240, 40, 40);
constexpr uint16_t kRowBg       = rgb(28, 32, 40);
constexpr uint16_t kRowBorder   = rgb(80, 90, 105);
constexpr uint16_t kRowSelBg    = rgb(0, 60, 28);
constexpr uint16_t kRowSelBd    = rgb(0, 210, 90);
constexpr uint16_t kSetupTitle  = rgb(255, 200, 0);   // yellow button colour

constexpr int kImageSize    = 240;   // diameter of the image circle
constexpr int kRingInner    = 127;
constexpr int kRingOuter    = 139;
constexpr int kSideOffset   = 186;   // left / right info columns
constexpr int kRowPitch     = 80;    // setup rows
constexpr int kRowHalfW     = 125;
constexpr int kRowHalfH     = 32;
constexpr int kTouchBtnOfs  = 186;   // setup [-] / [+]
constexpr int kTouchBtnR    = 34;

constexpr uint32_t kRxActiveMs        = 250;   // "receiving" indicator hold
constexpr uint32_t kRxNewSessionGapMs = 1500;  // quiet time before RX counts as a new call
constexpr uint32_t kBatteryPollMs     = 5000;

M5Canvas *s_image = nullptr;
bool s_image_ok = false;

volatile bool s_setup = false;
StopWatchSetupItem s_selected = StopWatchSetupItem::Channel;
int s_channel = 1;
int s_volume = 3;
uint8_t s_mode = 1;

volatile bool s_transmitting = false;
volatile bool s_continuous = false;
volatile bool s_receiving = false;

bool s_meter_is_tx = false;
int16_t s_meter_value = 0;
bool s_meter_valid = false;

int s_batt_level = -1;
bool s_batt_charging = false;
uint32_t s_batt_ms = 0;

uint8_t s_vib_steps = 0;       // remaining on/off steps of the pattern
uint32_t s_vib_next_ms = 0;

int cx() { return M5.Display.width() / 2; }
int cy() { return M5.Display.height() / 2; }

uint16_t state_color()
{
    if (s_transmitting) return s_continuous ? kContTx : kTransmit;
    if (s_receiving) return kReceiving;
    return kStandby;
}

const char *state_label()
{
    if (s_transmitting) return s_continuous ? "CONT TX" : "TRANSMIT";
    if (s_receiving) return "RECEIVING";
    return "RECEIVE";
}

void load_image()
{
    s_image = new M5Canvas(&M5.Display);
    s_image->setColorDepth(16);
    s_image->setPsram(true);
    if (!s_image->createSprite(kImageSize, kImageSize)) {
        Serial.println("StopWatch: image sprite allocation failed");
        return;
    }
    s_image->fillSprite(kBg);
    const size_t size = static_cast<size_t>(sw_image_end - sw_image_start);
    s_image_ok = s_image->drawJpg(sw_image_start, size, 0, 0, kImageSize, kImageSize,
                                  0, 0, 0.0f, 0.0f, middle_center);
    if (!s_image_ok) {
        Serial.println("StopWatch: JPEG decode failed");
        s_image->fillSprite(kBg);
        s_image->fillCircle(kImageSize / 2, kImageSize / 2, kImageSize / 2 - 1, kRowBg);
        s_image->setFont(&fonts::FreeSansBold12pt7b);
        s_image->setTextDatum(middle_center);
        s_image->setTextColor(kText);
        s_image->drawString("ESPTalkie", kImageSize / 2, kImageSize / 2);
        s_image_ok = true;
    }
    // Circular mask: paint everything outside the circle with the background.
    const float r = kImageSize / 2.0f;
    for (int y = 0; y < kImageSize; ++y) {
        const float dy = (y + 0.5f) - r;
        const float span = r * r - dy * dy;
        const int half = span > 0.0f ? static_cast<int>(sqrtf(span)) : 0;
        const int x0 = static_cast<int>(r) - half;
        const int x1 = static_cast<int>(r) + half;
        if (x0 > 0) s_image->fillRect(0, y, x0, 1, kBg);
        if (x1 < kImageSize) s_image->fillRect(x1, y, kImageSize - x1, 1, kBg);
    }
}

// ---------------------------------------------------------------- main screen

void draw_ring()
{
    if (s_setup) return;
    display_lock();
    M5.Display.fillArc(cx(), cy(), kRingInner, kRingOuter, 0.0f, 360.0f, state_color());
    display_unlock();
}

void draw_status_label()
{
    if (s_setup) return;
    display_lock();
    M5.Display.fillRect(cx() - 160, cy() - 205, 320, 60, kBg);
    M5.Display.setFont(&fonts::FreeSansBold18pt7b);
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(state_color());
    M5.Display.drawString(state_label(), cx(), cy() - 172);
    display_unlock();
}

void draw_meter()
{
    if (s_setup || !s_meter_valid) return;
    static const int16_t kRssiLevel[8] = { -90, -80, -70, -60, -50, -40, -30, -20 };
    const int col = cx() - kSideOffset;
    display_lock();
    M5.Display.fillRect(col - 42, cy() - 45, 84, 100, kBg);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextColor(kTextSub);
    M5.Display.drawString(s_meter_is_tx ? "TX dBm" : "RSSI", col, cy() - 34);
    M5.Display.setFont(&fonts::Font4);
    M5.Display.setTextColor(kText);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", s_meter_value);
    M5.Display.drawString(buf, col, cy() - 8);

    int active = 0;
    if (s_meter_is_tx) {
        active = s_meter_value / 3;
    } else {
        for (int i = 0; i < 8; ++i) {
            if (s_meter_value >= kRssiLevel[i]) active = i + 1;
        }
    }
    if (active < 0) active = 0;
    if (active > 8) active = 8;
    constexpr int kBarW = 6;
    constexpr int kBarGap = 2;
    const int x0 = col - (8 * kBarW + 7 * kBarGap) / 2;
    const int base_y = cy() + 50;
    for (int i = 0; i < 8; ++i) {
        const int h = 4 + i * 4;
        const uint16_t c = (i < active) ? ((i < 5) ? kBarLow : kBarHigh) : kBarOff;
        M5.Display.fillRect(x0 + i * (kBarW + kBarGap), base_y - h, kBarW, h, c);
    }
    display_unlock();
}

void draw_battery()
{
    if (s_setup || s_batt_level < 0) return;
    const int col = cx() + kSideOffset;
    const uint16_t c = s_batt_charging ? kSetupTitle
                     : (s_batt_level < 20 ? kTransmit : kReceiving);
    display_lock();
    M5.Display.fillRect(col - 42, cy() - 45, 84, 100, kBg);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextColor(kTextSub);
    M5.Display.drawString(s_batt_charging ? "CHG" : "BAT", col, cy() - 34);
    M5.Display.setFont(&fonts::Font4);
    M5.Display.setTextColor(kText);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", s_batt_level);
    M5.Display.drawString(buf, col, cy() - 8);
    // battery icon
    constexpr int kW = 38;
    constexpr int kH = 18;
    const int x = col - kW / 2 - 2;
    const int y = cy() + 22;
    M5.Display.drawRect(x, y, kW, kH, kText);
    M5.Display.fillRect(x + kW, y + kH / 3, 3, kH / 3, kText);
    const int fill_w = ((kW - 4) * s_batt_level) / 100;
    if (fill_w > 0) {
        M5.Display.fillRect(x + 2, y + 2, fill_w, kH - 4, c);
    }
    display_unlock();
}

void draw_settings_line()
{
    if (s_setup) return;
    display_lock();
    M5.Display.fillRect(cx() - 160, cy() + 146, 320, 76, kBg);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setFont(&fonts::FreeSansBold18pt7b);
    M5.Display.setTextColor(kText);
    char buf[24];
    snprintf(buf, sizeof(buf), "CH %02d", s_channel);
    M5.Display.drawString(buf, cx(), cy() + 172);
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextColor(kTextSub);
    snprintf(buf, sizeof(buf), "VOL %d   VOICE M%u", s_volume, static_cast<unsigned>(s_mode));
    M5.Display.drawString(buf, cx(), cy() + 204);
    display_unlock();
}

void read_battery()
{
    int level = M5.Power.getBatteryLevel();
    if (level < 0) level = 0;
    if (level > 100) level = 100;
    const bool charging = (M5.Power.isCharging() == m5::Power_Class::is_charging);
    const bool changed = (level != s_batt_level) || (charging != s_batt_charging);
    s_batt_level = level;
    s_batt_charging = charging;
    if (changed) draw_battery();
}

void draw_main()
{
    display_lock();
    M5.Display.startWrite();
    M5.Display.fillScreen(kBg);
    if (s_image && s_image_ok) {
        s_image->pushSprite(cx() - kImageSize / 2, cy() - kImageSize / 2);
    }
    draw_ring();
    draw_status_label();
    draw_meter();
    draw_battery();
    draw_settings_line();
    M5.Display.endWrite();
    display_unlock();
}

// --------------------------------------------------------------- setup screen

int row_center_y(int index)
{
    return cy() - kRowPitch + index * kRowPitch;
}

void draw_setup_row(int index)
{
    static const char *const kLabels[3] = { "CHANNEL", "VOLUME", "VOICE" };
    const bool sel = (static_cast<int>(s_selected) == index);
    const int yc = row_center_y(index);
    const int x = cx() - kRowHalfW;
    const int y = yc - kRowHalfH;
    const int w = kRowHalfW * 2;
    const int h = kRowHalfH * 2;
    display_lock();
    M5.Display.fillRoundRect(x, y, w, h, 14, sel ? kRowSelBg : kRowBg);
    M5.Display.drawRoundRect(x, y, w, h, 14, sel ? kRowSelBd : kRowBorder);
    if (sel) {
        M5.Display.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 13, kRowSelBd);
    }
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextDatum(middle_left);
    M5.Display.setTextColor(sel ? kRowSelBd : kTextSub);
    M5.Display.drawString(kLabels[index], x + 14, yc);

    char buf[8];
    switch (index) {
        case 0: snprintf(buf, sizeof(buf), "%02d", s_channel); break;
        case 1: snprintf(buf, sizeof(buf), "%d", s_volume); break;
        default: snprintf(buf, sizeof(buf), "M%u", static_cast<unsigned>(s_mode)); break;
    }
    M5.Display.setFont(&fonts::FreeSansBold18pt7b);
    M5.Display.setTextDatum(middle_right);
    M5.Display.setTextColor(kText);
    M5.Display.drawString(buf, x + w - 14, yc + 1);
    M5.Display.setTextDatum(middle_center);
    display_unlock();
}

void draw_touch_button(int x, const char *label)
{
    M5.Display.fillCircle(x, cy(), kTouchBtnR, kRowBg);
    M5.Display.drawCircle(x, cy(), kTouchBtnR, kRowBorder);
    M5.Display.setFont(&fonts::FreeSansBold18pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(kText);
    M5.Display.drawString(label, x, cy() + 1);
}

void draw_setup()
{
    display_lock();
    M5.Display.startWrite();
    M5.Display.fillScreen(kBg);
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(kSetupTitle);
    M5.Display.drawString("SETUP", cx(), cy() - 170);
    for (int i = 0; i < static_cast<int>(StopWatchSetupItem::Count); ++i) {
        draw_setup_row(i);
    }
    draw_touch_button(cx() - kTouchBtnOfs, "-");
    draw_touch_button(cx() + kTouchBtnOfs, "+");
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(kTextDim);
    M5.Display.drawString("YELLOW: item   BLUE: +1", cx(), cy() + 142);
    M5.Display.drawString("hold YELLOW: exit", cx(), cy() + 164);
    M5.Display.endWrite();
    display_unlock();
}

}  // namespace

void stopwatch_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode)
{
    s_channel = channel;
    s_volume = volume_level;
    s_mode = tx_pitch_mode;
    load_image();
    s_batt_ms = millis();
    read_battery();
    draw_main();
}

void stopwatch_ui_service(uint32_t last_rx_ms)
{
    const uint32_t now = millis();

    // New incoming call (first packet after a quiet channel) -> vibrate.
    static uint32_t prev_rx_ms = 0;
    if (last_rx_ms != 0 && last_rx_ms != prev_rx_ms) {
        if (!s_transmitting && (prev_rx_ms == 0 || last_rx_ms - prev_rx_ms >= kRxNewSessionGapMs)) {
            stopwatch_ui_vibrate();
        }
        prev_rx_ms = last_rx_ms;
    }

    const bool receiving = last_rx_ms != 0 && (now - last_rx_ms) < kRxActiveMs;
    if (receiving != s_receiving) {
        s_receiving = receiving;
        draw_ring();
        draw_status_label();
    }

    if (now - s_batt_ms >= kBatteryPollMs) {
        s_batt_ms = now;
        read_battery();
    }

    // Vibration pattern: on / off steps, even count = motor on.
    if (s_vib_steps > 0 && static_cast<int32_t>(now - s_vib_next_ms) >= 0) {
        const bool on = (s_vib_steps % 2) == 0;
        M5.Power.setVibration(on ? STOPWATCH_VIBRATION_LEVEL : 0);
        s_vib_next_ms = now + (on ? STOPWATCH_VIBRATION_ON_MS : STOPWATCH_VIBRATION_OFF_MS);
        --s_vib_steps;
    }
}

void stopwatch_ui_vibrate()
{
    if (s_vib_steps > 0) return;  // pattern already running
    s_vib_steps = STOPWATCH_VIBRATION_PULSES * 2;
    s_vib_next_ms = millis();
}

StopWatchTouch stopwatch_ui_poll_touch()
{
    if (!s_setup || !M5.Touch.isEnabled()) return StopWatchTouch::None;
    const auto t = M5.Touch.getDetail();
    if (!t.wasPressed()) return StopWatchTouch::None;
    const int x = t.x;
    const int y = t.y;
    const int hit_r = kTouchBtnR + 12;
    auto in_circle = [&](int bx) {
        const int dx = x - bx;
        const int dy = y - cy();
        return dx * dx + dy * dy <= hit_r * hit_r;
    };
    if (in_circle(cx() - kTouchBtnOfs)) return StopWatchTouch::Minus;
    if (in_circle(cx() + kTouchBtnOfs)) return StopWatchTouch::Plus;
    if (abs(x - cx()) <= kRowHalfW) {
        for (int i = 0; i < static_cast<int>(StopWatchSetupItem::Count); ++i) {
            if (abs(y - row_center_y(i)) <= kRowHalfH + 4) {
                return static_cast<StopWatchTouch>(static_cast<int>(StopWatchTouch::SelectChannel) + i);
            }
        }
    }
    return StopWatchTouch::None;
}

bool stopwatch_ui_ptt_pressed()
{
    return !s_setup && M5.BtnB.isPressed();
}

bool stopwatch_ui_setup_visible()
{
    return s_setup;
}

void stopwatch_ui_show_setup(bool show)
{
    display_lock();
    s_setup = show;
    if (show) {
        s_selected = StopWatchSetupItem::Channel;
        draw_setup();
    } else {
        draw_main();
    }
    display_unlock();
}

void stopwatch_ui_set_selected_item(StopWatchSetupItem item)
{
    const int prev = static_cast<int>(s_selected);
    s_selected = item;
    if (s_setup) {
        draw_setup_row(prev);
        draw_setup_row(static_cast<int>(item));
    }
}

StopWatchSetupItem stopwatch_ui_selected_item()
{
    return s_selected;
}

void stopwatch_ui_set_settings(int channel, int volume_level, uint8_t tx_pitch_mode)
{
    s_channel = channel;
    s_volume = volume_level;
    s_mode = tx_pitch_mode;
    if (s_setup) {
        draw_setup_row(static_cast<int>(s_selected));
    } else {
        draw_settings_line();
    }
}

void stopwatch_ui_set_status(bool transmitting, bool continuous)
{
    if (transmitting == s_transmitting && continuous == s_continuous) return;
    s_transmitting = transmitting;
    s_continuous = continuous;
    draw_ring();
    draw_status_label();
}

void stopwatch_ui_set_rssi(int16_t rssi)
{
    if (s_meter_valid && !s_meter_is_tx && s_meter_value == rssi) return;
    s_meter_is_tx = false;
    s_meter_value = rssi;
    s_meter_valid = true;
    draw_meter();
}

void stopwatch_ui_set_tx_power(int16_t dbm)
{
    if (s_meter_valid && s_meter_is_tx && s_meter_value == dbm) return;
    s_meter_is_tx = true;
    s_meter_value = dbm;
    s_meter_valid = true;
    draw_meter();
}

#else  // !TALKIE_TARGET_M5STOPWATCH

void stopwatch_ui_begin(int, int, uint8_t) {}
void stopwatch_ui_service(uint32_t) {}
StopWatchTouch stopwatch_ui_poll_touch() { return StopWatchTouch::None; }
bool stopwatch_ui_ptt_pressed() { return false; }
bool stopwatch_ui_setup_visible() { return false; }
void stopwatch_ui_show_setup(bool) {}
void stopwatch_ui_set_selected_item(StopWatchSetupItem) {}
StopWatchSetupItem stopwatch_ui_selected_item() { return StopWatchSetupItem::Channel; }
void stopwatch_ui_set_settings(int, int, uint8_t) {}
void stopwatch_ui_set_status(bool, bool) {}
void stopwatch_ui_set_rssi(int16_t) {}
void stopwatch_ui_set_tx_power(int16_t) {}
void stopwatch_ui_vibrate() {}

#endif
