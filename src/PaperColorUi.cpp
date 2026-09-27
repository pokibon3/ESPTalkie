#include "PaperColorUi.h"

#include "config.h"

#if TALKIE_TARGET_M5PAPERCOLOR

#include <Arduino.h>
#include <M5Unified.h>
#include <esp32-hal-rmt.h>

namespace {

constexpr uint8_t kLedBrightness = 48;
// Two WS2812-type LEDs on GPIO21 (power via M5PM1 LDO, enabled by M5.Power).
// M5Unified 0.2.15's LED driver only implements the IDF5 RMT API, so on
// Arduino-ESP32 2.x (IDF4.4) M5.Led.begin() always fails. Drive it directly.
constexpr int kLedPin = 21;
constexpr int kLedCount = 2;
rmt_obj_t *s_led_rmt = nullptr;
volatile int s_battery_level = -1;  // cached; read only from the main loop (I2C)

bool led_begin()
{
    s_led_rmt = rmtInit(kLedPin, RMT_TX_MODE, RMT_MEM_64);
    if (!s_led_rmt) {
        return false;
    }
    rmtSetTick(s_led_rmt, 100);  // 100 ns per tick
    return true;
}

// LED index of each lamp in the WS2812 chain (swap if left/right are reversed).
constexpr int kPowerLedIndex = 0;  // left: power lamp
constexpr int kRadioLedIndex = 1;  // right: TX/RX lamp
constexpr int kLowBatteryPercent = 20;
constexpr uint32_t kBatteryPollMs = 5000;
constexpr uint32_t kBlinkHalfPeriodMs = 500;

struct LedColor {
    uint8_t r, g, b;
    bool operator==(const LedColor &o) const { return r == o.r && g == o.g && b == o.b; }
};

void led_write(const LedColor (&colors)[kLedCount])
{
    if (!s_led_rmt) {
        return;
    }
    static rmt_data_t data[kLedCount * 24];
    int i = 0;
    for (int led = 0; led < kLedCount; ++led) {
        const uint8_t grb[3] = {
            static_cast<uint8_t>((colors[led].g * kLedBrightness) / 255),
            static_cast<uint8_t>((colors[led].r * kLedBrightness) / 255),
            static_cast<uint8_t>((colors[led].b * kLedBrightness) / 255),
        };
        for (int c = 0; c < 3; ++c) {
            for (int bit = 7; bit >= 0; --bit) {
                const bool one = (grb[c] >> bit) & 1U;
                data[i].level0 = 1;
                data[i].duration0 = one ? 8 : 4;
                data[i].level1 = 0;
                data[i].duration1 = one ? 4 : 8;
                ++i;
            }
        }
    }
    rmtWriteBlocking(s_led_rmt, data, kLedCount * 24);
}

extern "C" {
extern const uint8_t badge_image_start[] asm("_binary_assets_pokibon_transfer_png_start");
extern const uint8_t badge_image_end[] asm("_binary_assets_pokibon_transfer_png_end");
}

struct UiState {
    int channel = 1;
    int volume_level = 3;
    uint8_t tx_pitch_mode = 1;
    int16_t rssi = -127;
    bool settings_visible = false;
    uint8_t selected_item = 0;
    uint32_t revision = 0;
};

UiState s_ui;
portMUX_TYPE s_ui_mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t s_display_task = nullptr;
bool s_led_ready = false;
volatile PaperColorRadioState s_requested_led_state = PaperColorRadioState::Idle;
constexpr int kPttPin = 1;
volatile bool s_ptt_inhibit = false;

UiState snapshot_ui()
{
    taskENTER_CRITICAL(&s_ui_mux);
    UiState copy = s_ui;
    taskEXIT_CRITICAL(&s_ui_mux);
    return copy;
}

void request_redraw()
{
    taskENTER_CRITICAL(&s_ui_mux);
    ++s_ui.revision;
    taskEXIT_CRITICAL(&s_ui_mux);
    if (s_display_task) {
        xTaskNotifyGive(s_display_task);
    }
}

void draw_badge(M5Canvas &canvas)
{
    canvas.fillSprite(TFT_WHITE);
    const size_t image_size = static_cast<size_t>(badge_image_end - badge_image_start);
    const bool ok = canvas.drawPng(
        badge_image_start, image_size,
        0, 0, canvas.width(), canvas.height(),
        0, 0, 1.0F, 1.0F, middle_center);
    if (!ok) {
        canvas.setTextDatum(middle_center);
        canvas.setTextColor(TFT_RED, TFT_WHITE);
        canvas.setFont(&fonts::FreeSansBold18pt7b);
        canvas.drawString("BADGE IMAGE ERROR", canvas.width() / 2, canvas.height() / 2);
    }
}

void draw_item_frame(M5Canvas &canvas, int x, int y, int w, int h, bool selected, uint16_t color)
{
    if (selected) {
        for (int i = 0; i < 5; ++i) {
            canvas.drawRect(x + i, y + i, w - 2 * i, h - 2 * i, TFT_RED);
        }
    } else {
        canvas.drawRect(x, y, w, h, color);
    }
}

void draw_item_label(M5Canvas &canvas, const char *text, int cx, int cy, int bar_w, bool selected)
{
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    if (selected) {
        canvas.fillRect(cx - bar_w / 2, cy - 16, bar_w, 32, TFT_RED);
        canvas.setTextColor(TFT_WHITE, TFT_RED);
    } else {
        canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    }
    canvas.drawString(text, cx, cy);
}

void draw_settings(M5Canvas &canvas, const UiState &state)
{
    const int w = canvas.width();
    const auto item = static_cast<PaperColorSettingItem>(state.selected_item);
    const bool sel_channel = item == PaperColorSettingItem::Channel;
    const bool sel_volume = item == PaperColorSettingItem::Volume;
    const bool sel_voice = item == PaperColorSettingItem::Voice;
    canvas.fillSprite(TFT_WHITE);

    canvas.fillRect(0, 0, w, 68, TFT_RED);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(TFT_WHITE, TFT_RED);
    canvas.setFont(&fonts::FreeSansBold18pt7b);
    canvas.drawString("ESPTALKIE", w / 2, 34);

    draw_item_frame(canvas, 12, 78, w - 24, 196, sel_channel, TFT_BLACK);
    draw_item_label(canvas, "CHANNEL", w / 2, 104, 200, sel_channel);

    char text[48];
    snprintf(text, sizeof(text), "%02d", state.channel);
    canvas.setTextColor(TFT_BLUE, TFT_WHITE);
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextSize(2);
    canvas.drawString(text, w / 2, 184);
    canvas.setTextSize(1);

    const int frequency_mhz = 2407 + state.channel * 5;
    snprintf(text, sizeof(text), "%d MHz", frequency_mhz);
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.drawString(text, w / 2, 250);

    draw_item_frame(canvas, 20, 286, 172, 112, sel_volume, TFT_BLUE);
    draw_item_frame(canvas, 208, 286, 172, 112, sel_voice, TFT_GREEN);
    draw_item_label(canvas, "VOLUME", 106, 315, 140, sel_volume);
    draw_item_label(canvas, "VOICE", 294, 315, 140, sel_voice);
    snprintf(text, sizeof(text), "%d / 5", state.volume_level);
    canvas.setTextColor(TFT_BLUE, TFT_WHITE);
    canvas.setFont(&fonts::FreeSansBold18pt7b);
    canvas.drawString(text, 106, 364);
    snprintf(text, sizeof(text), "M%u", static_cast<unsigned>(state.tx_pitch_mode));
    canvas.setTextColor(TFT_GREEN, TFT_WHITE);
    canvas.drawString(text, 294, 364);

    canvas.drawFastHLine(20, 418, w - 40, TFT_BLACK);
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    if (state.rssi <= -127) {
        canvas.drawString("LAST RX  -- dBm", w / 2, 450);
    } else {
        snprintf(text, sizeof(text), "LAST RX  %d dBm", state.rssi);
        canvas.drawString(text, w / 2, 450);
    }

    const int battery = s_battery_level;
    if (battery >= 0) {
        snprintf(text, sizeof(text), "BATTERY %d%%", battery);
        canvas.drawString(text, w / 2, 490);
    }

    canvas.fillRect(0, 516, w, 84, TFT_YELLOW);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(TFT_BLACK, TFT_YELLOW);
    canvas.drawString("TOP: SELECT  CH / VOL / VOICE", w / 2, 531);
    canvas.drawString("LEFT-UP: +1   LEFT-MID: -1", w / 2, 557);
    canvas.drawString("A+B: BADGE / SETTINGS", w / 2, 583);
}

void display_task(void *)
{
    M5Canvas canvas(&M5.Display);
    canvas.setPsram(true);
    canvas.setColorDepth(16);
    if (!canvas.createSprite(M5.Display.width(), M5.Display.height())) {
        Serial.println("PaperColor: failed to allocate display canvas");
        papercolor_ui_set_radio_state(PaperColorRadioState::Error);
        vTaskDelete(nullptr);
    }

    uint32_t rendered_revision = UINT32_MAX;
    while (true) {
        const UiState state = snapshot_ui();
        if (state.revision != rendered_revision) {
            if (state.settings_visible) {
                draw_settings(canvas, state);
            } else {
                draw_badge(canvas);
            }
            canvas.pushSprite(0, 0);
            rendered_revision = state.revision;
        }
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

}  // namespace

void papercolor_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode)
{
    taskENTER_CRITICAL(&s_ui_mux);
    s_ui.channel = channel;
    s_ui.volume_level = volume_level;
    s_ui.tx_pitch_mode = tx_pitch_mode;
    s_ui.settings_visible = false;
    s_ui.revision = 1;
    taskEXIT_CRITICAL(&s_ui_mux);

    s_led_ready = led_begin();
    s_battery_level = M5.Power.getBatteryLevel();
    if (!s_led_ready) {
        Serial.println("PaperColor: RGB LED init failed");
    }

#if defined(CONFIG_FREERTOS_UNICORE) && CONFIG_FREERTOS_UNICORE
    xTaskCreate(display_task, "papercolor_display", 8192, nullptr, 1, &s_display_task);
#else
    xTaskCreatePinnedToCore(display_task, "papercolor_display", 8192, nullptr, 1, &s_display_task, 0);
#endif
    if (!s_display_task) {
        Serial.println("PaperColor: failed to create display task");
        papercolor_ui_set_radio_state(PaperColorRadioState::Error);
    }
}

void papercolor_ui_service()
{
    static uint32_t last_battery_poll_ms = 0;
    static bool written = false;
    static LedColor last[kLedCount];

    const uint32_t now = millis();
    if (now - last_battery_poll_ms >= kBatteryPollMs) {
        last_battery_poll_ms = now;
        s_battery_level = M5.Power.getBatteryLevel();
    }
    if (!s_led_ready) {
        return;
    }

    LedColor colors[kLedCount] = {};

    // Left: power lamp. Solid green; blinks at 1 Hz when battery <= 20 %.
    const int battery = s_battery_level;
    const bool low_battery = battery >= 0 && battery <= kLowBatteryPercent;
    const bool power_on = !low_battery || ((now / kBlinkHalfPeriodMs) & 1U) == 0;
    if (power_on) {
        colors[kPowerLedIndex] = { 0, 96, 0 };
    }

    // Right: radio lamp. Blue = RX, red = TX, orange = error, off = idle.
    switch (s_requested_led_state) {
        case PaperColorRadioState::Receiving:
            colors[kRadioLedIndex] = { 0, 0, 255 };
            break;
        case PaperColorRadioState::Transmitting:
            colors[kRadioLedIndex] = { 255, 0, 0 };
            break;
        case PaperColorRadioState::TransmittingContinuous:
            // Blink red so continuous TX is distinguishable from PTT hold.
            if (((now / kBlinkHalfPeriodMs) & 1U) == 0) {
                colors[kRadioLedIndex] = { 255, 0, 0 };
            }
            break;
        case PaperColorRadioState::Error:
            colors[kRadioLedIndex] = { 255, 48, 0 };
            break;
        case PaperColorRadioState::Idle:
        default:
            break;
    }

    bool changed = !written;
    for (int i = 0; i < kLedCount; ++i) {
        if (!(colors[i] == last[i])) {
            changed = true;
        }
    }
    if (!changed) {
        return;
    }
    led_write(colors);
    for (int i = 0; i < kLedCount; ++i) {
        last[i] = colors[i];
    }
    written = true;
}

void papercolor_ui_set_radio_state(PaperColorRadioState state)
{
    s_requested_led_state = state;
}

void papercolor_ui_update_settings(int channel, int volume_level, uint8_t tx_pitch_mode, int16_t rssi)
{
    taskENTER_CRITICAL(&s_ui_mux);
    s_ui.channel = channel;
    s_ui.volume_level = volume_level;
    s_ui.tx_pitch_mode = tx_pitch_mode;
    s_ui.rssi = rssi;
    taskEXIT_CRITICAL(&s_ui_mux);
}

void papercolor_ui_show_badge()
{
    taskENTER_CRITICAL(&s_ui_mux);
    s_ui.settings_visible = false;
    taskEXIT_CRITICAL(&s_ui_mux);
    request_redraw();
}

void papercolor_ui_show_settings()
{
    taskENTER_CRITICAL(&s_ui_mux);
    s_ui.settings_visible = true;
    taskEXIT_CRITICAL(&s_ui_mux);
    request_redraw();
}

bool papercolor_ui_settings_visible()
{
    taskENTER_CRITICAL(&s_ui_mux);
    const bool visible = s_ui.settings_visible;
    taskEXIT_CRITICAL(&s_ui_mux);
    return visible;
}

void papercolor_ui_set_selected_item(PaperColorSettingItem item)
{
    if (item >= PaperColorSettingItem::Count) {
        item = PaperColorSettingItem::Channel;
    }
    taskENTER_CRITICAL(&s_ui_mux);
    s_ui.selected_item = static_cast<uint8_t>(item);
    taskEXIT_CRITICAL(&s_ui_mux);
}

bool papercolor_ui_ptt_pressed()
{
    const bool raw = digitalRead(kPttPin) == LOW;
    if (papercolor_ui_settings_visible()) {
        // Top button is item-select here; keep PTT off until it is released
        // after returning to the badge page.
        s_ptt_inhibit = true;
        return false;
    }
    if (s_ptt_inhibit) {
        if (!raw) {
            s_ptt_inhibit = false;
        }
        return false;
    }
    return raw;
}

#else

void papercolor_ui_begin(int, int, uint8_t) {}
void papercolor_ui_service() {}
void papercolor_ui_set_radio_state(PaperColorRadioState) {}
void papercolor_ui_update_settings(int, int, uint8_t, int16_t) {}
void papercolor_ui_show_badge() {}
void papercolor_ui_show_settings() {}
bool papercolor_ui_settings_visible() { return false; }
void papercolor_ui_set_selected_item(PaperColorSettingItem) {}
bool papercolor_ui_ptt_pressed() { return false; }

#endif
