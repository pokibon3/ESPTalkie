#include "PaperColorUi.h"

#include "config.h"

#if TALKIE_TARGET_M5PAPERCOLOR

#include <Arduino.h>
#include <M5Unified.h>

namespace {

constexpr uint8_t kLedBrightness = 48;

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
    uint32_t revision = 0;
};

UiState s_ui;
portMUX_TYPE s_ui_mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t s_display_task = nullptr;
bool s_led_ready = false;
volatile PaperColorRadioState s_requested_led_state = PaperColorRadioState::Idle;
PaperColorRadioState s_applied_led_state = PaperColorRadioState::Error;

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

void draw_settings(M5Canvas &canvas, const UiState &state)
{
    const int w = canvas.width();
    canvas.fillSprite(TFT_WHITE);

    canvas.fillRect(0, 0, w, 68, TFT_RED);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(TFT_WHITE, TFT_RED);
    canvas.setFont(&fonts::FreeSansBold18pt7b);
    canvas.drawString("ESPTALKIE", w / 2, 34);

    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.drawString("CHANNEL", w / 2, 104);

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

    canvas.drawRect(20, 286, 172, 112, TFT_BLUE);
    canvas.drawRect(208, 286, 172, 112, TFT_GREEN);
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.drawString("VOLUME", 106, 315);
    canvas.drawString("VOICE", 294, 315);
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

    const int battery = M5.Power.getBatteryLevel();
    if (battery >= 0) {
        snprintf(text, sizeof(text), "BATTERY %d%%", battery);
        canvas.drawString(text, w / 2, 490);
    }

    canvas.fillRect(0, 516, w, 84, TFT_YELLOW);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(TFT_BLACK, TFT_YELLOW);
    canvas.drawString("LEFT-UP: VOL+ / Hold: MODE", w / 2, 531);
    canvas.drawString("LEFT-MID: CH+ / Hold: CH-", w / 2, 557);
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

    M5.Led.setAutoDisplay(false);
    s_led_ready = M5.Led.begin();
    if (s_led_ready) {
        M5.Led.setBrightness(kLedBrightness);
        M5.Led.setAllColor(0, 0, 0);
        M5.Led.display();
    } else {
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
    if (!s_led_ready || s_requested_led_state == s_applied_led_state) {
        return;
    }
    const PaperColorRadioState requested = s_requested_led_state;
    switch (requested) {
        case PaperColorRadioState::Receiving:
            M5.Led.setAllColor(0, 0, 255);
            break;
        case PaperColorRadioState::Transmitting:
            M5.Led.setAllColor(255, 0, 0);
            break;
        case PaperColorRadioState::Error:
            M5.Led.setAllColor(255, 48, 0);
            break;
        case PaperColorRadioState::Idle:
        default:
            M5.Led.setAllColor(0, 96, 0);
            break;
    }
    M5.Led.display();
    s_applied_led_state = requested;
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

#else

void papercolor_ui_begin(int, int, uint8_t) {}
void papercolor_ui_service() {}
void papercolor_ui_set_radio_state(PaperColorRadioState) {}
void papercolor_ui_update_settings(int, int, uint8_t, int16_t) {}
void papercolor_ui_show_badge() {}
void papercolor_ui_show_settings() {}
bool papercolor_ui_settings_visible() { return false; }

#endif
