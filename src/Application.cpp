#include <Arduino.h>
#include <WiFi.h>
#include <M5Unified.h>
#include <math.h>
#include <string.h>
#include <SPIFFS.h>
#include <esp_wifi.h>

#include "Application.h"
#include "DisplaySync.h"
#include "EspNowTransport.h"
#include "OutputBuffer.h"
#include "UiLayout.h"
#include "config.h"

namespace {

static uint32_t s_tx_session_id = 1;
static uint16_t s_tx_fade_samples_remaining = 0;
constexpr size_t kMicWavWriteCacheSize = 8192;
static uint8_t s_mic_wav_write_cache[kMicWavWriteCacheSize];

static void begin_tx_session()
{
    ++s_tx_session_id;
    if (s_tx_session_id == 0) {
        s_tx_session_id = 1;
    }
    // Short fade-in to suppress click/pop at TX start.
    s_tx_fade_samples_remaining = 320; // about 20ms @16kHz
}

static void application_task(void *param)
{
    auto *application = reinterpret_cast<Application *>(param);
    application->loop();
}

static void write_wav_header(File &f, uint32_t sample_rate, uint16_t bits_per_sample, uint16_t channels, uint32_t data_bytes)
{
    const uint32_t byte_rate = sample_rate * channels * (bits_per_sample / 8);
    const uint16_t block_align = channels * (bits_per_sample / 8);
    const uint32_t chunk_size = 36 + data_bytes;

    f.seek(0);
    f.write(reinterpret_cast<const uint8_t *>("RIFF"), 4);
    f.write(reinterpret_cast<const uint8_t *>(&chunk_size), 4);
    f.write(reinterpret_cast<const uint8_t *>("WAVE"), 4);
    f.write(reinterpret_cast<const uint8_t *>("fmt "), 4);
    const uint32_t subchunk1_size = 16;
    const uint16_t audio_format_pcm = 1;
    f.write(reinterpret_cast<const uint8_t *>(&subchunk1_size), 4);
    f.write(reinterpret_cast<const uint8_t *>(&audio_format_pcm), 2);
    f.write(reinterpret_cast<const uint8_t *>(&channels), 2);
    f.write(reinterpret_cast<const uint8_t *>(&sample_rate), 4);
    f.write(reinterpret_cast<const uint8_t *>(&byte_rate), 4);
    f.write(reinterpret_cast<const uint8_t *>(&block_align), 2);
    f.write(reinterpret_cast<const uint8_t *>(&bits_per_sample), 2);
    f.write(reinterpret_cast<const uint8_t *>("data"), 4);
    f.write(reinterpret_cast<const uint8_t *>(&data_bytes), 4);
}

static void apply_octave_up_simple_u8_block(uint8_t *buf, size_t n)
{
    // Naive chipmunk shift:
    // Compress 2 chunks worth of timeline (prev + current) into current chunk size.
    // This raises pitch and speech speed by about +1 octave with very low CPU cost.
    static bool has_prev = false;
    static uint8_t prev[256];
    static uint8_t curr[256];
    static uint32_t last_session_id = 0;

    if (!buf || n == 0 || n > sizeof(prev)) {
        return;
    }
    if (last_session_id != s_tx_session_id) {
        has_prev = false;
        last_session_id = s_tx_session_id;
    }

    memcpy(curr, buf, n);
    if (!has_prev) {
        memcpy(prev, curr, n);
        has_prev = true;
        return;
    }

    for (size_t i = 0; i < n; ++i) {
        const size_t src = i * 2;
        buf[i] = (src < n) ? prev[src] : curr[src - n];
    }

    memcpy(prev, curr, n);
}

static void apply_triple_speed_simple_u8_block(uint8_t *buf, size_t n)
{
    // Compress 3 chunks worth of timeline (prev2 + prev1 + current) into current chunk size.
    // This raises pitch and speech speed by about 3x with very low CPU cost.
    static uint8_t prev2[256];
    static uint8_t prev1[256];
    static uint8_t curr[256];
    static uint8_t history_count = 0;
    static uint32_t last_session_id = 0;

    if (!buf || n == 0 || n > sizeof(prev1)) {
        return;
    }
    if (last_session_id != s_tx_session_id) {
        history_count = 0;
        last_session_id = s_tx_session_id;
    }

    memcpy(curr, buf, n);
    if (history_count < 2) {
        if (history_count == 0) {
            memcpy(prev1, curr, n);
        } else {
            memcpy(prev2, prev1, n);
            memcpy(prev1, curr, n);
        }
        ++history_count;
        return;
    }

    for (size_t i = 0; i < n; ++i) {
        const size_t src = i * 3;
        if (src < n) {
            buf[i] = prev2[src];
        } else if (src < (2 * n)) {
            buf[i] = prev1[src - n];
        } else {
            buf[i] = curr[src - (2 * n)];
        }
    }

    memcpy(prev2, prev1, n);
    memcpy(prev1, curr, n);
}

static void apply_quad_speed_simple_u8_block(uint8_t *buf, size_t n)
{
    // Compress 4 chunks of timeline (prev3 + prev2 + prev1 + current) into current chunk size.
    // This raises pitch and speech speed by about 4x with very low CPU cost.
    static uint8_t prev3[256];
    static uint8_t prev2[256];
    static uint8_t prev1[256];
    static uint8_t curr[256];
    static uint8_t history_count = 0;
    static uint32_t last_session_id = 0;

    if (!buf || n == 0 || n > sizeof(prev1)) {
        return;
    }
    if (last_session_id != s_tx_session_id) {
        history_count = 0;
        last_session_id = s_tx_session_id;
    }

    memcpy(curr, buf, n);
    if (history_count < 3) {
        if (history_count == 0) {
            memcpy(prev1, curr, n);
        } else if (history_count == 1) {
            memcpy(prev2, prev1, n);
            memcpy(prev1, curr, n);
        } else {
            memcpy(prev3, prev2, n);
            memcpy(prev2, prev1, n);
            memcpy(prev1, curr, n);
        }
        ++history_count;
        return;
    }

    for (size_t i = 0; i < n; ++i) {
        const size_t src = i * 4;
        if (src < n) {
            buf[i] = prev3[src];
        } else if (src < (2 * n)) {
            buf[i] = prev2[src - n];
        } else if (src < (3 * n)) {
            buf[i] = prev1[src - (2 * n)];
        } else {
            buf[i] = curr[src - (3 * n)];
        }
    }

    memcpy(prev3, prev2, n);
    memcpy(prev2, prev1, n);
    memcpy(prev1, curr, n);
}

static void convert_i16_to_u8_tx_compatible(const int16_t* in, uint8_t* out, size_t n)
{
    if (!in || !out || n == 0) {
        return;
    }

    // TX-side only refinement for 8-bit linear PCM:
    // - gentle peak compression to use quantization range better
    // - small TPDF-like dither before quantization to reduce "grainy" artifacts
    static uint32_t lfsr = 0x12345678u;
    constexpr int kDrivePct = 108;
    constexpr int kKnee = 11000;
    constexpr int kCeil = 22000;
    constexpr int kDitherAmp = 96;  // about 0.75 LSB in 8-bit domain

    auto next_rand = [&]() -> int {
        lfsr ^= lfsr << 13;
        lfsr ^= lfsr >> 17;
        lfsr ^= lfsr << 5;
        return static_cast<int>(lfsr & 0xFF);
    };

    for (size_t i = 0; i < n; ++i) {
        int x = static_cast<int>(in[i]);
        x = (x * kDrivePct) / 100;

        int ax = abs(x);
        if (ax > kKnee) {
            const int sign = (x >= 0) ? 1 : -1;
            const int over = ax - kKnee;
            int y = kKnee + (over / 3);
            if (y > kCeil) y = kCeil;
            x = sign * y;
        }

        const int dither = (next_rand() - next_rand()) * kDitherAmp / 255;
        x += dither;

        int v = 128 + ((x + 128) >> 8);
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        out[i] = static_cast<uint8_t>(v);
    }
}

static void apply_tx_frontend_i16_to_u8_block(const int16_t* in, uint8_t* out, size_t n)
{
    if (!in || !out || n == 0) {
        return;
    }

    // Front-end processing before 8-bit transport:
    // - short fade-in to suppress TX-start click
    // - near-linear conversion (limiter effectively disabled)
    constexpr int kCenter = 128;
    constexpr int kFadeTotalSamples = 320;  // about 20ms @16kHz
    constexpr int kNoiseGate = 0;           // disabled (caused choppy voice)
    constexpr int kKnee = 32000;   // practically disables compression
    constexpr int kCeil = 32000;
    constexpr int kGainPct = 96;   // slight trim for headroom

    for (size_t i = 0; i < n; ++i) {
        int x = static_cast<int>(in[i]);
        x = (x * kGainPct) / 100;

        if (s_tx_fade_samples_remaining > 0) {
            const int done = kFadeTotalSamples - static_cast<int>(s_tx_fade_samples_remaining);
            x = (x * done) / kFadeTotalSamples;
            --s_tx_fade_samples_remaining;
        }

        int ax = abs(x);
        if (kNoiseGate > 0 && ax < kNoiseGate) {
            x = 0;
            ax = 0;
        }
        if (ax > kKnee) {
            const int sign = (x >= 0) ? 1 : -1;
            const int over = ax - kKnee;
            int y = kKnee + (over / 4);
            if (y > kCeil) y = kCeil;
            x = sign * y;
        }

        int v = kCenter + ((x + 128) >> 8);
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        out[i] = static_cast<uint8_t>(v);
    }
}

static uint8_t default_pitch_mode_from_config()
{
#if TX_PITCH_MODE == TX_PITCH_MODE_OCTAVE_UP_SIMPLE
    return Application::kTxPitchModeM2;
#elif TX_PITCH_MODE == TX_PITCH_MODE_TRIPLE_SPEED_SIMPLE
    return Application::kTxPitchModeM3;
#else
    return Application::kTxPitchModeM1;
#endif
}

#if TALKIE_TARGET_M5STICKS3
static void draw_battery_status_icon(uint16_t status_color)
{
    int32_t battery_level = M5.Power.getBatteryLevel();
    if (battery_level < 0) battery_level = 0;
    if (battery_level > 100) battery_level = 100;

    static bool charging_display = false;
    static m5::Power_Class::is_charging_t charging_candidate = m5::Power_Class::charge_unknown;
    static uint8_t charging_candidate_count = 0;
    constexpr uint8_t kChargeDebounceCount = 3;

    auto charging_raw = M5.Power.isCharging();
    if (charging_raw == m5::Power_Class::charge_unknown) {
        charging_raw = charging_display ? m5::Power_Class::is_charging : m5::Power_Class::is_discharging;
    }
    if (charging_raw == charging_candidate) {
        if (charging_candidate_count < kChargeDebounceCount) {
            ++charging_candidate_count;
        }
    } else {
        charging_candidate = charging_raw;
        charging_candidate_count = 1;
    }
    if (charging_candidate_count >= kChargeDebounceCount) {
        charging_display = (charging_candidate == m5::Power_Class::is_charging);
    }

    const bool charging = charging_display;
    const uint16_t outline_color = charging ? TFT_YELLOW : TFT_WHITE;
    const uint16_t fill_color = charging ? TFT_YELLOW : status_color;

    const int term_w = 2;
    const int body_w = 25;
    const int body_h = 14;
    const int margin_r = 3;
    const int x = M5.Display.width() - margin_r - term_w - body_w;
    const int y = (kUiLayout.status_h - body_h) / 2;
    const int inner_x = x + 1;
    const int inner_y = y + 1;
    const int inner_w = body_w - 2;
    const int inner_h = body_h - 2;

    M5.Display.fillRect(inner_x, inner_y, inner_w, inner_h, fill_color);
    M5.Display.drawRect(x, y, body_w, body_h, outline_color);
    M5.Display.fillRect(x + body_w, y + (body_h / 3), term_w, body_h / 3, outline_color);

    char batt_text[4];
    snprintf(batt_text, sizeof(batt_text), "%ld", static_cast<long>(battery_level));

    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(middle_center);
    const int tx = x + (body_w / 2);
    const int ty = y + (body_h / 2);
    if (charging) {
        M5.Display.setTextColor(TFT_BLACK, fill_color);
        M5.Display.drawString(batt_text, tx, ty);
    } else {
        M5.Display.setTextColor(TFT_BLACK, fill_color);
        M5.Display.drawString(batt_text, tx + 1, ty + 1);
        M5.Display.setTextColor(TFT_WHITE, fill_color);
        M5.Display.drawString(batt_text, tx, ty);
    }
    M5.Display.setTextDatum(top_left);
}
#endif

static void dump_mic_wav_to_spiffs_10s()
{
    if (!SPIFFS.begin(true)) {
        return;
    }

    SPIFFS.remove("/mic_10s.wav");
    File f = SPIFFS.open("/mic_10s.wav", FILE_WRITE);
    if (!f) {
        return;
    }

    write_wav_header(f, SAMPLE_RATE, 16, 1, 0);

    auto mic_cfg = M5.Mic.config();
    mic_cfg.magnification = MIC_MAGNIFICATION;
    mic_cfg.over_sampling = 2;
    M5.Mic.config(mic_cfg);

    constexpr size_t kChunkSamples = 256;
    int16_t buf[kChunkSamples];
    uint32_t data_bytes = 0;
    size_t write_cache_used = 0;
    constexpr uint16_t warmup_chunks = 8;   // about 128ms @ 256 samples/chunk, 16kHz
    constexpr uint16_t record_fade_samples = 256;  // about 16ms @ 16kHz
    const uint32_t target_samples = static_cast<uint32_t>(MIC_WAV_DUMP_SECONDS) * SAMPLE_RATE;
    int last_remaining = -1;
    const UBaseType_t old_prio = uxTaskPriorityGet(nullptr);
    const UBaseType_t raised_prio = (configMAX_PRIORITIES > 2) ? (configMAX_PRIORITIES - 2) : old_prio;

    auto flush_cache = [&]() {
        if (write_cache_used == 0) {
            return;
        }
        size_t written = 0;
        while (written < write_cache_used) {
            size_t n = f.write(s_mic_wav_write_cache + written, write_cache_used - written);
            if (n == 0) {
                break;
            }
            written += n;
        }
        data_bytes += static_cast<uint32_t>(written);
        write_cache_used = 0;
    };

    auto draw_countdown = [&](int remaining_sec) {
        const uint16_t bg = M5.Display.color565(20, 20, 20);
        const uint16_t fg = TFT_WHITE;
        M5.Display.fillRect(0, 0, M5.Display.width(), kUiLayout.status_h, bg);
        M5.Display.setFont(&fonts::Font0);
        M5.Display.setTextDatum(middle_center);
        M5.Display.setTextSize(2);
        M5.Display.setTextColor(fg, bg);
        char msg[24];
        snprintf(msg, sizeof(msg), "REC %2ds", remaining_sec);
        M5.Display.drawString(msg, M5.Display.width() / 2, kUiLayout.status_h / 2);
        M5.Display.setTextDatum(top_left);
    };

    vTaskPrioritySet(nullptr, raised_prio);
    M5.Mic.begin();

    // Discard initial DMA/mic startup chunks to avoid startup click/pop.
    uint16_t discarded_chunks = 0;
    while (discarded_chunks < warmup_chunks) {
        if (M5.Mic.record(buf, kChunkSamples, SAMPLE_RATE, true)) {
            ++discarded_chunks;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint32_t collected_samples = 0;
    uint16_t fade_samples_remaining = record_fade_samples;
    while (collected_samples < target_samples) {
        int remaining = static_cast<int>((target_samples - collected_samples + SAMPLE_RATE - 1) / SAMPLE_RATE);
        if (remaining < 0) remaining = 0;
        if (remaining != last_remaining) {
            draw_countdown(remaining);
            last_remaining = remaining;
        }
        bool recorded = M5.Mic.record(buf, kChunkSamples, SAMPLE_RATE, true);
        if (!recorded) {
            memset(buf, 0, sizeof(buf));
        }
        if (fade_samples_remaining > 0) {
            for (size_t i = 0; i < kChunkSamples && fade_samples_remaining > 0; ++i) {
                const int done = static_cast<int>(record_fade_samples - fade_samples_remaining);
                int32_t x = buf[i];
                x = (x * done) / static_cast<int>(record_fade_samples);
                buf[i] = static_cast<int16_t>(x);
                --fade_samples_remaining;
            }
        }
        // Save raw mic samples for diagnosis (no filter/effect).
        const uint8_t* src = reinterpret_cast<const uint8_t*>(buf);
        size_t bytes_remaining = sizeof(buf);
        while (bytes_remaining > 0) {
            size_t room = kMicWavWriteCacheSize - write_cache_used;
            size_t to_copy = (bytes_remaining < room) ? bytes_remaining : room;
            memcpy(s_mic_wav_write_cache + write_cache_used, src, to_copy);
            write_cache_used += to_copy;
            src += to_copy;
            bytes_remaining -= to_copy;
            if (write_cache_used == kMicWavWriteCacheSize) {
                flush_cache();
            }
        }
        collected_samples += static_cast<uint32_t>(kChunkSamples);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    flush_cache();
    M5.Mic.end();
    vTaskPrioritySet(nullptr, old_prio);

    write_wav_header(f, SAMPLE_RATE, 16, 1, data_bytes);
    f.close();
    Serial.printf("WAV dump completed: /mic_10s.wav (%lu bytes)\n", static_cast<unsigned long>(data_bytes));
}

}  // namespace

Application::Application() :
    m_transport(nullptr),
    m_output_buffer(nullptr),
    m_channel(ESP_NOW_WIFI_CHANNEL),
    m_speaker_volume(132),
    m_tx_pitch_mode(default_pitch_mode_from_config())
{
    m_output_buffer = new OutputBuffer(300 * 16);
    m_transport = new EspNowTransport(m_output_buffer, static_cast<uint8_t>(m_channel));
}

void Application::begin()
{
    Serial.print("My IDF Version is: ");
    Serial.println(esp_get_idf_version());

#if AUDIO_DIAG_SOURCE == AUDIO_DIAG_SRC_MIC
    auto mic_cfg = M5.Mic.config();
    mic_cfg.magnification = MIC_MAGNIFICATION;
    mic_cfg.over_sampling = 2;
    M5.Mic.config(mic_cfg);
#endif

#if MIC_WAV_DUMP_TO_SPIFFS
    // Capture diagnostic WAV before enabling radio/transport.
    dump_mic_wav_to_spiffs_10s();
#endif

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(true);

    Serial.print("My MAC Address is: ");
    Serial.println(WiFi.macAddress());

    m_transport->begin();

    M5.Speaker.begin();
    M5.Speaker.setVolume(m_speaker_volume);
    M5.Speaker.tone(1200, 80);

    TaskHandle_t task_handle;
    xTaskCreate(application_task, "application_task", 8192, this, 1, &task_handle);
}

void Application::setChannel(uint16_t ch)
{
    m_channel = ch;
    m_transport->setWifiChannel(ch);
}

void Application::setSpeakerVolume(uint8_t volume)
{
    m_speaker_volume = volume;
    M5.Speaker.setVolume(m_speaker_volume);
}

uint8_t Application::getSpeakerVolume() const
{
    return m_speaker_volume;
}

void Application::setTxPitchMode(uint8_t mode)
{
    if (mode < kTxPitchModeM1 || mode > kTxPitchModeM3) {
        mode = kTxPitchModeM1;
    }
    m_tx_pitch_mode = mode;
}

uint8_t Application::getTxPitchMode() const
{
    return m_tx_pitch_mode;
}

int16_t Application::getRSSI()
{
    return m_transport->getRSSI();
}

void Application::dispRSSI(int16_t rssi)
{
    display_lock();
    const uint16_t kBarLeftOn = TFT_GREEN;
    const uint16_t kBarRightOn = TFT_RED;
    static const int16_t rssi_level[] = { -90, -80, -70, -60, -50, -40, -30, -20 };
    const uint16_t panel = M5.Display.color565(44, 52, 62);
    const uint16_t text = M5.Display.color565(235, 245, 255);
    const uint16_t text_sub = M5.Display.color565(160, 205, 255);
    const uint16_t bar_bg = M5.Display.color565(232, 250, 255);
    const uint16_t bar_off = TFT_BLACK;

    // RSSI numeric (info row, right)
    M5.Display.fillRoundRect(kUiLayout.rssi_x, kUiLayout.info_y, kUiLayout.info_w, kUiLayout.info_h, kUiLayout.info_radius, panel);
    M5.Display.drawRoundRect(kUiLayout.rssi_x, kUiLayout.info_y, kUiLayout.info_w, kUiLayout.info_h, kUiLayout.info_radius, TFT_BLUE);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(text_sub, panel);
    M5.Display.setCursor(kUiLayout.rssi_label_x, kUiLayout.rssi_label_y);
#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
    M5.Display.print("RS");
#else
    M5.Display.print("RSSI");
#endif
    M5.Display.setTextSize(kUiLayout.rssi_value_text_size);
    M5.Display.setTextColor(text, panel);
    M5.Display.setCursor(kUiLayout.rssi_value_x, kUiLayout.rssi_value_y);
    M5.Display.printf("%d", rssi);

    // Level bar (bottom)
    M5.Display.fillRoundRect(kUiLayout.bar_x, kUiLayout.bar_y, kUiLayout.bar_w, kUiLayout.bar_h, kUiLayout.bar_radius, bar_bg);
    M5.Display.drawRoundRect(kUiLayout.bar_x, kUiLayout.bar_y, kUiLayout.bar_w, kUiLayout.bar_h, kUiLayout.bar_radius, TFT_BLUE);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_BLACK, bar_bg);
    M5.Display.setTextDatum(top_left);
    M5.Display.drawString("SIGNAL", kUiLayout.bar_label_x, kUiLayout.bar_label_y);
    const int base_y = kUiLayout.bar_base_y;
    const int bar_w = kUiLayout.bar_col_w;
    const int gap = kUiLayout.bar_col_gap;

    for (int i = 0; i < 8; ++i) {
        const int h = kUiLayout.bar_min_h + i * kUiLayout.bar_step_h;
        const int x = kUiLayout.bar_start_x + i * (bar_w + gap);
        const int y = base_y - h;
        uint16_t color = (rssi >= rssi_level[i]) ? ((i < 5) ? kBarLeftOn : kBarRightOn) : bar_off;
        M5.Display.fillRoundRect(x, y, bar_w, h, kUiLayout.bar_col_radius, color);
    }
    M5.Display.setTextDatum(top_left);
    display_unlock();
}

void Application::dispStatus(bool transmitting)
{
    display_lock();
    const uint16_t status_color = transmitting
        ? TFT_RED
        : TFT_BLUE;

    M5.Display.fillRect(0, 0, M5.Display.width(), kUiLayout.status_h, status_color);
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextDatum(middle_center);
    const char* label = transmitting ? "Transmit" : "Receive";
    int status_text_area_w = M5.Display.width();
#if TALKIE_TARGET_M5STICKS3
    constexpr int kBatteryAreaW = 31;  // battery icon + right margin on StickS3
    status_text_area_w -= kBatteryAreaW;
#endif
    const int cx = status_text_area_w / 2;
    const int cy = kUiLayout.status_h / 2;

#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
    int best_size = 1;
    for (int s = 1; s <= 6; ++s) {
        M5.Display.setTextSize(s);
        const int text_w = M5.Display.textWidth(label);
        const int text_h = M5.Display.fontHeight();
        if (text_w <= (M5.Display.width() - 4) && text_h <= (kUiLayout.status_h - 2)) {
            best_size = s;
        } else {
            break;
        }
    }
    M5.Display.setTextSize(best_size);
#else
    M5.Display.setTextSize(kUiLayout.status_text_size);
#endif
    M5.Display.setTextColor(TFT_BLACK, status_color);
    M5.Display.drawString(label, cx + 1, cy + 1);
    M5.Display.setTextColor(TFT_WHITE, status_color);
    M5.Display.drawString(label, cx, cy);
#if TALKIE_TARGET_M5STICKS3
    draw_battery_status_icon(status_color);
#endif
    M5.Display.setTextDatum(top_left);
    display_unlock();
}

void Application::dispTxPower(int16_t dbm)
{
    display_lock();
    const uint16_t kBarLeftOn = TFT_GREEN;
    const uint16_t kBarRightOn = TFT_RED;
    const uint16_t panel = M5.Display.color565(44, 52, 62);
    const uint16_t text = M5.Display.color565(235, 245, 255);
    const uint16_t text_sub = M5.Display.color565(160, 205, 255);
    const uint16_t bar_bg = M5.Display.color565(232, 250, 255);
    const uint16_t bar_off = TFT_BLACK;

    int active_bars = dbm / 3;
    if (active_bars < 0) active_bars = 0;
    if (active_bars > 8) active_bars = 8;

    // Tx Power numeric (info row, right)
    M5.Display.fillRoundRect(kUiLayout.rssi_x, kUiLayout.info_y, kUiLayout.info_w, kUiLayout.info_h, kUiLayout.info_radius, panel);
    M5.Display.drawRoundRect(kUiLayout.rssi_x, kUiLayout.info_y, kUiLayout.info_w, kUiLayout.info_h, kUiLayout.info_radius, TFT_BLUE);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(text_sub, panel);
    M5.Display.setCursor(kUiLayout.tx_label_x, kUiLayout.tx_label_y);
    M5.Display.print("TXdBm");
    M5.Display.setTextSize(kUiLayout.tx_value_text_size);
    M5.Display.setTextColor(text, panel);
    M5.Display.setCursor(kUiLayout.tx_value_x, kUiLayout.tx_value_y);
    M5.Display.printf("%d", dbm);

    // Level bar (bottom)
    M5.Display.fillRoundRect(kUiLayout.bar_x, kUiLayout.bar_y, kUiLayout.bar_w, kUiLayout.bar_h, kUiLayout.bar_radius, bar_bg);
    M5.Display.drawRoundRect(kUiLayout.bar_x, kUiLayout.bar_y, kUiLayout.bar_w, kUiLayout.bar_h, kUiLayout.bar_radius, TFT_BLUE);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_BLACK, bar_bg);
    M5.Display.setTextDatum(top_left);
    M5.Display.drawString("POWER", kUiLayout.bar_label_x, kUiLayout.bar_label_y);
    const int base_y = kUiLayout.bar_base_y;
    const int bar_w = kUiLayout.bar_col_w;
    const int gap = kUiLayout.bar_col_gap;
    for (int i = 0; i < 8; ++i) {
        const int h = kUiLayout.bar_min_h + i * kUiLayout.bar_step_h;
        const int x = kUiLayout.bar_start_x + i * (bar_w + gap);
        const int y = base_y - h;
        uint16_t color = (i < active_bars) ? ((i < 5) ? kBarLeftOn : kBarRightOn) : bar_off;
        M5.Display.fillRoundRect(x, y, bar_w, h, kUiLayout.bar_col_radius, color);
    }
    M5.Display.setTextDatum(top_left);
    display_unlock();
}

void Application::loop()
{
    constexpr size_t mic_chunk_samples = 128;
    constexpr size_t play_chunk_bytes = 128;  // 8-bit PCM from OutputBuffer
    constexpr bool enable_rssi_overlay = true;
    constexpr TickType_t synth_chunk_delay_ticks =
        pdMS_TO_TICKS((mic_chunk_samples * 1000 + SAMPLE_RATE - 1) / SAMPLE_RATE);
    int16_t *mic_samples = reinterpret_cast<int16_t *>(malloc(sizeof(int16_t) * mic_chunk_samples));
    uint8_t *mic_samples_u8 = reinterpret_cast<uint8_t *>(malloc(mic_chunk_samples));
    uint8_t *play_samples = reinterpret_cast<uint8_t *>(malloc(play_chunk_bytes));
    int16_t *play_samples_stereo = reinterpret_cast<int16_t *>(malloc(sizeof(int16_t) * play_chunk_bytes * 2));
    bool mic_active = false;
    bool spk_active = true;
    uint32_t last_rssi_draw_ms = 0;
    uint32_t last_diag_ms = 0;
    uint32_t mic_record_fail_count = 0;
    const uint32_t ptt_enable_after_ms = millis() + 1000;
    float tone_phase = 0.0f;
    constexpr float tone_freq_hz = 1000.0f;
    constexpr float two_pi = 6.28318530718f;
    const float tone_phase_step = two_pi * tone_freq_hz / static_cast<float>(SAMPLE_RATE);
    constexpr int16_t tone_amplitude = 12000;

    if (!mic_samples || !mic_samples_u8 || !play_samples || !play_samples_stereo) {
        Serial.println("Failed to allocate audio buffers");
        vTaskDelete(nullptr);
    }

    auto emit_diag = [&](const char* phase) {
        uint32_t now = millis();
        if (now - last_diag_ms < 1000) {
            return;
        }
        last_diag_ms = now;

        uint32_t rx_ok = 0;
        uint32_t rx_ok_bytes = 0;
        uint32_t rx_bad_header = 0;
        uint32_t rx_invalid_len = 0;
        uint32_t rx_gap_events = 0;
        uint32_t rx_max_gap_ms = 0;
        uint32_t tx_packets = 0;
        uint32_t tx_failures = 0;
        auto *espnow = static_cast<EspNowTransport*>(m_transport);
        espnow->snapshot_and_reset_stats(rx_ok, rx_ok_bytes, rx_bad_header, rx_invalid_len, rx_gap_events, rx_max_gap_ms, tx_packets, tx_failures);

        uint32_t underruns = 0;
        uint32_t overflows = 0;
        m_output_buffer->snapshot_and_reset_stats(underruns, overflows);
        int avail = m_output_buffer->get_available_samples();
        int cap = m_output_buffer->get_buffer_size();

        (void)phase;
        (void)rx_ok;
        (void)rx_ok_bytes;
        (void)rx_bad_header;
        (void)rx_invalid_len;
        (void)rx_gap_events;
        (void)rx_max_gap_ms;
        (void)tx_packets;
        (void)tx_failures;
        (void)avail;
        (void)cap;
        (void)underruns;
        (void)overflows;
        mic_record_fail_count = 0;
    };

    while (true) {
        bool ptt = (millis() > ptt_enable_after_ms) && M5.BtnA.isPressed();
        if (ptt) {
            begin_tx_session();
            if (enable_rssi_overlay) {
                dispStatus(true);
                int8_t tx_qdbm = 0;
                if (esp_wifi_get_max_tx_power(&tx_qdbm) == ESP_OK) {
                    dispTxPower(tx_qdbm / 4);
                }
            }
            if (spk_active) {
                M5.Speaker.stop();
                M5.Speaker.end();
                spk_active = false;
            }
#if AUDIO_DIAG_SOURCE == AUDIO_DIAG_SRC_MIC
            if (!mic_active) {
                M5.Mic.begin();
                mic_active = true;
            }
#endif

            unsigned long start_time = millis();
            while (millis() - start_time < 1000 || M5.BtnA.isPressed()) {
                if (enable_rssi_overlay) {
                    uint32_t now = millis();
                    if (now - last_rssi_draw_ms >= 500) {
                        dispStatus(true);
                        int8_t tx_qdbm = 0;
                        if (esp_wifi_get_max_tx_power(&tx_qdbm) == ESP_OK) {
                            dispTxPower(tx_qdbm / 4);
                        }
                        last_rssi_draw_ms = now;
                    }
                }
                bool ready = false;
                size_t send_samples = mic_chunk_samples;
#if AUDIO_DIAG_SOURCE == AUDIO_DIAG_SRC_MIC
                ready = M5.Mic.record(mic_samples, mic_chunk_samples, SAMPLE_RATE, false);
                if (!ready) {
                    ++mic_record_fail_count;
                }
#elif AUDIO_DIAG_SOURCE == AUDIO_DIAG_SRC_SILENCE
                memset(mic_samples, 0, sizeof(int16_t) * mic_chunk_samples);
                ready = true;
#elif AUDIO_DIAG_SOURCE == AUDIO_DIAG_SRC_TONE
                for (size_t i = 0; i < mic_chunk_samples; ++i) {
                    mic_samples[i] = static_cast<int16_t>(sinf(tone_phase) * tone_amplitude);
                    tone_phase += tone_phase_step;
                    if (tone_phase >= two_pi) {
                        tone_phase -= two_pi;
                    }
                }
                ready = true;
#endif

                if (ready) {
#if AUDIO_DIAG_SOURCE == AUDIO_DIAG_SRC_MIC
                    convert_i16_to_u8_tx_compatible(mic_samples, mic_samples_u8, send_samples);
                    switch (m_tx_pitch_mode) {
                        case kTxPitchModeM2:
                            apply_octave_up_simple_u8_block(mic_samples_u8, send_samples);
                            break;
                        case kTxPitchModeM3:
                            apply_triple_speed_simple_u8_block(mic_samples_u8, send_samples);
                            break;
                        default:
                            break;
                    }
                    for (size_t i = 0; i < send_samples; ++i) {
                        m_transport->add_sample_u8(mic_samples_u8[i]);
                    }
#else
                    for (size_t i = 0; i < send_samples; ++i) {
                        m_transport->add_sample(mic_samples[i]);
                    }
#endif
                }
#if AUDIO_DIAG_SOURCE == AUDIO_DIAG_SRC_MIC
                vTaskDelay(pdMS_TO_TICKS(1));
#else
                vTaskDelay(synth_chunk_delay_ticks);
#endif
                emit_diag("TX");
            }
            m_transport->flush();
#if AUDIO_DIAG_SOURCE == AUDIO_DIAG_SRC_MIC
            if (mic_active) {
                M5.Mic.end();
                mic_active = false;
            }
#endif
            if (!spk_active) {
                M5.Speaker.begin();
                M5.Speaker.setVolume(m_speaker_volume);
                spk_active = true;
            }
        }

        unsigned long start_time = millis();
        if (enable_rssi_overlay) {
            dispStatus(false);
        }
        while (millis() - start_time < 1000 || !M5.BtnA.isPressed()) {
            if (enable_rssi_overlay) {
                uint32_t now = millis();
                if (now - last_rssi_draw_ms >= 500) {  // lower UI refresh load
                    dispStatus(false);
                    dispRSSI(getRSSI());
                    last_rssi_draw_ms = now;
                }
            }

            if (!spk_active) {
                M5.Speaker.begin();
                M5.Speaker.setVolume(m_speaker_volume);
                spk_active = true;
            }
            m_output_buffer->remove_samples(play_samples, play_chunk_bytes);
            for (size_t i = 0; i < play_chunk_bytes; ++i) {
                // Convert unsigned 8-bit PCM (0..255, center=128) to signed 16-bit PCM.
                int16_t s = static_cast<int16_t>((static_cast<int16_t>(play_samples[i]) - 128) << 8);
                play_samples_stereo[i * 2 + 0] = s;
                play_samples_stereo[i * 2 + 1] = s;
            }
            while (M5.Speaker.isPlaying()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            M5.Speaker.playRaw(play_samples_stereo, play_chunk_bytes * 2, SAMPLE_RATE, true, 1, -1, true);
            vTaskDelay(pdMS_TO_TICKS(1));
            emit_diag("RX");
        }
    }
}
