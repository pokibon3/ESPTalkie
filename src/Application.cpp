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
constexpr size_t kRxPlayChunkSamples = RX_PLAY_CHUNK_SAMPLES;
constexpr size_t kRxPlayChunkBytes = kRxPlayChunkSamples;
static uint8_t s_mic_wav_write_cache[kMicWavWriteCacheSize];

static void begin_tx_session()
{
    ++s_tx_session_id;
    if (s_tx_session_id == 0) {
        s_tx_session_id = 1;
    }
    // Short fade-in to suppress click/pop at TX start.
    s_tx_fade_samples_remaining = 128; // about 8ms @16kHz
}

static void application_task(void *param)
{
    auto *application = reinterpret_cast<Application *>(param);
    application->loop();
}

static void scope_plot_chunk_i16(const int16_t *samples, size_t n);
static void scope_plot_chunk_u8_linear(const uint8_t *samples, size_t n);

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

static void apply_tx_pitch_mode_u8_block(uint8_t mode, uint8_t *buf, size_t n)
{
    switch (mode) {
        case Application::kTxPitchModeM2:
            apply_octave_up_simple_u8_block(buf, n);
            break;
        case Application::kTxPitchModeM3:
            apply_triple_speed_simple_u8_block(buf, n);
            break;
        case Application::kTxPitchModeM1:
        default:
            break;
    }
}

static void convert_i16_to_u8_tx_compatible(const int16_t* in, uint8_t* out, size_t n)
{
    if (!in || !out || n == 0) {
        return;
    }

#if TX_8BIT_COMPRESSOR_ENABLE
    // TX-side refinement for 8-bit linear PCM:
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
#else
    // Minimal conversion only: signed 16-bit PCM -> unsigned 8-bit PCM.
    for (size_t i = 0; i < n; ++i) {
        int v = 128 + (static_cast<int>(in[i]) >> 8);
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        out[i] = static_cast<uint8_t>(v);
    }
#endif
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
    constexpr int kFadeTotalSamples = 128;  // about 8ms @16kHz
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

static uint8_t linear16_to_mulaw(int16_t sample)
{
    static const int16_t kSegEnd[8] = { 0x00FF, 0x01FF, 0x03FF, 0x07FF, 0x0FFF, 0x1FFF, 0x3FFF, 0x7FFF };
    constexpr int kBias = 0x84;
    constexpr int kClip = 32635;

    int pcm = sample;
    uint8_t mask = 0xFF;
    if (pcm < 0) {
        pcm = -pcm;
        mask = 0x7F;
    }
    if (pcm > kClip) {
        pcm = kClip;
    }
    pcm += kBias;

    uint8_t seg = 0;
    while (seg < 8 && pcm > kSegEnd[seg]) {
        ++seg;
    }
    if (seg >= 8) {
        return static_cast<uint8_t>(0x7F ^ mask);
    }
    const uint8_t uval = static_cast<uint8_t>((seg << 4) | ((pcm >> (seg + 3)) & 0x0F));
    return static_cast<uint8_t>(uval ^ mask);
}

static int16_t mulaw_to_linear16(uint8_t uval)
{
    constexpr int kBias = 0x84;
    uval = static_cast<uint8_t>(~uval);
    int t = ((uval & 0x0F) << 3) + kBias;
    t <<= ((uval & 0x70) >> 4);
    return (uval & 0x80) ? static_cast<int16_t>(kBias - t)
                         : static_cast<int16_t>(t - kBias);
}

#if 1
struct ScopeState {
    bool initialized = false;
    int w = 0;
    int h = 0;
    int mid = 0;
    int x = 0;
    int clip_top = 0;
    int clip_bottom = 0;
    uint16_t bg = TFT_BLACK;
    uint16_t grid = TFT_DARKGREY;
    uint16_t wave = TFT_CYAN;
    uint16_t clip = TFT_RED;
};

static ScopeState s_scope;
constexpr int kScopeSweepStep = 3;

static constexpr int16_t scope_clip_pos()
{
#if PTT_TEST_AUDIO_PATH == PTT_TEST_AUDIO_PATH_16BIT
    return 32767;
#elif PTT_TEST_AUDIO_PATH == PTT_TEST_AUDIO_PATH_8BIT_MULAW
    return 32124;  // approx max for G.711 u-law decode
#else
    return 32512;  // dequantized from 8-bit linear value 255
#endif
}

static constexpr int16_t scope_clip_neg()
{
#if PTT_TEST_AUDIO_PATH == PTT_TEST_AUDIO_PATH_16BIT
    return -32768;
#elif PTT_TEST_AUDIO_PATH == PTT_TEST_AUDIO_PATH_8BIT_MULAW
    return -32124;  // approx min for G.711 u-law decode
#else
    return -32768;  // dequantized from 8-bit linear value 0
#endif
}

static int sample_to_scope_y(int16_t s)
{
    const int amp = (s_scope.h / 2) - 2;
    int y = s_scope.mid - static_cast<int>((static_cast<int32_t>(s) * amp) / 32768);
    if (y < 0) y = 0;
    if (y >= s_scope.h) y = s_scope.h - 1;
    return y;
}

static void scope_clear_and_grid()
{
    M5.Display.fillScreen(s_scope.bg);
    M5.Display.drawFastHLine(0, s_scope.mid, s_scope.w, s_scope.grid);
    M5.Display.drawFastHLine(0, s_scope.clip_top, s_scope.w, s_scope.grid);
    M5.Display.drawFastHLine(0, s_scope.clip_bottom, s_scope.w, s_scope.grid);
}

static void scope_init_fullscreen_landscape()
{
    display_lock();
    M5.Display.setRotation(1);  // landscape, full-width scope for test mode
    s_scope.w = M5.Display.width();
    s_scope.h = M5.Display.height();
    s_scope.mid = s_scope.h / 2;
    s_scope.x = 0;
    s_scope.clip_top = sample_to_scope_y(scope_clip_pos());
    s_scope.clip_bottom = sample_to_scope_y(scope_clip_neg());
    scope_clear_and_grid();
    display_unlock();
    s_scope.initialized = true;
}

static void scope_plot_chunk_i16(const int16_t *samples, size_t n)
{
    if (!s_scope.initialized) {
        scope_init_fullscreen_landscape();
    }
    if (!samples || n == 0) {
        return;
    }

    int16_t vmin = 32767;
    int16_t vmax = -32768;
    bool clipped = false;
    for (size_t i = 0; i < n; ++i) {
        const int16_t s = samples[i];
        if (s < vmin) vmin = s;
        if (s > vmax) vmax = s;
        if (s >= scope_clip_pos() || s <= scope_clip_neg()) {
            clipped = true;
        }
    }

    const int x = s_scope.x;
    int y1 = sample_to_scope_y(vmax);
    int y2 = sample_to_scope_y(vmin);
    if (y1 > y2) {
        const int tmp = y1;
        y1 = y2;
        y2 = tmp;
    }

    display_lock();
    M5.Display.drawFastVLine(x, 0, s_scope.h, s_scope.bg);
    if ((x % 20) == 0) {
        M5.Display.drawFastVLine(x, 0, s_scope.h, s_scope.grid);
    }
    M5.Display.drawPixel(x, s_scope.mid, s_scope.grid);
    M5.Display.drawPixel(x, s_scope.clip_top, s_scope.grid);
    M5.Display.drawPixel(x, s_scope.clip_bottom, s_scope.grid);
    M5.Display.drawFastVLine(x, y1, (y2 - y1) + 1, clipped ? s_scope.clip : s_scope.wave);
    if (clipped) {
        M5.Display.drawPixel(x, 0, s_scope.clip);
        M5.Display.drawPixel(x, s_scope.h - 1, s_scope.clip);
    }
    display_unlock();

    s_scope.x = (x + kScopeSweepStep) % s_scope.w;
    if (s_scope.x == 0) {
        display_lock();
        scope_clear_and_grid();
        display_unlock();
    }
}

static void scope_plot_chunk_u8_linear(const uint8_t *samples, size_t n)
{
    if (!s_scope.initialized) {
        scope_init_fullscreen_landscape();
    }
    if (!samples || n == 0) {
        return;
    }

    uint8_t vmin = 255;
    uint8_t vmax = 0;
    bool clipped = false;
    for (size_t i = 0; i < n; ++i) {
        const uint8_t s = samples[i];
        if (s < vmin) vmin = s;
        if (s > vmax) vmax = s;
        if (s == 0 || s == 255) {
            clipped = true;
        }
    }

    const int16_t vmax16 = static_cast<int16_t>((static_cast<int16_t>(vmax) - 128) << 8);
    const int16_t vmin16 = static_cast<int16_t>((static_cast<int16_t>(vmin) - 128) << 8);

    const int x = s_scope.x;
    int y1 = sample_to_scope_y(vmax16);
    int y2 = sample_to_scope_y(vmin16);
    if (y1 > y2) {
        const int tmp = y1;
        y1 = y2;
        y2 = tmp;
    }

    display_lock();
    M5.Display.drawFastVLine(x, 0, s_scope.h, s_scope.bg);
    if ((x % 20) == 0) {
        M5.Display.drawFastVLine(x, 0, s_scope.h, s_scope.grid);
    }
    M5.Display.drawPixel(x, s_scope.mid, s_scope.grid);
    M5.Display.drawPixel(x, s_scope.clip_top, s_scope.grid);
    M5.Display.drawPixel(x, s_scope.clip_bottom, s_scope.grid);
    M5.Display.drawFastVLine(x, y1, (y2 - y1) + 1, clipped ? s_scope.clip : s_scope.wave);
    if (clipped) {
        M5.Display.drawPixel(x, 0, s_scope.clip);
        M5.Display.drawPixel(x, s_scope.h - 1, s_scope.clip);
    }
    display_unlock();

    s_scope.x = (x + kScopeSweepStep) % s_scope.w;
    if (s_scope.x == 0) {
        display_lock();
        scope_clear_and_grid();
        display_unlock();
    }
}
#endif

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
    m_output_buffer = new OutputBuffer(120 * 16);
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

#if !PTT_LOCAL_PLAYBACK_TEST_MODE
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(true);

    Serial.print("My MAC Address is: ");
    Serial.println(WiFi.macAddress());

    m_transport->begin();
#endif

    M5.Speaker.begin();
    M5.Speaker.setVolume(m_speaker_volume);
#if !PTT_LOCAL_PLAYBACK_TEST_MODE
    M5.Speaker.tone(1200, 80);
#endif

    TaskHandle_t task_handle;
#if defined(CONFIG_FREERTOS_UNICORE) && CONFIG_FREERTOS_UNICORE
    xTaskCreate(application_task, "application_task", 8192, this, 1, &task_handle);
#else
    constexpr BaseType_t kApplicationTaskCore = 1;
    xTaskCreatePinnedToCore(application_task, "application_task", 8192, this, 1, &task_handle, kApplicationTaskCore);
#endif
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
#if PTT_LOCAL_PLAYBACK_TEST_MODE
    constexpr size_t kRecordChunkSamples = 128;
    constexpr uint32_t kMaxRecordMs = 5000;
    constexpr size_t kMaxRecordSamples = (SAMPLE_RATE * kMaxRecordMs) / 1000;
    int16_t *mic_chunk_samples = reinterpret_cast<int16_t *>(malloc(sizeof(int16_t) * kRecordChunkSamples));
    int16_t *record_samples_i16 = reinterpret_cast<int16_t *>(malloc(sizeof(int16_t) * kMaxRecordSamples));
    uint8_t *record_samples_u8 = reinterpret_cast<uint8_t *>(malloc(kMaxRecordSamples));
    bool mic_active = false;
    bool mic_primed = false;
    bool spk_active = true;
    bool idle_status_drawn = false;
    int16_t play_samples_mono[kRecordChunkSamples];
    uint32_t last_test_u8_log_ms = 0;
    uint8_t test_u8_min = 255;
    uint8_t test_u8_max = 0;

    if (!mic_chunk_samples || !record_samples_i16 || !record_samples_u8) {
        Serial.println("PTT test mode: failed to allocate record buffers");
        if (mic_chunk_samples) free(mic_chunk_samples);
        if (record_samples_i16) free(record_samples_i16);
        if (record_samples_u8) free(record_samples_u8);
        vTaskDelete(nullptr);
    }

    while (true) {
        if (!M5.BtnA.isPressed()) {
            if (!idle_status_drawn && s_scope.initialized) {
                idle_status_drawn = true;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        idle_status_drawn = false;
        if (spk_active) {
            M5.Speaker.stop();
            M5.Speaker.end();
            spk_active = false;
        }
        if (!mic_active) {
            M5.Mic.begin();
            mic_active = true;
        }

        scope_init_fullscreen_landscape();
        begin_tx_session();
        uint32_t start_ms = millis();
        size_t recorded_samples = 0;
        while (recorded_samples < kMaxRecordSamples) {
            if (!M5.BtnA.isPressed()) {
                break;
            }
            if (millis() - start_ms >= kMaxRecordMs) {
                break;
            }

            size_t chunk_samples = kRecordChunkSamples;
            const size_t remain = kMaxRecordSamples - recorded_samples;
            if (chunk_samples > remain) {
                chunk_samples = remain;
            }
            bool ok = M5.Mic.record(mic_chunk_samples, chunk_samples, SAMPLE_RATE, false);
            if (!ok) {
                memset(mic_chunk_samples, 0, chunk_samples * sizeof(int16_t));
            }
#if PTT_TEST_AUDIO_PATH == PTT_TEST_AUDIO_PATH_16BIT
            memcpy(record_samples_i16 + recorded_samples, mic_chunk_samples, chunk_samples * sizeof(int16_t));
#elif PTT_TEST_AUDIO_PATH == PTT_TEST_AUDIO_PATH_8BIT_MULAW
            for (size_t i = 0; i < chunk_samples; ++i) {
                record_samples_u8[recorded_samples + i] = linear16_to_mulaw(mic_chunk_samples[i]);
            }
#else
            {
                // Match wireless TX path: int16 mic -> 8bit transport (no additional processing).
                convert_i16_to_u8_tx_compatible(
                    mic_chunk_samples,
                    record_samples_u8 + recorded_samples,
                    chunk_samples);
            }
#endif
            recorded_samples += chunk_samples;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (mic_active) {
            M5.Mic.end();
            mic_active = false;
        }
        if (!spk_active) {
            M5.Speaker.begin();
            M5.Speaker.setVolume(m_speaker_volume);
            spk_active = true;
        }

        size_t played_samples = 0;
        while (played_samples < recorded_samples) {
            size_t chunk_samples = kRecordChunkSamples;
            const size_t remain = recorded_samples - played_samples;
            if (chunk_samples > remain) {
                chunk_samples = remain;
            }
#if PTT_TEST_AUDIO_PATH == PTT_TEST_AUDIO_PATH_16BIT
            while (M5.Speaker.isPlaying()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            M5.Speaker.playRaw(record_samples_i16 + played_samples, chunk_samples, SAMPLE_RATE, false, 1, -1, true);
            scope_plot_chunk_i16(record_samples_i16 + played_samples, chunk_samples);
#elif PTT_TEST_AUDIO_PATH == PTT_TEST_AUDIO_PATH_8BIT_LINEAR
            for (size_t i = 0; i < chunk_samples; ++i) {
                const uint8_t v = record_samples_u8[played_samples + i];
                if (v < test_u8_min) test_u8_min = v;
                if (v > test_u8_max) test_u8_max = v;
            }
            uint32_t now_ms = millis();
            if (now_ms - last_test_u8_log_ms >= 1000) {
                Serial.printf("TEST u8 range: min=%u max=%u\n",
                              static_cast<unsigned>(test_u8_min),
                              static_cast<unsigned>(test_u8_max));
                test_u8_min = 255;
                test_u8_max = 0;
                last_test_u8_log_ms = now_ms;
            }
            while (M5.Speaker.isPlaying()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            M5.Speaker.playRaw(record_samples_u8 + played_samples, chunk_samples, SAMPLE_RATE, false, 1, -1, true);
            scope_plot_chunk_u8_linear(record_samples_u8 + played_samples, chunk_samples);
#else
            for (size_t i = 0; i < chunk_samples; ++i) {
                int16_t s = 0;
#if PTT_TEST_AUDIO_PATH == PTT_TEST_AUDIO_PATH_8BIT_MULAW
                s = mulaw_to_linear16(record_samples_u8[played_samples + i]);
#else
                s = static_cast<int16_t>((static_cast<int16_t>(record_samples_u8[played_samples + i]) - 128) << 8);
#endif
                play_samples_mono[i] = s;
            }
            while (M5.Speaker.isPlaying()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            M5.Speaker.playRaw(play_samples_mono, chunk_samples, SAMPLE_RATE, false, 1, -1, true);
            scope_plot_chunk_i16(play_samples_mono, chunk_samples);
#endif
            played_samples += chunk_samples;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        while (M5.Speaker.isPlaying()) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Prevent immediate re-trigger while the button remains held after timeout.
        while (M5.BtnA.isPressed()) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
#else
    constexpr size_t mic_chunk_samples = 128;
    constexpr size_t play_chunk_bytes = kRxPlayChunkBytes;
    constexpr size_t kRxPrefillChunks = 3;
    constexpr bool enable_tx_overlay = true;
    constexpr bool enable_rx_overlay = true;
    constexpr size_t rx_buffered_samples = SAMPLE_RATE * RX_RAM_BUFFERED_SECONDS;
    constexpr TickType_t synth_chunk_delay_ticks =
        pdMS_TO_TICKS((mic_chunk_samples * 1000 + SAMPLE_RATE - 1) / SAMPLE_RATE);
    int16_t *mic_samples = reinterpret_cast<int16_t *>(malloc(sizeof(int16_t) * mic_chunk_samples));
    uint8_t *mic_samples_u8 = reinterpret_cast<uint8_t *>(malloc(mic_chunk_samples));
    uint8_t *rx_buffered_samples_u8 = reinterpret_cast<uint8_t *>(malloc(rx_buffered_samples));
    uint8_t *rx_play_buffers[3] = { nullptr, nullptr, nullptr };
    bool mic_active = false;
    bool mic_primed = false;
    bool spk_active = true;
    uint32_t last_rssi_draw_ms = 0;
    uint32_t last_rx_level_log_ms = millis();
    uint8_t rx_level_min = 255;
    uint8_t rx_level_max = 0;
    const uint32_t ptt_enable_after_ms = millis() + 1000;
    float tone_phase = 0.0f;
    constexpr float tone_freq_hz = 1000.0f;
    constexpr float two_pi = 6.28318530718f;
    const float tone_phase_step = two_pi * tone_freq_hz / static_cast<float>(SAMPLE_RATE);
    constexpr int16_t tone_amplitude = 12000;

    rx_play_buffers[0] = reinterpret_cast<uint8_t *>(malloc(kRxPlayChunkBytes));
    rx_play_buffers[1] = reinterpret_cast<uint8_t *>(malloc(kRxPlayChunkBytes));
    rx_play_buffers[2] = reinterpret_cast<uint8_t *>(malloc(kRxPlayChunkBytes));
    size_t rx_play_buf_index = 0;
    bool rx_play_pending = false;
    uint8_t *rx_play_pending_ptr = nullptr;

    if (!mic_samples || !mic_samples_u8 || !rx_buffered_samples_u8 ||
        !rx_play_buffers[0] || !rx_play_buffers[1] || !rx_play_buffers[2]) {
        Serial.println("Failed to allocate audio buffers");
        vTaskDelete(nullptr);
    }
    while (true) {
        bool ptt = (millis() > ptt_enable_after_ms) && M5.BtnA.isPressed();
        if (ptt) {
            begin_tx_session();
            if (enable_tx_overlay) {
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
                if (!mic_primed) {
                    // First mic start after boot can include transient noise.
                    // Prime input by discarding a couple of chunks once.
                    for (int i = 0; i < 2; ++i) {
                        if (!M5.Mic.record(mic_samples, mic_chunk_samples, SAMPLE_RATE, true)) {
                            break;
                        }
                    }
                    mic_primed = true;
                }
            }
#endif

            unsigned long start_time = millis();
            while (millis() - start_time < 1000 || M5.BtnA.isPressed()) {
                if (enable_tx_overlay) {
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
                    const uint8_t tx_pitch_mode = m_tx_pitch_mode;
                    apply_tx_pitch_mode_u8_block(tx_pitch_mode, mic_samples_u8, send_samples);
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

        if (enable_rx_overlay) {
            dispStatus(false);
        }

#if RX_RAM_BUFFERED_PLAYBACK_MODE
        size_t captured = 0;
        while (captured < rx_buffered_samples && !M5.BtnA.isPressed()) {
            const size_t n = (rx_buffered_samples - captured > play_chunk_bytes)
                ? play_chunk_bytes
                : (rx_buffered_samples - captured);
            m_output_buffer->remove_samples(rx_buffered_samples_u8 + captured, static_cast<int>(n));
            for (size_t i = 0; i < n; ++i) {
                const uint8_t v = rx_buffered_samples_u8[captured + i];
                if (v < rx_level_min) rx_level_min = v;
                if (v > rx_level_max) rx_level_max = v;
            }
            uint32_t now_ms = millis();
            if (now_ms - last_rx_level_log_ms >= 1000) {
                Serial.printf("RX u8 range: min=%u max=%u\n",
                              static_cast<unsigned>(rx_level_min),
                              static_cast<unsigned>(rx_level_max));
                rx_level_min = 255;
                rx_level_max = 0;
                last_rx_level_log_ms = now_ms;
            }
            captured += n;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!M5.BtnA.isPressed() && captured > 0) {
            if (!spk_active) {
                M5.Speaker.begin();
                M5.Speaker.setVolume(m_speaker_volume);
                spk_active = true;
            }
            size_t ofs = 0;
            while (ofs < captured && !M5.BtnA.isPressed()) {
                const size_t n = (captured - ofs > play_chunk_bytes)
                    ? play_chunk_bytes
                    : (captured - ofs);
                while (M5.Speaker.isPlaying()) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                M5.Speaker.playRaw(rx_buffered_samples_u8 + ofs, n, SAMPLE_RATE, false, 1, -1, true);
                ofs += n;
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            while (M5.Speaker.isPlaying()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
#else
        while (!M5.BtnA.isPressed()) {
            if (enable_rx_overlay) {
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
                rx_play_pending = false;
                rx_play_pending_ptr = nullptr;
            }
            size_t queued_now = 0;
            while (queued_now < kRxPrefillChunks) {
                if (!rx_play_pending) {
                    uint8_t *chunk_ptr = rx_play_buffers[rx_play_buf_index];
                    m_output_buffer->remove_samples(chunk_ptr, static_cast<int>(kRxPlayChunkBytes));
                    for (size_t i = 0; i < kRxPlayChunkBytes; ++i) {
                        const uint8_t v = chunk_ptr[i];
                        if (v < rx_level_min) rx_level_min = v;
                        if (v > rx_level_max) rx_level_max = v;
                    }
                    uint32_t now_ms = millis();
                    if (now_ms - last_rx_level_log_ms >= 1000) {
                        Serial.printf("RX u8 range: min=%u max=%u\n",
                                      static_cast<unsigned>(rx_level_min),
                                      static_cast<unsigned>(rx_level_max));
                        rx_level_min = 255;
                        rx_level_max = 0;
                        last_rx_level_log_ms = now_ms;
                    }
                    rx_play_pending_ptr = chunk_ptr;
                    rx_play_pending = true;
                }

                const bool queued = M5.Speaker.playRaw(
                    rx_play_pending_ptr, kRxPlayChunkBytes, SAMPLE_RATE, false, 1, 0, false);
                if (!queued) {
                    break;
                }
                rx_play_pending = false;
                rx_play_pending_ptr = nullptr;
                rx_play_buf_index = (rx_play_buf_index + 1) % 3;
                ++queued_now;
            }

            if (queued_now == 0) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
#endif
    }
#endif
}
