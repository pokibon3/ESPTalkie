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

    if (!buf || n == 0 || n > sizeof(prev)) {
        return;
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

static void dump_mic_wav_to_spiffs_10s()
{
    if (!SPIFFS.begin(true)) {
        return;
    }

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
    int32_t hpf_prev_x = 0;
    int32_t hpf_prev_y = 0;
    uint32_t data_bytes = 0;
    const uint32_t start_ms = millis();
    const uint32_t record_ms = static_cast<uint32_t>(MIC_WAV_DUMP_SECONDS) * 1000U;

    M5.Mic.begin();
    while (millis() - start_ms < record_ms) {
        if (M5.Mic.record(buf, kChunkSamples, SAMPLE_RATE, false)) {
            for (size_t i = 0; i < kChunkSamples; ++i) {
                int32_t x = buf[i];
                int32_t y = x - hpf_prev_x + ((hpf_prev_y * 31) >> 5);
                hpf_prev_x = x;
                hpf_prev_y = y;
                if (y > 32767) y = 32767;
                if (y < -32768) y = -32768;
                buf[i] = static_cast<int16_t>(y);
            }
            size_t n = f.write(reinterpret_cast<const uint8_t *>(buf), sizeof(buf));
            data_bytes += static_cast<uint32_t>(n);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    M5.Mic.end();

    write_wav_header(f, SAMPLE_RATE, 16, 1, data_bytes);
    f.close();
}

}  // namespace

Application::Application() :
    m_transport(nullptr),
    m_output_buffer(nullptr),
    m_channel(ESP_NOW_WIFI_CHANNEL),
    m_speaker_volume(132)
{
    m_output_buffer = new OutputBuffer(300 * 16);
    m_transport = new EspNowTransport(m_output_buffer, static_cast<uint8_t>(m_channel));
}

void Application::begin()
{
    Serial.print("My IDF Version is: ");
    Serial.println(esp_get_idf_version());

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);

    Serial.print("My MAC Address is: ");
    Serial.println(WiFi.macAddress());

    m_transport->begin();

#if AUDIO_DIAG_SOURCE == AUDIO_DIAG_SRC_MIC
    auto mic_cfg = M5.Mic.config();
    mic_cfg.magnification = MIC_MAGNIFICATION;
    mic_cfg.over_sampling = 2;
    M5.Mic.config(mic_cfg);
#endif

#if MIC_WAV_DUMP_TO_SPIFFS
    dump_mic_wav_to_spiffs_10s();
#endif

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
    const int cx = M5.Display.width() / 2;
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

    while (true) {
        bool ptt = (millis() > ptt_enable_after_ms) && M5.BtnA.isPressed();
        if (ptt) {
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
                ready = M5.Mic.record(mic_samples_u8, mic_chunk_samples, SAMPLE_RATE, false);
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
#if TX_PITCH_MODE == TX_PITCH_MODE_OCTAVE_UP_SIMPLE
                    apply_octave_up_simple_u8_block(mic_samples_u8, send_samples);
#endif
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

        unsigned long start_time = millis();
        if (enable_rssi_overlay) {
            dispStatus(false);
        }
        while (millis() - start_time < 1000 || !M5.BtnA.isPressed()) {
            if (enable_rssi_overlay) {
                uint32_t now = millis();
                if (now - last_rssi_draw_ms >= 500) {  // lower UI refresh load
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
        }
    }
}
