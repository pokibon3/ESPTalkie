#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>

// Build target selection (set from platformio.ini build_flags)
#if defined(TARGET_M5ATOMS3_ECHO_BASE)
#define TALKIE_TARGET_M5ATOMS3_ECHO_BASE 1
#else
#define TALKIE_TARGET_M5ATOMS3_ECHO_BASE 0
#endif

#if defined(TARGET_M5STICKS3)
#define TALKIE_TARGET_M5STICKS3 1
#else
#define TALKIE_TARGET_M5STICKS3 0
#endif

#if defined(TARGET_M5PAPERCOLOR)
#define TALKIE_TARGET_M5PAPERCOLOR 1
#else
#define TALKIE_TARGET_M5PAPERCOLOR 0
#endif

#if defined(TARGET_M5STOPWATCH)
#define TALKIE_TARGET_M5STOPWATCH 1
#else
#define TALKIE_TARGET_M5STOPWATCH 0
#endif

#if defined(TARGET_M5TAB5)
#define TALKIE_TARGET_M5TAB5 1
#else
#define TALKIE_TARGET_M5TAB5 0
#endif

// WiFi credentials
//#define WIFI_SSID 
//#define WIFI_PSWD 
#define USE_ESP_NOW
// sample rate for the system
#define SAMPLE_RATE 16000

// Microphone gain
#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
#define MIC_MAGNIFICATION 28
#elif TALKIE_TARGET_M5PAPERCOLOR
#define MIC_MAGNIFICATION 20
#elif TALKIE_TARGET_M5STICKS3
#define MIC_MAGNIFICATION 20
#elif TALKIE_TARGET_M5TAB5
#define MIC_MAGNIFICATION 40
#elif TALKIE_TARGET_M5STOPWATCH
#define MIC_MAGNIFICATION 20
#else
#define MIC_MAGNIFICATION 20
#endif
// ESP-NOW Long Range mode
#define ESPNOW_LONG_RANGE
// ESP-NOW payload magic header text for packet filtering
#define ESPNOW_PACKET_MAGIC_TEXT  "ESPT1"

// Which channel is the I2S microphone on? I2S_CHANNEL_FMT_ONLY_LEFT or I2S_CHANNEL_FMT_ONLY_RIGHT
// Generally they will default to LEFT - but you may need to attach the L/R pin to GND
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ALL_RIGHT
#define I2S_MIC_SERIAL_CLOCK    6
#define I2S_MIC_SERIAL_DATA     5

// speaker settings
#define PWM_SPEAKER_PIN         D0
#define PWM_SPEAKER_ENABLE_PIN  -1
#define PWM_SPEAKER_LEDC_CHANNEL 0

// transmit button
#define GPIO_TRANSMIT_BUTTON    D9         

// On which wifi channel (1-11) should ESP-Now transmit? The default ESP-Now channel on ESP32 is channel 1
#define ESP_NOW_WIFI_CHANNEL    1

// Audio diagnostic source selector
#define AUDIO_DIAG_SRC_MIC      0
#define AUDIO_DIAG_SRC_SILENCE  1
#define AUDIO_DIAG_SRC_TONE     2
#define AUDIO_DIAG_SOURCE       AUDIO_DIAG_SRC_MIC

// Transmit pitch effect mode
#define TX_PITCH_MODE_NONE               0
#define TX_PITCH_MODE_OCTAVE_UP_SIMPLE   1
#define TX_PITCH_MODE_TRIPLE_SPEED_SIMPLE 2
#define TX_PITCH_MODE_QUAD_SPEED_SIMPLE  3
#define TX_PITCH_MODE                    TX_PITCH_MODE_TRIPLE_SPEED_SIMPLE

// M5Unified external speaker selector
#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
#define M5UNIFIED_USE_ATOMIC_ECHO_BASE 1
#else
#define M5UNIFIED_USE_ATOMIC_ECHO_BASE 0
#endif

// Mic WAV dump (diagnostic)
#define MIC_WAV_DUMP_TO_SPIFFS  0
#define MIC_WAV_DUMP_SECONDS    10

// PTT local record/playback test mode:
// BtnA press starts recording (max 5s), and playback starts when BtnA is released
// or when 5s elapses. Uses current speaker volume setting.
#define PTT_LOCAL_PLAYBACK_TEST_MODE 0

// RX diagnostic mode:
// Buffer received 8-bit PCM in RAM for a fixed window, then play back as a block.
#define RX_RAM_BUFFERED_PLAYBACK_MODE 0
#define RX_RAM_BUFFERED_SECONDS       5

// RX playback chunk size (samples). Larger value reduces task wakeups but adds latency.
#define RX_PLAY_CHUNK_SAMPLES 320

// Test mode audio path selector
#define PTT_TEST_AUDIO_PATH_16BIT        0
#define PTT_TEST_AUDIO_PATH_8BIT_LINEAR  1
#define PTT_TEST_AUDIO_PATH_8BIT_MULAW   2
#define PTT_TEST_AUDIO_PATH              PTT_TEST_AUDIO_PATH_8BIT_LINEAR

// 8-bit linear quantization compressor switch (used by conversion function)
#define TX_8BIT_COMPRESSOR_ENABLE 0

// Horizontal shake to change current setting (same effect as BtnB click)
#if TALKIE_TARGET_M5TAB5 || TALKIE_TARGET_M5STOPWATCH
#define SHAKE_SWITCH_ENABLED     0
#else
#define SHAKE_SWITCH_ENABLED     1
#endif
#define SHAKE_SENSITIVITY_LOW    1
#define SHAKE_SENSITIVITY_MID    2
#define SHAKE_SENSITIVITY_HIGH   3
#define SHAKE_SENSITIVITY_LEVEL  SHAKE_SENSITIVITY_MID

#if SHAKE_SENSITIVITY_LEVEL == SHAKE_SENSITIVITY_LOW
#define SHAKE_X_THRESHOLD_G      1.60f
#define SHAKE_X_DOMINANCE_G      0.38f
#define SHAKE_REARM_G            0.90f
#elif SHAKE_SENSITIVITY_LEVEL == SHAKE_SENSITIVITY_HIGH
#define SHAKE_X_THRESHOLD_G      1.10f
#define SHAKE_X_DOMINANCE_G      0.20f
#define SHAKE_REARM_G            0.60f
#else
#define SHAKE_X_THRESHOLD_G      1.35f
#define SHAKE_X_DOMINANCE_G      0.30f
#define SHAKE_REARM_G            0.75f
#endif
#define SHAKE_Y_THRESHOLD_G      SHAKE_X_THRESHOLD_G
#define SHAKE_Y_DOMINANCE_G      SHAKE_X_DOMINANCE_G
#define SHAKE_Z_THRESHOLD_G      SHAKE_X_THRESHOLD_G
#define SHAKE_Z_DOMINANCE_G      SHAKE_X_DOMINANCE_G
#define SHAKE_COOLDOWN_MS        450

// M5Stack Tab5: background image on the SD card (JPG/PNG, fitted to 720x1280)
#define TAB5_IMAGE_PATH            "/images/pokibon.jpeg"

// M5Stack StopWatch
// Yellow button (BtnA) must be held this long to open / close SETUP.
#define STOPWATCH_SETUP_HOLD_MS        1000
// SETUP closes by itself after this much inactivity.
#define STOPWATCH_SETUP_TIMEOUT_MS     15000
// Vibration on incoming call: pulses x (on + off) ms, level 0-255.
#define STOPWATCH_VIBRATION_PULSES     3
#define STOPWATCH_VIBRATION_ON_MS      90
#define STOPWATCH_VIBRATION_OFF_MS     80
#define STOPWATCH_VIBRATION_LEVEL      200
// Clock: hands over the photo are translucent (alpha 0-255); a yellow click
// shows them opaque for STOPWATCH_HANDS_CLEAR_MS.
#define STOPWATCH_HANDS_ALPHA          80
#define STOPWATCH_HANDS_CLEAR_MS       5000
#define STOPWATCH_SHOW_SECONDS         1
// Time zone (POSIX TZ) and NTP. Wi-Fi credentials: src/wifi_secrets.h
#define STOPWATCH_TZ                   "JST-9"
#define STOPWATCH_NTP_SERVER1          "ntp.nict.jp"
#define STOPWATCH_NTP_SERVER2          "pool.ntp.org"
#define STOPWATCH_NTP_ON_BOOT          1
// Boot-time sync is skipped while the RTC is valid and the last NTP sync is
// newer than this (seconds).
#define STOPWATCH_NTP_INTERVAL_S       86400
#define STOPWATCH_WIFI_TIMEOUT_MS      10000
#define STOPWATCH_NTP_TIMEOUT_MS       5000
// SETUP > WIFI: phone setup page closes after this long
#define STOPWATCH_WIFI_SETUP_TIMEOUT_MS 300000
