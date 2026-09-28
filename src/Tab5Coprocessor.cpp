#include "Tab5Coprocessor.h"

#include "config.h"

#if TALKIE_TARGET_M5TAB5

#include <Arduino.h>
#include <Preferences.h>
#include <esp32-hal-hosted.h>

#include "Tab5Ui.h"
#include "esp_now_hosted.h"

// ESPHome esp-hosted-firmware v2.12.13 (ESP-Hosted slave + ESP-NOW overlay),
// embedded via board_build.embed_files.
extern const uint8_t c6_fw_start[] asm("_binary_assets_c6_network_adapter_esp32c6_bin_start");
extern const uint8_t c6_fw_end[] asm("_binary_assets_c6_network_adapter_esp32c6_bin_end");

namespace {

bool flash_embedded_firmware()
{
    const size_t total = static_cast<size_t>(c6_fw_end - c6_fw_start);
    Serial.printf("C6: writing embedded firmware (%u bytes)\n", static_cast<unsigned>(total));
    if (!hostedBeginUpdate()) {
        Serial.println("C6: hostedBeginUpdate failed");
        return false;
    }
    constexpr size_t kChunk = 1400;
    static uint8_t buf[kChunk];
    size_t done = 0;
    int last_pct = -1;
    while (done < total) {
        const size_t n = (total - done) < kChunk ? (total - done) : kChunk;
        memcpy(buf, c6_fw_start + done, n);  // API takes a non-const buffer
        if (!hostedWriteUpdate(buf, n)) {
            Serial.printf("C6: write failed at %u\n", static_cast<unsigned>(done));
            return false;
        }
        done += n;
        const int pct = static_cast<int>(done * 100 / total);
        if (pct / 10 != last_pct / 10) {
            last_pct = pct;
            char msg[48];
            snprintf(msg, sizeof(msg), "Updating C6 firmware %d%%", pct);
            tab5_ui_message(msg);
        }
    }
    if (!hostedEndUpdate()) {
        Serial.println("C6: hostedEndUpdate failed");
        return false;
    }
    if (!hostedActivateUpdate()) {
        Serial.println("C6: hostedActivateUpdate failed");
        return false;
    }
    return true;
}

}  // namespace

bool tab5_coprocessor_ensure_espnow()
{
    uint32_t hmaj = 0, hmin = 0, hpat = 0, smaj = 0, smin = 0, spat = 0;
    hostedGetHostVersion(&hmaj, &hmin, &hpat);
    hostedGetSlaveVersion(&smaj, &smin, &spat);
    Serial.printf("esp-hosted host %lu.%lu.%lu, C6 %lu.%lu.%lu\n",
                  (unsigned long)hmaj, (unsigned long)hmin, (unsigned long)hpat,
                  (unsigned long)smaj, (unsigned long)smin, (unsigned long)spat);

    // Factory Tab5 C6 firmware (ESP-Hosted 1.x) has no OTA partitions and is
    // not protocol-compatible with the 2.x host: it must be flashed over UART
    // once (assets/c6/wired).
    if (smaj < 2) {
        char msg[64];
        snprintf(msg, sizeof(msg), "C6 fw %lu.%lu.%lu too old: wired flash needed",
                 (unsigned long)smaj, (unsigned long)smin, (unsigned long)spat);
        tab5_ui_message(msg);
        return false;
    }

    Preferences prefs;
    prefs.begin("tab5c6", false);

    if (esp_now_hosted_available(1500)) {
        Serial.println("C6: ESP-NOW overlay present");
        prefs.putUChar("tries", 0);
        prefs.end();
        return true;
    }

    // Avoid an update/reboot loop if the new image still does not answer.
    const uint8_t tries = prefs.getUChar("tries", 0);
    if (tries >= 2) {
        Serial.println("C6: ESP-NOW overlay missing after update; giving up");
        tab5_ui_message("C6 has no ESP-NOW support");
        prefs.end();
        return false;
    }
    prefs.putUChar("tries", tries + 1);
    prefs.end();

    tab5_ui_message("Updating C6 firmware (ESP-NOW)...");
    if (!flash_embedded_firmware()) {
        tab5_ui_message("C6 firmware update failed");
        return false;
    }
    tab5_ui_message("C6 updated. Restarting...");
    delay(1000);
    ESP.restart();
    return false;  // not reached
}

#else

bool tab5_coprocessor_ensure_espnow() { return true; }

#endif
