#pragma once

// M5Stack Tab5: make sure the ESP32-C6 co-processor runs esp-hosted firmware
// with the ESP-NOW overlay. If it does not answer ESP-NOW requests, the
// embedded firmware (assets/c6) is written to it over esp-hosted OTA and the
// Tab5 restarts. Call after WiFi.mode(WIFI_STA). Returns true when ESP-NOW
// is usable. Always true on other targets.
bool tab5_coprocessor_ensure_espnow();
