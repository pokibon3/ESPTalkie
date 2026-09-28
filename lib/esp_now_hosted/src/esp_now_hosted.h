#pragma once

#include <sdkconfig.h>
#include <stdint.h>

#if defined(CONFIG_ESP_HOSTED_ENABLED) && !defined(CONFIG_SOC_WIFI_SUPPORTED)
#define ESP_NOW_HOSTED_SHIM 1
// True when the co-processor answers ESP-NOW-over-CustomRpc requests
// (i.e. it runs the esp-hosted firmware with the ESP-NOW overlay).
// esp-hosted Wi-Fi must already be started (WiFi.mode(WIFI_STA)).
bool esp_now_hosted_available(uint32_t timeout_ms = 1000);
#else
#define ESP_NOW_HOSTED_SHIM 0
#endif
