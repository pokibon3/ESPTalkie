#pragma once

#include <stddef.h>

// Clock support for boards with an RTC (M5Stack StopWatch).
// The RTC holds UTC; the local time zone is STOPWATCH_TZ (config.h).
// NTP sync uses the Wi-Fi credentials saved in NVS (entered from a phone via
// SETUP > WIFI), or, if none are saved, the ones in src/wifi_secrets.h (not in
// git; see src/wifi_secrets.h.example). All functions are no-ops on other
// targets.

// Set the time zone and load the system clock from the RTC if the RTC holds
// a valid time (oscillator never stopped, sane date).
void time_sync_init();
// True if the RTC time is invalid or the last NTP sync is older than
// STOPWATCH_NTP_INTERVAL_S: sync at boot. Call after time_sync_init().
bool time_sync_needed();
// True if Wi-Fi credentials are available (NVS or compiled in).
bool time_sync_available();
// Wi-Fi credentials: NVS first, then src/wifi_secrets.h. False if none.
bool wifi_credentials_get(char *ssid, size_t ssid_len, char *pass, size_t pass_len);
void wifi_credentials_save(const char *ssid, const char *pass);
// True once the system clock holds a plausible date (RTC set or NTP done).
bool time_is_valid();
// Connect to the access point, get the time by NTP, write it to the RTC.
// radio_running: ESP-NOW is already up; its channel / LR mode are restored
// afterwards (restore_channel is the ESP-NOW channel).
bool time_sync_ntp(bool radio_running, int restore_channel);
