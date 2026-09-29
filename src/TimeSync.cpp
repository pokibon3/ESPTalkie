#include "TimeSync.h"

#include "config.h"

#if TALKIE_TARGET_M5STOPWATCH

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <sys/time.h>
#include <time.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

namespace {

constexpr int kValidYear = 2024;
// RX8130CE (StopWatch RTC)
constexpr uint8_t kRtcAddr = 0x32;
constexpr uint8_t kRtcRegFlag = 0x1D;
constexpr uint8_t kRtcFlagVLF = 0x02;  // oscillation stopped / supply lost: time invalid
constexpr uint32_t kRtcFreq = 400000;

bool s_rtc_valid = false;

// Firmware build time (UTC is close enough): the RTC can't be older than this.
time_t build_epoch()
{
    static const char *const kMonths = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *d = __DATE__;  // "Sep 29 2026"
    tm t = {};
    const char *m = strstr(kMonths, String(d).substring(0, 3).c_str());
    t.tm_mon = m ? static_cast<int>((m - kMonths) / 3) : 0;
    t.tm_mday = atoi(d + 4);
    t.tm_year = atoi(d + 7) - 1900;
    // mktime() uses the local zone; the result is only a lower bound anyway.
    return mktime(&t);
}

// RTC holds a trustworthy time: VLF clear, readable date, not before the build.
bool rtc_time_valid()
{
    if (!M5.Rtc.isEnabled()) return false;
    uint8_t flag = 0;
    if (!M5.In_I2C.readRegister(kRtcAddr, kRtcRegFlag, &flag, 1, kRtcFreq)) {
        Serial.println("Clock: RTC flag read failed");
        return false;
    }
    if (flag & kRtcFlagVLF) {
        Serial.println("Clock: RTC VLF set (time lost)");
        return false;
    }
    m5::rtc_datetime_t dt;
    if (!M5.Rtc.getDateTime(&dt)) {
        Serial.println("Clock: RTC date unreadable");
        return false;
    }
    tm t = {};
    t.tm_year = dt.date.year - 1900;
    t.tm_mon = dt.date.month - 1;
    t.tm_mday = dt.date.date;
    t.tm_hour = dt.time.hours;
    t.tm_min = dt.time.minutes;
    t.tm_sec = dt.time.seconds;
    const time_t rtc = mktime(&t);
    if (rtc < build_epoch() - 86400) {
        Serial.printf("Clock: RTC date %04d-%02d-%02d is before the firmware build\n",
                      dt.date.year, dt.date.month, dt.date.date);
        return false;
    }
    return true;
}

void rtc_clear_vlf()
{
    // Flags are write-0-to-clear: write 1 everywhere except VLF.
    M5.In_I2C.writeRegister8(kRtcAddr, kRtcRegFlag, static_cast<uint8_t>(~kRtcFlagVLF), kRtcFreq);
}

}  // namespace

void time_sync_init()
{
    setenv("TZ", STOPWATCH_TZ, 1);
    tzset();
    s_rtc_valid = rtc_time_valid();
    if (s_rtc_valid) {
        M5.Rtc.setSystemTimeFromRtc();  // RTC holds UTC
        setenv("TZ", STOPWATCH_TZ, 1);
        tzset();
    }
    Serial.printf("Clock: %s\n", s_rtc_valid ? "RTC time loaded" : "RTC time invalid");
}

bool time_sync_needed()
{
    if (!s_rtc_valid) return true;
    Preferences p;
    uint32_t last = 0;
    if (p.begin("esptalkie", true)) {
        last = p.getUInt("ntp_last", 0);
        p.end();
    }
    const time_t now = time(nullptr);
    const bool due = last == 0 || now < static_cast<time_t>(last) ||
                     now - static_cast<time_t>(last) >= STOPWATCH_NTP_INTERVAL_S;
    Serial.printf("Clock: last NTP sync %lu s ago -> %s\n",
                  last ? static_cast<unsigned long>(now - last) : 0UL, due ? "sync" : "skip");
    return due;
}

bool wifi_credentials_get(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    Preferences p;
    bool ok = false;
    if (p.begin("esptalkie", true)) {
        const String s = p.getString("wifi_ssid", "");
        const String k = p.getString("wifi_pass", "");
        p.end();
        if (s.length() > 0) {
            snprintf(ssid, ssid_len, "%s", s.c_str());
            snprintf(pass, pass_len, "%s", k.c_str());
            ok = true;
        }
    }
#if defined(WIFI_SSID) && defined(WIFI_PSWD)
    if (!ok) {
        snprintf(ssid, ssid_len, "%s", WIFI_SSID);
        snprintf(pass, pass_len, "%s", WIFI_PSWD);
        ok = true;
    }
#endif
    return ok;
}

void wifi_credentials_save(const char *ssid, const char *pass)
{
    Preferences p;
    if (p.begin("esptalkie", false)) {
        p.putString("wifi_ssid", ssid);
        p.putString("wifi_pass", pass);
        p.end();
    }
}

bool time_sync_available()
{
    char ssid[33];
    char pass[65];
    return wifi_credentials_get(ssid, sizeof(ssid), pass, sizeof(pass));
}

bool time_is_valid()
{
    const time_t now = time(nullptr);
    tm t;
    localtime_r(&now, &t);
    return (t.tm_year + 1900) >= kValidYear;
}

bool time_sync_ntp(bool radio_running, int restore_channel)
{
    char ssid[33];
    char pass[65];
    if (!wifi_credentials_get(ssid, sizeof(ssid), pass, sizeof(pass))) {
        Serial.println("NTP: no Wi-Fi credentials (SETUP > WIFI)");
        return false;
    }
    Serial.printf("NTP: connecting to %s\n", ssid);
    if (radio_running) {
        esp_wifi_set_promiscuous(false);
    }
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    WiFi.begin(ssid, pass);
    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < STOPWATCH_WIFI_TIMEOUT_MS) {
        delay(100);
    }
    bool ok = false;
    if (WiFi.status() == WL_CONNECTED) {
        configTzTime(STOPWATCH_TZ, STOPWATCH_NTP_SERVER1, STOPWATCH_NTP_SERVER2);
        tm t;
        const uint32_t ntp_start = millis();
        while (millis() - ntp_start < STOPWATCH_NTP_TIMEOUT_MS) {
            if (getLocalTime(&t, 200) && (t.tm_year + 1900) >= kValidYear) {
                ok = true;
                break;
            }
        }
        if (ok) {
            const time_t now = time(nullptr);
            if (M5.Rtc.isEnabled()) {
                tm utc;
                gmtime_r(&now, &utc);
                M5.Rtc.setDateTime(&utc);
                rtc_clear_vlf();
                s_rtc_valid = true;
            }
            Preferences p;
            if (p.begin("esptalkie", false)) {
                p.putUInt("ntp_last", static_cast<uint32_t>(now));
                p.end();
            }
        }
        Serial.printf("NTP: %s\n", ok ? "time set" : "no reply");
    } else {
        Serial.println("NTP: Wi-Fi connect failed");
    }
    WiFi.disconnect(false, false);
    delay(50);

    if (radio_running) {
#ifdef ESPNOW_LONG_RANGE
        esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#endif
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_channel(static_cast<uint8_t>(restore_channel), WIFI_SECOND_CHAN_NONE);
    }
    return ok;
}

#else  // !TALKIE_TARGET_M5STOPWATCH

void time_sync_init() {}
bool time_sync_needed() { return false; }
bool time_sync_available() { return false; }
bool wifi_credentials_get(char *, size_t, char *, size_t) { return false; }
void wifi_credentials_save(const char *, const char *) {}
bool time_is_valid() { return false; }
bool time_sync_ntp(bool, int) { return false; }

#endif
