#include "WifiSetup.h"

#include "config.h"

#if TALKIE_TARGET_M5STOPWATCH

#include <Arduino.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <M5Unified.h>

#include "DisplaySync.h"
#include "TimeSync.h"

namespace {

constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
constexpr uint16_t kBg     = rgb(0, 0, 0);
constexpr uint16_t kText   = rgb(240, 245, 255);
constexpr uint16_t kSub    = rgb(150, 190, 240);
constexpr uint16_t kDim    = rgb(120, 130, 145);
constexpr uint16_t kTitle  = rgb(255, 200, 0);
constexpr uint16_t kOk     = rgb(0, 210, 90);

const IPAddress kApIp(192, 168, 4, 1);
constexpr int kQrSize = 220;

enum class Page : uint8_t { Join, Open, Saved, Cancelled };

String html_escape(const String &s)
{
    String o;
    for (size_t i = 0; i < s.length(); ++i) {
        const char c = s[i];
        switch (c) {
            case '&': o += "&amp;"; break;
            case '<': o += "&lt;"; break;
            case '>': o += "&gt;"; break;
            case '"': o += "&quot;"; break;
            default: o += c; break;
        }
    }
    return o;
}

// Escape for the WIFI: QR payload.
String qr_escape(const char *s)
{
    String o;
    for (; *s; ++s) {
        if (*s == '\\' || *s == ';' || *s == ',' || *s == ':' || *s == '"') o += '\\';
        o += *s;
    }
    return o;
}

void draw_page(Page page, const char *ap_ssid, const char *ap_pass, const char *saved_ssid)
{
    const int cx = M5.Display.width() / 2;
    const int cy = M5.Display.height() / 2;
    display_lock();
    M5.Display.startWrite();
    M5.Display.fillScreen(kBg);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.setTextColor(kTitle);
    M5.Display.drawString("Wi-Fi SETUP", cx, cy - 170);
    M5.Display.setFont(&fonts::Font2);

    char buf[160];
    switch (page) {
        case Page::Join:
            snprintf(buf, sizeof(buf), "WIFI:T:WPA;S:%s;P:%s;;",
                     qr_escape(ap_ssid).c_str(), qr_escape(ap_pass).c_str());
            M5.Display.fillRect(cx - kQrSize / 2 - 8, cy - kQrSize / 2 - 8, kQrSize + 16, kQrSize + 16, TFT_WHITE);
            M5.Display.qrcode(buf, cx - kQrSize / 2, cy - kQrSize / 2, kQrSize, 1);
            M5.Display.setTextColor(kText);
            M5.Display.drawString("1. Scan with your phone", cx, cy - 138);
            M5.Display.setTextColor(kSub);
            snprintf(buf, sizeof(buf), "AP: %s  PW: %s", ap_ssid, ap_pass);
            M5.Display.drawString(buf, cx, cy + 134);
            break;
        case Page::Open:
            M5.Display.fillRect(cx - kQrSize / 2 - 8, cy - kQrSize / 2 - 8, kQrSize + 16, kQrSize + 16, TFT_WHITE);
            M5.Display.qrcode("http://192.168.4.1/", cx - kQrSize / 2, cy - kQrSize / 2, kQrSize, 1);
            M5.Display.setTextColor(kText);
            M5.Display.drawString("2. Open the setup page", cx, cy - 138);
            M5.Display.setTextColor(kSub);
            M5.Display.drawString("http://192.168.4.1/", cx, cy + 134);
            break;
        case Page::Saved:
            M5.Display.setFont(&fonts::FreeSansBold18pt7b);
            M5.Display.setTextColor(kOk);
            M5.Display.drawString("SAVED", cx, cy - 20);
            M5.Display.setFont(&fonts::Font2);
            M5.Display.setTextColor(kText);
            M5.Display.drawString(saved_ssid ? saved_ssid : "", cx, cy + 20);
            break;
        case Page::Cancelled:
            M5.Display.setFont(&fonts::FreeSansBold18pt7b);
            M5.Display.setTextColor(kDim);
            M5.Display.drawString("CANCELLED", cx, cy);
            break;
    }
    if (page == Page::Join || page == Page::Open) {
        M5.Display.setTextColor(kDim);
        M5.Display.drawString("YELLOW: cancel", cx, cy + 160);
    }
    M5.Display.endWrite();
    display_unlock();
}

String build_form(const String &options)
{
    String h;
    h.reserve(2048 + options.length());
    h += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
           "<meta name='viewport' content='width=device-width,initial-scale=1'>"
           "<title>ESPTalkie Wi-Fi</title><style>"
           "body{font-family:-apple-system,sans-serif;margin:24px;background:#111;color:#eee}"
           "h1{font-size:20px;color:#ffc800}label{display:block;margin-top:16px;font-size:14px;color:#9bd}"
           "input,select{width:100%;box-sizing:border-box;font-size:18px;padding:10px;margin-top:6px;"
           "border-radius:8px;border:1px solid #555;background:#222;color:#eee}"
           "button{margin-top:24px;width:100%;font-size:18px;padding:12px;border:0;border-radius:8px;"
           "background:#1e78ff;color:#fff}</style></head><body>"
           "<h1>ESPTalkie StopWatch<br>Wi-Fi setup</h1>"
           "<form method='POST' action='/save'>"
           "<label>Network</label><select onchange=\"document.getElementById('s').value=this.value\">"
           "<option value=''>-- select --</option>");
    h += options;
    h += F("</select><label>SSID</label><input id='s' name='ssid' maxlength='32' required>"
           "<label>Password</label><input name='pass' type='password' maxlength='64'>"
           "<button type='submit'>Save</button></form>"
           "<p style='font-size:12px;color:#888'>Used only to set the clock (NTP).</p>"
           "</body></html>");
    return h;
}

}  // namespace

bool wifi_setup_run(int restore_channel)
{
    const int cx = M5.Display.width() / 2;
    const int cy = M5.Display.height() / 2;
    display_lock();
    M5.Display.fillScreen(kBg);
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(kText);
    M5.Display.drawString("Scanning...", cx, cy);
    display_unlock();

    // Leave ESP-NOW listening mode: normal 11b/g/n, no promiscuous.
    esp_wifi_set_promiscuous(false);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    // Scan nearby networks for the drop-down list.
    String options;
    const int n = WiFi.scanNetworks();
    for (int i = 0; i < n && i < 20; ++i) {
        const String s = WiFi.SSID(i);
        if (s.length() == 0 || options.indexOf("value=\"" + html_escape(s) + "\"") >= 0) continue;
        options += "<option value=\"" + html_escape(s) + "\">" + html_escape(s) +
                   " (" + String(WiFi.RSSI(i)) + ")</option>";
    }
    WiFi.scanDelete();

    // Own access point with a random password (shown in the QR code).
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char ap_ssid[24];
    char ap_pass[12];
    snprintf(ap_ssid, sizeof(ap_ssid), "ESPTalkie-%02X%02X", mac[4], mac[5]);
    snprintf(ap_pass, sizeof(ap_pass), "%08lu", static_cast<unsigned long>(esp_random() % 100000000UL));
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(kApIp, kApIp, IPAddress(255, 255, 255, 0));
    WiFi.softAP(ap_ssid, ap_pass, restore_channel);

    DNSServer dns;
    dns.start(53, "*", kApIp);  // captive portal: every name -> 192.168.4.1
    WebServer server(80);
    const String form = build_form(options);
    bool saved = false;
    String saved_ssid;
    server.on("/", HTTP_GET, [&]() { server.send(200, "text/html", form); });
    server.on("/save", HTTP_POST, [&]() {
        const String ssid = server.arg("ssid");
        const String pass = server.arg("pass");
        if (ssid.length() == 0 || ssid.length() > 32 || pass.length() > 64) {
            server.send(400, "text/html", "<meta charset='utf-8'>Invalid SSID / password. <a href='/'>Back</a>");
            return;
        }
        wifi_credentials_save(ssid.c_str(), pass.c_str());
        saved_ssid = ssid;
        saved = true;
        server.send(200, "text/html",
                    "<!DOCTYPE html><meta charset='utf-8'>"
                    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                    "<body style='font-family:sans-serif;background:#111;color:#eee;margin:24px'>"
                    "<h2 style='color:#00d25a'>Saved</h2><p>" + html_escape(ssid) +
                    "</p><p>You can close this page and leave the ESPTalkie network.</p></body>");
    });
    // Captive-portal probes (iOS / Android / Windows) and anything else -> form
    server.onNotFound([&]() {
        server.sendHeader("Location", "http://192.168.4.1/", true);
        server.send(302, "text/plain", "");
    });
    server.begin();

    Page page = Page::Join;
    draw_page(page, ap_ssid, ap_pass, nullptr);
    const uint32_t start = millis();
    bool cancelled = false;
    while (!saved) {
        dns.processNextRequest();
        server.handleClient();
        M5.update();
        if (M5.BtnA.wasClicked() || M5.BtnA.wasHold()) {
            cancelled = true;
            break;
        }
        if (millis() - start >= STOPWATCH_WIFI_SETUP_TIMEOUT_MS) {
            cancelled = true;
            break;
        }
        const Page want = WiFi.softAPgetStationNum() > 0 ? Page::Open : Page::Join;
        if (want != page) {
            page = want;
            draw_page(page, ap_ssid, ap_pass, nullptr);
        }
        delay(5);
    }
    if (saved) {
        // let the phone receive the confirmation page
        const uint32_t t0 = millis();
        while (millis() - t0 < 1500) {
            server.handleClient();
            delay(5);
        }
    }

    server.stop();
    dns.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    delay(50);
#ifdef ESPNOW_LONG_RANGE
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#endif
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(static_cast<uint8_t>(restore_channel), WIFI_SECOND_CHAN_NONE);

    draw_page(saved ? Page::Saved : Page::Cancelled, ap_ssid, ap_pass, saved_ssid.c_str());
    Serial.printf("Wi-Fi setup: %s\n", saved ? "saved" : (cancelled ? "cancelled" : "ended"));
    delay(1200);
    return saved;
}

#else

bool wifi_setup_run(int) { return false; }

#endif
