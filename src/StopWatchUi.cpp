#include "StopWatchUi.h"

#include "config.h"

#if TALKIE_TARGET_M5STOPWATCH

#include <Arduino.h>
#include <M5Unified.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "DisplaySync.h"
#include "TimeSync.h"

// Main screen (round 466x466). Angles are clock angles (0 = 12 o'clock,
// clockwise), radii from the screen centre.
//
//   r   0-199  photo (translucent clock hands on top); the outer part
//              (r 172-199) is shaded and carries the ticks, the hour /
//              minute markers and the second dot, drawn over the photo
//   r 201-231  outer ring:
//                status   -56..56  (RECEIVE / RECEIVING / TRANSMIT / CONT TX)
//                battery   64..124 (gauge + "BAT nn%")
//                info     134..226 ("CH 01 · VOL 3 · M1")
//                signal   236..304 (value + 8 segments)
//
// The whole screen is composed in a PSRAM frame buffer and pushed at once
// (no flicker): static layer (photo, dial, ring background) + dynamic parts.
// It is redrawn every second and whenever the radio state changes.

extern const uint8_t sw_image_start[] asm("_binary_assets_stopwatch_dog_jpg_start");
extern const uint8_t sw_image_end[] asm("_binary_assets_stopwatch_dog_jpg_end");

namespace {

constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

constexpr uint16_t kBg          = rgb(0, 0, 0);
constexpr uint16_t kRingBg      = rgb(18, 20, 26);
constexpr uint16_t kTick        = rgb(200, 205, 215);
constexpr uint16_t kTickDot     = rgb(170, 175, 185);
constexpr uint16_t kText        = rgb(240, 245, 255);
constexpr uint16_t kTextSub     = rgb(150, 190, 240);
constexpr uint16_t kTextDim     = rgb(120, 130, 145);
constexpr uint16_t kStandby     = rgb(30, 120, 255);
constexpr uint16_t kReceiving   = rgb(0, 200, 90);
constexpr uint16_t kTransmit    = rgb(235, 40, 40);
constexpr uint16_t kContTx      = rgb(255, 140, 0);
constexpr uint16_t kGaugeBg     = rgb(55, 60, 70);
constexpr uint16_t kBarLow      = rgb(0, 200, 90);
constexpr uint16_t kBarHigh     = rgb(235, 40, 40);
constexpr uint16_t kHourColor   = rgb(255, 255, 255);
constexpr uint16_t kMinuteColor = rgb(0, 220, 255);
constexpr uint16_t kSecondColor = rgb(255, 60, 60);
constexpr uint16_t kRowBg       = rgb(28, 32, 40);
constexpr uint16_t kRowBorder   = rgb(80, 90, 105);
constexpr uint16_t kRowSelBg    = rgb(0, 60, 28);
constexpr uint16_t kRowSelBd    = rgb(0, 210, 90);
constexpr uint16_t kSetupTitle  = rgb(255, 200, 0);

constexpr int kPhotoR   = 199;   // photo radius (JPEG is 398x398)
constexpr int kDialIn   = 172;   // shaded dial zone kDialIn..kDialOut
constexpr int kDialOut  = 199;
constexpr int kDialShadeAlpha = 110;
constexpr int kBandIn   = 201;
constexpr int kBandOut  = 231;
constexpr float kBandMid = (kBandIn + kBandOut) / 2.0f;

constexpr int kRowPitch    = 56;   // setup rows
constexpr int kRowHalfW    = 125;
constexpr int kRowHalfH    = 24;
constexpr int kTouchBtnOfs = 186;
constexpr int kTouchBtnR   = 34;

constexpr uint32_t kRxActiveMs        = 250;
constexpr uint32_t kRxNewSessionGapMs = 1500;
constexpr uint32_t kBatteryPollMs     = 5000;

constexpr int kGlyphSize = 40;   // scratch sprite for one character

M5Canvas *s_static = nullptr;   // photo + dial + ring background
M5Canvas *s_frame = nullptr;    // composed screen
M5Canvas *s_glyph = nullptr;
uint16_t *s_fb = nullptr;       // blend target: s_frame buffer (byte-swapped RGB565)
int s_w = 0;
int s_h = 0;
float s_cx = 0;
float s_cy = 0;

volatile bool s_setup = false;
StopWatchSetupItem s_selected = StopWatchSetupItem::Channel;
volatile int s_channel = 1;
volatile int s_volume = 3;
volatile uint8_t s_mode = 1;

volatile bool s_transmitting = false;
volatile bool s_continuous = false;
volatile bool s_receiving = false;
volatile bool s_meter_is_tx = false;
volatile int16_t s_meter_value = -127;
volatile bool s_dirty = true;

int s_batt_level = -1;
bool s_batt_charging = false;
uint32_t s_batt_ms = 0;

uint32_t s_hands_clear_until = 0;
bool s_hands_clear = false;
time_t s_last_drawn_sec = 0;
int s_setup_minute = -1;
const char *s_time_msg = nullptr;

uint8_t s_vib_steps = 0;
uint32_t s_vib_next_ms = 0;

// ------------------------------------------------------------------ helpers

inline uint16_t swap16(uint16_t v) { return static_cast<uint16_t>((v >> 8) | (v << 8)); }

inline float rad(float deg) { return deg * 0.017453292f; }

// Point at clock angle deg, radius r.
inline void polar(float deg, float r, float &x, float &y)
{
    x = s_cx + r * sinf(rad(deg));
    y = s_cy - r * cosf(rad(deg));
}

// Blend color over the frame buffer pixel with alpha 0..255.
inline void blend_px(int x, int y, uint16_t c, int a)
{
    if (a <= 0 || x < 0 || y < 0 || x >= s_w || y >= s_h) return;
    uint16_t *p = &s_fb[y * s_w + x];
    if (a >= 255) {
        *p = swap16(c);
        return;
    }
    const uint16_t d = swap16(*p);
    const int dr = d >> 11, dg = (d >> 5) & 0x3F, db = d & 0x1F;
    const int sr = c >> 11, sg = (c >> 5) & 0x3F, sb = c & 0x1F;
    const int r = dr + ((sr - dr) * a) / 255;
    const int g = dg + ((sg - dg) * a) / 255;
    const int b = db + ((sb - db) * a) / 255;
    *p = swap16(static_cast<uint16_t>((r << 11) | (g << 5) | b));
}

// Anti-aliased thick segment, alpha 0..255.
void blend_segment(float x0, float y0, float x1, float y1, float width, uint16_t c, int alpha)
{
    const float hw = width * 0.5f;
    const int minx = static_cast<int>(floorf(fminf(x0, x1) - hw - 1));
    const int maxx = static_cast<int>(ceilf(fmaxf(x0, x1) + hw + 1));
    const int miny = static_cast<int>(floorf(fminf(y0, y1) - hw - 1));
    const int maxy = static_cast<int>(ceilf(fmaxf(y0, y1) + hw + 1));
    const float vx = x1 - x0, vy = y1 - y0;
    const float len2 = vx * vx + vy * vy;
    for (int y = miny; y <= maxy; ++y) {
        for (int x = minx; x <= maxx; ++x) {
            const float px = x + 0.5f - x0, py = y + 0.5f - y0;
            float t = len2 > 0 ? (px * vx + py * vy) / len2 : 0;
            if (t < 0) t = 0;
            if (t > 1) t = 1;
            const float dx = px - t * vx, dy = py - t * vy;
            float cov = hw + 0.5f - sqrtf(dx * dx + dy * dy);
            if (cov <= 0) continue;
            if (cov > 1) cov = 1;
            blend_px(x, y, c, static_cast<int>(cov * alpha));
        }
    }
}

void blend_disc(float cx, float cy, float r, uint16_t c, int alpha)
{
    for (int y = static_cast<int>(cy - r - 1); y <= static_cast<int>(cy + r + 1); ++y) {
        for (int x = static_cast<int>(cx - r - 1); x <= static_cast<int>(cx + r + 1); ++x) {
            const float dx = x + 0.5f - cx, dy = y + 0.5f - cy;
            float cov = r + 0.5f - sqrtf(dx * dx + dy * dy);
            if (cov <= 0) continue;
            if (cov > 1) cov = 1;
            blend_px(x, y, c, static_cast<int>(cov * alpha));
        }
    }
}

// Arc on a canvas between clock angles a0..a1 (a0 < a1), radii r0..r1.
void clock_arc(M5Canvas *cv, float a0, float a1, int r0, int r1, uint16_t c)
{
    cv->fillArc(static_cast<int>(s_cx), static_cast<int>(s_cy), r1, r0, a0 - 90.0f, a1 - 90.0f, c);
}

inline float glyph_mask(int x, int y)
{
    if (x < 0 || y < 0 || x >= kGlyphSize || y >= kGlyphSize) return 0.0f;
    const uint16_t *g = static_cast<const uint16_t *>(s_glyph->getBuffer());
    return g[y * kGlyphSize + x] ? 1.0f : 0.0f;
}

// Draw one character centred at (px, py), rotated clockwise by rot degrees.
void rotated_char(const char *ch, float px, float py, float rot, uint16_t color)
{
    s_glyph->fillScreen(0);
    s_glyph->setTextColor(0xFFFF);
    s_glyph->setTextDatum(middle_center);
    s_glyph->drawString(ch, kGlyphSize / 2, kGlyphSize / 2);
    const float cs = cosf(rad(rot)), sn = sinf(rad(rot));
    const int R = static_cast<int>(kGlyphSize * 0.72f);
    const float gc = kGlyphSize / 2.0f;
    for (int dy = -R; dy <= R; ++dy) {
        for (int dx = -R; dx <= R; ++dx) {
            // inverse rotation (screen coords, y down)
            const float sx = dx * cs + dy * sn + gc - 0.5f;
            const float sy = -dx * sn + dy * cs + gc - 0.5f;
            const int ix = static_cast<int>(floorf(sx)), iy = static_cast<int>(floorf(sy));
            if (ix < -1 || iy < -1 || ix >= kGlyphSize || iy >= kGlyphSize) continue;
            const float fx = sx - ix, fy = sy - iy;
            const float m = glyph_mask(ix, iy) * (1 - fx) * (1 - fy) +
                            glyph_mask(ix + 1, iy) * fx * (1 - fy) +
                            glyph_mask(ix, iy + 1) * (1 - fx) * fy +
                            glyph_mask(ix + 1, iy + 1) * fx * fy;
            if (m <= 0.02f) continue;
            blend_px(static_cast<int>(px) + dx, static_cast<int>(py) + dy, color, static_cast<int>(m * 255));
        }
    }
}

// Text along the ring centred on clock angle center_deg. bottom=true keeps it
// upright on the lower half of the ring.
void curved_text(const char *text, float center_deg, float r, const lgfx::IFont *font,
                 uint16_t color, bool bottom)
{
    s_glyph->setFont(font);
    constexpr float kSpacing = 1.0f;
    const float deg_per_px = 57.29578f / r;
    float total = 0;
    char ch[2] = { 0, 0 };
    for (const char *p = text; *p; ++p) {
        ch[0] = *p;
        total += s_glyph->textWidth(ch) + kSpacing;
    }
    float a = bottom ? center_deg + total * deg_per_px / 2 : center_deg - total * deg_per_px / 2;
    for (const char *p = text; *p; ++p) {
        ch[0] = *p;
        const float da = (s_glyph->textWidth(ch) + kSpacing) * deg_per_px;
        const float mid = bottom ? a - da / 2 : a + da / 2;
        if (*p != ' ') {
            float x, y;
            polar(mid, r, x, y);
            rotated_char(ch, x, y, bottom ? mid - 180.0f : mid, color);
        }
        a = bottom ? a - da : a + da;
    }
}

// --------------------------------------------------------------- main screen

uint16_t state_color()
{
    if (s_transmitting) return s_continuous ? kContTx : kTransmit;
    if (s_receiving) return kReceiving;
    return kStandby;
}

const char *state_label()
{
    if (s_transmitting) return s_continuous ? "CONT TX" : "TRANSMIT";
    if (s_receiving) return "RECEIVING";
    return "RECEIVE";
}

void build_static()
{
    M5Canvas *cv = s_static;
    cv->fillScreen(kBg);
    const size_t size = static_cast<size_t>(sw_image_end - sw_image_start);
    const bool ok = cv->drawJpg(sw_image_start, size,
                                static_cast<int>(s_cx) - kPhotoR, static_cast<int>(s_cy) - kPhotoR,
                                kPhotoR * 2, kPhotoR * 2, 0, 0, 0.0f, 0.0f, middle_center);
    if (!ok) {
        Serial.println("StopWatch: JPEG decode failed");
        cv->fillCircle(static_cast<int>(s_cx), static_cast<int>(s_cy), kPhotoR, kRowBg);
    }
    // Shade the outer part of the photo so ticks and markers stand out.
    uint16_t *saved = s_fb;
    s_fb = static_cast<uint16_t *>(cv->getBuffer());
    for (int y = static_cast<int>(s_cy) - kDialOut - 1; y <= static_cast<int>(s_cy) + kDialOut + 1; ++y) {
        for (int x = static_cast<int>(s_cx) - kDialOut - 1; x <= static_cast<int>(s_cx) + kDialOut + 1; ++x) {
            const float dx = x + 0.5f - s_cx, dy = y + 0.5f - s_cy;
            const float r = sqrtf(dx * dx + dy * dy);
            if (r < kDialIn - 12 || r > kDialOut + 1) continue;
            // soft inner edge (12 px ramp), full shade from kDialIn outward
            float k = (r - (kDialIn - 12)) / 12.0f;
            if (k > 1) k = 1;
            blend_px(x, y, kBg, static_cast<int>(k * kDialShadeAlpha));
        }
    }
    s_fb = saved;
    clock_arc(cv, 0, 360, kDialOut + 1, kBandIn - 1, kBg);
    clock_arc(cv, 0, 360, kBandIn, kBandOut, kRingBg);
    clock_arc(cv, 0, 360, kBandOut + 1, kBandOut + 80, kBg);
    for (int i = 0; i < 60; ++i) {
        const float deg = i * 6.0f;
        float x0, y0, x1, y1;
        if (i % 5 == 0) {
            const int len = (i % 15 == 0) ? 10 : 7;
            polar(deg, kDialOut - 2 - len, x0, y0);
            polar(deg, kDialOut - 2, x1, y1);
            cv->drawWideLine(static_cast<int>(x0), static_cast<int>(y0),
                             static_cast<int>(x1), static_cast<int>(y1),
                             (i % 15 == 0) ? 1.5f : 1.0f, kTick);
        } else {
            polar(deg, kDialOut - 5, x0, y0);
            cv->fillCircle(static_cast<int>(x0), static_cast<int>(y0), 1, kTickDot);
        }
    }
}

void draw_ring_dynamic()
{
    char buf[32];

    // status (top)
    clock_arc(s_frame, -56, 56, kBandIn, kBandOut, state_color());
    curved_text(state_label(), 0, kBandMid, &fonts::FreeSansBold12pt7b, kText, false);

    // battery (right)
    if (s_batt_level >= 0) {
        const uint16_t c = s_batt_charging ? kSetupTitle
                         : (s_batt_level < 20 ? kTransmit : kReceiving);
        clock_arc(s_frame, 64, 124, kBandIn + 7, kBandOut - 7, kGaugeBg);
        if (s_batt_level > 0) {
            clock_arc(s_frame, 64, 64 + 60.0f * s_batt_level / 100.0f, kBandIn + 7, kBandOut - 7, c);
        }
        snprintf(buf, sizeof(buf), "%s %d%%", s_batt_charging ? "CHG" : "BAT", s_batt_level);
        curved_text(buf, 94, kBandMid, &fonts::Font2, kText, false);
    }

    // CH / VOL / VOICE (bottom)
    snprintf(buf, sizeof(buf), "CH %02d  VOL %d  M%u", s_channel, s_volume, static_cast<unsigned>(s_mode));
    curved_text(buf, 180, kBandMid, &fonts::FreeSansBold9pt7b, kText, true);

    // signal (left): value + 8 segments
    static const int16_t kRssiLevel[8] = { -90, -80, -70, -60, -50, -40, -30, -20 };
    const int16_t v = s_meter_value;
    int active = 0;
    if (s_meter_is_tx) {
        active = v / 3;
        snprintf(buf, sizeof(buf), "%ddBm", v);
    } else if (v <= -120) {
        snprintf(buf, sizeof(buf), "---");
    } else {
        for (int i = 0; i < 8; ++i) {
            if (v >= kRssiLevel[i]) active = i + 1;
        }
        snprintf(buf, sizeof(buf), "%d", v);
    }
    if (active < 0) active = 0;
    if (active > 8) active = 8;
    for (int i = 0; i < 8; ++i) {
        const float a0 = 250 + i * 6.6f;
        const uint16_t c = (i < active) ? ((i < 5) ? kBarLow : kBarHigh) : kGaugeBg;
        const int h = 6 + (i * 13) / 5;  // 6..24 px
        const int r0 = static_cast<int>(kBandMid - h / 2.0f);
        clock_arc(s_frame, a0, a0 + 5, r0, r0 + h, c);
    }
    curved_text(buf, 240, kBandMid, &fonts::Font2, kText, true);
}

void draw_clock(const tm &t)
{
    const float ah = (t.tm_hour % 12) * 30.0f + t.tm_min * 0.5f + t.tm_sec / 120.0f;
    const float am = t.tm_min * 6.0f + t.tm_sec * 0.1f;
    const float as = t.tm_sec * 6.0f;
    const int alpha = s_hands_clear ? 255 : STOPWATCH_HANDS_ALPHA;

    auto hand = [&](float deg, float len, float tail, float w, uint16_t c, int a) {
        float x0, y0, x1, y1;
        polar(deg + 180.0f, tail, x0, y0);
        polar(deg, len, x1, y1);
        if (s_hands_clear) {
            blend_segment(x0, y0, x1, y1, w + 3, kBg, 255);
        }
        blend_segment(x0, y0, x1, y1, w, c, a);
    };
    hand(ah, 100, 10, 9, kHourColor, alpha);
    hand(am, 150, 12, 6, kMinuteColor, alpha);
#if STOPWATCH_SHOW_SECONDS
    hand(as, 166, 20, 2, kSecondColor, alpha + 40 > 255 ? 255 : alpha + 40);
#endif
    if (s_hands_clear) blend_disc(s_cx, s_cy, 7, kBg, 255);
    blend_disc(s_cx, s_cy, 6, kSecondColor, alpha);

    // markers on the dial ring (always opaque)
    auto marker = [&](float deg, float tip_r, float half_deg, uint16_t c) {
        float x0, y0, x1, y1, x2, y2;
        polar(deg, tip_r, x0, y0);
        polar(deg - half_deg, kDialOut - 1, x1, y1);
        polar(deg + half_deg, kDialOut - 1, x2, y2);
        s_frame->fillTriangle(x0, y0, x1, y1, x2, y2, c);
        s_frame->drawTriangle(x0, y0, x1, y1, x2, y2, kBg);
    };
    marker(am, kDialIn + 2, 4.5f, kMinuteColor);
    marker(ah, kDialIn + 4, 7.0f, kHourColor);
#if STOPWATCH_SHOW_SECONDS
    float sx, sy;
    polar(as, kDialIn + 14, sx, sy);
    s_frame->fillCircle(static_cast<int>(sx), static_cast<int>(sy), 4, kSecondColor);
#endif
}

void render_main()
{
    if (!s_frame || !s_static) return;
    memcpy(s_fb, s_static->getBuffer(), static_cast<size_t>(s_w) * s_h * 2);
    draw_ring_dynamic();
    if (time_is_valid()) {
        const time_t now = time(nullptr);
        tm t;
        localtime_r(&now, &t);
        draw_clock(t);
        s_last_drawn_sec = now;
    }
    display_lock();
    s_frame->pushSprite(0, 0);
    display_unlock();
}

void read_battery()
{
    int level = M5.Power.getBatteryLevel();
    if (level < 0) level = 0;
    if (level > 100) level = 100;
    const bool charging = (M5.Power.isCharging() == m5::Power_Class::is_charging);
    if (level != s_batt_level || charging != s_batt_charging) {
        s_batt_level = level;
        s_batt_charging = charging;
        s_dirty = true;
    }
}

// --------------------------------------------------------------- setup screen

int cxi() { return static_cast<int>(s_cx); }
int cyi() { return static_cast<int>(s_cy); }

int row_center_y(int index)
{
    return cyi() + (index - 2) * kRowPitch;
}

void draw_setup_row(int index)
{
    static const char *const kLabels[5] = { "CHANNEL", "VOLUME", "VOICE", "TIME", "WIFI" };
    const bool sel = (static_cast<int>(s_selected) == index);
    const int yc = row_center_y(index);
    const int x = cxi() - kRowHalfW;
    const int y = yc - kRowHalfH;
    const int w = kRowHalfW * 2;
    const int h = kRowHalfH * 2;
    display_lock();
    M5.Display.fillRoundRect(x, y, w, h, 11, sel ? kRowSelBg : kRowBg);
    M5.Display.drawRoundRect(x, y, w, h, 11, sel ? kRowSelBd : kRowBorder);
    if (sel) {
        M5.Display.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 10, kRowSelBd);
    }
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextDatum(middle_left);
    M5.Display.setTextColor(sel ? kRowSelBd : kTextSub);
    M5.Display.drawString(kLabels[index], x + 14, yc);

    char buf[40];
    const lgfx::IFont *font = &fonts::FreeSansBold18pt7b;
    switch (index) {
        case 0: snprintf(buf, sizeof(buf), "%02d", s_channel); break;
        case 1: snprintf(buf, sizeof(buf), "%d", s_volume); break;
        case 2: snprintf(buf, sizeof(buf), "M%u", static_cast<unsigned>(s_mode)); break;
        case 4: {
            char ssid[33];
            char pass[65];
            if (wifi_credentials_get(ssid, sizeof(ssid), pass, sizeof(pass))) {
                if (strlen(ssid) > 12) {
                    ssid[11] = '~';
                    ssid[12] = 0;
                }
                snprintf(buf, sizeof(buf), "%s", ssid);
            } else {
                snprintf(buf, sizeof(buf), "NOT SET");
            }
            font = &fonts::FreeSansBold9pt7b;
            break;
        }
        default:
            if (s_time_msg) {
                snprintf(buf, sizeof(buf), "%s", s_time_msg);
                font = &fonts::FreeSansBold12pt7b;
            } else if (time_is_valid()) {
                const time_t now = time(nullptr);
                tm t;
                localtime_r(&now, &t);
                snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
            } else {
                snprintf(buf, sizeof(buf), "--:--");
            }
            break;
    }
    M5.Display.setFont(font);
    M5.Display.setTextDatum(middle_right);
    M5.Display.setTextColor(kText);
    M5.Display.drawString(buf, x + w - 14, yc + 1);
    M5.Display.setTextDatum(middle_center);
    display_unlock();
}

void draw_touch_button(int x, const char *label)
{
    M5.Display.fillCircle(x, cyi(), kTouchBtnR, kRowBg);
    M5.Display.drawCircle(x, cyi(), kTouchBtnR, kRowBorder);
    M5.Display.setFont(&fonts::FreeSansBold18pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(kText);
    M5.Display.drawString(label, x, cyi() + 1);
}

void draw_setup()
{
    display_lock();
    M5.Display.startWrite();
    M5.Display.fillScreen(kBg);
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(kSetupTitle);
    M5.Display.drawString("SETUP", cxi(), cyi() - 165);
    for (int i = 0; i < static_cast<int>(StopWatchSetupItem::Count); ++i) {
        draw_setup_row(i);
    }
    draw_touch_button(cxi() - kTouchBtnOfs, "-");
    draw_touch_button(cxi() + kTouchBtnOfs, "+");
    M5.Display.setFont(&fonts::Font2);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(kTextDim);
    M5.Display.drawString("YELLOW: item   BLUE: +1 / run", cxi(), cyi() + 156);
    M5.Display.drawString("hold YELLOW: exit", cxi(), cyi() + 176);
    M5.Display.endWrite();
    display_unlock();
}

}  // namespace

void stopwatch_ui_begin(int channel, int volume_level, uint8_t tx_pitch_mode)
{
    s_channel = channel;
    s_volume = volume_level;
    s_mode = tx_pitch_mode;
    s_w = M5.Display.width();
    s_h = M5.Display.height();
    s_cx = s_w / 2.0f;
    s_cy = s_h / 2.0f;

    s_glyph = new M5Canvas();
    s_glyph->setColorDepth(16);
    s_glyph->setPsram(false);
    s_glyph->createSprite(kGlyphSize, kGlyphSize);

    s_static = new M5Canvas(&M5.Display);
    s_static->setColorDepth(16);
    s_static->setPsram(true);
    s_frame = new M5Canvas(&M5.Display);
    s_frame->setColorDepth(16);
    s_frame->setPsram(true);
    if (!s_static->createSprite(s_w, s_h) || !s_frame->createSprite(s_w, s_h)) {
        Serial.println("StopWatch: frame buffer allocation failed");
        s_static->deleteSprite();
        s_frame->deleteSprite();
        delete s_static;
        delete s_frame;
        s_static = nullptr;
        s_frame = nullptr;
        return;
    }
    s_fb = static_cast<uint16_t *>(s_frame->getBuffer());
    build_static();
    s_batt_ms = millis();
    read_battery();
    render_main();
}

void stopwatch_ui_message(const char *msg)
{
    display_lock();
    M5.Display.fillScreen(kBg);
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(kText);
    M5.Display.drawString(msg, M5.Display.width() / 2, M5.Display.height() / 2);
    display_unlock();
}

void stopwatch_ui_service(uint32_t last_rx_ms)
{
    const uint32_t now = millis();

    // New incoming call (first packet after a quiet channel) -> vibrate.
    static uint32_t prev_rx_ms = 0;
    if (last_rx_ms != 0 && last_rx_ms != prev_rx_ms) {
        if (!s_transmitting && (prev_rx_ms == 0 || last_rx_ms - prev_rx_ms >= kRxNewSessionGapMs)) {
            stopwatch_ui_vibrate();
        }
        prev_rx_ms = last_rx_ms;
    }

    const bool receiving = last_rx_ms != 0 && (now - last_rx_ms) < kRxActiveMs;
    if (receiving != s_receiving) {
        s_receiving = receiving;
        s_dirty = true;
    }

    if (now - s_batt_ms >= kBatteryPollMs) {
        s_batt_ms = now;
        read_battery();
    }

    // Vibration pattern: on / off steps, even count = motor on.
    if (s_vib_steps > 0 && static_cast<int32_t>(now - s_vib_next_ms) >= 0) {
        const bool on = (s_vib_steps % 2) == 0;
        M5.Power.setVibration(on ? STOPWATCH_VIBRATION_LEVEL : 0);
        s_vib_next_ms = now + (on ? STOPWATCH_VIBRATION_ON_MS : STOPWATCH_VIBRATION_OFF_MS);
        --s_vib_steps;
    }

    if (s_hands_clear && static_cast<int32_t>(now - s_hands_clear_until) >= 0) {
        s_hands_clear = false;
        s_dirty = true;
    }

    const time_t sec = time(nullptr);
    if (s_setup) {
        // keep the TIME row current
        tm t;
        localtime_r(&sec, &t);
        if (t.tm_min != s_setup_minute) {
            s_setup_minute = t.tm_min;
            draw_setup_row(static_cast<int>(StopWatchSetupItem::Time));
        }
        return;
    }
    if (s_dirty || sec != s_last_drawn_sec) {
        s_dirty = false;
        render_main();
    }
}

void stopwatch_ui_vibrate()
{
    if (s_vib_steps > 0) return;  // pattern already running
    s_vib_steps = STOPWATCH_VIBRATION_PULSES * 2;
    s_vib_next_ms = millis();
}

void stopwatch_ui_show_hands()
{
    s_hands_clear = true;
    s_hands_clear_until = millis() + STOPWATCH_HANDS_CLEAR_MS;
    s_dirty = true;
}

StopWatchTouch stopwatch_ui_poll_touch()
{
    if (!s_setup || !M5.Touch.isEnabled()) return StopWatchTouch::None;
    const auto t = M5.Touch.getDetail();
    if (!t.wasPressed()) return StopWatchTouch::None;
    const int x = t.x;
    const int y = t.y;
    const int hit_r = kTouchBtnR + 12;
    auto in_circle = [&](int bx) {
        const int dx = x - bx;
        const int dy = y - cyi();
        return dx * dx + dy * dy <= hit_r * hit_r;
    };
    if (in_circle(cxi() - kTouchBtnOfs)) return StopWatchTouch::Minus;
    if (in_circle(cxi() + kTouchBtnOfs)) return StopWatchTouch::Plus;
    if (abs(x - cxi()) <= kRowHalfW) {
        for (int i = 0; i < static_cast<int>(StopWatchSetupItem::Count); ++i) {
            if (abs(y - row_center_y(i)) <= kRowHalfH + 2) {
                return static_cast<StopWatchTouch>(static_cast<int>(StopWatchTouch::SelectChannel) + i);
            }
        }
    }
    return StopWatchTouch::None;
}

bool stopwatch_ui_ptt_pressed()
{
    return !s_setup && M5.BtnB.isPressed();
}

bool stopwatch_ui_setup_visible()
{
    return s_setup;
}

void stopwatch_ui_show_setup(bool show)
{
    const bool reopen = show && s_setup;  // redraw after a sub-page (Wi-Fi setup)
    s_setup = show;
    if (show) {
        if (!reopen) s_selected = StopWatchSetupItem::Channel;
        s_time_msg = nullptr;
        s_setup_minute = -1;
        draw_setup();
    } else {
        s_dirty = true;
        render_main();
    }
}

void stopwatch_ui_set_selected_item(StopWatchSetupItem item)
{
    const int prev = static_cast<int>(s_selected);
    s_selected = item;
    if (s_setup) {
        draw_setup_row(prev);
        draw_setup_row(static_cast<int>(item));
    }
}

StopWatchSetupItem stopwatch_ui_selected_item()
{
    return s_selected;
}

void stopwatch_ui_set_time_message(const char *msg)
{
    s_time_msg = msg;
    if (s_setup) {
        draw_setup_row(static_cast<int>(StopWatchSetupItem::Time));
    }
}

void stopwatch_ui_set_settings(int channel, int volume_level, uint8_t tx_pitch_mode)
{
    s_channel = channel;
    s_volume = volume_level;
    s_mode = tx_pitch_mode;
    if (s_setup) {
        draw_setup_row(static_cast<int>(s_selected));
    }
    s_dirty = true;
}

void stopwatch_ui_set_status(bool transmitting, bool continuous)
{
    if (transmitting == s_transmitting && continuous == s_continuous) return;
    s_transmitting = transmitting;
    s_continuous = continuous;
    s_dirty = true;
}

void stopwatch_ui_set_rssi(int16_t rssi)
{
    if (!s_meter_is_tx && s_meter_value == rssi) return;
    s_meter_is_tx = false;
    s_meter_value = rssi;
    s_dirty = true;
}

void stopwatch_ui_set_tx_power(int16_t dbm)
{
    if (s_meter_is_tx && s_meter_value == dbm) return;
    s_meter_is_tx = true;
    s_meter_value = dbm;
    s_dirty = true;
}

#else  // !TALKIE_TARGET_M5STOPWATCH

void stopwatch_ui_begin(int, int, uint8_t) {}
void stopwatch_ui_service(uint32_t) {}
StopWatchTouch stopwatch_ui_poll_touch() { return StopWatchTouch::None; }
bool stopwatch_ui_ptt_pressed() { return false; }
bool stopwatch_ui_setup_visible() { return false; }
void stopwatch_ui_show_setup(bool) {}
void stopwatch_ui_set_selected_item(StopWatchSetupItem) {}
StopWatchSetupItem stopwatch_ui_selected_item() { return StopWatchSetupItem::Channel; }
void stopwatch_ui_show_hands() {}
void stopwatch_ui_set_time_message(const char *) {}
void stopwatch_ui_message(const char *) {}
void stopwatch_ui_set_settings(int, int, uint8_t) {}
void stopwatch_ui_set_status(bool, bool) {}
void stopwatch_ui_set_rssi(int16_t) {}
void stopwatch_ui_set_tx_power(int16_t) {}
void stopwatch_ui_vibrate() {}

#endif
