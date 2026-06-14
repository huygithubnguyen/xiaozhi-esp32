#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "led/single_led.h"
#include "assets/lang_config.h"
#include "pca9685.h"
#include "clock_manager.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <led_strip.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <time.h>
#include <sys/time.h>
#include <cmath>
#include <cstring>
#include <esp_heap_caps.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#define TAG "WatchInterfaceBoard"

// =============================================================================
// Boot face test (ILI9341 240x320). Cycles every procedural face through the
// LVGL canvas (the same render path the runtime uses) at boot, so the LCD is
// verified through LVGL's flush rather than a raw panel write.
// Set to 0 to disable once the LCD is verified.
// =============================================================================
#define WATCH_ENABLE_FACES_TEST 1

// Face rendering primitives are always compiled: WatchFaceLcdDisplay uses them at
// runtime, and the boot test (RunFaceTest) is still gated by the macro above.
namespace {
constexpr int kSfW = DISPLAY_WIDTH;
constexpr int kSfH = DISPLAY_HEIGHT;
constexpr uint16_t kSfBlack = 0x0000;
constexpr uint16_t kSfWhite = 0xFFFF;

// Rotation applied to the face on screen: 0 = upright, 1 = 90 CW, -1 = 90 CCW.
constexpr int kSfRotate = 1;
constexpr int kSfCx = kSfW / 2;   // rotation centre (120, 160)
constexpr int kSfCy = kSfH / 2;

inline bool SfInCircle(int x, int y, int cx, int cy, int r) {
    int dx = x - cx, dy = y - cy;
    return dx * dx + dy * dy <= r * r;
}

inline bool SfInRect(int x, int y, int x0, int y0, int x1, int y1) {
    return x >= x0 && x <= x1 && y >= y0 && y <= y1;
}

inline int SfEdge(int ax, int ay, int bx, int by, int px, int py) {
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

inline bool SfInTri(int x, int y, int ax, int ay, int bx, int by, int cx, int cy) {
    int e1 = SfEdge(ax, ay, bx, by, x, y);
    int e2 = SfEdge(bx, by, cx, cy, x, y);
    int e3 = SfEdge(cx, cy, ax, ay, x, y);
    return (e1 >= 0 && e2 >= 0 && e3 >= 0) || (e1 <= 0 && e2 <= 0 && e3 <= 0);
}

// Heart shape (two lobes + a downward point) used by the love face.
inline bool SfInHeart(int x, int y, int cx, int cy, int s) {
    if (SfInCircle(x, y, cx - s / 2, cy - s / 3, s * 3 / 4)) return true;   // left lobe
    if (SfInCircle(x, y, cx + s / 2, cy - s / 3, s * 3 / 4)) return true;   // right lobe
    if (SfInTri(x, y, cx - s, cy - s / 4, cx + s, cy - s / 4, cx, cy + s)) return true;  // point
    return false;
}

// Diamond (rotated square) used for sparkle / star eyes.
inline bool SfInDiamond(int x, int y, int cx, int cy, int r) {
    int ax = x - cx; if (ax < 0) ax = -ax;
    int ay = y - cy; if (ay < 0) ay = -ay;
    return ax + ay <= r;
}

// Normal / neutral face: level brows, straight mouth. Eyes/pupils use the same
// coordinates as the sad face so the eye layout is identical between the two.
// eyes_open = false draws closed-eye slits (used for the blink).
inline uint16_t SfNormalPixel(int x, int y, bool eyes_open) {
    uint16_t c = kSfBlack;
    if (eyes_open) {
        if (SfInCircle(x, y, 82, 135, 38) || SfInCircle(x, y, 158, 135, 38))
            c = kSfWhite;                                               // eyes (same as sad)
        if (SfInCircle(x, y, 82, 150, 15) || SfInCircle(x, y, 158, 150, 15))
            c = kSfBlack;                                               // pupils (same as sad)
    } else {
        if (SfInRect(x, y, 44, 133, 120, 138) || SfInRect(x, y, 120, 133, 196, 138))
            c = kSfWhite;                                               // closed-eye slits
    }
    if (SfInRect(x, y, 50, 88, 114, 93) || SfInRect(x, y, 126, 88, 190, 93))
        c = kSfBlack;                                                   // level (neutral) brows
    if (SfInRect(x, y, 98, 229, 142, 235))
        c = kSfWhite;                                                   // straight mouth
    return c;
}

// Sad face with a tear at (tear_y, tear_r). Shapes applied in paint order;
// later ones sit on top (eyes, pupils, brows, mouth, eraser, tear).
inline uint16_t SfSadPixel(int x, int y, int tear_y, int tear_r) {
    uint16_t c = kSfBlack;
    if (SfInCircle(x, y, 82, 135, 38) || SfInCircle(x, y, 158, 135, 38))
        c = kSfWhite;                                                   // eyes
    if (SfInCircle(x, y, 82, 150, 15) || SfInCircle(x, y, 158, 150, 15))
        c = kSfBlack;                                                   // pupils looking down
    if (SfInTri(x, y, 36, 96, 124, 126, 124, 96) ||                    // brows, slant to centre
        SfInTri(x, y, 204, 96, 116, 126, 116, 96))
        c = kSfBlack;
    if (SfInCircle(x, y, 120, 235, 22))
        c = kSfWhite;                                                   // mouth
    if (SfInCircle(x, y, 120, 228, 24))
        c = kSfBlack;                                                   // eraser above -> frown
    if (SfInCircle(x, y, 158, tear_y, tear_r))
        c = kSfWhite;                                                   // tear
    return c;
}

// Complete face = happy expression. wink: 0 = both eyes open, 1 = left closed,
// 2 = right closed (the wink animation).
inline uint16_t SfCompletePixel(int x, int y, int wink) {
    uint16_t c = kSfBlack;
    if (wink == 1) {
        if (SfInRect(x, y, 44, 133, 120, 138))
            c = kSfWhite;                                              // left wink (slit)
    } else {
        if (SfInCircle(x, y, 82, 135, 38)) c = kSfWhite;             // left eye open
        if (SfInCircle(x, y, 82, 150, 15)) c = kSfBlack;             // left pupil
    }
    if (wink == 2) {
        if (SfInRect(x, y, 120, 133, 196, 138))
            c = kSfWhite;                                              // right wink (slit)
    } else {
        if (SfInCircle(x, y, 158, 135, 38)) c = kSfWhite;            // right eye open
        if (SfInCircle(x, y, 158, 150, 15)) c = kSfBlack;            // right pupil
    }
    if (SfInRect(x, y, 50, 80, 114, 85) || SfInRect(x, y, 126, 80, 190, 85))
        c = kSfBlack;                                                  // raised (happy) brows
    if (SfInCircle(x, y, 120, 222, 28))
        c = kSfWhite;                                                  // smile base
    if (SfInRect(x, y, 84, 194, 156, 222))
        c = kSfBlack;                                                  // erase top half -> open grin
    return c;
}

// Scared face: same eyes/pupils, raised brows and a round "O" mouth. big =
// true widens the mouth (the gasp animation).
inline uint16_t SfScaredPixel(int x, int y, bool big) {
    uint16_t c = kSfBlack;
    if (SfInCircle(x, y, 82, 135, 38) || SfInCircle(x, y, 158, 135, 38))
        c = kSfWhite;                                                  // eyes (same layout)
    if (SfInCircle(x, y, 82, 150, 15) || SfInCircle(x, y, 158, 150, 15))
        c = kSfBlack;                                                  // pupils (same layout)
    if (SfInRect(x, y, 50, 74, 114, 79) || SfInRect(x, y, 126, 74, 190, 79))
        c = kSfBlack;                                                  // raised (scared) brows
    int ro = big ? 26 : 20;
    int ri = big ? 15 : 11;
    if (SfInCircle(x, y, 120, 225, ro))
        c = kSfWhite;                                                  // open mouth outer
    if (SfInCircle(x, y, 120, 225, ri))
        c = kSfBlack;                                                  // hollow centre -> "O" gasp
    return c;
}

// Remind face: one raised brow (the "hmm?" look), a small off-centre mouth,
// and a notification dot. show_dot toggles the dot (the pulse animation).
inline uint16_t SfRemindPixel(int x, int y, bool show_dot) {
    uint16_t c = kSfBlack;
    if (SfInCircle(x, y, 82, 135, 38) || SfInCircle(x, y, 158, 135, 38))
        c = kSfWhite;                                                  // eyes (same layout)
    if (SfInCircle(x, y, 82, 150, 15) || SfInCircle(x, y, 158, 150, 15))
        c = kSfBlack;                                                  // pupils (same layout)
    if (SfInRect(x, y, 50, 80, 114, 85))
        c = kSfBlack;                                                  // left brow raised
    if (SfInRect(x, y, 126, 88, 190, 93))
        c = kSfBlack;                                                  // right brow level
    if (SfInRect(x, y, 128, 230, 172, 236))
        c = kSfWhite;                                                  // small mouth, offset right
    if (show_dot && SfInCircle(x, y, 210, 110, 12))
        c = kSfWhite;                                                  // notification dot
    return c;
}

// Sleep face: closed eyes, relaxed (droopy) brows, a peaceful mouth, and Zzz
// dots. show_zzz toggles the dots (the sleep animation).
inline uint16_t SfSleepPixel(int x, int y, bool show_zzz) {
    uint16_t c = kSfBlack;
    if (SfInRect(x, y, 44, 133, 120, 138) || SfInRect(x, y, 120, 133, 196, 138))
        c = kSfWhite;                                                  // closed eyes (slits)
    if (SfInRect(x, y, 50, 92, 114, 97) || SfInRect(x, y, 126, 92, 190, 97))
        c = kSfBlack;                                                  // relaxed brows
    if (SfInRect(x, y, 104, 234, 136, 239))
        c = kSfWhite;                                                  // peaceful mouth
    if (show_zzz) {
        if (SfInCircle(x, y, 208, 90, 7)) c = kSfWhite;              // Z
        if (SfInCircle(x, y, 216, 76, 6)) c = kSfWhite;             // z
        if (SfInCircle(x, y, 224, 64, 5)) c = kSfWhite;             // z
    }
    return c;
}


// Angry face: steep V-shaped brows (inner ends pulled down) over a frown.
inline uint16_t SfAngryPixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInCircle(x, y, 82, 135, 38) || SfInCircle(x, y, 158, 135, 38))
        c = kSfWhite;                                                   // eyes (same layout)
    if (SfInCircle(x, y, 82, 150, 15) || SfInCircle(x, y, 158, 150, 15))
        c = kSfBlack;                                                   // pupils looking ahead
    if (SfInTri(x, y, 44, 78, 114, 130, 114, 78) ||                    // angry brows, steep toward centre
        SfInTri(x, y, 196, 78, 126, 130, 126, 78))
        c = kSfBlack;
    if (SfInCircle(x, y, 120, 235, 22))
        c = kSfWhite;                                                   // frown base
    if (SfInCircle(x, y, 120, 228, 24))
        c = kSfBlack;                                                   // erase top -> frown
    return c;
}

// Flat / expressionless face: droopy upper lids, no brow expression, straight mouth.
inline uint16_t SfFlatPixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInCircle(x, y, 82, 135, 38) || SfInCircle(x, y, 158, 135, 38))
        c = kSfWhite;                                                   // eyes
    if (SfInCircle(x, y, 82, 150, 15) || SfInCircle(x, y, 158, 150, 15))
        c = kSfBlack;                                                   // pupils
    if (SfInRect(x, y, 44, 108, 120, 122) || SfInRect(x, y, 120, 108, 196, 122))
        c = kSfBlack;                                                   // droopy upper lids (tired/blank)
    if (SfInRect(x, y, 92, 233, 148, 238))
        c = kSfWhite;                                                   // straight, slightly wider mouth
    return c;
}

// Love face: heart eyes and a big open smile.
inline uint16_t SfLovePixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInHeart(x, y, 82, 135, 16) || SfInHeart(x, y, 158, 135, 16))
        c = kSfWhite;                                                   // heart eyes
    if (SfInRect(x, y, 50, 80, 114, 85) || SfInRect(x, y, 126, 80, 190, 85))
        c = kSfBlack;                                                   // raised (happy) brows
    if (SfInCircle(x, y, 120, 222, 28))
        c = kSfWhite;                                                   // smile base
    if (SfInRect(x, y, 84, 194, 156, 222))
        c = kSfBlack;                                                   // erase top half -> open grin
    return c;
}

// Cool face: dark sunglasses over the eyes and a small smirk.
inline uint16_t SfCoolPixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInCircle(x, y, 82, 135, 38) || SfInCircle(x, y, 158, 135, 38))
        c = kSfWhite;                                                   // eye rims (face behind shades)
    if (SfInRect(x, y, 48, 118, 116, 156) || SfInRect(x, y, 124, 118, 192, 156))
        c = kSfBlack;                                                   // dark lenses
    if (SfInRect(x, y, 116, 128, 124, 138))
        c = kSfBlack;                                                   // bridge
    if (SfInCircle(x, y, 120, 228, 18))
        c = kSfWhite;                                                   // smirk base
    if (SfInRect(x, y, 90, 210, 150, 228))
        c = kSfBlack;                                                   // erase top -> smirk
    return c;
}

// Wink face: left eye open, right eye a closed slit, playful brows and a smirk.
inline uint16_t SfWinkPixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInCircle(x, y, 82, 135, 38))
        c = kSfWhite;                                                   // left eye open
    if (SfInCircle(x, y, 82, 150, 15))
        c = kSfBlack;                                                   // left pupil
    if (SfInRect(x, y, 120, 133, 196, 138))
        c = kSfWhite;                                                   // right eye closed (slit)
    if (SfInRect(x, y, 50, 80, 114, 85))
        c = kSfBlack;                                                   // left brow raised
    if (SfInRect(x, y, 126, 88, 190, 93))
        c = kSfBlack;                                                   // right brow level
    if (SfInCircle(x, y, 128, 228, 18))
        c = kSfWhite;                                                   // smirk base (offset right)
    if (SfInRect(x, y, 98, 210, 158, 228))
        c = kSfBlack;                                                   // erase top -> smirk
    return c;
}


// Laugh / funny face: closed happy eyes (upward arcs) and a big open grin.
inline uint16_t SfLaughPixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInTri(x, y, 60, 148, 104, 148, 82, 128) ||               // ^ closed-happy left eye
        SfInTri(x, y, 136, 148, 180, 148, 158, 128))               // ^ closed-happy right eye
        c = kSfWhite;
    if (SfInRect(x, y, 50, 80, 114, 85) || SfInRect(x, y, 126, 80, 190, 85))
        c = kSfBlack;                                                   // raised (happy) brows
    if (SfInCircle(x, y, 120, 222, 28))
        c = kSfWhite;                                                   // grin base
    if (SfInRect(x, y, 84, 194, 156, 222))
        c = kSfBlack;                                                   // erase top -> open grin
    return c;
}

// Dead / dizzy face: "+" mark eyes and an open "O" mouth.
inline uint16_t SfDeadPixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInCircle(x, y, 82, 135, 30) || SfInCircle(x, y, 158, 135, 30))
        c = kSfWhite;                                                   // eye discs
    if (SfInRect(x, y, 76, 121, 88, 149) || SfInRect(x, y, 152, 121, 164, 149) ||
        SfInRect(x, y, 68, 129, 96, 141) || SfInRect(x, y, 144, 129, 172, 141))
        c = kSfBlack;                                                   // crossed bars -> "+" eyes
    if (SfInCircle(x, y, 120, 225, 22))
        c = kSfWhite;                                                   // open mouth outer
    if (SfInCircle(x, y, 120, 225, 13))
        c = kSfBlack;                                                   // hollow centre -> "O"
    return c;
}

// Star-struck face: sparkle (diamond) eyes and a big open grin.
inline uint16_t SfStarPixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInDiamond(x, y, 82, 135, 26) || SfInDiamond(x, y, 158, 135, 26))
        c = kSfWhite;                                                   // sparkle eyes
    if (SfInRect(x, y, 50, 80, 114, 85) || SfInRect(x, y, 126, 80, 190, 85))
        c = kSfBlack;                                                   // raised (happy) brows
    if (SfInCircle(x, y, 120, 222, 28))
        c = kSfWhite;                                                   // grin base
    if (SfInRect(x, y, 84, 194, 156, 222))
        c = kSfBlack;                                                   // erase top -> open grin
    return c;
}

// Worried / nervous face: uneven brows, a small mouth and a sweat drop.
inline uint16_t SfSweatPixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInCircle(x, y, 82, 135, 38) || SfInCircle(x, y, 158, 135, 38))
        c = kSfWhite;                                                   // eyes (same layout)
    if (SfInCircle(x, y, 82, 150, 15) || SfInCircle(x, y, 158, 150, 15))
        c = kSfBlack;                                                   // pupils
    if (SfInRect(x, y, 50, 80, 114, 85))
        c = kSfBlack;                                                   // left brow raised
    if (SfInRect(x, y, 126, 88, 190, 93))
        c = kSfBlack;                                                   // right brow level
    if (SfInRect(x, y, 104, 232, 136, 237))
        c = kSfWhite;                                                   // small worried mouth
    if (SfInCircle(x, y, 210, 108, 9))
        c = kSfWhite;                                                   // sweat drop (right temple)
    return c;
}

// Razz / tongue-out face: squinting eyes and a tongue poking below the mouth.
inline uint16_t SfTonguePixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInRect(x, y, 44, 133, 120, 138) || SfInRect(x, y, 120, 133, 196, 138))
        c = kSfWhite;                                                   // squinting eye slits
    if (SfInRect(x, y, 50, 80, 114, 85) || SfInRect(x, y, 126, 80, 190, 85))
        c = kSfBlack;                                                   // raised playful brows
    if (SfInRect(x, y, 96, 222, 144, 227))
        c = kSfWhite;                                                   // mouth line
    if (SfInRect(x, y, 104, 227, 136, 250))
        c = kSfWhite;                                                   // tongue poking out
    return c;
}

// Inverse rotation: screen pixel -> design pixel.
inline void SfScreenToDesign(int sx, int sy, int* dx, int* dy) {
    if (kSfRotate == 1) {
        *dx = kSfCx - (sy - kSfCy);
        *dy = kSfCy + (sx - kSfCx);
    } else if (kSfRotate == -1) {
        *dx = kSfCx + (sy - kSfCy);
        *dy = kSfCy - (sx - kSfCx);
    } else {
        *dx = sx;
        *dy = sy;
    }
}

}  // namespace

// =============================================================================
// WatchFaceLcdDisplay — face-only display.
//
// Renders the procedural watch faces (normal / sad / complete / scared / remind
// / sleep) into a full-screen lv_canvas. No chat bubbles, status text or emoji
// widgets — just the face, driven by SetEmotion(). LVGL owns the panel I/O,
// double-buffering and backlight; we only paint the canvas pixels.
// =============================================================================
class WatchFaceLcdDisplay : public SpiLcdDisplay {
public:
    using SpiLcdDisplay::SpiLcdDisplay;

    void SetupUI() override {
        // Idempotent: the boot face test calls SetupUI() early; Application calls
        // it again later. Skip if the canvas is already built.
        if (setup_ui_called_) return;
        // Skip LcdDisplay::SetupUI (chat widgets) — build only a full-screen canvas.
        Display::SetupUI();  // mark SetupUI as called
        DisplayLockGuard lock(this);

        lv_obj_t* scr = lv_screen_active();
        lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

        canvas_ = lv_canvas_create(scr);
        canvas_buf_ = (uint8_t*)heap_caps_malloc((size_t)kSfW * kSfH * sizeof(uint16_t),
                                                 MALLOC_CAP_SPIRAM);
        if (canvas_buf_ == nullptr) {
            ESP_LOGE(TAG, "face: no PSRAM for canvas buffer");
            return;
        }
        lv_canvas_set_buffer(canvas_, canvas_buf_, kSfW, kSfH, LV_COLOR_FORMAT_RGB565);
        lv_obj_set_pos(canvas_, 0, 0);
        lv_obj_clear_flag(canvas_, LV_OBJ_FLAG_SCROLLABLE);

        RenderFace(current_face_);  // paint the initial face
    }

    void SetEmotion(const char* emotion) override {
        current_face_ = FaceForEmotion(emotion);
        if (canvas_buf_ == nullptr) return;  // SetupUI() not done yet — painted there
        DisplayLockGuard lock(this);
        RenderFace(current_face_);
    }

    // Faces only — swallow the chat / status / notification UI calls so the base
    // implementations don't touch LVGL widgets that were never created.
    void SetStatus(const char* status) override {}
    void SetChatMessage(const char* role, const char* content) override {}
    void ShowNotification(const char* notification, int duration_ms) override {}
    void ShowNotification(const std::string& notification, int duration_ms) override {}
    void ClearChatMessages() override {}
    void SetPreviewImage(std::unique_ptr<LvglImage> image) override { (void)image; }
    void SetTheme(Theme* theme) override {}
    void UpdateStatusBar(bool update_all) override {}

private:
    enum Face { kNormal, kSad, kComplete, kScared, kRemind, kSleep,
                kAngry, kFlat, kLove, kCool, kWink,
                kLaugh, kDead, kStar, kSweat, kTongue };

    static Face FaceForEmotion(const char* emotion) {
        if (emotion == nullptr) return kNormal;
        if (strcmp(emotion, "happy") == 0 || strcmp(emotion, "laughing") == 0 ||
            strcmp(emotion, "smile") == 0 || strcmp(emotion, "excited") == 0)
            return kComplete;
        if (strcmp(emotion, "sad") == 0 || strcmp(emotion, "crying") == 0)
            return kSad;
        if (strcmp(emotion, "angry") == 0)
            return kAngry;
        if (strcmp(emotion, "surprised") == 0 || strcmp(emotion, "scared") == 0 ||
            strcmp(emotion, "shocked") == 0)
            return kScared;
        if (strcmp(emotion, "thinking") == 0 || strcmp(emotion, "confused") == 0 ||
            strcmp(emotion, "remind") == 0)
            return kRemind;
        if (strcmp(emotion, "sleeping") == 0 || strcmp(emotion, "sleepy") == 0)
            return kSleep;
        if (strcmp(emotion, "loving") == 0 || strcmp(emotion, "kissy") == 0)
            return kLove;
        if (strcmp(emotion, "cool") == 0 || strcmp(emotion, "confident") == 0)
            return kCool;
        if (strcmp(emotion, "winking") == 0 || strcmp(emotion, "silly") == 0)
            return kWink;
        if (strcmp(emotion, "embarrassed") == 0 || strcmp(emotion, "expressionless") == 0)
            return kFlat;
        if (strcmp(emotion, "funny") == 0)
            return kLaugh;
        if (strcmp(emotion, "dizzy") == 0)
            return kDead;
        if (strcmp(emotion, "star") == 0 || strcmp(emotion, "starstruck") == 0)
            return kStar;
        if (strcmp(emotion, "worried") == 0 || strcmp(emotion, "nervous") == 0)
            return kSweat;
        if (strcmp(emotion, "razz") == 0 || strcmp(emotion, "tongue") == 0)
            return kTongue;
        return kNormal;  // "neutral" + anything unknown
    }

    // Paint the whole canvas from the per-pixel face function (same screen→design
    // rotation the boot test uses) and invalidate so LVGL flushes it to the panel.
    void RenderFace(Face f) {
        if (canvas_buf_ == nullptr) return;
        uint16_t* buf = reinterpret_cast<uint16_t*>(canvas_buf_);
        for (int y = 0; y < kSfH; y++) {
            for (int x = 0; x < kSfW; x++) {
                int dx, dy;
                SfScreenToDesign(x, y, &dx, &dy);
                buf[y * kSfW + x] = PixelFor(f, dx, dy);
            }
        }
        lv_obj_invalidate(canvas_);
    }

    static uint16_t PixelFor(Face f, int dx, int dy) {
        switch (f) {
            case kComplete: return SfCompletePixel(dx, dy, 0);    // eyes open
            case kSad:      return SfSadPixel(dx, dy, 184, 7);    // static tear
            case kScared:   return SfScaredPixel(dx, dy, false);  // small "O" mouth
            case kRemind:   return SfRemindPixel(dx, dy, true);   // dot on
            case kSleep:    return SfSleepPixel(dx, dy, false);   // no Zzz
            case kAngry:    return SfAngryPixel(dx, dy);          // angry brows + frown
            case kFlat:     return SfFlatPixel(dx, dy);           // deadpan
            case kLove:     return SfLovePixel(dx, dy);           // heart eyes + grin
            case kCool:     return SfCoolPixel(dx, dy);           // sunglasses + smirk
            case kWink:     return SfWinkPixel(dx, dy);           // one eye closed + smirk
            case kLaugh:    return SfLaughPixel(dx, dy);          // closed happy eyes + grin
            case kDead:     return SfDeadPixel(dx, dy);           // "+" eyes + O mouth
            case kStar:     return SfStarPixel(dx, dy);           // sparkle eyes + grin
            case kSweat:    return SfSweatPixel(dx, dy);          // worried + sweat drop
            case kTongue:   return SfTonguePixel(dx, dy);         // tongue out
            case kNormal:
            default:        return SfNormalPixel(dx, dy, true);   // eyes open
        }
    }

    lv_obj_t* canvas_ = nullptr;
    uint8_t* canvas_buf_ = nullptr;
    Face current_face_ = kNormal;
};

class WatchInterfaceBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Pca9685* pca9685_ = nullptr;
    ClockManager* clock_manager_ = nullptr;

    static constexpr gpio_num_t kGreenLeds[GREEN_LED_COUNT] = {
        GREEN_LED_HOUR_0, GREEN_LED_HOUR_1, GREEN_LED_HOUR_2, GREEN_LED_HOUR_3,
        GREEN_LED_HOUR_4, GREEN_LED_HOUR_5, GREEN_LED_HOUR_6, GREEN_LED_HOUR_7,
    };
    static constexpr const char* kHourLabels[GREEN_LED_COUNT] = {
        "8am", "9am", "10am", "11am", "1pm", "2pm", "3pm", "4pm"
    };

    void InitializeI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = I2C_SDA_PIN,
            .scl_io_num = I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGI(TAG, "Install ILI9341 LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;

#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#else
#error "Unsupported LCD type"
#endif

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new WatchFaceLcdDisplay(panel_io, panel,
                                     DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                     DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);

#if WATCH_ENABLE_FACES_TEST
        // Boot face test rendered through LVGL (the canvas + flush path the
        // runtime uses), not a raw panel write. SetupUI() builds the canvas now;
        // the later Application::Initialize() call is a no-op (idempotent).
        display_->SetupUI();
        RunFaceTest();
#endif
    }

    // Boot face test rendered through LVGL: turn the backlight on and cycle every
    // face on the canvas via SetEmotion() (the same render path the runtime
    // uses). The LVGL port task, started in the SpiLcdDisplay constructor, flushes
    // each frame while we sleep between faces. Backlight is later handed to
    // PwmBacklight by GetBacklight()->RestoreBrightness().
    void RunFaceTest() {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            gpio_set_direction(DISPLAY_BACKLIGHT_PIN, GPIO_MODE_OUTPUT);
            gpio_set_level(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT ? 0 : 1);
        }
        static const char* kFaces[] = {
            "neutral", "happy", "laughing", "sad", "angry",
            "surprised", "thinking", "sleepy", "loving", "cool",
            "winking", "expressionless",
            "funny", "dizzy", "star", "worried", "razz",
        };
        ESP_LOGI(TAG, "face test (LVGL): cycling %zu faces",
                 sizeof(kFaces) / sizeof(kFaces[0]));
        for (const char* emotion : kFaces) {
            display_->SetEmotion(emotion);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        display_->SetEmotion("neutral");
    }

    void InitializePca9685() {
        /* Probe first — PCA9685 may not be connected yet */
        esp_err_t probe = i2c_master_probe(display_i2c_bus_, PCA9685_I2C_ADDR, 100);
        if (probe != ESP_OK) {
            ESP_LOGW(TAG, "PCA9685 not found at 0x%02X — skipping (not connected?)", PCA9685_I2C_ADDR);
            pca9685_ = nullptr;
            return;
        }
        pca9685_ = new Pca9685(display_i2c_bus_, PCA9685_I2C_ADDR);
        ESP_ERROR_CHECK(pca9685_->Init(50.0f));

        /* Red LEDs (CH8–CH15) are active-LOW: FullOn = output HIGH = LED off.
         * Init() sets all channels FullOff (output LOW = red ON), so override here. */
        for (int ch = PCA9685_LED_CH_FIRST; ch <= PCA9685_LED_CH_LAST; ch++) {
            pca9685_->SetFullOn(ch);
        }
        ESP_LOGI(TAG, "PCA9685 red LEDs forced off (active LOW)");

        /* Ensure all servos at 0° */
        for (int ch = PCA9685_SERVO_CH_FIRST; ch <= PCA9685_SERVO_CH_LAST; ch++) {
            pca9685_->SetServoAngle(ch, 0);
            pca9685_->SetFullOff(ch);
        }
        ESP_LOGI(TAG, "PCA9685 servos set to 0°");
    }

    void InitializeGreenLeds() {
        for (int i = 0; i < GREEN_LED_COUNT; i++) {
            gpio_config_t cfg = {
                .pin_bit_mask = (1ULL << kGreenLeds[i]),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&cfg);
            gpio_set_level(kGreenLeds[i], 1); // HIGH = off
        }
        ESP_LOGI(TAG, "Green LEDs initialized (active LOW)");
    }

    void InitializeLimitSwitch() {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << LIMIT_SWITCH_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
        ESP_LOGI(TAG, "Limit switch initialized on GPIO %d (LOW = pressed)", LIMIT_SWITCH_GPIO);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeTools() {
        // Create a separate WS2812 strip for MCP control
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = BUILTIN_LED_GPIO;
        strip_config.max_leds = 1;
        strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
        strip_config.led_model = LED_MODEL_WS2812;

        led_strip_rmt_config_t rmt_config = {};
        rmt_config.resolution_hz = 10 * 1000 * 1000;

        led_strip_handle_t mcp_strip = nullptr;
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &mcp_strip));

        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool("self.led.turn_on",
            "Turn on the LED light. Color can be red, green, blue, yellow, cyan, magenta, or white.",
            PropertyList({
                Property("color", kPropertyTypeString, "green")
            }),
            [mcp_strip](const PropertyList& props) -> ReturnValue {
                auto color = props["color"].value<std::string>();
                uint8_t r = 0, g = 0, b = 0;
                if (color == "red")         { r = 255; g = 0;   b = 0;   }
                else if (color == "green")  { r = 0;   g = 255; b = 0;   }
                else if (color == "blue")   { r = 0;   g = 0;   b = 255; }
                else if (color == "yellow") { r = 255; g = 255; b = 0;   }
                else if (color == "cyan")   { r = 0;   g = 255; b = 255; }
                else if (color == "magenta"){ r = 255; g = 0;   b = 255; }
                else if (color == "white")  { r = 255; g = 255; b = 255; }
                else                        { r = 0;   g = 255; b = 0;   }
                led_strip_set_pixel(mcp_strip, 0, r, g, b);
                led_strip_refresh(mcp_strip);
                return true;
            });

        mcp_server.AddTool("self.led.turn_off",
            "Turn off the LED light",
            PropertyList(),
            [mcp_strip](const PropertyList& props) -> ReturnValue {
                led_strip_clear(mcp_strip);
                return true;
            });
    }

public:
    WatchInterfaceBoard() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializePca9685();
        InitializeGreenLeds();
        InitializeLimitSwitch();
        clock_manager_ = new ClockManager(pca9685_);
        clock_manager_->Start();
        InitializeButtons();
        InitializeTools();

        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }

        // TODO: Remove after hardware verification
        // TestPca9685();
        // TestGreenLeds();
        // TestLimitSwitch();
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    Pca9685* GetPca9685() { return pca9685_; }

    void TestPca9685() {
        if (!pca9685_) {
            ESP_LOGW(TAG, "PCA9685 not available — test skipped");
            return;
        }
        ESP_LOGI(TAG, "=== PCA9685 Test Start ===");

        /* 1. Sweep each servo CH0–CH7: 0° → 90° → 0° */
        for (int ch = 0; ch <= 7; ch++) {
            ESP_LOGI(TAG, "Servo CH%d → 90°", ch);
            pca9685_->SetServoAngle(ch, 90);
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG, "Servo CH%d → 0°", ch);
            pca9685_->SetServoAngle(ch, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
            pca9685_->SetFullOff(ch);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* 2. Ramp each LED CH8–CH15: 0 → full brightness */
        for (int ch = 8; ch <= 15; ch++) {
            ESP_LOGI(TAG, "LED CH%d → ramp up", ch);
            for (int duty = 0; duty <= 4095; duty += 256) {
                pca9685_->SetDuty(ch, duty);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            pca9685_->SetFullOff(ch);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        /* 3. Combined: servo CH0 pop + LED CH8 on (simulates 1 hour alert) */
        ESP_LOGI(TAG, "Combined test: servo CH0 + LED CH8");
        pca9685_->SetServoAngle(0, 90);
        pca9685_->SetDuty(8, 4095);
        vTaskDelay(pdMS_TO_TICKS(2000));
        pca9685_->SetServoAngle(0, 0);
        pca9685_->SetFullOff(8);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "=== PCA9685 Test Done ===");
    }

    void TestGreenLeds() {
        ESP_LOGI(TAG, "=== Green LED Test Start ===");

        /* 1. Turn on each green LED one by one */
        for (int i = 0; i < GREEN_LED_COUNT; i++) {
            ESP_LOGI(TAG, "Green LED %d (%s) ON", i, kHourLabels[i]);
            gpio_set_level(kGreenLeds[i], 0); // LOW = on
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* 2. All off */
        ESP_LOGI(TAG, "All green LEDs OFF");
        for (int i = 0; i < GREEN_LED_COUNT; i++) {
            gpio_set_level(kGreenLeds[i], 1); // HIGH = off
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* 3. All on */
        ESP_LOGI(TAG, "All green LEDs ON");
        for (int i = 0; i < GREEN_LED_COUNT; i++) {
            gpio_set_level(kGreenLeds[i], 0); // LOW = on
        }
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 4. All off */
        for (int i = 0; i < GREEN_LED_COUNT; i++) {
            gpio_set_level(kGreenLeds[i], 1);
        }

        ESP_LOGI(TAG, "=== Green LED Test Done ===");
    }

    void TestLimitSwitch() {
        ESP_LOGI(TAG, "=== Limit Switch Test Start ===");
        ESP_LOGI(TAG, "Press the switch (GPIO %d)... will check for 15 seconds", LIMIT_SWITCH_GPIO);

        bool last_pressed = false;
        for (int i = 0; i < 150; i++) { // 15 sec, check every 100ms
            bool pressed = (gpio_get_level(LIMIT_SWITCH_GPIO) == 0); // LOW = pressed
            if (pressed != last_pressed) {
                ESP_LOGI(TAG, "Switch %s", pressed ? "PRESSED" : "RELEASED");
                last_pressed = pressed;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ESP_LOGI(TAG, "=== Limit Switch Test Done ===");
    }
};

DECLARE_BOARD(WatchInterfaceBoard);
