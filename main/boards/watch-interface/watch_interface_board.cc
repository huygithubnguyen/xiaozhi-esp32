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
#include <esp_heap_caps.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#define TAG "WatchInterfaceBoard"

// =============================================================================
// Sad-face boot test (ILI9341 240x320). Plays a 3-frame tear-drop animation
// directly on the panel, before LVGL takes over the display.
// Set to 0 to disable once the LCD is verified.
// =============================================================================
#define WATCH_ENABLE_FACES_TEST 1

#if WATCH_ENABLE_FACES_TEST
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

// Complete face = happy expression: same eyes/pupils as the other faces, but
// raised brows and a big open smile, so it is distinct from normal and sad.
inline uint16_t SfCompletePixel(int x, int y) {
    uint16_t c = kSfBlack;
    if (SfInCircle(x, y, 82, 135, 38) || SfInCircle(x, y, 158, 135, 38))
        c = kSfWhite;                                                  // eyes (same layout)
    if (SfInCircle(x, y, 82, 150, 15) || SfInCircle(x, y, 158, 150, 15))
        c = kSfBlack;                                                  // pupils (same layout)
    if (SfInRect(x, y, 50, 80, 114, 85) || SfInRect(x, y, 126, 80, 190, 85))
        c = kSfBlack;                                                  // raised (happy) brows
    if (SfInCircle(x, y, 120, 222, 28))
        c = kSfWhite;                                                  // smile base
    if (SfInRect(x, y, 84, 194, 156, 222))
        c = kSfBlack;                                                  // erase top half -> open grin
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

// Forward rotation: design point -> screen point.
inline void SfDesignToScreen(int dx, int dy, int* sx, int* sy) {
    if (kSfRotate == 1) {
        *sx = kSfCx + (dy - kSfCy);
        *sy = kSfCy - (dx - kSfCx);
    } else if (kSfRotate == -1) {
        *sx = kSfCx - (dy - kSfCy);
        *sy = kSfCy + (dx - kSfCx);
    } else {
        *sx = dx;
        *sy = dy;
    }
}

// Screen-space bounding box of a design rectangle under the current rotation,
// expanded by `margin`.
inline void SfDesignRectToScreen(int dx0, int dy0, int dx1, int dy1, int margin,
                                 int* sx0, int* sy0, int* sx1, int* sy1) {
    int xs[4], ys[4];
    SfDesignToScreen(dx0, dy0, &xs[0], &ys[0]);
    SfDesignToScreen(dx1, dy0, &xs[1], &ys[1]);
    SfDesignToScreen(dx0, dy1, &xs[2], &ys[2]);
    SfDesignToScreen(dx1, dy1, &xs[3], &ys[3]);
    int minx = xs[0], maxx = xs[0], miny = ys[0], maxy = ys[0];
    for (int i = 1; i < 4; i++) {
        if (xs[i] < minx) minx = xs[i];
        if (xs[i] > maxx) maxx = xs[i];
        if (ys[i] < miny) miny = ys[i];
        if (ys[i] > maxy) maxy = ys[i];
    }
    *sx0 = minx - margin;
    *sy0 = miny - margin;
    *sx1 = maxx + margin + 1;
    *sy1 = maxy + margin + 1;
}

// Paint the whole frame into fb (screen space) and send it in one transfer.
// Used only for the one-time appearance of each face.
template <class PixelFn>
void SfBlitFull(uint16_t* fb, esp_lcd_panel_handle_t panel, PixelFn pixel) {
    for (int y = 0; y < kSfH; y++) {
        for (int x = 0; x < kSfW; x++) {
            int dx, dy;
            SfScreenToDesign(x, y, &dx, &dy);
            fb[y * kSfW + x] = pixel(dx, dy);
        }
    }
    esp_lcd_panel_draw_bitmap(panel, 0, 0, kSfW, kSfH, fb);
}

// Paint only a screen rectangle into buf (packed) and send just that rectangle.
// Used for small dirty updates so the rest of the screen is never rewritten ->
// no flash/tearing between frames.
template <class PixelFn>
void SfBlitRect(uint16_t* buf, esp_lcd_panel_handle_t panel,
                int sx0, int sy0, int sx1, int sy1, PixelFn pixel) {
    if (sx0 < 0) sx0 = 0;
    if (sy0 < 0) sy0 = 0;
    if (sx1 > kSfW) sx1 = kSfW;
    if (sy1 > kSfH) sy1 = kSfH;
    int w = sx1 - sx0;
    int h = sy1 - sy0;
    if (w <= 0 || h <= 0) return;
    for (int row = 0; row < h; row++) {
        int sy = sy0 + row;
        for (int sx = sx0; sx < sx1; sx++) {
            int dx, dy;
            SfScreenToDesign(sx, sy, &dx, &dy);
            buf[row * w + (sx - sx0)] = pixel(dx, dy);
        }
    }
    esp_lcd_panel_draw_bitmap(panel, sx0, sy0, sx1, sy1, buf);
}

// Shows both faces at boot (normal with a blink, then the sad tear animation),
// before LVGL takes over the panel. Each face is drawn once in full; afterwards
// only the changing region (eyes for the blink, tear for the drop) is updated,
// so the screen does not flash between frames.
void RunFaceTest(esp_lcd_panel_handle_t panel) {
    // Backlight is normally enabled later in Start(); turn it on now so the
    // boot animation is actually visible while it plays.
    if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
        gpio_set_direction(DISPLAY_BACKLIGHT_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT ? 0 : 1);
    }
    esp_lcd_panel_disp_on_off(panel, true);

    uint16_t* fb = (uint16_t*)heap_caps_malloc((size_t)kSfW * kSfH * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!fb) {
        ESP_LOGW(TAG, "face: no DMA RAM for framebuffer, skipping face test");
        return;
    }

    // Complete face: draw once and hold.
    ESP_LOGI(TAG, "face: complete");
    SfBlitFull(fb, panel, [](int x, int y) { return SfCompletePixel(x, y); });
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Screen rectangle covering both eyes (open or closed): the only region
    // that changes during a blink.
    int ex0, ey0, ex1, ey1;
    SfDesignRectToScreen(42, 95, 198, 175, 2, &ex0, &ey0, &ex1, &ey1);

    // Normal face: full draw (eyes open) once, then blink only the eye region.
    ESP_LOGI(TAG, "face: normal (blinking)");
    SfBlitFull(fb, panel, [](int x, int y) { return SfNormalPixel(x, y, true); });
    for (int b = 0; b < 3; b++) {
        vTaskDelay(pdMS_TO_TICKS(800));
        SfBlitRect(fb, panel, ex0, ey0, ex1, ey1,
                   [](int x, int y) { return SfNormalPixel(x, y, false); });  // close
        vTaskDelay(pdMS_TO_TICKS(120));
        SfBlitRect(fb, panel, ex0, ey0, ex1, ey1,
                   [](int x, int y) { return SfNormalPixel(x, y, true); });   // open
    }
    vTaskDelay(pdMS_TO_TICKS(700));

    // Sad face: full draw (tear at frame 0) once, then only the tear's region
    // is updated each frame.
    const struct {
        int y, r;
    } frames[3] = {{184, 7}, {210, 9}, {236, 10}};
    ESP_LOGI(TAG, "face: sad (tear animation)");
    SfBlitFull(fb, panel, [&](int x, int y) { return SfSadPixel(x, y, frames[0].y, frames[0].r); });

    int shown_ty = frames[0].y, shown_tr = frames[0].r;
    for (int loop = 0; loop < 3; loop++) {
        for (int f = 0; f < 3; f++) {
            int ty = frames[f].y, tr = frames[f].r;
            if (ty != shown_ty || tr != shown_tr) {
                int px0, py0, px1, py1, cx0, cy0, cx1, cy1;
                SfDesignRectToScreen(158 - shown_tr, shown_ty - shown_tr, 158 + shown_tr,
                                     shown_ty + shown_tr, 1, &px0, &py0, &px1, &py1);
                SfDesignRectToScreen(158 - tr, ty - tr, 158 + tr, ty + tr, 1,
                                     &cx0, &cy0, &cx1, &cy1);
                int sx0 = px0 < cx0 ? px0 : cx0;
                int sy0 = py0 < cy0 ? py0 : cy0;
                int sx1 = px1 > cx1 ? px1 : cx1;
                int sy1 = py1 > cy1 ? py1 : cy1;
                SfBlitRect(fb, panel, sx0, sy0, sx1, sy1,
                           [ty, tr](int x, int y) { return SfSadPixel(x, y, ty, tr); });
                shown_ty = ty;
                shown_tr = tr;
            }
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

    heap_caps_free(fb);
}
}  // namespace
#endif  // WATCH_ENABLE_FACES_TEST

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

#if WATCH_ENABLE_FACES_TEST
        // Show both boot faces before LVGL takes over the panel.
        RunFaceTest(panel);
#endif

        display_ = new SpiLcdDisplay(panel_io, panel,
                                     DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                     DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
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
