#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
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

#if defined(LCD_TYPE_SH1106)
#include "esp_lcd_panel_sh1106.h"
#endif

#define TAG "WatchInterfaceBoard"

// =============================================================================
// Boot face test (SH1106 OLED 128x64). Cycles faces through the OLED display
// at boot to verify the display is working.
// Set to 0 to disable once the OLED is verified.
// =============================================================================
#define WATCH_ENABLE_FACES_TEST 0

// =============================================================================
// WatchInterfaceBoard — Smart Clock Board with SH1106 OLED
// =============================================================================

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

    void InitializeOledDisplay() {
        ESP_LOGI(TAG, "Install SH1106 OLED panel IO via I2C");

        // Probe I2C bus for SH1106 display
        // Note: 0x78 (8-bit) = 0x3C (7-bit). ESP-IDF uses 7-bit addressing.
        bool display_found = false;
        uint8_t display_addr = 0x3C;  // SH1106 default address (7-bit format)

        esp_err_t probe = i2c_master_probe(display_i2c_bus_, display_addr, 100);
        if (probe == ESP_OK) {
            ESP_LOGI(TAG, "SH1106 display found at I2C address 0x%02X", display_addr);
            display_found = true;
        } else {
            ESP_LOGW(TAG, "SH1106 display not found at 0x%02X - check wiring!", display_addr);
            ESP_LOGW(TAG, "Continuing anyway - display may still work...");
        }

        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = display_addr,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
            .scl_speed_hz = 100 * 1000,  // Lower speed for better compatibility
        };

        ESP_LOGI(TAG, "Creating panel IO...");
        esp_err_t ret = esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &panel_io_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create panel IO: 0x%x (%s)", ret, esp_err_to_name(ret));
            return;
        }
        ESP_LOGI(TAG, "Panel IO created");

        ESP_LOGI(TAG, "Install SH1106 OLED driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;
        panel_config.bits_per_pixel = 1;

#if defined(LCD_TYPE_SH1106)
        ESP_LOGI(TAG, "Creating SH1106 panel...");
        ret = esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create SH1106 panel: 0x%x (%s)", ret, esp_err_to_name(ret));
            return;
        }
        ESP_LOGI(TAG, "SH1106 panel created");
#else
#error "Unsupported OLED type"
#endif

        ESP_LOGI(TAG, "Resetting panel...");
        ret = esp_lcd_panel_reset(panel_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Panel reset failed: 0x%x (%s)", ret, esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay after reset

        ESP_LOGI(TAG, "Initializing panel...");
        ret = esp_lcd_panel_init(panel_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Panel init failed: 0x%x (%s)", ret, esp_err_to_name(ret));
            return;
        }
        ESP_LOGI(TAG, "Panel initialized");
        vTaskDelay(pdMS_TO_TICKS(50));  // Delay after init

        ESP_LOGI(TAG, "Turning display ON...");
        ret = esp_lcd_panel_disp_on_off(panel_, true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to turn display on: 0x%x (%s)", ret, esp_err_to_name(ret));
        }

        ESP_LOGI(TAG, "SH1106 OLED driver installed");

        ESP_LOGI(TAG, "Creating OledDisplay object...");
        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                    DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        ESP_LOGI(TAG, "OledDisplay created");

#if WATCH_ENABLE_FACES_TEST
        // Boot face test rendered through LVGL
        ESP_LOGI(TAG, "Setting up UI...");
        display_->SetupUI();
        ESP_LOGI(TAG, "Running face test...");
        RunFaceTest();
        ESP_LOGI(TAG, "Face test done");
#endif
    }

    // Boot face test rendered through LVGL: cycle every face on the canvas via
    // SetEmotion() (the same render path the runtime uses). The LVGL port task,
    // started in the OledDisplay constructor, flushes each frame while we sleep
    // between faces.
    void RunFaceTest() {
        static const char* kFaces[] = {
            "neutral", "happy", "laughing", "sad", "angry",
            "surprised", "thinking", "sleepy", "loving", "cool",
            "winking", "expressionless",
            "funny", "dizzy", "star", "worried", "razz",
            "grin", "unamused", "pensive", "disappointed",
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
        InitializeOledDisplay();
        InitializePca9685();
        InitializeGreenLeds();
        InitializeLimitSwitch();
        clock_manager_ = new ClockManager(pca9685_);
        clock_manager_->Start();
        InitializeButtons();
        InitializeTools();

        // TODO: Remove after hardware verification
        // TestPca9685();
        // TestGreenLeds();
        // TestLimitSwitch();
        // TestOledDisplay();
    }

    virtual Backlight* GetBacklight() override {
        // OLED displays don't require backlight control
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

    void TestOledDisplay() {
        ESP_LOGI(TAG, "=== SH1106 OLED Display Test Start ===");

        if (!display_ || !panel_) {
            ESP_LOGW(TAG, "Display not available — test skipped");
            return;
        }

        /* 1. Display ON/OFF toggle test */
        ESP_LOGI(TAG, "Test 1: Display ON/OFF");
        for (int i = 0; i < 3; i++) {
            esp_lcd_panel_disp_on_off(panel_, false);
            ESP_LOGI(TAG, "  Display OFF");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_lcd_panel_disp_on_off(panel_, true);
            ESP_LOGI(TAG, "  Display ON");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        /* 2. Display clear and basic pattern test */
        ESP_LOGI(TAG, "Test 2: Clear display");
        display_->SetEmotion("");
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* 3. Display pattern test using emotions */
        ESP_LOGI(TAG, "Test 3: Display emotion patterns");
        static const char* kTestFaces[] = {
            "neutral", "happy", "sad", "angry", "surprised", "cool"
        };

        for (const char* emotion : kTestFaces) {
            ESP_LOGI(TAG, "  Displaying: %s", emotion);
            display_->SetEmotion(emotion);
            vTaskDelay(pdMS_TO_TICKS(1500));
        }

        /* 4. Status text test */
        ESP_LOGI(TAG, "Test 4: Status text display");
        display_->SetStatus("TEST OK");
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 5. Notification test */
        ESP_LOGI(TAG, "Test 5: Notification display");
        display_->ShowNotification("OLED OK");
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 6. Restore neutral face */
        ESP_LOGI(TAG, "Test 6: Restore neutral face");
        display_->SetEmotion("neutral");
        display_->SetStatus("");

        ESP_LOGI(TAG, "=== SH1106 OLED Display Test Done ===");
    }
};

DECLARE_BOARD(WatchInterfaceBoard);
