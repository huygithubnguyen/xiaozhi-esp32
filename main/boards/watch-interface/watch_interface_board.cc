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

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#define TAG "WatchInterfaceBoard"

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
