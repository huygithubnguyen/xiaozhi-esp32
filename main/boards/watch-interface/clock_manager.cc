#include "clock_manager.h"

#define TAG "ClockManager"

/* ── Construction ──────────────────────────────────── */

ClockManager::ClockManager(Pca9685* pca9685)
    : pca9685_(pca9685)
{
    static constexpr gpio_num_t leds[CLOCK_HOUR_COUNT] = {
        GREEN_LED_HOUR_0, GREEN_LED_HOUR_1, GREEN_LED_HOUR_2, GREEN_LED_HOUR_3,
        GREEN_LED_HOUR_4, GREEN_LED_HOUR_5, GREEN_LED_HOUR_6, GREEN_LED_HOUR_7,
    };
    static constexpr const char* labels[CLOCK_HOUR_COUNT] = {
        "8am", "9am", "10am", "11am", "1pm", "2pm", "3pm", "4pm",
    };

    for (int i = 0; i < CLOCK_HOUR_COUNT; i++) {
        hours_[i] = {
            .servo_channel = static_cast<uint8_t>(PCA9685_SERVO_CH_FIRST + i),
            .green_led     = leds[i],
            .label         = labels[i],
            .state         = HourState::Idle,
        };
    }
}

ClockManager::~ClockManager()
{
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
    }
}

/* ── Background task ───────────────────────────────── */

void ClockManager::Start()
{
    xTaskCreate(
        TaskEntry,
        "clock_mgr",
        4096,          // stack size
        this,          // arg
        2,             // priority (low)
        &task_handle_
    );
    ESP_LOGI(TAG, "started — dev interval %d min", kDevIntervalMs / 60000);
}

/* static */
void ClockManager::TaskEntry(void* arg)
{
    static_cast<ClockManager*>(arg)->RunLoop();
}

void ClockManager::RunLoop()
{
    while (true) {
        ESP_LOGI(TAG, "=== new cycle ===");

        for (int i = 0; i < CLOCK_HOUR_COUNT; i++) {
            /* --- Activate this hour --- */
            ActivateHour(i);

            /* --- Wait for acknowledgment or timeout --- */
            int elapsed_ms = 0;
            while (elapsed_ms < kAckTimeoutMs) {
                if (IsSwitchPressed()) {
                    ESP_LOGI(TAG, "[%s] switch pressed — acknowledged", hours_[i].label);
                    AcknowledgeHour(i);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(kSwitchPollMs));
                elapsed_ms += kSwitchPollMs;
            }

            /* Timeout: auto-acknowledge */
            if (hours_[i].state == HourState::Active) {
                ESP_LOGW(TAG, "[%s] ack timeout — auto-acknowledged", hours_[i].label);
                AcknowledgeHour(i);
            }

            /* --- Wait remainder of the 7-min interval --- */
            int remaining = kDevIntervalMs - elapsed_ms;
            if (remaining > 0) {
                ESP_LOGI(TAG, "[%s] next hour in %d s", hours_[i].label, remaining / 1000);
                vTaskDelay(pdMS_TO_TICKS(remaining));
            }
        }

        /* --- End of cycle: reset everything --- */
        ESP_LOGI(TAG, "=== cycle done — resetting ===");
        ResetAll();
        vTaskDelay(pdMS_TO_TICKS(5000));  // 5 s pause before next cycle
    }
}

/* ── Hour actions ──────────────────────────────────── */

void ClockManager::ActivateHour(int index)
{
    auto& h = hours_[index];
    ESP_LOGI(TAG, "[%s] activating servo CH%d + green LED", h.label, h.servo_channel);

    /* Pop servo up to 90° */
    if (pca9685_ != nullptr) {
        pca9685_->SetServoAngle(h.servo_channel, 90);
    }

    /* Turn on green LED (active LOW) */
    gpio_set_level(h.green_led, 0);

    h.state = HourState::Active;
}

void ClockManager::AcknowledgeHour(int index)
{
    auto& h = hours_[index];

    /* Retract servo to 0° */
    if (pca9685_ != nullptr) {
        pca9685_->SetServoAngle(h.servo_channel, 0);
        pca9685_->SetFullOff(h.servo_channel);
    }

    /* Green LED stays on (acknowledged indicator) */
    h.state = HourState::Acknowledged;
}

void ClockManager::ResetAll()
{
    for (int i = 0; i < CLOCK_HOUR_COUNT; i++) {
        /* Retract servo */
        if (pca9685_ != nullptr) {
            pca9685_->SetServoAngle(hours_[i].servo_channel, 0);
            pca9685_->SetFullOff(hours_[i].servo_channel);
        }
        /* Turn off green LED */
        gpio_set_level(hours_[i].green_led, 1);
        hours_[i].state = HourState::Idle;
    }
}

/* ── Limit switch ──────────────────────────────────── */

bool ClockManager::IsSwitchPressed() const
{
    return gpio_get_level(LIMIT_SWITCH_GPIO) == 0;  // LOW = pressed
}
