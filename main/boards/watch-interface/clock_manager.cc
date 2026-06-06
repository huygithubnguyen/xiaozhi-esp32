#include "clock_manager.h"

#include <esp_timer.h>

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
    BaseType_t ret = xTaskCreatePinnedToCore(
        TaskEntry,
        "clock_mgr",
        4096,          // stack size
        this,          // arg
        2,             // priority (low)
        &task_handle_,
        1              // pin to core 1 (app runs on core 0)
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create task (ret=%d)", ret);
        return;
    }
    ESP_LOGI(TAG, "started — dev interval %d s, ack timeout %d s",
             kDevIntervalMs / 1000, kAckTimeoutMs / 1000);
}

/* static */
void ClockManager::TaskEntry(void* arg)
{
    static_cast<ClockManager*>(arg)->RunLoop();
}

void ClockManager::RunLoop()
{
    ESP_LOGI(TAG, ">>> RunLoop entered on core %d <<<", xPortGetCoreID());

    while (true) {
        ESP_LOGI(TAG, "=== new cycle ===");

        for (int i = 0; i < CLOCK_HOUR_COUNT; i++) {
            int64_t t0 = esp_timer_get_time();  // µs

            /* --- Activate this hour --- */
            ActivateHour(i);

            /* --- Wait for acknowledgment or timeout --- */
            int elapsed_ms = 0;
            int last_log_sec = -1;
            while (elapsed_ms < kAckTimeoutMs) {
                if (IsSwitchPressed()) {
                    int64_t ack_ms = (esp_timer_get_time() - t0) / 1000;
                    ESP_LOGI(TAG, "[%s] switch pressed after %.1f s — acknowledged",
                             hours_[i].label, ack_ms / 1000.0);
                    AcknowledgeHour(i);
                    break;
                }
                int sec = elapsed_ms / 1000;
                if (sec != last_log_sec && sec % 30 == 0) {
                    ESP_LOGI(TAG, "[%s] waiting for ack... %d / %d s",
                             hours_[i].label, sec, kAckTimeoutMs / 1000);
                    last_log_sec = sec;
                }
                vTaskDelay(pdMS_TO_TICKS(kSwitchPollMs));
                elapsed_ms += kSwitchPollMs;
            }

            /* Timeout: auto-acknowledge */
            if (hours_[i].state == HourState::Active) {
                ESP_LOGW(TAG, "[%s] ack timeout (%d s) — auto-acknowledged",
                         hours_[i].label, kAckTimeoutMs / 1000);
                AcknowledgeHour(i);
            }

            /* --- Wait remainder of the dev interval --- */
            int remaining = kDevIntervalMs - elapsed_ms;
            if (remaining > 0) {
#if 1   // Enable logging
                ESP_LOGI(TAG, "[%s] done — next hour in %d s",
                         hours_[i].label, remaining / 1000);
                /* Log every 60 s while waiting */
                int wait_ms = 0;
                last_log_sec = -1;
                while (wait_ms < remaining) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    wait_ms += 1000;
                    int sec = wait_ms / 1000;
                    if (sec != last_log_sec && sec % 60 == 0) {
                        ESP_LOGI(TAG, "[%s] next hour in %d s",
                                 hours_[i].label, (remaining - wait_ms) / 1000);
                        last_log_sec = sec;
                    }
                }
#else
                vTaskDelay(pdMS_TO_TICKS(remaining));
#endif
            }
        }

        /* --- End of cycle: reset everything --- */
        ESP_LOGI(TAG, "=== cycle done (%d hours) — resetting, restart in 5 s ===",
                 CLOCK_HOUR_COUNT);
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
