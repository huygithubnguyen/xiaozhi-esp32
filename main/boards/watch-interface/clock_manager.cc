#include "clock_manager.h"
#include "application.h"
#include "assets/lang_config.h"

#include <esp_timer.h>
#include <string>

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
    static constexpr const char* messages[CLOCK_HOUR_COUNT] = {
        "It's 8 AM. Time to start working!",
        "It's 9 AM. Stay focused!",
        "It's 10 AM. Keep it up!",
        "It's 11 AM. Almost lunch time!",
        "It's 1 PM. Back to work!",
        "It's 2 PM. Stay productive!",
        "It's 3 PM. Keep going!",
        "It's 4 PM. Last hour!",
    };

    for (int i = 0; i < CLOCK_HOUR_COUNT; i++) {
        hours_[i] = {
            .servo_channel = static_cast<uint8_t>(PCA9685_SERVO_CH_FIRST + i),
            .green_led     = leds[i],
            .label         = labels[i],
            .message       = messages[i],
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
    ESP_LOGI(TAG, "started — dev interval %d s, alarm repeat %d s",
             kDevIntervalMs / 1000, kAlarmRepeatMs / 1000);
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
            int64_t t0 = esp_timer_get_time();

            /* ── Alarm 1: activate servo + LED + sound ── */
            ESP_LOGI(TAG, "[%s] alarm 1", hours_[i].label);
            ActivateHour(i);

            bool acked = false;

            /* ── Wait for ack (alarm repeat interval) ── */
            acked = WaitForAck(i, kAlarmRepeatMs, t0);

            /* ── Alarm 2 if not acknowledged ── */
            if (!acked) {
                ESP_LOGI(TAG, "[%s] alarm 2 (repeat)", hours_[i].label);
                PlayAlarm(i);

                acked = WaitForAck(i, kAlarmRepeatMs, t0);
            }

            /* ── Auto-acknowledge if still active ── */
            if (hours_[i].state == HourState::Active) {
                ESP_LOGW(TAG, "[%s] no ack after 2 alarms — auto-acknowledged", hours_[i].label);
                AcknowledgeHour(i);
            }

            /* ── Wait remainder of the 7-min dev interval ── */
            int64_t elapsed_us = esp_timer_get_time() - t0;
            int remaining = kDevIntervalMs - static_cast<int>(elapsed_us / 1000);
            if (remaining > 0) {
                ESP_LOGI(TAG, "[%s] idle — next hour in %d s", hours_[i].label, remaining / 1000);
                int wait_ms = 0;
                while (wait_ms < remaining) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    wait_ms += 1000;
                    if (wait_ms % 60 == 0) {
                        ESP_LOGI(TAG, "[%s] next hour in %d s",
                                 hours_[i].label, (remaining - wait_ms) / 1000);
                    }
                }
            }
        }

        /* --- End of cycle: reset everything --- */
        ESP_LOGI(TAG, "=== cycle done (%d hours) — resetting, restart in 5 s ===",
                 CLOCK_HOUR_COUNT);
        ResetAll();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

bool ClockManager::WaitForAck(int index, int timeout_ms, int64_t t0)
{
    int elapsed_ms = 0;
    while (elapsed_ms < timeout_ms) {
        if (IsSwitchPressed()) {
            int64_t ack_ms = (esp_timer_get_time() - t0) / 1000;
            ESP_LOGI(TAG, "[%s] switch pressed after %.1f s — acknowledged",
                     hours_[index].label, ack_ms / 1000.0);
            AcknowledgeHour(index);
            return true;
        }
        int sec = elapsed_ms / 1000;
        if (sec > 0 && sec % 30 == 0 && sec != (elapsed_ms - kSwitchPollMs) / 1000) {
            ESP_LOGI(TAG, "[%s] waiting for ack... %d / %d s",
                     hours_[index].label, sec, timeout_ms / 1000);
        }
        vTaskDelay(pdMS_TO_TICKS(kSwitchPollMs));
        elapsed_ms += kSwitchPollMs;
    }
    return false;
}

void ClockManager::PlayAlarm(int index)
{
    auto& h = hours_[index];
    auto& app = Application::GetInstance();
    app.Schedule([label = std::string(h.label), msg = std::string(h.message)]() {
        Application::GetInstance().Alert(label.c_str(), msg.c_str(), "alarm", Lang::Sounds::OGG_POPUP);
    });
}

/* ── Hour actions ──────────────────────────────────── */

void ClockManager::ActivateHour(int index)
{
    auto& h = hours_[index];
    ESP_LOGI(TAG, "[%s] activating servo CH%d + green LED + sound", h.label, h.servo_channel);

    /* Pop servo up to 90° */
    if (pca9685_ != nullptr) {
        pca9685_->SetServoAngle(h.servo_channel, 90);
    }

    /* Turn on green LED (active LOW) */
    gpio_set_level(h.green_led, 0);

    /* Play notification sound + show alert on OLED (via main task) */
    auto& app = Application::GetInstance();
    app.Schedule([label = std::string(h.label), msg = std::string(h.message)]() {
        Application::GetInstance().Alert(label.c_str(), msg.c_str(), "alarm", Lang::Sounds::OGG_POPUP);
    });

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

    /* Dismiss the alert on display (via main task) */
    Application::GetInstance().Schedule([]() {
        Application::GetInstance().DismissAlert();
    });

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
    Application::GetInstance().Schedule([]() {
        Application::GetInstance().DismissAlert();
    });
}

/* ── Limit switch ──────────────────────────────────── */

bool ClockManager::IsSwitchPressed() const
{
    return gpio_get_level(LIMIT_SWITCH_GPIO) == 0;  // LOW = pressed
}
