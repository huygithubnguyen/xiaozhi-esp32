#include "clock_manager.h"
#include "application.h"
#include "board.h"
#include "display.h"
#include "assets/lang_config.h"

#include <esp_timer.h>
#include <string>
#include <ctime>

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
    static constexpr int hours24[CLOCK_HOUR_COUNT] = {
        8, 9, 10, 11, 13, 14, 15, 16,
    };

    for (int i = 0; i < CLOCK_HOUR_COUNT; i++) {
        hours_[i] = {
            .servo_channel   = static_cast<uint8_t>(PCA9685_SERVO_CH_FIRST + i),
            .red_led_channel = static_cast<uint8_t>(PCA9685_LED_CH_FIRST + i),
            .green_led       = leds[i],
            .label           = labels[i],
            .message         = messages[i],
            .hour_24         = hours24[i],
            .state           = HourState::Idle,
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
        TaskEntry, "clock_mgr", 4096, this, 2, &task_handle_, 1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create task (ret=%d)", ret);
        return;
    }
    ESP_LOGI(TAG, "started — %s, alarm repeat %d s, max alarms %d",
#if CLOCK_DEV_MODE
             "DEV 7-min cycle",
#else
             "PRODUCTION real-time",
#endif
             kAlarmRepeatMs / 1000, kMaxAlarms);
}

/* static */
void ClockManager::TaskEntry(void* arg)
{
    static_cast<ClockManager*>(arg)->RunLoop();
}

/* ── Time helpers ──────────────────────────────────── */

void ClockManager::WaitForTimeSync()
{
    setenv("TZ", "UTC0", 1);
    tzset();

    ESP_LOGI(TAG, "waiting for time sync...");
    while (time(nullptr) < 1700000000) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    time_t now = time(nullptr);
    struct tm t;

#if 0
    localtime_r(&now, &t);
#else
    // DEV: fake time at 8:59 AM today (so first trigger is 9am)
    localtime_r(&now, &t);
    t.tm_hour = 7;
    t.tm_min  = 59;
    t.tm_sec  = 0;
    time_t fake_now = mktime(&t);
    struct timeval tv = { .tv_sec = fake_now, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    now = fake_now;
    localtime_r(&now, &t);
#endif
    ESP_LOGI(TAG, "time synced — %04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
}

int ClockManager::FindNextHourIndex() const
{
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    int current_hm = t.tm_hour * 60 + t.tm_min;

    for (int i = 0; i < CLOCK_HOUR_COUNT; i++) {
        if (current_hm < hours_[i].hour_24 * 60) {
            return i;
        }
    }
    return -1;
}

int ClockManager::SecondsUntilHour(int index) const
{
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);

    struct tm target = t;
    target.tm_hour = hours_[index].hour_24;
    target.tm_min  = 0;
    target.tm_sec  = 0;

    return static_cast<int>(difftime(mktime(&target), now));
}

/* Sleep with periodic log */
void ClockManager::SleepLogged(int seconds, const char* label)
{
    if (seconds <= 0) return;
    ESP_LOGI(TAG, "[%s] sleeping %d s", label, seconds);

    int waited = 0;
    while (waited < seconds) {
        int chunk = (seconds - waited > 60) ? 60 : (seconds - waited);
        vTaskDelay(pdMS_TO_TICKS(chunk * 1000));
        waited += chunk;
        int left = seconds - waited;
        if (left > 0 && left % 60 == 0) {
            ESP_LOGI(TAG, "[%s] %d s remaining", label, left);
        }
    }
}

/* ── Alarm logic (shared) ──────────────────────────── */

void ClockManager::TriggerHourWithAlarms(int index)
{
    int64_t t0 = esp_timer_get_time();

    for (int alarm = 0; alarm < kMaxAlarms; alarm++) {
        if (alarm == 0) {
            ActivateHour(index);
        } else {
            ESP_LOGI(TAG, "[%s] alarm %d/%d (repeat)",
                     hours_[index].label, alarm + 1, kMaxAlarms);
            PlayAlarm(index);
        }

        if (WaitForAck(index, kAlarmRepeatMs, t0)) {
            return;  // acknowledged
        }
    }

    /* Not acknowledged after all alarms */
    if (hours_[index].state == HourState::Active) {
        MissHour(index);
    }
}

/* ── Main loop (one flow for dev & production) ─────── */

void ClockManager::RunLoop()
{
    ESP_LOGI(TAG, ">>> RunLoop entered on core %d <<<", xPortGetCoreID());
    WaitForTimeSync();

    while (true) {
        /* Reset all LEDs/servos clean before starting a new cycle */
        ResetAll();

        /* Find next work hour based on real time */
        int start = FindNextHourIndex();

        if (start < 0) {
            /* All hours passed today — wait until tomorrow 00:00 */
            time_t now = time(nullptr);
            struct tm t;
            localtime_r(&now, &t);
            struct tm midnight = t;
            midnight.tm_hour = 0;
            midnight.tm_min  = 0;
            midnight.tm_sec  = 0;
            midnight.tm_mday += 1;
            int wait_sec = static_cast<int>(difftime(mktime(&midnight), now));
            ESP_LOGI(TAG, "all hours done — sleeping %d s until midnight", wait_sec);
            SleepLogged(wait_sec, "midnight");
            ResetAll();
            start = 0;
        }

        /* Process each remaining hour */
        for (int i = start; i < CLOCK_HOUR_COUNT; i++) {
            /* Wait until it's time for this hour */
            if constexpr (kHourIntervalMs > 0) {
                /* Dev: no pre-wait, alarms fire immediately */
            } else {
                /* Production: wait until the actual work hour */
                int wait_sec = SecondsUntilHour(i);
                if (wait_sec <= 0) continue;  // hour already passed, skip
                SleepLogged(wait_sec, hours_[i].label);
            }

            /* Trigger alarm sequence */
            int64_t t0 = esp_timer_get_time();
            TriggerHourWithAlarms(i);

            /* After alarms, idle until next interval */
            if constexpr (kHourIntervalMs > 0) {
                /* Dev: sleep remainder of the interval */
                int64_t alarm_ms = (esp_timer_get_time() - t0) / 1000;
                int idle_ms = kHourIntervalMs - static_cast<int>(alarm_ms);
                if (idle_ms > 0) {
                    SleepLogged(idle_ms / 1000, hours_[i].label);
                }
            }
            /* Production: loop will calculate SecondsUntilHour for next i */
        }

        /* All hours processed — wait until next day */
        if constexpr (kHourIntervalMs > 0) {
            /* Dev: one cycle done, idle forever (don't exit task) */
            ESP_LOGI(TAG, "all %d hours processed — dev cycle complete, idling", CLOCK_HOUR_COUNT);
            while (true) { vTaskDelay(pdMS_TO_TICKS(3600000)); }
        } else {
            /* Production: wait until midnight, then restart */
            time_t now = time(nullptr);
            struct tm t;
            localtime_r(&now, &t);
            struct tm midnight = t;
            midnight.tm_hour = 0;
            midnight.tm_min  = 0;
            midnight.tm_sec  = 0;
            midnight.tm_mday += 1;
            int wait_sec = static_cast<int>(difftime(mktime(&midnight), now));
            ESP_LOGI(TAG, "all hours done — sleeping %d s until midnight", wait_sec);
            SleepLogged(wait_sec, "midnight");
            /* Loop continues → ResetAll + FindNextHourIndex for new day */
        }
    }
}

/* ── Ack wait ──────────────────────────────────────── */

bool ClockManager::WaitForAck(int index, int timeout_ms, int64_t t0)
{
    int elapsed_ms = 0;
    while (elapsed_ms < timeout_ms) {
        if (IsSwitchPressed()) {
            int64_t ack_ms = (esp_timer_get_time() - t0) / 1000;
            ESP_LOGI(TAG, "[%s] switch pressed after %.1f s",
                     hours_[index].label, ack_ms / 1000.0);
            AcknowledgeHour(index);
            return true;
        }
        int sec = elapsed_ms / 1000;
        if (sec > 0 && sec % 30 == 0 && sec != (elapsed_ms - kSwitchPollMs) / 1000) {
            ESP_LOGI(TAG, "[%s] waiting... %d / %d s",
                     hours_[index].label, sec, timeout_ms / 1000);
        }
        vTaskDelay(pdMS_TO_TICKS(kSwitchPollMs));
        elapsed_ms += kSwitchPollMs;
    }
    return false;
}

/* ── Hour actions ──────────────────────────────────── */

void ClockManager::ActivateHour(int index)
{
    auto& h = hours_[index];
    ESP_LOGI(TAG, "[%s] servo CH%d pop + red LED CH%d on",
             h.label, h.servo_channel, h.red_led_channel);

    /* Pop servo up briefly, then return to 0° */
    if (pca9685_ != nullptr) {
        ESP_LOGI(TAG, "[%s] servo CH%d → 90°", h.label, h.servo_channel);
        pca9685_->SetServoAngle(h.servo_channel, 90);
        vTaskDelay(pdMS_TO_TICKS(2000));     // stay up 2 s
        ESP_LOGI(TAG, "[%s] servo CH%d → 0°", h.label, h.servo_channel);
        pca9685_->SetServoAngle(h.servo_channel, 0);
        vTaskDelay(pdMS_TO_TICKS(500));      // wait for servo to reach 0°
        pca9685_->SetFullOff(h.servo_channel);
    } else {
        ESP_LOGW(TAG, "[%s] PCA9685 not available — servo skipped", h.label);
    }

    /* Red LED on (active LOW: FullOff = output LOW = LED on) */
    if (pca9685_ != nullptr) {
        pca9685_->SetFullOff(h.red_led_channel);
    }
    gpio_set_level(h.green_led, 1);                 // green OFF

    PlayAlarm(index);
    h.state = HourState::Active;
}

void ClockManager::AcknowledgeHour(int index)
{
    auto& h = hours_[index];
    ESP_LOGI(TAG, "[%s] acknowledged — green on, red off", h.label);

    /* Red LED off (FullOn = output HIGH = LED off) */
    if (pca9685_ != nullptr) {
        pca9685_->SetFullOn(h.red_led_channel);
    }
    /* Green LED on */
    gpio_set_level(h.green_led, 0);

    Application::GetInstance().Schedule([]() {
        Application::GetInstance().DismissAlert();
    });
    h.state = HourState::Acknowledged;
}

void ClockManager::MissHour(int index)
{
    auto& h = hours_[index];
    ESP_LOGW(TAG, "[%s] missed — red LED stays on", h.label);

    /* Servo already returned to 0° after pop. Red stays on. */

    Application::GetInstance().Schedule([]() {
        Application::GetInstance().DismissAlert();
    });
    h.state = HourState::Missed;
}

void ClockManager::ResetAll()
{
    for (int i = 0; i < CLOCK_HOUR_COUNT; i++) {
        if (pca9685_ != nullptr) {
            pca9685_->SetServoAngle(hours_[i].servo_channel, 0);
            pca9685_->SetFullOff(hours_[i].servo_channel);
            pca9685_->SetFullOn(hours_[i].red_led_channel);
        }
        gpio_set_level(hours_[i].green_led, 1);
        hours_[i].state = HourState::Idle;
    }
    Application::GetInstance().Schedule([]() {
        Application::GetInstance().DismissAlert();
    });
}

/* ── Sound + OLED alert ────────────────────────────── */

void ClockManager::PlayAlarm(int index)
{
    auto& h = hours_[index];

    // Per-hour notification clip: Notify_08.ogg → 8am, Notify_09.ogg → 9am, …
    // Hours without a clip yet (14/15/16) show the OLED alert silently.
    const std::string_view* notify = nullptr;
    switch (h.hour_24) {
        case 8:  notify = &Lang::Sounds::OGG_NOTIFY_08; break;
        case 9:  notify = &Lang::Sounds::OGG_NOTIFY_09; break;
        case 10: notify = &Lang::Sounds::OGG_NOTIFY_10; break;
        case 11: notify = &Lang::Sounds::OGG_NOTIFY_11; break;
        case 13: notify = &Lang::Sounds::OGG_NOTIFY_13; break;
        default: break;
    }

    Application::GetInstance().Schedule(
        [label = std::string(h.label), msg = std::string(h.message), notify]() {
            // Show the RemindFace for this alarm
            Board::GetInstance().GetDisplay()->SetEmotion("remind");
            // Show message on OLED + play the hour's notification clip
            Application::GetInstance().Alert(label.c_str(), msg.c_str(), "alarm");
            if (notify != nullptr) {
                Application::GetInstance().PlaySound(*notify);
            }
        });
}

/* ── Limit switch ──────────────────────────────────── */

bool ClockManager::IsSwitchPressed() const
{
    return gpio_get_level(LIMIT_SWITCH_GPIO) == 0;
}
