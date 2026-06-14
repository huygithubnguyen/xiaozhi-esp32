#pragma once

#include "pca9685.h"
#include "config.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

enum class HourState {
    Idle,
    Active,        // servo up, red LED on, waiting for ack
    Acknowledged,  // switch pressed: servo down, green on, red off
    Missed,        // no ack after max alarms: servo down, red stays on
};

struct HourSlot {
    uint8_t servo_channel;    // PCA9685 servo channel (0–7)
    uint8_t red_led_channel;  // PCA9685 red LED channel (8–15)
    gpio_num_t green_led;     // GPIO for green LED
    const char* label;        // e.g. "8am"
    const char* message;      // e.g. "It's 8 AM"
    int hour_24;              // 24h: 8, 9, 10, 11, 13, 14, 15, 16
    HourState state;
};

class ClockManager {
public:
    explicit ClockManager(Pca9685* pca9685);
    ~ClockManager();

    void Start();

private:
    void RunLoop();
    static void TaskEntry(void* arg);

    /* Shared flow — same in dev and production */
    void WaitForTimeSync();
    int  FindNextHourIndex() const;
    int  SecondsUntilHour(int index) const;
    void TriggerHourWithAlarms(int index);   // alarm loop: activate → repeat → ack/miss
    void SleepLogged(int seconds, const char* label);

    void ActivateHour(int index);
    void AcknowledgeHour(int index);
    void MissHour(int index);
    void ResetAll();
    void PlayAlarm(int index, bool first = false);
    bool WaitForAck(int index, int timeout_ms, int64_t t0);
    bool IsSwitchPressed() const;

    Pca9685* pca9685_;
    TaskHandle_t task_handle_ = nullptr;
    HourSlot hours_[CLOCK_HOUR_COUNT];

    /* ── Timing (same constants for dev & production) ── */
#if CLOCK_DEV_MODE
    static constexpr int kHourIntervalMs = 2 * 60 * 1000;   // 5 min per "hour"
#else
    static constexpr int kHourIntervalMs = 0;                // 0 = use real-time
#endif
    static constexpr int kAlarmRepeatMs  = 2 * 60 * 1000;   // 2 min between repeats
    static constexpr int kMaxAlarms      = 2;
    static constexpr int kSwitchPollMs   = 100;
};
