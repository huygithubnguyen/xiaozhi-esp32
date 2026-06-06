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
    Acknowledged,  // switch pressed: servo down, green LED on, red off
    Missed,        // no ack after 2 alarms: servo down, red LED stays on
};

struct HourSlot {
    uint8_t servo_channel;    // PCA9685 servo channel (0–7)
    uint8_t red_led_channel;  // PCA9685 red LED channel (8–15)
    gpio_num_t green_led;     // GPIO for green LED
    const char* label;        // e.g. "8am"
    const char* message;      // e.g. "It's 8 AM"
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

    void WaitForTimeSync();            // block until NTP/server time is available
    void ActivateHour(int index);      // servo pop + red LED on + sound
    void AcknowledgeHour(int index);   // servo down + green on, red off
    void MissHour(int index);          // servo down + red stays on
    void ResetAll();                   // everything off
    void PlayAlarm(int index);         // play notification sound + OLED alert
    bool WaitForAck(int index, int timeout_ms, int64_t t0);
    bool IsSwitchPressed() const;

    Pca9685* pca9685_;
    TaskHandle_t task_handle_ = nullptr;
    HourSlot hours_[CLOCK_HOUR_COUNT];

    /* Dev-mode timing */
    static constexpr int kDevIntervalMs  = 7 * 60 * 1000;   // 7 min per "hour"
    static constexpr int kAlarmRepeatMs  = 2 * 60 * 1000;   // 2 min between alarm repeats
    static constexpr int kSwitchPollMs   = 100;               // poll switch every 100 ms
};
