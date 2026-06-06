#pragma once

#include "pca9685.h"
#include "config.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

enum class HourState {
    Idle,
    Active,        // servo up, LED on, waiting for ack
    Acknowledged,  // switch pressed (or timeout), servo retracted, LED stays
};

struct HourSlot {
    uint8_t servo_channel;    // PCA9685 channel (0–7)
    gpio_num_t green_led;     // GPIO for green LED
    const char* label;        // e.g. "8am"
    const char* message;      // e.g. "It's 8 AM"
    HourState state;
};

class ClockManager {
public:
    /**
     * @param pca9685  PCA9685 driver (may be nullptr if not connected).
     */
    explicit ClockManager(Pca9685* pca9685);
    ~ClockManager();

    /** Launch the background task (call once from board constructor). */
    void Start();

private:
    void RunLoop();
    static void TaskEntry(void* arg);

    void ActivateHour(int index);    // pop servo + LED on + sound alert
    void AcknowledgeHour(int index); // retract servo, LED stays green
    void ResetAll();                 // retract all servos, turn off all LEDs
    bool IsSwitchPressed() const;
    bool WaitForAck(int index, int timeout_ms, int64_t t0);  // poll switch, return true if acked
    void PlayAlarm(int index);       // play notification sound + OLED alert

    Pca9685* pca9685_;
    TaskHandle_t task_handle_ = nullptr;
    HourSlot hours_[CLOCK_HOUR_COUNT];

    /* Dev-mode timing — replace 1 real hour with this interval */
    static constexpr int kDevIntervalMs   = 7 * 60 * 1000;   // 7 min per "hour"
    static constexpr int kAlarmRepeatMs   = 2 * 60 * 1000;   // 2 min between alarm repeats
    static constexpr int kSwitchPollMs    = 100;               // poll switch every 100 ms
};
