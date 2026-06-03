#pragma once

#include "i2c_device.h"
#include "esp_err.h"

class Pca9685 : public I2cDevice {
public:
    Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x40);

    /**
     * Initialize PCA9685: set PWM frequency, configure outputs.
     * @param freq_hz  PWM frequency in Hz (default 50 Hz for servos).
     */
    esp_err_t Init(float freq_hz = 50.0f);

    /** Set raw 12-bit PWM on a channel (on/off tick counters, 0–4095). */
    void SetPwm(uint8_t channel, uint16_t on, uint16_t off);

    /** Set servo angle (0–180°) on a channel. Channel must be in servo range. */
    void SetServoAngle(uint8_t channel, uint8_t angle);

    /** Set LED brightness (0 = off, 4095 = full on). */
    void SetDuty(uint8_t channel, uint16_t duty);

    /** Force channel fully on (100 % duty). */
    void SetFullOn(uint8_t channel);

    /** Force channel fully off (0 % duty). */
    void SetFullOff(uint8_t channel);

private:
    void SetFrequency(float freq_hz);

    static constexpr uint16_t kPwmMax = (1 << 12) - 1;   // 4095
    static constexpr float    kOscFreq = 25000000.0f;     // 25 MHz internal oscillator
};
