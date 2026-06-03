#include "pca9685.h"
#include "config.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>

#define TAG "PCA9685"

/* ── Register map ──────────────────────────────────── */
namespace reg {
    static constexpr uint8_t MODE1    = 0x00;
    static constexpr uint8_t MODE2    = 0x01;
    static constexpr uint8_t PRESCALE = 0xFE;
    static constexpr uint8_t ALL_ON_L = 0xFA;
    static constexpr uint8_t ALL_ON_H = 0xFB;
    static constexpr uint8_t ALL_OFF_L = 0xFC;
    static constexpr uint8_t ALL_OFF_H = 0xFD;

    /* Per-channel base: LEDx_ON_L = 0x06 + 4*ch */
    static constexpr uint8_t LED_ON_L(uint8_t ch)  { return 0x06 + 4 * ch; }
    static constexpr uint8_t LED_ON_H(uint8_t ch)  { return 0x07 + 4 * ch; }
    static constexpr uint8_t LED_OFF_L(uint8_t ch) { return 0x08 + 4 * ch; }
    static constexpr uint8_t LED_OFF_H(uint8_t ch) { return 0x09 + 4 * ch; }
}

/* ── MODE1 bits ────────────────────────────────────── */
namespace mode1 {
    static constexpr uint8_t RESTART     = 0x80;
    static constexpr uint8_t EXTCLK      = 0x40;
    static constexpr uint8_t AI          = 0x20;   // auto-increment
    static constexpr uint8_t SLEEP       = 0x10;
    static constexpr uint8_t SUB1        = 0x08;
    static constexpr uint8_t SUB2        = 0x04;
    static constexpr uint8_t SUB3        = 0x02;
    static constexpr uint8_t ALLCALL     = 0x01;
}

/* ── MODE2 bits ────────────────────────────────────── */
namespace mode2 {
    static constexpr uint8_t INVRT       = 0x10;
    static constexpr uint8_t OCH         = 0x08;
    static constexpr uint8_t OUTDRV      = 0x04;   // totem-pole
}

/* ── ON/OFF H-register bits ────────────────────────── */
static constexpr uint8_t FULL_ON_OFF_BIT = 4;       // bit 4 = full-on / full-off

/* ── Construction ──────────────────────────────────── */

Pca9685::Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr)
{
}

/* ── Initialisation ────────────────────────────────── */

esp_err_t Pca9685::Init(float freq_hz)
{
    /* 1. Compute and write prescaler (must be in sleep mode) */
    float prescale_f = kOscFreq / (4096.0f * freq_hz) - 1.0f;
    uint8_t prescale = static_cast<uint8_t>(roundf(prescale_f));

    uint8_t old_mode1 = ReadReg(reg::MODE1);
    WriteReg(reg::MODE1, (old_mode1 & 0x7F) | mode1::SLEEP);   // enter sleep
    WriteReg(reg::PRESCALE, prescale);

    /* 2. Wake up with auto-increment enabled */
    WriteReg(reg::MODE1, (old_mode1 & 0x0F) | mode1::AI);
    vTaskDelay(pdMS_TO_TICKS(1));   // allow oscillator to stabilise

    /* 3. Enable restart (clears SLEEP flag that may have been set) */
    uint8_t cur = ReadReg(reg::MODE1);
    WriteReg(reg::MODE1, cur | mode1::RESTART);

    /* 4. Configure outputs: totem-pole, non-inverted */
    WriteReg(reg::MODE2, mode2::OUTDRV);

    /* 5. Turn all channels off */
    for (uint8_t ch = 0; ch < 16; ch++) {
        SetFullOff(ch);
    }

    ESP_LOGI(TAG, "initialized at %.1f Hz, prescale=%d, addr 0x%02X", freq_hz, prescale, PCA9685_I2C_ADDR);
    return ESP_OK;
}

/* ── Frequency (runtime change) ────────────────────── */

void Pca9685::SetFrequency(float freq_hz)
{
    float prescale_f = kOscFreq / (4096.0f * freq_hz) - 1.0f;
    uint8_t prescale = static_cast<uint8_t>(roundf(prescale_f));

    uint8_t old = ReadReg(reg::MODE1);
    WriteReg(reg::MODE1, (old & 0x7F) | mode1::SLEEP);   // must sleep to write PRESCALE
    WriteReg(reg::PRESCALE, prescale);
    WriteReg(reg::MODE1, old & ~mode1::SLEEP);             // wake up

    ESP_LOGD(TAG, "prescale=%d → %.1f Hz", prescale, freq_hz);
}

/* ── Raw PWM ───────────────────────────────────────── */

void Pca9685::SetPwm(uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel > 15) return;
    /* Clear full-on / full-off bits, write 4 bytes */
    WriteReg(reg::LED_ON_L(channel),  on  & 0xFF);
    WriteReg(reg::LED_ON_H(channel),  (on  >> 8) & 0x0F);
    WriteReg(reg::LED_OFF_L(channel), off & 0xFF);
    WriteReg(reg::LED_OFF_H(channel), (off >> 8) & 0x0F);
}

/* ── Servo ─────────────────────────────────────────── */

void Pca9685::SetServoAngle(uint8_t channel, uint8_t angle)
{
    if (angle > 180) angle = 180;

    /* Convert angle → pulse width (µs) → off tick count.
     * Period at 50 Hz = 20 000 µs.
     * Pulse: 500 µs (0°) … 2500 µs (180°).
     * off_tick = pulse_us * 4096 / period_us
     */
    uint32_t pulse_us = SERVO_US_MIN + (uint32_t)angle * (SERVO_US_MAX - SERVO_US_MIN) / 180;
    uint16_t off_tick = static_cast<uint16_t>(pulse_us * 4096 / 20000);

    SetPwm(channel, 0, off_tick);
}

/* ── LED brightness ────────────────────────────────── */

void Pca9685::SetDuty(uint8_t channel, uint16_t duty)
{
    if (duty > kPwmMax) duty = kPwmMax;
    if (duty == 0) {
        SetFullOff(channel);
        return;
    }
    SetPwm(channel, 0, duty);
}

/* ── Full on / off ─────────────────────────────────── */

void Pca9685::SetFullOn(uint8_t channel)
{
    if (channel > 15) return;
    WriteReg(reg::LED_ON_H(channel), (1 << FULL_ON_OFF_BIT));
    WriteReg(reg::LED_OFF_H(channel), 0x00);
}

void Pca9685::SetFullOff(uint8_t channel)
{
    if (channel > 15) return;
    WriteReg(reg::LED_ON_H(channel), 0x00);
    WriteReg(reg::LED_OFF_H(channel), (1 << FULL_ON_OFF_BIT));
}
