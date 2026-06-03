#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX

#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16

#else

#define AUDIO_I2S_GPIO_WS GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7

#endif

#define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0

#define DISPLAY_SDA_PIN GPIO_NUM_41
#define DISPLAY_SCL_PIN GPIO_NUM_42
#define DISPLAY_WIDTH   128

#if CONFIG_OLED_SSD1306_128X32
#define DISPLAY_HEIGHT  32
#elif CONFIG_OLED_SSD1306_128X64
#define DISPLAY_HEIGHT  64
#elif CONFIG_OLED_SH1106_128X64
#define DISPLAY_HEIGHT  64
#define SH1106
#else
#error "OLED display type is not selected"
#endif

#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true

/* ── PCA9685 16-channel PWM driver ─────────────────── */
#define PCA9685_I2C_ADDR       0x40

/* Channel allocation: CH0–CH7 = servos, CH8–CH15 = LEDs */
#define PCA9685_SERVO_CH_FIRST  0
#define PCA9685_SERVO_CH_LAST   7
#define PCA9685_LED_CH_FIRST    8
#define PCA9685_LED_CH_LAST     15

/* Servo pulse range (µs) — 500 µs = 0°, 2500 µs = 180° */
#define SERVO_US_MIN  500
#define SERVO_US_MAX  2500

/* ── Limit switch (user acknowledgment) ─────────────── */
#define LIMIT_SWITCH_GPIO      GPIO_NUM_38

/* ── Interactive Clock hours ────────────────────────── */
#define CLOCK_HOUR_COUNT       8
/* 24 h format: 8, 9, 10, 11, 13, 14, 15, 16 (skip 12 = lunch) */

#endif // _BOARD_CONFIG_H_
