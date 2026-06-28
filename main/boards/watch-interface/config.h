#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

/*
 * Smart Clock — ESP32-S3 WROOM
 *
 *  GPIO  Function
 *  ───── ─────────────────────────────
 *   0    BOOT button
 *   4    INMP441 WS
 *   5    INMP441 SCK
 *   6    INMP441 SD
 *   7    MAX98357A DIN
 *   8    Green LED 0  (8am)
 *   9    Green LED 1  (9am)
 *  10    Green LED 2  (10am)
 *  11    Green LED 3  (11am)
 *  12    Green LED 4  (1pm)
 *  13    Green LED 5  (2pm)
 *  14    Green LED 6  (3pm)
 *  15    MAX98357A BCLK
 *  16    MAX98357A LRCK
 *  17    Green LED 7  (4pm)
 *  18    ILI9341 RST
 *  21    ILI9341 SCK  (SPI3)
 *  38    Limit switch
 *  39    ILI9341 CS   (SPI3)
 *  40    ILI9341 DC
 *  41    I2C SDA  (PCA9685)
 *  42    I2C SCL  (PCA9685)
 *  45    ILI9341 BL  (backlight)
 *  47    ILI9341 MOSI (SPI3)
 *  48    WS2812 status LED
 *
 * PCA9685 (I2C addr 0x40):
 *  CH0–CH7   8 servos  (8am–5pm, skip 12pm)
 *  CH8–CH15  8 red LEDs
 */

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

/* ── I2C bus (PCA9685 @ 0x40 + OLED @ 0x3C) ───── */
#define I2C_SDA_PIN             GPIO_NUM_41
#define I2C_SCL_PIN             GPIO_NUM_42

/* ── Display: SH1106 OLED 128×64 — I2C ─────────────── */
#define DISPLAY_SDA_PIN         GPIO_NUM_41  /* Share with PCA9685 I2C */
#define DISPLAY_SCL_PIN         GPIO_NUM_42  /* Share with PCA9685 I2C */

#if CONFIG_OLED_SH1106_128X64
#define DISPLAY_WIDTH           128
#define DISPLAY_HEIGHT          64
#define LCD_TYPE_SH1106
#else
#error "OLED display type is not selected"
#endif

#define DISPLAY_MIRROR_X        true
#define DISPLAY_MIRROR_Y        false

/* ── PCA9685 16-channel PWM driver ─────────────────── */
#define PCA9685_I2C_ADDR       0x40

/* Channel allocation: CH0–CH7 = servos, CH8–CH15 = red LEDs */
#define PCA9685_SERVO_CH_FIRST  0
#define PCA9685_SERVO_CH_LAST   7
#define PCA9685_LED_CH_FIRST    8
#define PCA9685_LED_CH_LAST     15

/* Servo pulse range (µs) — 500 µs = 0°, 2500 µs = 180° */
#define SERVO_US_MIN  500
#define SERVO_US_MAX  2500

/* ── Limit switch (active LOW, pressed = LOW) ──────── */
#define LIMIT_SWITCH_GPIO      GPIO_NUM_38

/* ── Green LEDs (active LOW: 0 = on, 1 = off) ──────── */
#define GREEN_LED_COUNT        8
#define GREEN_LED_HOUR_0       GPIO_NUM_8    /* 8am  */
#define GREEN_LED_HOUR_1       GPIO_NUM_9    /* 9am  */
#define GREEN_LED_HOUR_2       GPIO_NUM_10   /* 10am */
#define GREEN_LED_HOUR_3       GPIO_NUM_11   /* 11am */
#define GREEN_LED_HOUR_4       GPIO_NUM_12   /* 1pm  */
#define GREEN_LED_HOUR_5       GPIO_NUM_13   /* 2pm  */
#define GREEN_LED_HOUR_6       GPIO_NUM_14   /* 3pm  */
#define GREEN_LED_HOUR_7       GPIO_NUM_17   /* 4pm  */

/* ── Interactive Clock hours ────────────────────────── */
#define CLOCK_HOUR_COUNT       8
/* 24 h format: 8, 9, 10, 11, 13, 14, 15, 16 (skip 12 = lunch) */

/* Set to 1 for dev mode (7-min cycle), 0 for production (real hours) */
#define CLOCK_DEV_MODE         1

#endif // _BOARD_CONFIG_H_
