#ifndef SEVENSEG_H
#define SEVENSEG_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pin definitions for 74HC595
#define SHIFT_DATA_PIN 14  // SER pin
#define SHIFT_CLOCK_PIN 26 // SRCLK pin
#define SHIFT_LATCH_PIN 27 // RCLK pin

// Common cathode 7-segment display patterns (0-9, A-F)
#define SEG_A 0xEE // 11101110
#define SEG_AS 0xEF
#define SEG_B 0xFE // 11111110
#define SEG_C 0x9C // 10011100
#define SEG_CS 0x9D
#define SEG_D 0xFC // 11111100
#define SEG_DS 0xFD
#define SEG_E 0x9E // 10011110
#define SEG_F 0x8E // 10001110
#define SEG_FS 0x8F
#define SEG_G 0xBC // 10111100
#define SEG_GS 0xBD

/**
 * @brief Initialize GPIO pins for the 74HC595 shift register
 */
void init_7seg();

/**
 * @brief Display a hex value (0-F) on the 7-segment display
 * @param value The hex value to display (0-15)
 */
void display_7seg(uint8_t value);

#endif