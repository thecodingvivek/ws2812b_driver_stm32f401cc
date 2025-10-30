/*
 * ws2812b_driver.h
 * Modified for STM32F401CC @ 16MHz
 */

#ifndef WS2812B_H_
#define WS2812B_H_

#include "stdbool.h"
#include "string.h"
#include "stdint.h"

// Configuration for your LED strip
#define LED_AMOUNT          30   // Change to your LED count (or 3 for testing)
#define RESET_BYTES_AMOUNT  80   // Reset timing (~100 µs)

// PWM values for 16MHz clock, ARR=19 (period = 1.25us)
#define PWM_VALUE_ONE       12   // Logic '1' = ~0.75us high
#define PWM_VALUE_ZERO      6    // Logic '0' = ~0.375us high
#define TIMER_PERIOD        19   // ARR value (20 cycles total)

typedef struct ws2812bLedStruct {
    uint16_t length;
    uint16_t resetBytes;
    uint16_t bytesToSend;
    uint16_t pwmData[RESET_BYTES_AMOUNT + 24*LED_AMOUNT];
    uint32_t ledArray[LED_AMOUNT];
    volatile bool dataSentFlag;
} ws2812bLedStruct;

void ws2812bLedStructInit(ws2812bLedStruct *ledsStrip);
void ws2812bGetBytesArray(ws2812bLedStruct *ledsStrip);
void ws2812bSetRGB(ws2812bLedStruct *ledsStrip, uint16_t index, uint8_t Red, uint8_t Green, uint8_t Blue);
void ws2812bClearAll(ws2812bLedStruct *ledsStrip);
void ws2812bWave(ws2812bLedStruct *ledStrip, uint16_t width, uint8_t Red, uint8_t Green, uint8_t Blue);

#endif
