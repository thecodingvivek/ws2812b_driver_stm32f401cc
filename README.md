# WS2812B DRIVER FOR STM332F401CC

![working](https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2016/09/GIF3.gif?resize=480%2C270&quality=100&strip=all&ssl=1)

The WS2812B is very compatible with ESP32, but for STM32F401 its a different story. After searching for a very long time, I got this driver from https://github.com/FRSH-0109/WS2812B_LED_Strip_Driver thanks to him,which I have modified specifically for the *STM32F401CCU6*.  

This project provides a DMA-based WS2812B LED driver for the STM32F401CC microcontroller (running at 16 MHz).
It allows precise timing generation using TIM1 + DMA, suitable for driving WS2812B addressable RGB LEDs.

You can get datasheet for ws2812b here  https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf

## File Overview
### ws2812b_driver.h
- Configuration header file for LED strip:
- Defines LED count, PWM values, and structure
- Function declarations for LED control
  
### ws2812b_driver.c
- Converts RGB data to WS2812B timing
- Uses bit-reversal LUT for proper WS2812 bit order
- Generates pwmData[] for DMA transfer

### main.c
- Initializes TIM1 + DMA
- Demonstrates several LED patterns:
- Static RGB colors
- Rainbow gradient
- Wave animation
- All white
- Clear LEDs and repeat
