#include "stm32f4xx.h"
#include "ws2812b_driver.h"

#define LED_PIN 8 // PA8 = TIM1_CH1

// Create LED strip structure
ws2812bLedStruct myLedStrip;

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1600; i++) __NOP();
}

// Initialize TIM1 for WS2812B PWM + DMA
void ws2812b_timer_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3U<<28);
    GPIOC->MODER |= (1U<<28);
    GPIOC->ODR |= (1U<<14);  // PC14 HIGH

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // PA8 = AF1 (TIM1_CH1)
    GPIOA->MODER &= ~(3 << (LED_PIN * 2));
    GPIOA->MODER |= (2 << (LED_PIN * 2));
    GPIOA->AFR[1] |= (1 << ((LED_PIN - 8) * 4));

    // TIM1: PWM @ 800 kHz (for 16MHz clock)
    TIM1->PSC = 0;
    TIM1->ARR = 19;  // Period = 20 cycles = 1.25us @ 16MHz
    TIM1->CCR1 = 0;

    TIM1->CCMR1 = (6 << 4);  // PWM mode 1
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->BDTR |= TIM_BDTR_MOE;  // Main output enable
    TIM1->CR1 |= TIM_CR1_ARPE;

    // DMA2 Stream5 -> TIM1_CH1
    DMA2_Stream5->CR = 0;
    while (DMA2_Stream5->CR & DMA_SxCR_EN);

    DMA2_Stream5->PAR = (uint32_t)&TIM1->CCR1;
    DMA2_Stream5->M0AR = (uint32_t)myLedStrip.pwmData;
    DMA2_Stream5->NDTR = myLedStrip.bytesToSend;

    DMA2_Stream5->CR = (6 << DMA_SxCR_CHSEL_Pos) |  // Channel 6
                       DMA_SxCR_DIR_0 |              // Memory to peripheral
                       DMA_SxCR_MINC |               // Increment memory
                       DMA_SxCR_PSIZE_0 |            // Peripheral size = 16-bit
                       DMA_SxCR_MSIZE_0 |            // Memory size = 16-bit
                       DMA_SxCR_PL_1 |               // High priority
                       DMA_SxCR_TCIE;                // Transfer complete interrupt

    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    TIM1->DIER |= TIM_DIER_UDE;  // DMA request on update
}

// DMA Transfer Complete Interrupt
void DMA2_Stream5_IRQHandler(void) {
    if (DMA2->HISR & DMA_HISR_TCIF5) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF5;
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        TIM1->CR1 &= ~TIM_CR1_CEN;
        myLedStrip.dataSentFlag = true;
    }
}

// Send LED data via DMA
void ws2812b_send(void) {
    // Wait for previous transfer to complete
    while (!myLedStrip.dataSentFlag);
    myLedStrip.dataSentFlag = false;

    // Prepare PWM data array from LED colors
    ws2812bGetBytesArray(&myLedStrip);

    // Reset DMA
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CNT = 0;
    TIM1->CCR1 = 0;

    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream5->CR & DMA_SxCR_EN);

    DMA2->HIFCR = 0x3F << 6;  // Clear all flags
    DMA2_Stream5->M0AR = (uint32_t)myLedStrip.pwmData;
    DMA2_Stream5->NDTR = myLedStrip.bytesToSend;

    // Start DMA and timer
    DMA2_Stream5->CR |= DMA_SxCR_EN;
    TIM1->CR1 |= TIM_CR1_CEN;
}

int main(void) {
    // Initialize LED strip structure
    ws2812bLedStructInit(&myLedStrip);
    myLedStrip.dataSentFlag = true;

    // Initialize timer and DMA
    ws2812b_timer_init();

    // Clear all LEDs initially
    ws2812bClearAll(&myLedStrip);
    ws2812b_send();
    delay_ms(100);

    while (1) {
        // Example 1: Display RGB on first 3 LEDs
        ws2812bSetRGB(&myLedStrip, 0, 255, 0, 0);    // LED 0 = RED
        ws2812bSetRGB(&myLedStrip, 1, 0, 255, 0);    // LED 1 = GREEN
        ws2812bSetRGB(&myLedStrip, 2, 0, 0, 255);    // LED 2 = BLUE
        ws2812b_send();
        delay_ms(2000);

        // Example 2: Rainbow pattern
        for (uint16_t i = 0; i < LED_AMOUNT; i++) {
            uint8_t hue = (i * 255) / LED_AMOUNT;
            if (hue < 85) {
                ws2812bSetRGB(&myLedStrip, i, 255 - hue * 3, hue * 3, 0);
            } else if (hue < 170) {
                hue -= 85;
                ws2812bSetRGB(&myLedStrip, i, 0, 255 - hue * 3, hue * 3);
            } else {
                hue -= 170;
                ws2812bSetRGB(&myLedStrip, i, hue * 3, 0, 255 - hue * 3);
            }
        }
        ws2812b_send();
        delay_ms(2000);

        // Example 3: Wave effect
        for (int j = 0; j < 50; j++) {
            ws2812bWave(&myLedStrip, 10, 255, 100, 50);
            ws2812b_send();
            delay_ms(50);
        }

        // Example 4: All white
        for (uint16_t i = 0; i < LED_AMOUNT; i++) {
            ws2812bSetRGB(&myLedStrip, i, 255, 255, 255);
        }
        ws2812b_send();
        delay_ms(2000);

        // Clear all and repeat
        ws2812bClearAll(&myLedStrip);
        ws2812b_send();
        delay_ms(1000);
    }
}
