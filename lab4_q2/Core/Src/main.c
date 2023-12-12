#include "stm32g0xx.h"  // Include the device header

void delayMs(int delay) {
    int i;
    for (; delay > 0; delay--) {
        for (i = 0; i < 3195; i++);
    }
}

void setupPWM() {
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN;   // Enable GPIOC clock
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;  // Enable TIM3 clock

    GPIOC->MODER &= ~GPIO_MODER_MODE6;   // Clear mode bits
    GPIOC->MODER |= GPIO_MODER_MODE6_1;  // Set to Alternate Function mode
    GPIOC->AFR[0] |= GPIO_AFRL_AFSEL6_0; // AF1 for TIM3_CH1

    TIM3->PSC = 1600 - 1;  // Prescaler for 1kHz PWM frequency
    TIM3->ARR = 100;       // Auto-reload value for 100 steps
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1 on Channel 1
    TIM3->CCER |= TIM_CCER_CC1E;        // Enable capture/compare for channel 1
    TIM3->CR1 |= TIM_CR1_CEN;           // Enable timer
}

int main(void) {
    setupPWM();
    int dutyCycle = 0;
    int direction = 1;

    while (1) {
        TIM3->CCR1 = dutyCycle;  // Set duty cycle

        // Update duty cycle for next iteration
        dutyCycle += direction;

        // Ensure duty cycle stays within 0 to 100
        if (dutyCycle >= 100) {
            dutyCycle = 100;
            direction = -1;
        } else if (dutyCycle <= 0) {
            dutyCycle = 0;
            direction = 1;
        }

        delayMs(1);  // Control speed of brightness change
    }
}
