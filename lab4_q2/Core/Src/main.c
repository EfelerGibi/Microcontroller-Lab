#include "stm32g0xx.h"  // Include the device header

volatile uint32_t millis = 0;

void SysTickInit();
void delay_ms(uint32_t delay);
void PWM_Init();
void setDutyCycle(uint16_t dutyCycle);

volatile int dutyCycle = 0;
volatile int direction = 1;


int main(void) {
    SysTickInit();
    PWM_Init();

    while (1) {

        delay_ms(10);  // Control speed of brightness change
    }
}

void SysTickInit() {
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   // Enable SysTick
    SysTick->LOAD = 16000 - 1;                  // Load 16000 for 1ms tick at 16 MHz clock
    SysTick->VAL = 0;                           // Reset SysTick
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  // Enable SysTick interrupt
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;// Set clock source

    NVIC_EnableIRQ(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

void delay_ms(uint32_t delay) {
    millis = 0;
    while (millis < delay);
}

void SysTick_Handler(void) {
    millis++;  // Increment millis value
}

void setDutyCycle(uint16_t dutyCycle) {
    TIM1->CCR3 = dutyCycle;  // Set duty cycle for TIM1 channel 3
}

void PWM_Init() {
    // Enable clock for GPIOA and TIM1
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN;

    // Set PA10 to Alternate Function mode for TIM1_CH3
    GPIOA->MODER &= ~GPIO_MODER_MODE10;
    GPIOA->MODER |= GPIO_MODER_MODE10_1;

    // Set the correct AF for PA10 (AF2 for TIM1_CH3)
    GPIOA->AFR[1] &= ~(0xF << ((10 - 8) * 4));
    GPIOA->AFR[1] |= (2 << ((10 - 8) * 4));

    // Configure TIM1 for PWM
    TIM1->PSC = 1600 - 1;  // Prescaler for 1kHz PWM frequency
    TIM1->ARR = 100;       // Auto-reload value for 100 steps
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;  // PWM mode 1 on Channel 3
    TIM1->CCER |= TIM_CCER_CC3E;    // Enable capture/compare for channel 3
    TIM1->BDTR |= TIM_BDTR_MOE;     // Main output enable (needed for TIM1)
    TIM1->CR1 |= TIM_CR1_CEN;       // Enable timer

    //for interrupts
    TIM1->DIER |= TIM_DIER_CC1IE;

    // Enable TIM1 interrupt in NVIC
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    NVIC_SetPriority(TIM1_CC_IRQn, 0);
}


void TIM1_CC_IRQHandler(void) {
    if (TIM1->SR & TIM_SR_CC1IF) { // Check capture/compare 1 interrupt flag
        TIM1->SR &= ~TIM_SR_CC1IF; // Clear the interrupt flag

        // Update the duty cycle
        dutyCycle += direction;
        if (dutyCycle >= 100) {
            dutyCycle = 100;
            direction = -1;
        } else if (dutyCycle <= 0) {
            dutyCycle = 0;
            direction = 1;
        }

        setDutyCycle(dutyCycle); // Update TIM1 channel 3 duty cycle
    }
}
