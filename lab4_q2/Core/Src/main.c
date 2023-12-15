#include "stm32g0xx.h"  // Include the device header

volatile uint32_t millis = 0;
void setDutyCycle(uint16_t dutyCycle);
void delay_ms(uint32_t delay);
void SysTickInit();
void PWM_Init();

int main(void) {
	SysTickInit();
    PWM_Init();
    int dutyCycle = 0;
    int direction = 1;

    while (1) {
    	setDutyCycle(dutyCycle);
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

        delay_ms(10);  // Control speed of brightness change
    }
}

void SysTickInit() {
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   // Enable SysTick
    SysTick->LOAD = 16000;                      // Load 16000 for 1ms tick
    SysTick->VAL = 0;                           // Reset SysTick
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  // SysTick Enable Interrupt
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // SysTick clock source = AHB

    NVIC_EnableIRQ(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

void delay_ms(uint32_t delay) {
    millis = 0;
    while (millis < delay)
    {
        // Wait for the specified duration
    }
}

void SysTick_Handler(void) {
    millis++; // Increment millis value
}

void setDutyCycle(uint16_t dutyCycle)
{
	TIM1->CCR3 = dutyCycle;
}

void PWM_Init() {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;   // Enable GPIOA clock
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN;  // Enable TIM1 clock

    GPIOA->MODER &= ~GPIO_MODER_MODE10;   // Clear mode bits for PA10
    GPIOA->MODER |= GPIO_MODER_MODE10_1;  // Set PA10 to Alternate Function mode


    GPIOA->AFR[1] &= ~(0xF << ((10 - 8) * 4));  // Clear the current AF setting for PA10
    GPIOA->AFR[1] |= (2 << ((10 - 8) * 4));    // Set the correct AF (AF2) for TIM1_CH3 for PA10

    TIM1->PSC = 1600 - 1;  // Prescaler for 1kHz PWM frequency
    TIM1->ARR = 100;       // Auto-reload value for 100 steps
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // PWM mode 1 on Channel 3
    TIM1->CCER |= TIM_CCER_CC3E;        // Enable capture/compare for channel 3
    TIM1->BDTR |= TIM_BDTR_MOE;         // Main output enable (needed for TIM1)
    TIM1->CR1 |= TIM_CR1_CEN;           // Enable timer
}
