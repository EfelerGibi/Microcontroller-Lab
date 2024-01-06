#include "stm32g0xx.h"

void PWM_Init();
void ADC_Init();
void SysTickInit();
void SysTick_Handler();
void setDutyCycle(uint16_t dutyCycle, uint8_t channel);
uint32_t readADC();

volatile uint32_t millis = 0;
uint32_t dutyCycle = 0;

int main(){
	SysTickInit();
    PWM_Init();
    ADC_Init();

    setDutyCycle(10, 2);
    setDutyCycle(10, 3);

    while(1)
    {
    	dutyCycle = (uint16_t) readADC()*100/4095;
    	setDutyCycle(dutyCycle, 2);
    	setDutyCycle(100-dutyCycle, 3);
    }
    return 0;
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

void setDutyCycle(uint16_t dutyCycle, uint8_t channel)
{
	if (channel==2){
		TIM1->CCR2 = dutyCycle;
	}
	else if (channel==3){
		TIM1->CCR3 = dutyCycle;
	}
}
void SysTick_Handler(void) {
    millis++; // Increment millis value
}

void PWM_Init() {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;   // Enable GPIOA clock
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN;  // Enable TIM1 clock

    GPIOA->MODER &= ~GPIO_MODER_MODE10;   // Clear mode bits for PA10
    GPIOA->MODER &= ~GPIO_MODER_MODE9;   // Clear mode bits for PA09

    GPIOA->MODER |= GPIO_MODER_MODE10_1;  // Set PA10 to Alternate Function mode
    GPIOA->MODER |= GPIO_MODER_MODE9_1;  // Set PA9 to Alternate Function mode


    GPIOA->AFR[1] &= ~(0xF << ((10 - 8) * 4));  // Clear the current AF setting for PA10
    GPIOA->AFR[1] |= (2 << ((10 - 8) * 4));    // Set the AF (AF2) for TIM1_CH3 for PA10

    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4));  // Clear the current AF setting for PA9
    GPIOA->AFR[1] |= (2 << ((9 - 8) * 4));    // Set the AF (AF2) for TIM1_CH2 for PA9

    TIM1->PSC = 1600 - 1;  // Prescaler for 1kHz PWM frequency
    TIM1->ARR = 100;       // Auto-reload value for 100 steps, this is for 1% increments

    TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // PWM mode 1 on Channel 3
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // PWM mode 1 on Channel 2

    TIM1->CCER |= TIM_CCER_CC3E;        // Enable capture/compare for channel 3
    TIM1->CCER |= TIM_CCER_CC2E;        // Enable capture/compare for channel 3

    TIM1->BDTR |= TIM_BDTR_MOE;         // Main output enable (needed for TIM1)
    TIM1->CR1 |= TIM_CR1_CEN;           // Enable timer
    setDutyCycle(0, 2); //set initial duty cycle to 0
    setDutyCycle(0, 3); //set initial duty cycle to 0

}


void ADC_Init()
{
	//PA7
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	RCC->APBENR2 |= RCC_APBENR2_ADCEN;
	GPIOA->MODER |= GPIO_MODER_MODE7; //Set PA7 as analog

	//ADC1->CFGR1 &= ~ADC_CFGR1_CHSELRMOD;
	ADC1->CFGR1 |= ADC_CFGR1_CONT; //Enable continuous conversion mode
	ADC1->SMPR |= ADC_SMPR_SMPSEL7;
	ADC1->SMPR |= 0b111;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL7;

	ADC1->CR |= ADC_CR_ADEN;
	ADC1->CR |= ADC_CR_ADSTART;
}

uint32_t readADC()
{
	while(!(ADC1->ISR & ADC_ISR_EOC))
	{
	}
	return ADC1->DR;
}


