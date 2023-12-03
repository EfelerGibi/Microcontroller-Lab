#include "stm32g0xx.h"

int counter = 0;
int incrementing = 1;
int delay;

void check_counter();
void LedInit();
void ToggleLed();
void TimerInit();
void ButtonInit();

int main(void) {

	LedInit();
	TimerInit();
	ButtonInit();

    while(1) {
    }
    return 0;
}

void LedInit()
{
    /* Enable GPIOC clock */
    RCC->IOPENR |= (1U << 2);

    /* Setup PC6 as output */
    GPIOC->MODER &= ~(3U << 2*6);
    GPIOC->MODER |= (1U << 2*6);

    /* Turn on LED */
    GPIOC->ODR |= (1U << 6);
}

void ToggleLed()
{
	GPIOC->ODR ^= (1U << 6);
}


void TimerInit()
{
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN_Msk;

	TIM2->CNT = 0;
	TIM2->PSC = 0;
	TIM2->ARR = (uint32_t) 16000000;
	TIM2->DIER |= (1U<<0);

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn,3);

	TIM2->SR &= ~(1U<<0);

	TIM2->EGR |= (1U<<0); // Reset timer
	TIM2->CR1 |= (1U<<0); // Enable timer
}


void TIM2_IRQHandler(void){
	TIM2->SR &= ~(1<<0); // Clear UIF update interrupt flag
	ToggleLed();
}

void ButtonInit()
{
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN_Msk;
	GPIOA->MODER &= ~GPIO_MODER_MODE0_Msk;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
	GPIOA->PUPDR |=  (1U<<0);

	RCC->APBENR2 |= (1U<<0);
	EXTI->RTSR1 |= (1U<<0);
	EXTI->IMR1 |= (1U<<0);

    EXTI->FTSR1 |= (1U<<0);        // Enable EXTI on Falling edge
	EXTI->RTSR1 &= ~(1U<<0);       // Disable EXTI on Rising edge

	NVIC_SetPriority(EXTI0_1_IRQn,0);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void EXTI0_1_IRQHandler(void){
	EXTI->FPR1 |= (1<<0);

	if(incrementing)
	{
		counter++;
		TIM2->PSC++;

		if(counter>=9)
		{
			incrementing = 0;
		}
	}
	else
	{
		counter--;
		TIM2->PSC--;
		if(counter <= 0)
		{
			incrementing = 1;
		}
	}
	delay = counter + 1;
}

