#include "stm32g0xx.h"

#define TIM_AutoReload 16000//1600
uint16_t counter = 0;
int D1;
int D2;
int D3;
int D4;

void check_counter();
void LedInit();
void ToggleLed();
void Timer2Init();
void Timer3Init();
void ButtonInit();
void EnableTimer();
void DisableTimer();
void setPanel();
void PanelInit();
void setDigit(uint32_t mask,int digit);


static const uint8_t digitPins[16] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // B
    0x39, // C
    0x5E, // D
    0x79, // E
    0x71  // F
};



int main(void) {
	LedInit();

	Timer2Init();

	ButtonInit();
	PanelInit();
	setDigit(GPIO_ODR_OD6,0);
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


void Timer2Init()
{
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN_Msk;

	TIM2->CNT = 0;
	TIM2->PSC = 160;
	TIM2->ARR = (uint32_t) 25;//1600
	TIM2->DIER |= (1U<<0);

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn,3);
	TIM2->SR &= ~(1U<<0);

	TIM2->EGR |= (1U<<0); // Reset timer

	DisableTimer(TIM2);
}

void EnableTimer(TIM_TypeDef* TIM)
{
	TIM->CR1 |= (1U<<0);
}

void DisableTimer(TIM_TypeDef* TIM)
{
	TIM->CR1 &= ~(1U<<0);
}


void TIM2_IRQHandler(void){
	TIM2->SR &= ~(1<<0); // Clear UIF update interrupt flag
	counter++;

	if(counter>=40000)
	{
		DisableTimer(TIM2);
		GPIOC->ODR ^= (1U << 6);
		counter = 0;
	}
	int number_counter = counter/4;
	D1 = number_counter/1000;
	D2 = (number_counter%1000)/100;
	D3 = (number_counter%100)/10;
	D4 = number_counter%10;
	if (counter%4 == 0){
		setDigit(GPIO_ODR_OD6,D4);
	}else if(counter%3 == 0){
		setDigit(GPIO_ODR_OD5,D3);
	}else if(counter%2 == 0){
		setDigit(GPIO_ODR_OD4,D2);
	}else if(counter%1 == 0){
		setDigit(GPIO_ODR_OD1,D1);
	}
}

void setDigit(uint32_t mask,int digit)
{
    //Disable D1,D2,D3,D4
    GPIOA->ODR |= GPIO_ODR_OD1;
	GPIOA->ODR |= GPIO_ODR_OD4;
	GPIOA->ODR |= GPIO_ODR_OD5;
	GPIOA->ODR |= GPIO_ODR_OD6;

    GPIOA->ODR &= ~mask; //Enable mask2
    GPIOB->ODR |= digitPins[digit];
    GPIOB->ODR &= digitPins[digit];
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
	EXTI->FPR1 |= (1<<0); //EXTI falling edge pending register 1 (EXTI_FPR1)
	counter = 0;
	GPIOC->ODR &= ~(1U << 6);
	EnableTimer(TIM2);
}

void PanelInit()
{
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN_Msk;
    GPIOB->MODER &= ~0xFFFF;
    GPIOB->MODER |= 0x5555;
    GPIOB->ODR |= digitPins[0];


    GPIOA->MODER &= ~GPIO_MODER_MODE1_Msk;
    GPIOA->MODER |= GPIO_MODER_MODE1_0;

    GPIOA->MODER &= ~GPIO_MODER_MODE4_Msk;
    GPIOA->MODER |= GPIO_MODER_MODE4_0;

    GPIOA->MODER &= ~GPIO_MODER_MODE5_Msk;
    GPIOA->MODER |= GPIO_MODER_MODE5_0;

    GPIOA->MODER &= ~GPIO_MODER_MODE6_Msk;
    GPIOA->MODER |= GPIO_MODER_MODE6_0; //pa1 pa4 pa5 pa6

    //Disable D1,D2,D3,D4
    GPIOA->ODR |= GPIO_ODR_OD1;
	GPIOA->ODR |= GPIO_ODR_OD4;
	GPIOA->ODR |= GPIO_ODR_OD5;
	GPIOA->ODR |= GPIO_ODR_OD6;
}



