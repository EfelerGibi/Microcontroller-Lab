#include "stm32g0xx.h"
#define TIM_AutoReload 16000//1600


uint32_t count = 0;
uint32_t isr = 0;
uint32_t tdr = 0;
void UART_Init();
void printChar(uint8_t c);
int _print(int f,char *ptr, int len);
void print(char *s);
uint8_t uart_rx(void);
void TimerInit();
void TIM2_IRQHandler(void);


int main()
{
	UART_Init();
	while(1)
	{

		print("hello world \n");

	}
}

void UART_Init()
{
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN; //Enable clock for line A
	RCC->APBENR1 |= RCC_APBENR1_USART2EN; //Enable clock for UART2

	GPIOA->MODER &= ~GPIO_MODER_MODE2_Msk; //Clear MODE2
	GPIOA->MODER |= GPIO_MODER_MODE2_1; //Set PA2 as alternate function

	GPIOA->MODER &= ~GPIO_MODER_MODE3_Msk; //Clear MODE3
	GPIOA->MODER |= GPIO_MODER_MODE3_1; //Set PA3 as alternate function

	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL3_0;

	USART2->BRR = (104<<4) | (3<<0);//(104<<4) | (3<<0); //0x683

	USART2->CR1 |= USART_CR1_RE; //Enable receive
	USART2->CR1 |= USART_CR1_TE; //Enable transmit
	USART2->CR1 |= USART_CR1_UE; //Enable uart
}

void TimerInit()
{
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN_Msk;

	TIM2->CNT = 0;
	TIM2->PSC = 1;
	TIM2->ARR = (uint32_t) 16000000;
	TIM2->DIER |= (1U<<0); //Enable interrupt

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn,3);

	TIM2->SR &= ~(1U<<0);

	TIM2->EGR |= (1U<<0); // Reset timer
	TIM2->CR1 |= (1U<<0); // Enable timer
}

void TIM2_IRQHandler(void){
	TIM2->SR &= ~(1<<0); // Clear UIF update interrupt flag

	count++;
}


void printChar(uint8_t c)
{
	USART2->TDR = c;
	while(!(USART2->ISR & USART_ISR_TC))
	{
	}
}

int _print(int f,char *ptr, int len)
{
	(void)f;
	for(int i=0; i<len; i++)
	{
		printChar(ptr[i]);
	}
	return len; // return length
}


void print(char *s)
{
	// count number of characters in s string until a null byte comes `\0`
	uint8_t length = 0;
	while(s[length]!='\0')
	{
		length++;
	}
	_print(0, s, length);
}

uint8_t uart_rx(void)
{
	while(!(USART2->ISR & USART_ISR_RXNE_RXFNE))
	{
	}
	return (uint8_t) USART2->RDR;
}



