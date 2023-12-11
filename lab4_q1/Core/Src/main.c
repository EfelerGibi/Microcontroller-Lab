#include "stm32g0xx.h"
uint32_t count = 0;
uint32_t isr = 0;
uint32_t tdr = 0;
void UART_Init();
void printChar(char c);

int main()
{
	UART_Init();
	while(1)
	{
		printChar('a');
		while(count<1000000){

			count++;
			isr = USART1->ISR;
			tdr = USART1->TDR;
		}
		count=0;
	}
}

void UART_Init()
{
	RCC->IOPENR = RCC_IOPENR_GPIOAEN; //Enable clock
	RCC->APBENR2 = RCC_APBENR2_USART1EN;

	GPIOA->MODER &= ~GPIO_MODER_MODE2_Msk; //Clear MODE2
	GPIOA->MODER |= GPIO_MODER_MODE2_1; //Set PA2 as alternate function

	GPIOA->MODER &= ~GPIO_MODER_MODE3_Msk; //Clear MODE3
	GPIOA->MODER |= GPIO_MODER_MODE3_1; //Set PA3 as alternate function

	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL3_0;

	USART1->BRR = (104<<4) | (3<<0);//(104<<4) | (3<<0); //0x683

	USART1->CR1 |= USART_CR1_RE; //Enable receive
	USART1->CR1 |= USART_CR1_TE; //Enable transmit
	USART1->CR1 |= USART_CR1_UE; //Enable uart
}

void printChar(char c)
{
	USART1->TDR = 'a';
	while(!(USART1->ISR & USART_ISR_TC))
	{
	}
}











