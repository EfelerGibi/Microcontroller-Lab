#include "stm32g0xx.h"


uint32_t count = 0;
uint32_t isr = 0;
uint32_t tdr = 0;
void UART_Init();
void printChar(uint8_t c);
int _print(int f,char *ptr, int len);
void print(char *s);
uint8_t uart_rx(void);
void uart_tx(uint8_t c);

int main()
{
	UART_Init();
	while(1)
	{

		uart_tx(uart_rx());

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
	int length = 0;
	while(s[length]!='\0')
	{
		length++;
	}
	_print(0, s, length);
}

void uart_tx(uint8_t c)
{
	printChar(c);
}

uint8_t uart_rx(void)
{
	while(!(USART2->ISR & USART_ISR_RXNE_RXFNE))
	{
	}
	return (uint8_t) USART2->RDR;
}



