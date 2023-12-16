#include "stm32g0xx.h"
#include "stdlib.h"

uint8_t updateStatus();

void PWM_Init();

volatile uint8_t counter = 0;
uint32_t isr = 0;
uint32_t tdr = 0;
volatile uint32_t millis = 0;
void setDutyCycle(uint16_t dutyCycle);
void delay_ms(uint32_t delay);
void SysTickInit();

void UART_Init();
void printChar(uint8_t c);
int _print(int f,char *ptr, int len);
void print(char *s);
uint8_t uart_rx(void);
void uart_tx(uint8_t c);

#define A 50
#define B A
#define C A
#define D A
#define clear 10
#define send 11
uint8_t temp_counter;
int matrix[4][4] = {{1,2,3,A},
					{4,5,6,B},
					{7,8,9,C},
					{clear,0,send,D}};

uint8_t uart_transmit_flag = 0;
uint8_t dutycycle_raw = 0;
uint8_t dutycycle;


void init_keypad();
void scan_keypad(uint8_t row);

void EXTI0_1_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void EXTI4_15_IRQHandler(void);



int main(){
	UART_Init();
	init_keypad();
	SysTickInit();
    PWM_Init();
	char str_dutycycle[4];
	while(1){
		utoa(dutycycle,str_dutycycle, 10);
		print("Current Duty Cycle: %");
		for(int i=0;i<4;i++)
		{
			uart_tx(str_dutycycle[i]);
		}
		print("\n\r");

		delay_ms(2000);
		temp_counter = counter;

	}
	return 0;
}


void init_keypad(){

    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

    GPIOA->MODER &= ~(GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE8_Msk);
    GPIOA->MODER |= GPIO_MODER_MODE8_0;

    GPIOB->MODER &= ~(GPIO_MODER_MODE0_Msk |
                      GPIO_MODER_MODE2_Msk |
                      GPIO_MODER_MODE4_Msk |
                      GPIO_MODER_MODE5_Msk |
                      GPIO_MODER_MODE8_Msk |
                      GPIO_MODER_MODE9_Msk);

    GPIOB->MODER |= GPIO_MODER_MODE4_0 |
                    GPIO_MODER_MODE5_0 |
                    GPIO_MODER_MODE9_0;

    // Set initial output levels
    GPIOA->ODR |= GPIO_ODR_OD8;
    GPIOB->ODR |= GPIO_ODR_OD9 | GPIO_ODR_OD5 | GPIO_ODR_OD4;

	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9_Msk;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk | GPIO_PUPDR_PUPD2_Msk | GPIO_PUPDR_PUPD8_Msk;

	GPIOA->PUPDR |= GPIO_PUPDR_PUPD9_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD2_1 | GPIO_PUPDR_PUPD8_1;
	RCC->APBENR2 |= (1U<<0); //APB peripheral clock enable register

	EXTI->EXTICR[2] |= (0U << 8*1);
	EXTI->EXTICR[0] |= (1U << 8*0);
	EXTI->EXTICR[0] |= (1U << 8*2);
	EXTI->EXTICR[2] |= (1U << 8*0);

	EXTI->RTSR1 |= (1U << 9);
	EXTI->RTSR1 |= (1U << 0);
	EXTI->RTSR1 |= (1U << 2);
	EXTI->RTSR1 |= (1U << 8);

	EXTI->IMR1 |= (1U<<9);
	EXTI->IMR1 |= (1U<<0);
	EXTI->IMR1 |= (1U<<2);
	EXTI->IMR1 |= (1U<<8);

	NVIC_SetPriority(EXTI0_1_IRQn, 2);
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	NVIC_SetPriority(EXTI2_3_IRQn, 2);
	NVIC_EnableIRQ(EXTI2_3_IRQn);

	NVIC_SetPriority(EXTI4_15_IRQn, 2);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void scan_keypad(uint8_t row)
{
	delay_ms(20); //for debouncing
	uint8_t column;

	GPIOA->ODR &= ~GPIO_ODR_OD8;
	if (!updateStatus()){
		column = 0;
	}
	GPIOA->ODR |= GPIO_ODR_OD8;

	GPIOB->ODR &= ~GPIO_ODR_OD9;

	if (!updateStatus()){
		column = 1;
	}
	GPIOB->ODR |= GPIO_ODR_OD9;

	GPIOB->ODR &= ~GPIO_ODR_OD5;
	if (!updateStatus()){
		column = 2;
	}
	GPIOB->ODR |= GPIO_ODR_OD5;

	GPIOB->ODR &= ~GPIO_ODR_OD4;
	if (!updateStatus()){
		column = 3;
	}
	GPIOB->ODR |= GPIO_ODR_OD4;//if not first 3 column its the 4th column

	uint8_t current_num = matrix[row-1][column];
	if(current_num == send)
	{
		setDutyCycle(dutycycle_raw);
		dutycycle = dutycycle_raw;
		dutycycle_raw = 0;
	}
	else if(current_num == clear)
	{
		dutycycle_raw = 0;
	}
	else if(current_num < 10)
	{
			dutycycle_raw = 10*dutycycle_raw + current_num;
		if(dutycycle_raw > 100)
		{
			dutycycle_raw=100;
		}
	}
}

uint8_t updateStatus()
{
	return (GPIOA->IDR & GPIO_IDR_ID9) ||
			(GPIOB->IDR & GPIO_IDR_ID0) ||
			(GPIOB->IDR & GPIO_IDR_ID2) ||
			(GPIOB->IDR & GPIO_IDR_ID8);
}

void EXTI0_1_IRQHandler(void) {
	counter++;

    scan_keypad(2);
    // Clear the interrupt flag
    EXTI->RPR1 = EXTI_RPR1_RPIF0;  // Clear the pending bit for EXTI line 0
}

void EXTI2_3_IRQHandler(void) {
	counter++;

    scan_keypad(3);
    // Clear the interrupt flag
    EXTI->RPR1 = EXTI_RPR1_RPIF2;  // Clear the pending bit for EXTI line 0
}

void EXTI4_15_IRQHandler(void) {
	counter++;

    // Clear the interrupt flag
    if((EXTI->RPR1&EXTI_RPR1_RPIF8))
    {
    	scan_keypad(4);
    }
    if((EXTI->RPR1&EXTI_RPR1_RPIF9))
    {
    	scan_keypad(1);
    }
    EXTI->RPR1 |= EXTI_RPR1_RPIF8;
    EXTI->RPR1 |= EXTI_RPR1_RPIF9;
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
    setDutyCycle(0);
}

