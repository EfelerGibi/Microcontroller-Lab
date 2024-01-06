#include "stm32g0xx.h"

#define TIM_AutoReload 16000//1600
uint16_t counter = 0;
int D1;
int D2;
int D3;
int D4;

volatile uint32_t millis = 0;

void initMic();
void SysTickInit();
void SysTick_Handler();
void ButtonInit();
void setPanel();
void PanelInit();
void setDigit(uint32_t mask,int digit);
void I2C_Init();
void I2C_Read();

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
	ButtonInit();
	PanelInit();
	SysTickInit();
	initMic();

	setDigit(GPIO_ODR_OD6,0);
	I2C_Init();
    while(1) {

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
void SysTick_Handler(void) {
    millis++; // Increment millis value
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
	counter = 0;
	GPIOC->ODR &= ~(1U << 6);
	EXTI->FPR1 |= (1<<0); //EXTI falling edge pending register 1 (EXTI_FPR1)
}

void EXTI4_15_IRQHandler(){

	counter++;
	delay_ms(100); // Delay for debouncing
	EXTI->RPR1 = EXTI_RPR1_RPIF9;  // Clear the pending bit for EXTI line 2

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

void I2C_Init()
{
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

	GPIOA->MODER &= ~(GPIO_MODER_MODE9|
					  GPIO_MODER_MODE10);
	GPIOA->MODER |= (GPIO_MODER_MODE9_1|
					 GPIO_MODER_MODE10_1);

	GPIOA->OTYPER |= (GPIO_OTYPER_OT9|
				 	  GPIO_OTYPER_OT10);

	GPIOB->AFR[1] &= (0xFU << 0);
	GPIOB->AFR[1] |= (6 << 0);

	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9|GPIO_AFRH_AFSEL10);
	GPIOA->AFR[1] |= (GPIO_AFRH_AFSEL9|GPIO_AFRH_AFSEL10);

	RCC->APBENR1 |= RCC_APBENR1_I2C1EN;

	I2C1->CR1 = 0;
	I2C1->CR1 |= I2C_CR1_ERRIE;

	I2C1->TIMINGR |= (3<<28);
	I2C1->TIMINGR |= (0x13<<0);
	I2C1->TIMINGR |= (0xF<<8);
	I2C1->TIMINGR |= (0x2<<16);
	I2C1->TIMINGR |= (0x4<<20);

	I2C1->CR1 = I2C_CR1_PE;

	NVIC_SetPriority(I2C1_IRQn, 1);
	NVIC_EnableIRQ(I2C1_IRQn);
}

void read_I2C(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint32_t num)
{
	I2C1->CR2 = 0; //Clear
	I2C1->CR2 |= (uint32_t)devAddr << 1; //Slave adress
	I2C1->CR2 |= 1U << 16;
	I2C1->CR2 |= I2C_CR2_START;
	while(!(I2C1->ISR & I2C_ISR_TXIS)); //

	I2C1->TXDR = (uint32_t)regAddr;
	while(!(I2C1->ISR & I2C_ISR_TC));

	I2C1->CR2 = 0;
	I2C1->CR2 |= ((uint32_t)devAddr << 1);
	I2C1->CR2 |= I2C_CR2_RD_WRN;
	I2C1->CR2 |= (num << 16);
	I2C1->CR2 |= I2C_CR2_NACK;
	I2C1->CR2 |= I2C_CR2_AUTOEND;
	I2C1->CR2 |= I2C_CR2_START;

	for(int i=0; i<num; i++)
	{
		while(!(I2C1->ISR & I2C_ISR_RXNE));
		data[i] = (uint8_t) I2C1->RXDR;
	}
}

void I2C_Write(uint8_t devAddr, uint16_t regAddr, uint8_t data)
{
	I2C1->CR2 = 0;
	I2C1->CR2 |= ((uint32_t)devAddr << 1); // slave adress
	I2C1->CR2 |= (3U << 16);
	I2C1->CR2 |= I2C_CR2_AUTOEND;
	I2C1->CR2 |= I2C_CR2_START;

	while(!(I2C1->ISR & I2C_ISR_TXIS));
	I2C1->TXDR = (uint32_t)regAddr;

	while(!(I2C1->ISR & I2C_ISR_TXIS));
	I2C1->TXDR = (uint32_t)regAddr;

	while(!(I2C1->ISR & I2C_ISR_TXIS));
	I2C1->TXDR = (uint32_t)data;
}


void initMic(){
	 // Enable the clock for GPIO port B
	    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

	    // Configure PB9 as input
	    GPIOB->MODER &= ~GPIO_MODER_MODE9_Msk;  // Clear mode bits for PB9


		EXTI->EXTICR[2]|= (1U << 8*1);

	    // Enable SYSCFG clock
	    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;

	    EXTI->IMR1 |= (1U<<9);
	    EXTI->RTSR1 |= (1U << 9);

	    NVIC_SetPriority(EXTI4_15_IRQn, 2);
	    NVIC_EnableIRQ(EXTI4_15_IRQn);
}


