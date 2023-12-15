#include "stm32g0xx.h"



void init_keypad();

/*

void EXT0_1_IRQHandler(void){
	Clearrows();

	GPIOA->ODR ^= (1U<<8);
	if ((GPIOB->IDR >>0)&1){
		//DO Something
	};
	GPIOA->ODR ^= (1U<<8);

	GPIOB->ODR ^= (1U<<9);
		if ((GPIOB->IDR >>0)&1){
			//DO Something
		};
	GPIOB->ODR ^= (1U<<9);

	GPIOB->ODR ^= (1U<<5);
			if ((GPIOB->IDR >>0)&1){
				//DO Something
			};
	GPIOB->ODR ^= (1U<<5);

	GPIOB->ODR ^= (1U<<4);
			if ((GPIOB->IDR >>0)&1){
				//DO Something
			};
	GPIOB->ODR ^= (1U<<4);
};
*/
void EXT0_1_IRQHandler(void) {
    scan_keypad();
    // Clear the interrupt flag
    EXTI->RPR1 = (1U << 0);  // Clear the pending bit for EXTI line 0
}

void EXT2_3_IRQHandler(void) {
    scan_keypad();
    // Clear the interrupt flag
    EXTI->RPR2 = (1U << 0);  // Clear the pending bit for EXTI line 0
}

void EXT4_15_IRQHandler(void) {
    scan_keypad();
    // Clear the interrupt flag
    EXTI->RPR4 = (1U << 0);  // Clear the pending bit for EXTI line 0
}


void main(){
	while(1){

	}
}




void init_keypad(){
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN |  RCC_IOPENR_GPIOBEN;

	GPIOA ->MODER &= ~GPIO_MODER_MODE9_Msk;
	GPIOA ->MODER |= GPIO_MODER_MODE9_0;
	GPIOA ->MODER &= ~GPIO_MODER_MODE8_Msk;
	GPIOA ->MODER |= GPIO_MODER_MODE8_1;

	GPIOB ->MODER &= ~GPIO_MODER_MODE0_Msk;
	GPIOB ->MODER |= GPIO_MODER_MODE0_0;
	GPIOB ->MODER &= ~GPIO_MODER_MODE2_Msk;
	GPIOB ->MODER |= GPIO_MODER_MODE2_0;

	GPIOB ->MODER &= ~GPIO_MODER_MODE4_Msk;
	GPIOB ->MODER |= GPIO_MODER_MODE4_1;
	GPIOB ->MODER &= ~GPIO_MODER_MODE5_Msk;
	GPIOB ->MODER |= GPIO_MODER_MODE5_1;


	GPIOB ->MODER &= ~GPIO_MODER_MODE8_Msk;
	GPIOB ->MODER |= GPIO_MODER_MODE8_0;
	GPIOB ->MODER &= ~GPIO_MODER_MODE9_Msk;
	GPIOB ->MODER |= GPIO_MODER_MODE9_1;

	GPIOA->PUPDR |= GPIO_PUPDR_PUPD9_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD2_1 | GPIO_PUPDR_PUPD8_1;

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

	NVIC_SetPriority(EXTI0_1_IRQn, 0);
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	NVIC_SetPriority(EXTI2_3_IRQn, 0);
	NVIC_EnableIRQ(EXTI2_3_IRQn);

	NVIC_SetPriority(EXTI4_15_IRQn, 0);
	NVIC_EnableIRQ(EXTI4_15_IRQn);

}


int scan_keypad() {


}