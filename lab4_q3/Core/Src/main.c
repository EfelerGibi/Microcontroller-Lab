#include "stm32g0xx.h"


void init_keypad();
uint8_t scan_keypad();

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
    //scan_keypad();
    // Clear the interrupt flag
    EXTI->RPR1 = EXTI_RPR1_RPIF0;  // Clear the pending bit for EXTI line 0
}

void EXT2_3_IRQHandler(void) {
    //scan_keypad();
    // Clear the interrupt flag
    EXTI->RPR1 = EXTI_RPR1_RPIF2;  // Clear the pending bit for EXTI line 0
}

void EXT4_15_IRQHandler(void) {
    //scan_keypad();
    // Clear the interrupt flag
    if(!(EXTI->RPR1|EXTI_RPR1_RPIF8))
    {
        EXTI->RPR1 |= EXTI_RPR1_RPIF8;  // Set the pending bit for EXTI line 0
    }
    if(!(EXTI->RPR1|EXTI_RPR1_RPIF9))
    {
        EXTI->RPR1 |= EXTI_RPR1_RPIF9;  // Clear the pending bit for EXTI line 0
    }
}


int main(){
	while(1){

	}
	return 0;
}




void init_keypad(){
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN |  RCC_IOPENR_GPIOBEN;
/*
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
*/
	GPIOA->MODER &= ~(GPIO_MODER_MODE9_Msk|GPIO_MODER_MODE8_Msk);
	GPIOA->MODER |= GPIO_MODER_MODE9_0 | GPIO_MODER_MODE8_1;

	GPIOB->MODER &= ~(GPIO_MODER_MODE0_Msk|
					GPIO_MODER_MODE2_Msk|
					GPIO_MODER_MODE4_Msk|
					GPIO_MODER_MODE5_Msk|
					GPIO_MODER_MODE8_Msk|
					GPIO_MODER_MODE9_Msk);

	GPIOB->MODER |= GPIO_MODER_MODE0_0|
					GPIO_MODER_MODE2_0|
					GPIO_MODER_MODE4_1|
					GPIO_MODER_MODE5_1|
					GPIO_MODER_MODE8_0|
					GPIO_MODER_MODE9_1;


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


uint8_t scan_keypad()
{

	GPIOB->ODR ^= GPIO_ODR_OD8;
	if (~((GPIOA->IDR | GPIO_IDR_ID8)| (GPIOB->IDR | GPIO_IDR_ID4)|(GPIOB->IDR | GPIO_IDR_ID5)|(GPIOB->IDR | GPIO_IDR_ID9))){
		GPIOB->ODR ^= GPIO_ODR_OD8;
		return 0;
	}
	GPIOB->ODR ^= GPIO_ODR_OD8;

	GPIOB->ODR ^= GPIO_ODR_OD9;
	if (~((GPIOA->IDR | GPIO_IDR_ID8)| (GPIOB->IDR | GPIO_IDR_ID4)|(GPIOB->IDR | GPIO_IDR_ID5)|(GPIOB->IDR | GPIO_IDR_ID9))){
		GPIOB->ODR ^= GPIO_ODR_OD9;
		return 1;
	}
	GPIOB->ODR ^= GPIO_ODR_OD9;

	GPIOB->ODR ^= GPIO_ODR_OD5;
	if (~((GPIOA->IDR | GPIO_IDR_ID8)| (GPIOB->IDR | GPIO_IDR_ID4)|(GPIOB->IDR | GPIO_IDR_ID5)|(GPIOB->IDR | GPIO_IDR_ID9))){
		GPIOB->ODR ^= GPIO_ODR_OD5;
		return 2;
	}
	GPIOB->ODR ^= GPIO_ODR_OD5;

	return 3;//if not first 3 column its the 4th column
}
