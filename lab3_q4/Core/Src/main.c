#include "stm32g0xx.h"

#define LEDDELAY    1000
#define WDG_RELOAD_COMMAND 0xAAAA
#define WDG_RELOAD_VAL 1000 //1000de calısıyor, 100'de reset atıyor

volatile int counter = 0;
volatile uint32_t millis = 0;

void delay_ms(uint32_t delay);
void SysTickInit();
void IWDGInit();
void LedInit();
void IWDG_Refresh(void);

int main(void) {
    LedInit();
    SysTickInit();
    IWDGInit();

    while (1) {
        delay_ms(LEDDELAY);
        /* Toggle LED */
        GPIOC->ODR ^= (1U << 6);
        IWDG_Refresh();

    }
    return 0;
}

void LedInit() {
    /* Enable GPIOC clock */
    RCC->IOPENR |= (1U << 2);

    /* Setup PC6 as output */
    GPIOC->MODER &= ~(3U << 2 * 6);
    GPIOC->MODER |= (1U << 2 * 6);

    /* Turn on LED */
    GPIOC->ODR |= (1U << 6);
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


void IWDGInit() {
	// Erişim açılıyor.
    IWDG->KR = 0x5555;

    IWDG->PR = 4;    // Prescaler: 4
    IWDG->RLR = WDG_RELOAD_VAL; // Reload degeri: 1000

    // Watchdog counterı yenile
    IWDG_Refresh();

    // Sayıcı saymaya başlıyor.
    IWDG->KR = 0xCCCC;
}

void IWDG_Refresh(void) {
    // Watchdog counterı yenile
    IWDG->KR = WDG_RELOAD_COMMAND;
}

void check_counter(){
	counter++;
	if (counter==1000)
	{
		GPIOC->ODR ^= (1U << 6);
	}
}

void delay_ms(uint32_t delay)
{
    millis = 0;
    while (millis < delay) {
        // Wait for the specified duration
    }
}

void SysTick_Handler(void) {
    millis++; // Increment millis value
}
