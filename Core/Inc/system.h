#pragma once

void system_config(void) {
	// Enable RCC
	RCC->CR |= RCC_CR_HSEON;
	// Wait for RCC to become ready
	while(!(RCC->CR & RCC_CR_HSERDY));

	// Configure Power
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// Configure Flash
	FLASH->ACR |= FLASH_ACR_PRFTBE;
	FLASH->ACR |= FLASH_ACR_LATENCY_2;

	// Configure Clock
	RCC->CFGR |= RCC_CFGR_PLLNODIV;
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV;
	RCC->CFGR |= RCC_CFGR_PLLMUL9;
	RCC->CFGR |= RCC_CFGR_USBPRE_DIV1_5;
	//RCC->CFGR |= RCC_CFGR_MCO_PLL;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	// Enable PLL
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));
	// Configure System Clock
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	// Set prescaler for adc 1 and 2 to be 1
	RCC->CFGR2 |= 0x10 << 4;

	// Peripheral clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // GPIO A
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIO C
	RCC->AHBENR |= RCC_AHBENR_ADC12EN; // ADC 1 and 2
	RCC->AHBENR |= RCC_AHBENR_DMA2EN; // DMA1
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // USART2
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // TIM 7
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // TIM 7
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // SYSCFG

}
