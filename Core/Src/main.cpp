#include <stm32f303xe.h>
#include <array>

#include "usart.h"
#include "printf.h"
#include "system.h"

constexpr size_t n_channels = 3;
constexpr size_t n_samples = 512;

// 1024 measurements for each channel
std::array<uint16_t, n_channels * n_samples> measurements;
volatile bool do_measurement = false;
bool led_on = false;

constexpr size_t max_packet_size = 16;
struct USARTRXStateMachine {
	uint16_t n; // What byte we are on
	uint16_t packet_size_temp; // Not including the packet_size
	uint16_t packet_size; // Not including the packet_size
	uint8_t data[max_packet_size]; // Data from the transmission
	uint16_t ignore_n; // Number of bytes to ignore due to over sized data
	bool ready;
	bool handled;
};
volatile USARTRXStateMachine rx_state = {};

extern "C" {
	void EXTI15_10_IRQHandler() {
		do_measurement = !do_measurement;
		EXTI->PR = 1 << 13;
	}

	void USART2_IRQHandler() {
		// Read one byte from the transmission
		uint8_t data = *(__IO uint8_t*)&USART2->RDR;

		// Skip if we need to ignore bytes
		if(rx_state.ignore_n > 0) {
			//USART2->ICR |= USART_ICR_RTOCF
			rx_state.ignore_n--;
			rx_state.n = 0;
			return;
		}

		switch(rx_state.n) {
		case 0:
			// byte is lower half of packet size
			rx_state.packet_size_temp = data;
			break;
		case 1:
			// byte is upper half of packet size
			((uint8_t*)&rx_state.packet_size_temp)[1] = data;
			// If there is an overrun or the previous packet not handled, ignore all data
			if(rx_state.packet_size > max_packet_size || !rx_state.handled) {
				rx_state.ignore_n = rx_state.packet_size_temp;
			} else {
				rx_state.handled = false;
				rx_state.ready = false;
				rx_state.packet_size = rx_state.packet_size_temp;
			}
			break;
		default:
			// byte is a part of the transmission data
			rx_state.data[rx_state.n-2] = data;
			// Check if this is the last byte in the packet
			if(rx_state.n - 1 == rx_state.packet_size) {
				rx_state.ready = true;
				rx_state.n = 0;
				return;
			}
		}
		rx_state.n++;
	}
}


int main() {
	system_config();

	GPIOA->MODER |= 1 << 10;
	GPIOA->OSPEEDR |= 3 << 10;

	// Enable external interrupts for C13
	SYSCFG->EXTICR[3] |= 2 << 4;
	EXTI->IMR |= 1 << 13;
	EXTI->FTSR |= 1 << 13;
	NVIC_SetPriority(EXTI15_10_IRQn, 0);
	NVIC_EnableIRQ(EXTI15_10_IRQn);


	// Configure C13
	GPIOC->OSPEEDR |= 3 << 26;
	GPIOC->PUPDR |= 2 << 26;

	USART_init();

	// Set up A4-A6 to be analog input
	GPIOA->MODER |= 0x3F << 8;

	// Set sampling times
	constexpr size_t sample = 3;
	ADC2->SMPR1 |= sample << 3 | sample << 6 | sample << 9; // 1.5 cycles
	ADC2->SQR1 |= 1 << 6 | 2 << 12 | 3 << 18; // enable channel 1, 2 and 3
	ADC2->SQR1 |= 3 - 1; // 3 Channels in sequence

	// Configure DMA
	ADC2->CFGR |= ADC_CFGR_DMAEN;
	// Auto increment memory
	DMA2_Channel1->CCR |= DMA_CCR_MINC;
	DMA2_Channel1->CCR |= 1 << 10 | 1 << 8; // 16 bit mem size for both peripheral and buffer
	DMA2_Channel1->CPAR = reinterpret_cast<uint32_t>(&ADC2->DR); // Read from ADC2 DR

	// Configure Timer for ADC
	ADC2->CFGR |= 0xD << 6; // TIM 6 TRGO
	// Enable Timer on rising edge
	ADC2->CFGR |= 1 << 10;
	// Set Timer to have 1 us resolution
	TIM6->PSC = 72 - 1; // 72 MHz in -> 1 MHz out
	TIM6->ARR = 10; // Period of ADC in us, 100 KHz = 10 us
	// Enable TRGO on reload
	TIM6->CR2 |= 0x2 << 4;
	TIM6->EGR |= TIM_EGR_UG;

	// Enable ADC
	ADC2->CR |= ADC_CR_ADEN;
	while(!(ADC2->ISR & ADC_ISR_ADRDY));
	TIM7->ARR = 0xFFFF;
	TIM7->PSC = 72-1;
	TIM7->CR1 |= TIM_CR1_CEN;
	rx_state.handled = true;
	uint16_t arr = 50000;
	for(;;) {
		__disable_irq();
		if(rx_state.ready) {
			arr = *(uint16_t*)rx_state.data;
			rx_state.handled = true;
		}
		__enable_irq();
		if(do_measurement && TIM7->CNT > arr) {
			// Configure DMA
			DMA2_Channel1->CMAR = reinterpret_cast<uint32_t>(measurements.data());
			DMA2_Channel1->CNDTR = measurements.size();
			// Enable DMA and timer
			DMA2_Channel1->CCR |= DMA_CCR_EN;
			TIM6->CR1 |= TIM_CR1_CEN;
			TIM6->EGR |= TIM_EGR_UG;
			ADC2->CR |= ADC_CR_ADSTART;
			// Wait for DMA to complete
			while(!(DMA2->ISR & DMA_ISR_TCIF1));
			TIM6->CR1 &= ~TIM_CR1_CEN;
			DMA2_Channel1->CCR &= ~DMA_CCR_EN;
			DMA2->IFCR |= DMA_IFCR_CTCIF1;
			// Upload results to UART
			USART_write((uint16_t)measurements.size());
			USART_send_buff(std::span(reinterpret_cast<uint8_t*>(measurements.data()), measurements.size()*2));
			TIM7->CNT = 0;
		}
	}
}
