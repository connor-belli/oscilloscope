#include "usart.h"


#include <array>
#include <cstring>



void USART_init() {
	// GPIO Initialization
	// Set GPIO to be alternate pins
	GPIOA->MODER |= (2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER3_Pos);
	// Set GPIO speed to high speed
	GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDER_OSPEEDR2_Pos) | (3 << GPIO_OSPEEDER_OSPEEDR3_Pos);
	// Set alternate function to USART1
	GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFRL2_Pos) | (7 << GPIO_AFRL_AFRL3_Pos);

	// Clear USART control register
	USART2->CR1 = 0;
	// Set Baud rate to 400,000
	USART2->BRR = 90;
	// Enable Receiving Interrupts
	USART2->CR1 |= USART_CR1_RXNEIE;

	NVIC_SetPriority(USART2_IRQn, 0);
	NVIC_EnableIRQ(USART2_IRQn);

	// Enable USART1
	USART2->CR1 |= USART_CR1_UE;
	// Enable transfers
	USART2->CR1 |= USART_CR1_TE;
	// Enable receiving
	USART2->CR1 |= USART_CR1_RE;
}

void USART_send_buff(std::span<const uint8_t> buff) {
	// Wait for current transfer to complete
	USART_send_buff_unsafe(buff);
}

void USART_send_buff_cancelable(std::span<const uint8_t> buff) {
	// Cancel if current transfer is not complete
	USART_send_buff_unsafe(buff);
}

void USART_send_buff_unsafe(std::span<const uint8_t> buff) {
	const uint8_t* data = buff.data();
	size_t size = buff.size();
	USART_write_ptr<uint8_t>(data, size);
	// Reset USART transfer complete flag to start DMA transfer
}

void USART_console_data(std::span<const uint8_t> buff) {
	USART_write<uint8_t>(0); // Packet type console data
	USART_write<uint16_t>(buff.size()); // Write message size
	USART_write_ptr<uint8_t>(buff.data(), buff.size()); // Gyro Data

}

void USART_gyro_data(float data[3]) {
	USART_write<uint8_t>(1); // Packet type gyro data
	USART_write_ptr<float>(data, 3); // Gyro Data
}




