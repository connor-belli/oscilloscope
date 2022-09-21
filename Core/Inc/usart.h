#pragma once
#include <span>
#include <stdint.h>
#include "stm32f303xe.h"

// USART asynchronous communication
// Uses USART2, DMA1 Channel 4, GPIO A2, and GPIO A3

/* USART protocol
 * First byte of message is the packet type
 * Data is then sent afterwards (varies by packet type)
 * List of packet types:
 * 0: Console string - Prints a message to console
 * struct {
 *	uint16_t size;
 *	char data[size];
 * }
 * 1: Gyro data
 * struct {
 * 	float x, y, z;
 * }
 */

// Creates span from string literal
template<size_t N>
constexpr std::span<const uint8_t> s2s(const char (&str)[N]) {
	return std::span{reinterpret_cast<const uint8_t *>(str), N};
}

template<class T>
void USART_write(const T&& v) {
	constexpr size_t s = sizeof(T);
	const uint8_t* data = (uint8_t*)&v;
	for(size_t i = 0; i < s; i++) {
		while(!(USART2->ISR & USART_ISR_TC));
		*((__IO uint8_t*) &USART2->TDR) = data[i];
	}
}

template<class T>
void USART_write_ptr(const T* v, size_t n) {
	constexpr size_t size = sizeof(T);
	size_t s = size*n;
	const uint8_t* data = (uint8_t*)v;
	for(size_t i = 0; i < s; i++) {
		while(!(USART2->ISR & USART_ISR_TXE));
		USART2->TDR = data[i];
		USART2->ISR &= ~USART_ISR_TXE;
	}
}

// Initializes the USART bus
// REQUIREMENTS: DMA1, GPIOC, and USART1 clock enabled
void USART_init();

// Sends buffer via DMA
// Waits for current transfer to complete before sending
// If the size of buff > UART_BUFF_SIZE, UART_BUFF_SIZE bytes will be transfered
void USART_send_buff(std::span<const uint8_t> buff);

// Attempts to send buffer via DMA
// If there is an incomplete transfer, cancel the request
// If the size of buff > UART_BUFF_SIZE, UART_BUFF_SIZE bytes will be transfered
void USART_send_buff_cancelable(std::span<const uint8_t> buff);

// Sends buffer without checking if a transfer is occurring
// If the size of buff > UART_BUFF_SIZE, UART_BUFF_SIZE bytes will be transfered
void USART_send_buff_unsafe(std::span<const uint8_t> buff);

void USART_console_data(std::span<const uint8_t> buff);

void USART_gyro_data(float data[3]);

