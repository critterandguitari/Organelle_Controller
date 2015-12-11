/*
 * uart.c
 *
 *  Created on: May 23, 2014
 *      Author: owen
 */

#include "uart.h"
#include "BlinkLed.h"

uint8_t uart2_recv_buf[UART2_BUFFER_SIZE];
uint16_t uart2_recv_buf_head = 0;
uint16_t uart2_recv_buf_tail = 0;

void uart2_init(void) {

	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	USART_StructInit(&USART_InitStructure);


	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	//Configure USART2 pins:  Rx and Tx ----------------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Configure USART2 setting:         ----------------------------
	USART_InitStructure.USART_BaudRate = 500000;
	//USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	/* Enable USART1 IRQ */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);
}

void uart2_send(uint8_t c) {
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		; // Wait for Empty

	USART_SendData(USART2, c);
}

int uart2_available(void) {
	return (int) (UART2_BUFFER_SIZE + uart2_recv_buf_head - uart2_recv_buf_tail)
			% UART2_BUFFER_SIZE;
}

int uart2_peek(void) {
	if (uart2_recv_buf_head == uart2_recv_buf_tail) {
		return -1;
	} else {
		return uart2_recv_buf[uart2_recv_buf_tail];
	}
}

int uart2_read(void) {
	// if the head isn't ahead of the tail, we don't have any characters
	if (uart2_recv_buf_head == uart2_recv_buf_tail) {
		return -1;
	} else {
		unsigned char c = uart2_recv_buf[uart2_recv_buf_tail];
		uart2_recv_buf_tail = (unsigned int) (uart2_recv_buf_tail + 1)
				% UART2_BUFFER_SIZE;
		return c;
	}
}

void USART2_IRQHandler(void) {

	// check if the USART1 receive interrupt flag was set

	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {

		uart2_recv_buf[uart2_recv_buf_head] = USART_ReceiveData(USART2);

		uart2_recv_buf_head++;
		uart2_recv_buf_head %= UART2_BUFFER_SIZE;  //

	}
}
