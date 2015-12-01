/*
 * uart.h
 *
 *  Created on: May 23, 2014
 *      Author: owen
 */

#ifndef UART_H_
#define UART_H_

#define UART2_BUFFER_SIZE 256

#include "stm32f0xx.h"

void uart2_init(void);
void uart2_send(uint8_t c);
int uart2_available(void);
int uart2_peek(void);
int uart2_read(void);

#endif /* UART_H_ */
