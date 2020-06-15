/*
 * uart.h
 *
 *  Created on: 3 maj 2020
 *      Author: MIKE
 */

#ifndef UART_UART_H_
#define UART_UART_H_

#define BAUD 9600
#define UBRR_VAL F_CPU/16/BAUD-1

#define USE_FLOAT 1
#if USE_DOUBLE
#define USE_SPRINTF 1
#endif


#define UART_TX_BUFF_SIZE 32
#define UART_TX_BUFF_MASK (UART_TX_BUFF_SIZE  - 1)

#define UART_RX_BUFF_SIZE 32
#define UART_RX_BUFF_MASK (UART_RX_BUFF_SIZE  - 1)


#if USE_SPRINTF == 1
#include <string.h>
#endif

#if USE_FLOAT == 1
#include <stdlib.h>
#endif

#if USE_DOUBLE == 1
	void uart_putdobule(double value);
#endif


void uart_init(uint16_t baud);
void uart_putc(char c);
void uart_puts(char *str);
void uart_putsn(char *str);
void uart_putint(int value, int radix);
char uart_getc();
void uart_gets(char *str);
void uart_on_received(char *str);
void uart_set_on_received_callback(void (*callback)(char *str));




#endif /* UART_UART_H_ */
