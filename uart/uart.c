/*
 * uart.c
 *
 *  Created on: 3 maj 2020
 *      Author: MIKE
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"

volatile char tx_buff[UART_TX_BUFF_SIZE];
volatile uint8_t tx_head;
volatile uint8_t tx_tail;

volatile char rx_buff[UART_RX_BUFF_SIZE];
volatile uint8_t rx_head;
volatile uint8_t rx_tail;
volatile uint8_t line;

void (*rx_callback)(char * str);


void uart_init(uint16_t baud) {
	UBRR0H = (uint8_t)(baud>>8);
	UBRR0L = (uint8_t)baud;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);

}


void uart_putc(char c) {
	uint8_t tmp_head;
	tmp_head = (tx_head + 1) & UART_TX_BUFF_MASK;
	while (tmp_head == tx_tail);
	tx_buff[tmp_head] = c;
	tx_head = tmp_head;
	UCSR0B |= (1<<UDRIE0);
}

void uart_puts(char *str) {
	register char c;
	while((c = *str++)) uart_putc(c);
}


void uart_putsn(char *str) {
	uart_puts(str);
	uart_puts("\r\n");
}

void uart_putint(int val, int radix) {
	char buff[16];
#if USE_SPRINTF
	sprintf(buff, "%d", val);
#else
	itoa(val, buff, radix);
#endif
	uart_puts(buff);
}

#if USE_FLOAT
	void uart_putdouble(double val) {
		char buff[16];
		dtostrf(val, 4, 1, buff);
		uart_puts(buff);
	}
#endif


char uart_getc() {
	if (rx_head == rx_tail) return 0;
	rx_tail = (rx_tail + 1) & UART_RX_BUFF_MASK;
	return rx_buff[rx_tail];
}

void uart_gets(char *str) {
	char c;
	if (line) {
		while ((c = uart_getc())) {
			if (13 == c || c < 0) break;
			*str++ = c;
		}
		*str = 0;
		line--;
	}
}

void uart_on_received(char *str) {
	if (line) {
		uart_gets(str);
		(*rx_callback)(str);
	}
}

void uart_set_on_received_callback(void (*callback)(char *str)) {
	rx_callback = callback;
}


ISR(USART_UDRE_vect) {
	if (tx_head != tx_tail) {
		tx_tail = (tx_tail + 1) & UART_TX_BUFF_MASK;
		UDR0 = tx_buff[tx_tail];
	} else {
		UCSR0B &= ~(1<<UDRIE0);
	}
}

ISR(USART_RX_vect) {
	uint8_t temp_head;
	char data;
	data = UDR0;
	temp_head = (rx_head + 1) & UART_RX_BUFF_MASK;

	if (temp_head == rx_tail) {

	} else {
		switch (data) {
		case 0:
		case 10: break;
		case 13: line++;
		default:
			rx_head = temp_head;
			rx_buff[temp_head] = data;
		}
	}
}
