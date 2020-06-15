/*
 * main.c
 *
 *  Created on: 15 cze 2020
 *      Author: MIKE
 */

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "spi/spi.h"
#include "uart/uart.h"
#include "timeout/timeout.h"
#include "nrf24l01/nrf24l01.h"
#include "nrf24l01/nrf24l01_memory_map.h"

unsigned long previus_millis = 0;
uint8_t status = 0xFF;
uint8_t init_status;
uint64_t addr =  0xF0F0F0F0F101;
uint8_t cnt = 0;
char print_buff[32];
void error_handler(char *str);

int main(void) {
	DDRD |= (1<<7);
	PORTD &= ~(1<<7);
	timeout_init();
	init_status = nrf24l01_init();
	nrf24l01_init_external_interrupt();
	nrf24l01_open_wrting_pipe(addr);
	nrf24l01_open_reading_pipe(1, 0xF0F0F0F0E101);
	nrf24l01_set_pa_level(RF24_PA_MAX);
	uart_init(UBRR_VAL);
	sei();
	uart_putsn("SPI TEST");
	uart_puts("Init result: ");
	uart_putint(init_status, 16);
	uart_putsn("");
	status = nrf24l01_get_status();
	sprintf(print_buff, "STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n", status,
			(status & _BV(RX_DR))?1:0,
					(status & _BV(TX_DS))?1:0,
							(status & _BV(MAX_RT))?1:0,
									((status >> RX_P_NO) & 0x07),
									(status & _BV(TX_FULL))?1:0);
	uart_puts(print_buff);
	nrf24l01_print_reg_addr("TX_ADDR: ", TX_ADDR);
	nrf24l01_print_reg_addr("RX_ADDR: ", RX_ADDR_P1);

	nrf24l01_set_start_char('#');
	nrf24l01_set_end_char('$');
	nrf24l01_set_separator('-');
	nrf24l01_set_on_error_callback(error_handler);

	while (1) {
		nrf24l01_write_long_package("This message is too long to send via NRF24L01 and so far working fine and so far working fine and so far working fine");
		PORTD ^= (1<<7);
		_delay_ms(1000);

	}

}

void error_handler(char *str) {
	uart_puts("Error: ");
	uart_putsn(str);
}
