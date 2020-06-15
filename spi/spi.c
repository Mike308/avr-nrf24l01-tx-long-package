/*
 * spi.c
 *
 *  Created on: 6 maj 2020
 *      Author: MIKE
 *      based on SPI library by Julien Delvaux
 */

#include "spi.h"

volatile uint8_t spi_tx_buff[TX_BUFF_SIZE];
volatile uint8_t spi_tx_buff_mask;
volatile uint8_t spi_rx_buff[RX_BUFF_SIZE];
volatile uint8_t spi_rx_buff_mask;
volatile uint8_t cts;
volatile uint8_t bytes_request;
volatile uint8_t spi_tx_head;
volatile uint8_t spi_tx_tail;
volatile uint8_t spi_rx_head;
volatile uint8_t spi_rx_tail;

volatile uint8_t tx_flag = 0;
volatile uint8_t rx_flag = 0;


void spi_init(uint8_t mode, uint8_t clock) {
	// Pin Configuration
	SPI_DDR |= (1<<SPI_PIN_SS);
	SPI_PORT|= (1<<SPI_PIN_SS);

	cts	 = SPI_INACTIVE;
	// Set MOSI and SCK output, all others input
	SPI_DDR |= (1<<SPI_PIN_MOSI)|(1<<SPI_PIN_SCK);
	// Enable SPI, Master, set clock rate
	SPCR = (1<<SPE)|(1<<MSTR)|(mode<<CPHA)|(clock<<SPR0);
}



uint8_t spi_transfer(uint8_t data) {
	SPDR = data;
	/*
	 * The following NOP introduces a small delay that can prevent the wait
	 * loop form iterating when running at the maximum speed. This gives
	 * about 10% more speed, even if it seems counter-intuitive. At lower
	 * speeds it is unnoticed.
	 */
	asm volatile("nop");
	while (!(SPSR & _BV(SPIF))) ; // wait
	return SPDR;
}


