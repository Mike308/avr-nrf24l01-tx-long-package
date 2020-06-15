/*
 * spi.h
 *
 *  Created on: 6 maj 2020
 *      Author: MIKE
 *      based on SPI library by Julien Delvaux
 */

#ifndef SPI_SPI_H_
#define SPI_SPI_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#define MOSI PB5	//   <---- A (SER IN)
#define SCK PB7		//   <---- SHIFT CLOCK (SC)
#define CS PB4		//	 <---- LATCH CLOCK (LC)

#define TX_BUFF_SIZE 64
#define TX_BUFF_MASK (TX_BUFF_SIZE - 1)
#define RX_BUFF_SIZE 64
#define RX_BUFF_MASK (RX_BUFF_SIZE - 1)


#define SPI_ACTIVE			0 // SS Pin put Low
#define SPI_INACTIVE		1 // SS Pin put High

#define SPI_PIN_SS		2		//SS PIN
#define SPI_PIN_MOSI	3		//MOSI PIN
#define SPI_PIN_MISO	4		//MISO PIN
#define SPI_PIN_SCK		5		//SCK PIN
#define SPI_DDR			DDRB	//SPI on PORTB
#define SPI_PORT		PORTB	//SPI on PORTB

#define SPI_CLOCK_DIV4		0x00
#define SPI_CLOCK_DIV16		0x01
#define SPI_CLOCK_DIV64		0x02
#define SPI_CLOCK_DIV128	0x03
#define SPI_CLOCK_DIV2		0x04
#define SPI_CLOCK_DIV8		0x05
#define SPI_CLOCK_DIV32		0x06

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

void spi_init(uint8_t mode, uint8_t clock);

uint8_t spi_transfer(uint8_t data);


#endif /* SPI_SPI_H_ */
