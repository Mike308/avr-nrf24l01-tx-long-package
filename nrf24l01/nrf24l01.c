/*
 * nrf24l01.c
 *
 *  Created on: 8 maj 2020
 *      Author: MIKE
 */

/*
 * BASED ON RF24 library https://github.com/nRF24/RF24
 */


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "nrf24l01.h"
#include "nrf24l01_config.h"
#include "nrf24l01_memory_map.h"
#include "../spi/spi.h"
#include "../timeout/timeout.h"
#if DEBUG
#include "../uart/uart.h"
#endif

void nrf24l01_csn(uint8_t mode);
void nrf24l01_ce(uint8_t mode);

void nrf24l01_start_trans(void);
void nrf24l01_end_trans(void);

uint8_t nrf24l01_read_register_buff(uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t nrf24l01_read_register(uint8_t reg);

uint8_t nrf24l01_write_register_buff(uint8_t reg, const uint8_t* buf, uint8_t len);
uint8_t nrf24l01_write_register(uint8_t reg, uint8_t value);

uint8_t nrf24l01_write_payload(const void* buf, uint8_t data_len, const uint8_t write_type);
uint8_t nrf24l01_read_payload(void* buf, uint8_t data_len);

uint8_t nrf24l01_spi_trans(uint8_t cmd);

uint8_t nrf24l01_flush_rx(void);
uint8_t nrf24l01_flush_tx(void);

uint8_t nrf24l01_get_status(void);

void nrf24l01_set_channel(uint8_t channel);
uint8_t nrf24l01_get_channel();

void nrf24l01_set_retries(uint8_t delay, uint8_t count);

uint8_t nrf24l01_set_data_rate(rf24_datarate_e speed);

void nrf24l01_toggle_features(void);

void nrf24l01_re_use_tx(void);

void nrf24l01_re_use_tx(void);

uint8_t dynamic_payloads_enabled = 0;
uint8_t payload_size = 32;
uint32_t tx_delay;
uint8_t p_variant = 0;
uint8_t addr_width = 5;
uint8_t pipe0_reading_address[5];
static void (*rx_callback)(void *buff);

#if USE_RX_LONG_PACKAGE
static void (*rx_long_pocket_callback)(char *buff);
#endif
#if USE_RX_LONG_PACKAGE || USE_TX_LONG_PACKAGE
static char start_frame_char;
static char end_frame_char;
#endif
#if USE_TX_LONG_PACKAGE
static char separator[] = "-";
#endif

#if FAILURE_HANDLING
static void (*error_callback)(char *str);
#endif


static const uint8_t child_pipe[] PROGMEM =
{
		RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};

static const uint8_t child_payload_size[] PROGMEM =
{
		RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};


static const uint8_t child_pipe_enable[] PROGMEM =
{
		ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};


void nrf24l01_csn(uint8_t mode) {
	if (mode) {
		SPI_PORT |= (1<<SPI_PIN_SS);
		_delay_us(5);
	} else {
		SPI_PORT &= ~(1<<SPI_PIN_SS);
		_delay_us(5);
	}
}

void nrf24l01_ce(uint8_t mode) {
	if (mode) {
		CE_PORT |= CE_PIN;
	} else {
		CE_PORT &= ~CE_PIN;
	}
}

void nrf24l01_start_trans(void) {
	nrf24l01_csn(0);
}

void nrf24l01_end_trans(void) {
	nrf24l01_csn(1);
}

uint8_t nrf24l01_read_register_buff(uint8_t reg, uint8_t* buf, uint8_t len) {
	uint8_t status;
	//	nrf24l01_start_trans();
	nrf24l01_csn(0);
	status = spi_transfer(R_REGISTER | ( REGISTER_MASK & reg ));
	while ( len-- ){
		*buf++ = spi_transfer(0xff);
	}
	//	nrf24l01_end_trans();
	nrf24l01_csn(1);
	return status;
}

uint8_t nrf24l01_read_register(uint8_t reg) {
	uint8_t result;
	nrf24l01_csn(0);
	spi_transfer(R_REGISTER | ( REGISTER_MASK & reg ));
	result = spi_transfer(0xff);
	nrf24l01_csn(1);
	return result;
}


uint8_t nrf24l01_write_register_buff(uint8_t reg, const uint8_t* buf, uint8_t len) {
	uint8_t status;
	nrf24l01_csn(0);
	status = spi_transfer(W_REGISTER | ( REGISTER_MASK & reg ));
	while(len--)
		spi_transfer(*buf++);
	nrf24l01_csn(1);
	return status;
}

uint8_t nrf24l01_write_register(uint8_t reg, uint8_t value) {
	uint8_t status;
	nrf24l01_csn(0);
	status = spi_transfer( W_REGISTER | ( REGISTER_MASK & reg ));
	spi_transfer(value);
	nrf24l01_csn(1);
	return status;
}

uint8_t nrf24l01_write_payload(const void* buf, uint8_t data_len, const uint8_t write_type) {
	uint8_t status;
	const uint8_t* current = (const uint8_t*)(buf);

	data_len = rf24_min(data_len, payload_size);
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
	nrf24l01_csn(0);
	status = spi_transfer(write_type);
	while ( data_len-- ) {
		spi_transfer(*current++);
	}
	while ( blank_len-- ) {
		spi_transfer(0);
	}
	nrf24l01_csn(1);
	return status;
}

uint8_t nrf24l01_read_payload(void* buf, uint8_t data_len) {
	uint8_t status;
	uint8_t* current = (uint8_t *)(buf);

	if(data_len > payload_size) data_len = payload_size;
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

	nrf24l01_csn(0);
	status = spi_transfer( R_RX_PAYLOAD );
	while ( data_len-- ) {
		*current++ = spi_transfer(0xFF);
	}
	while ( blank_len-- ) {
		spi_transfer(0xff);
	}
	nrf24l01_csn(1);
	return status;
}

uint8_t nrf24l01_spi_trans(uint8_t cmd) {
	uint8_t status;
	nrf24l01_csn(0);
	status = spi_transfer(cmd);
	nrf24l01_csn(1);
	return status;
}

uint8_t nrf24l01_flush_rx(void) {
	uint8_t status = nrf24l01_spi_trans(FLUSH_RX);
	return status;
}

uint8_t nrf24l01_flush_tx(void) {
	uint8_t status = nrf24l01_spi_trans(FLUSH_TX);
	return status;
}

uint8_t nrf24l01_get_status(void) {
	uint8_t status = nrf24l01_spi_trans(RF24_NOP);
	return status;
}

void nrf24l01_set_channel(uint8_t channel) {
	nrf24l01_write_register(RF_CH, rf24_min(channel, 125));
}

uint8_t nrf24l01_get_channel() {
	uint8_t result = nrf24l01_read_register(RF_CH);
	return result;
}

uint8_t nrf24l01_init(void) {
	spi_init(SPI_MODE0, SPI_CLOCK_DIV4);
	CE_DDR |= CE_PIN;
	uint8_t setup = 0;
	pipe0_reading_address[0] = 0;
	nrf24l01_ce(0);
	nrf24l01_csn(1);
	// Must allow the radio time to settle else configuration bits will not necessarily stick.
	// This is actually only required following power up but some settling time also appears to
	// be required after resets too. For full coverage, we'll always assume the worst.
	// Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
	// Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
	// WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
	_delay_ms( 5 ) ;

	// Reset NRF_CONFIG and enable 16-bit CRC.
	nrf24l01_write_register( NRF_CONFIG, 0x0C ) ;

	// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	// WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	// sizes must never be used. See documentation for a more complete explanation.
	nrf24l01_set_retries(5,15);

	// Reset value is MAX
	//setPALevel( RF24_PA_MAX ) ;

	// check for connected module and if this is a p nRF24l01 variant
	//

	if( nrf24l01_set_data_rate( RF24_250KBPS ) )
	{
		p_variant = 1 ;
	}
	setup = nrf24l01_read_register(RF_SETUP);
	/*if( setup == 0b00001110 )     // register default for nRF24L01P
		  {
		    p_variant = true ;
		  }*/

	// Then set the data rate to the slowest (and most reliable) speed supported by all
	// hardware.
	nrf24l01_set_data_rate( RF24_1MBPS ) ;

	// Initialize CRC and request 2-byte (16bit) CRC
	//setCRCLength( RF24_CRC_16 ) ;

	// Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
	nrf24l01_toggle_features();
	nrf24l01_write_register(FEATURE,0 );
	nrf24l01_write_register(DYNPD,0);
	dynamic_payloads_enabled = 0;

	// Reset current status
	// Notice reset and flush is the last thing we do
	nrf24l01_write_register(NRF_STATUS,(1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT) );

	// Set up default configuration.  Callers can always change it later.
	// This channel should be universally safe and not bleed over into adjacent
	// spectrum.
	nrf24l01_set_channel(76);

	// Flush buffers
	nrf24l01_flush_rx();
	nrf24l01_flush_tx();

	nrf24l01_power_up(); //Power up by default when begin() is called

	// Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
	// PTX should use only 22uA of power
	nrf24l01_write_register(NRF_CONFIG, ( nrf24l01_read_register(NRF_CONFIG) ) & ~(1<<PRIM_RX) );
	uart_puts("Inner status: ");
	uart_putint(setup, 16);
	uart_putsn("");
	// if setup is 0 or ff then there was no response from module
	return ( setup != 0 && setup != 0xff );
}

void nrf24l01_set_payload_size(uint8_t size) {
	payload_size = rf24_min(size, 32);
}

uint8_t nrf24l01_get_payload_size() {
	return payload_size;
}

void nrf24l01_set_retries(uint8_t delay, uint8_t count) {
	nrf24l01_write_register(SETUP_RETR, (delay&0xf)<<ARD | (count&0xf)<<ARC);
}


void nrf24l01_disable_crc(void) {
	uint8_t disable = nrf24l01_read_register(NRF_CONFIG) & ~(1<<EN_CRC);
	nrf24l01_write_register(NRF_CONFIG, disable);
}

void nrf24l01_set_crc_lenght(rf24_crclength_e length) {
	uint8_t config = nrf24l01_read_register(NRF_CONFIG) & ~( (1<<CRCO) | (1<<EN_CRC));

	// switch uses RAM (evil!)
	if (length == RF24_CRC_DISABLED) {
		// Do nothing, we turned it off above.
	} else if (length == RF24_CRC_8) {
		config |= (1<<EN_CRC);
	} else {
		config |= (1<<EN_CRC);
		config |= (1<<CRCO);
	}
	nrf24l01_write_register( NRF_CONFIG, config);
}

uint8_t nrf24l01_set_data_rate(rf24_datarate_e speed) {
	uint8_t result = 0;
	uint8_t setup = nrf24l01_read_register(RF_SETUP);

	// HIGH and LOW '00' is 1Mbs - our default
	setup &= ~((1<<RF_DR_LOW) | (1<<RF_DR_HIGH));

#if !defined(F_CPU) || F_CPU > 20000000
	tx_delay=250;
#else //16Mhz Arduino
	tx_delay = 85;
#endif
	if (speed == RF24_250KBPS) {
		// Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
		// Making it '10'.
		setup |= (1<<RF_DR_LOW);
#if !defined(F_CPU) || F_CPU > 20000000
		tx_delay=450;
#else //16Mhz Arduino
		tx_delay = 155;
#endif
	} else {
		// Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
		// Making it '01'
		if (speed == RF24_2MBPS) {
			setup |= (1<<RF_DR_HIGH);
#if !defined(F_CPU) || F_CPU > 20000000
			tx_delay=190;
#else //16Mhz Arduino
			tx_delay = 65;
#endif
		}
	}
	nrf24l01_write_register(RF_SETUP, setup);

	// Verify our result
	if (nrf24l01_read_register(RF_SETUP) == setup) {
		result = 1;
	}
	return result;
}

rf24_datarate_e nrf24l01_get_data_rate(void) {
	rf24_datarate_e result;
	uint8_t data_rate = nrf24l01_read_register(RF_SETUP) & ((1<<RF_DR_LOW) | (1<<RF_DR_HIGH));
	// switch uses RAM (evil!)
	// Order matters in our case below
	if (data_rate == (1<<RF_DR_LOW)) {
		// '10' = 250KBPS
		result = RF24_250KBPS;
	} else if (data_rate == (1<<RF_DR_HIGH)) {
		// '01' = 2MBPS
		result = RF24_2MBPS;
	} else {
		// '00' = 1MBPS
		result = RF24_1MBPS;
	}
	return result;
}

void nrf24l01_set_pa_level(uint8_t level) {
	uint8_t setup = nrf24l01_read_register(RF_SETUP) & 0xF8;
	if (level > 3) {  						// If invalid level, go to max PA
		level = (RF24_PA_MAX << 1) + 1;	// +1 to support the SI24R1 chip extra bit
	} else {
		level = (level << 1) + 1;	 		// Else set level as requested
	}
	nrf24l01_write_register( RF_SETUP, setup |= level);	// Write it to the chip
}

uint8_t nrf24l01_get_pa_level(void) {
	return (nrf24l01_read_register(RF_SETUP) & ((1<<RF_PWR_LOW) | (1<<RF_PWR_HIGH))) >> 1 ;
}

uint8_t nrf24l01_test_rpd(void) {
	return ( nrf24l01_read_register(RPD) & 1 ) ;
}

uint8_t nrf24l01_test_carrier(void) {
	return ( nrf24l01_read_register(CD) & 1 );
}

void nrf24l01_set_auto_ack_of_pipe(uint8_t pipe, uint8_t enable) {
	if (pipe <= 6) {
		uint8_t en_aa = nrf24l01_read_register( EN_AA);
		if (enable) {
			en_aa |= (1<<pipe);
		} else {
			en_aa &= ~(1<<pipe);
		}
		nrf24l01_write_register( EN_AA, en_aa);
	}
}

void nrf24l01_set_auto_ack(uint8_t enable) {
	if (enable)
		nrf24l01_write_register(EN_AA, 0x3F);
	else
		nrf24l01_write_register(EN_AA, 0);
}

uint8_t nrf24l01_is_p_variant(void) {
	return p_variant;
}

uint8_t nrf24l01_is_ack_payload_available(void) {
	return !(nrf24l01_read_register(FIFO_STATUS) & (1<<RX_EMPTY));
}

void nrf24l01_write_ack_payload(uint8_t pipe, const void* buf, uint8_t len) {
	const uint8_t* current = (const uint8_t*)(buf);
	uint8_t data_len = rf24_min(len,32);
	spi_transfer(W_ACK_PAYLOAD | ( pipe & 0x07 ));

	while(data_len--)
		spi_transfer(*current++);
}

void nrf24l01_enable_dynamic_ack_payload(void) {
	nrf24l01_write_register(FEATURE, nrf24l01_read_register(FEATURE) | (1<<EN_DYN_ACK) );
}

void nrf24l01_enable_ack_payload(void) {
	// enable ack payload and dynamic payload features

	nrf24l01_write_register(FEATURE,nrf24l01_read_register(FEATURE) | (1<<EN_ACK_PAY) | (1<<EN_DPL) );

	// Enable dynamic payload on pipes 0 & 1
	nrf24l01_write_register(DYNPD,nrf24l01_read_register(DYNPD) | (1<<DPL_P1) | (1<<DPL_P0));
	dynamic_payloads_enabled = 1;
}

void nrf24l01_disable_dynamic_payloads(void) {
	// Disables dynamic payload throughout the system.  Also disables Ack Payloads

	//toggle_features();
	nrf24l01_write_register(FEATURE, 0);

	// Disable dynamic payload on all pipes
	//
	// Not sure the use case of only having dynamic payload on certain
	// pipes, so the library does not support it.
	nrf24l01_write_register(DYNPD, 0);

	dynamic_payloads_enabled = 0;
}

void nrf24l01_enable_dynamic_payloads(void) {
	// Enable dynamic payload throughout the system

	//toggle_features();
	nrf24l01_write_register(FEATURE,nrf24l01_read_register(FEATURE) | (1<<EN_DPL) );

	// Enable dynamic payload on all pipes
	//
	// Not sure the use case of only having dynamic payload on certain
	// pipes, so the library does not support it.
	nrf24l01_write_register(DYNPD,nrf24l01_read_register(DYNPD) | (1<<DPL_P5) | (1<<DPL_P4) | (1<<DPL_P3) | (1<<DPL_P2) | (1<<DPL_P1) | (1<<DPL_P0));

	dynamic_payloads_enabled = 1;
}

void nrf24l01_toggle_features(void) {
	spi_transfer( ACTIVATE );
	spi_transfer( 0x73 );
}

void nrf24l01_close_reading_pipe(uint8_t pipe) {
	nrf24l01_write_register(EN_RXADDR, nrf24l01_read_register(EN_RXADDR) & ~(1<<pgm_read_byte(&child_pipe_enable[pipe])));
}

void nrf24l01_open_reading_pipe_by_pointer(uint8_t child,
		const uint8_t *address) {
	// If this is pipe 0, cache the address.  This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
	if (child == 0) {
		memcpy(pipe0_reading_address, address, addr_width);
	}
	if (child <= 6) {
		// For pipes 2-5, only write the LSB
		if (child < 2) {
			nrf24l01_write_register_buff(pgm_read_byte(&child_pipe[child]),
					address, addr_width);
		} else {
			nrf24l01_write_register_buff(pgm_read_byte(&child_pipe[child]),
					address, 1);
		}
		nrf24l01_write_register(pgm_read_byte(&child_payload_size[child]),
				payload_size);

		// Note it would be more efficient to set all of the bits for all open
		// pipes at once.  However, I thought it would make the calling code
		// more simple to do it this way.
		nrf24l01_write_register(EN_RXADDR, nrf24l01_read_register(
				EN_RXADDR) | (1<<pgm_read_byte(&child_pipe_enable[child])));

	}
}

void nrf24l01_set_address_width(uint8_t width) {
	if(width -= 2){
		nrf24l01_write_register(SETUP_AW, width % 4);
		addr_width = (width % 4) + 2;
	}else{
		nrf24l01_write_register(SETUP_AW,0);
		addr_width = 2;
	}
}

void nrf24l01_open_reading_pipe(uint8_t child, uint64_t address) {
	// If this is pipe 0, cache the address.  This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
	if (child == 0) {
		memcpy(pipe0_reading_address, &address, addr_width);
	}

	if (child <= 6) {
		// For pipes 2-5, only write the LSB
		if (child < 2)
			nrf24l01_write_register_buff(pgm_read_byte(&child_pipe[child]),
					(const uint8_t*) (&address), addr_width);
		else
			nrf24l01_write_register_buff(pgm_read_byte(&child_pipe[child]),
					(const uint8_t*) (&address), 1);

		nrf24l01_write_register(pgm_read_byte(&child_payload_size[child]), payload_size);

		// Note it would be more efficient to set all of the bits for all open
		// pipes at once.  However, I thought it would make the calling code
		// more simple to do it this way.
		nrf24l01_write_register(EN_RXADDR,
				nrf24l01_read_register(
						EN_RXADDR) | (1<<pgm_read_byte(&child_pipe_enable[child])));
	}
}

void nrf24l01_open_writing_pipe_by_pointer(const uint8_t *address) {
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.

	nrf24l01_write_register_buff(RX_ADDR_P0 ,address, addr_width);
	nrf24l01_write_register_buff(TX_ADDR, address, addr_width);

	//const uint8_t max_payload_size = 32;
	//write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
	nrf24l01_write_register(RX_PW_P0,payload_size);
}

void nrf24l01_open_wrting_pipe(uint64_t value) {
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.
	nrf24l01_write_register_buff(RX_ADDR_P0, (uint8_t*)(&value), addr_width);
	nrf24l01_write_register_buff(TX_ADDR, (uint8_t*)(&value), addr_width);
	//const uint8_t max_payload_size = 32;
	//write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
	nrf24l01_write_register(RX_PW_P0,payload_size);
}

void nrf24l01_read(void *buff, uint8_t len) {
	nrf24l01_read_payload(buff, len);
	nrf24l01_write_register(NRF_STATUS,(1<<RX_DR) | (1<<MAX_RT) | (1<<TX_DS) );
}

uint8_t nrf24l01_available_by_pipe(uint8_t *pipe_num) {
	if (!( nrf24l01_read_register(FIFO_STATUS) & (1<<RX_EMPTY) )){
		// If the caller wants the pipe number, include that
		if ( pipe_num ){
			uint8_t status = nrf24l01_get_status();
			*pipe_num = ( status >> RX_P_NO ) & 0x07;
		}
		return 1;
	}
	return 0;
}

uint8_t nrf24l01_available() {
	return nrf24l01_available_by_pipe(NULL);
}

uint8_t nrf24l01_get_dynamic_payload_size(void) {
	uint8_t result = 0;
	nrf24l01_csn(0);
	spi_transfer( R_RX_PL_WID);
	result = spi_transfer(0xff);
	nrf24l01_csn(1);
	if (result > 32) {
		nrf24l01_flush_rx();
		_delay_ms(2);
		return 0;
	}
	return result;
}

void nrf24l01_mask_irq(uint8_t tx, uint8_t fail, uint8_t rx) {
	uint8_t config = nrf24l01_read_register(NRF_CONFIG);
	/* clear the interrupt flags */
	config &= ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR);
	/* set the specified interrupt flags */
	config |= fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR;
	nrf24l01_write_register(NRF_CONFIG, config);
}

void nrf24l01_stop_listening(void) {
	nrf24l01_ce(0);

	timeout_delay_us(tx_delay);

	if(nrf24l01_read_register(FEATURE) & (1<<EN_ACK_PAY)){
		timeout_delay_us(tx_delay); //200
		nrf24l01_flush_tx();
	}
	//flush_rx();
	nrf24l01_write_register(NRF_CONFIG, ( nrf24l01_read_register(NRF_CONFIG) ) & ~(1<<PRIM_RX) );

	nrf24l01_write_register(EN_RXADDR, nrf24l01_read_register(EN_RXADDR) | (1<<pgm_read_byte(&child_pipe_enable[0]))); // Enable RX on pipe0
}

uint8_t nrf24l01_tx_standby_with_timeout(uint32_t timeout, uint8_t start_tx) {
	if (start_tx) {
		nrf24l01_stop_listening();
		nrf24l01_ce(1);
	}
	uint32_t start = timeout_millis();

	while (!(nrf24l01_read_register(FIFO_STATUS) & (1<<TX_EMPTY))) {
		if (nrf24l01_get_status() & (1<<MAX_RT)) {
			nrf24l01_write_register(NRF_STATUS, (1<<MAX_RT));
			nrf24l01_ce(0);									//Set re-transmit
			nrf24l01_ce(1);
			if (timeout_millis() - start >= timeout) {
				nrf24l01_ce(0);
				nrf24l01_flush_tx();
				return 0;
			}
		}
	}
	nrf24l01_ce(0);				   //Set STANDBY-I mode
	return 1;
}

uint8_t nrf24l01_tx_standby(void) {
	while (!(nrf24l01_read_register(FIFO_STATUS) & (1<<TX_EMPTY))) {
		if (nrf24l01_get_status() & (1<<MAX_RT)) {
			nrf24l01_write_register(NRF_STATUS, (1<<MAX_RT));
			nrf24l01_ce(0);
			nrf24l01_flush_tx();    //Non blocking, flush the data
			return 0;
		}
	}
	nrf24l01_ce(0);			   //Set STANDBY-I mode
	return 1;
}

uint8_t nrf24l01_is_fifo_full(void) {
	return nrf24l01_read_register(FIFO_STATUS) & (1<<RX_FULL);
}

void nrf24l01_start_write(const void *buff, uint8_t len, const uint8_t multicast) {
	// Send the payload

	//write_payload( buf, len );
	nrf24l01_write_payload( buff , len,multicast? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
	nrf24l01_ce(1);
#if !defined(F_CPU) || F_CPU > 20000000
	_delay_us(10);
#endif
	nrf24l01_ce(0);
}

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void nrf24l01_start_fast_write(const void* buf, uint8_t len, const uint8_t multicast, uint8_t start_tx) {
	nrf24l01_write_payload( buf, len,multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
	if(start_tx){
		nrf24l01_ce(1);
	}
}


uint8_t nrf24l01_fast_write(const void* buf, uint8_t len, uint8_t multicast) {
	while( ( nrf24l01_get_status()  & ( (1<<TX_FULL) ))) {			  //Blocking only if FIFO is full. This will loop and block until TX is successful or fail
		if( nrf24l01_get_status() & (1<<MAX_RT)){
			//reUseTX();										  //Set re-transmit
			nrf24l01_write_register(NRF_STATUS,(1<<MAX_RT) );			  //Clear max retry flag
			return 0;										  //Return 0. The previous payload has been retransmitted
			//From the user perspective, if you get a 0, just keep trying to send the same payload
		}
	}
	nrf24l01_start_fast_write(buf, len, multicast, 1);
	return 1;
}

void nrf24l01_re_use_tx(void) {
	nrf24l01_write_register(NRF_STATUS,(1<<MAX_RT) );			  //Clear max retry flag
	nrf24l01_spi_trans( REUSE_TX_PL );
	nrf24l01_ce(0);										  //Re-Transfer packet
	nrf24l01_ce(1);
}

uint8_t nrf24l01_write_blocking(const void *buff, uint8_t len, uint32_t timeout) {
	uint32_t timer = timeout_millis();//Get the time that the payload transmission started

	while ((nrf24l01_get_status() & ((1<<TX_FULL)))) {//Blocking only if FIFO is full. This will loop and block until TX is successful or timeout

		if (nrf24l01_get_status() & (1<<MAX_RT)) {//If MAX Retries have been reached
			nrf24l01_re_use_tx();//Set re-transmit and clear the MAX_RT interrupt flag
			if (timeout_millis() - timer > timeout) {
				return 0;
			}//If this payload has exceeded the user-defined timeout, exit and return 0
		}
#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
		if(timeout_millis() - timer > (timeout+95) ){
			nrf24l01_error_notify();
#if defined (FAILURE_HANDLING)
			return 0;
#endif
		}
#endif

	}

	//Start Writing
	nrf24l01_start_fast_write(buff, len, 0, 1);//Write the payload if a buffer is clear

	return 1;					//Return 1 to indicate successful transmission
}

uint8_t nrf24l01_write(const void* buf, uint8_t len, uint8_t multicast) {
	//Start Writing
	nrf24l01_start_fast_write(buf,len,multicast, 1);

	//Wait until complete or failed
#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
	uint32_t timer = timeout_millis();
#endif



	while( ! ( nrf24l01_get_status()  & ( (1<<TX_DS) | (1<<MAX_RT) ))) {

#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
		if(timeout_millis() - timer > 95){
			nrf24l01_error_notify();
#if defined (FAILURE_HANDLING)
			return 0;
#else
			_delay_ms(100);
#endif
		}
#endif
		//_delay_ms(100);
	}

	//_delay_ms(200);

	nrf24l01_ce(0);
	uint8_t status = nrf24l01_write_register(NRF_STATUS,(1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT) );

	//Max retries exceeded
	if( status & (1<<MAX_RT)){
		nrf24l01_flush_tx(); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
		return 0;
	}
	//TX OK 1 or 0
	return 1;
}

uint8_t nrf24l01_write_without_multicast(const void *buff, uint8_t len) {
	return nrf24l01_write(buff, len, 0);
}

//#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
//void RF24::errNotify(){
//#if defined (SERIAL_DEBUG) || defined (RF24_LINUX)
//	//printf_P(PSTR("RF24 HARDWARE FAIL: Radio not responding, verify pin connections, wiring, etc.\r\n"));
//#endif
//#if defined (FAILURE_HANDLING)
//	failureDetected = 1;
//#else
//	delay(5000);
//#endif
//}
//#endif



void nrf24l01_power_up(void) {
	uint8_t cfg = nrf24l01_read_register(NRF_CONFIG);
	// if not powered up then power up and wait for the radio to initialize
	if (!(cfg & (1<<PWR_UP))){
		nrf24l01_write_register(NRF_CONFIG, cfg | (1<<PWR_UP));

		// For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
		// There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
		// the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
		_delay_ms(5);
	}
}

void nrf24l01_power_down(void) {
	nrf24l01_ce(0);
	nrf24l01_write_register(NRF_CONFIG,nrf24l01_read_register(NRF_CONFIG) & ~(1<<PWR_UP));
}

void nrf24l01_start_listening() {
#if !defined (RF24_TINY) && ! defined(LITTLEWIRE)
	nrf24l01_power_up();
#endif
	nrf24l01_write_register(NRF_CONFIG, nrf24l01_read_register(NRF_CONFIG) | (1<<PRIM_RX));
	nrf24l01_write_register(NRF_STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT) );
	nrf24l01_ce(1);
	// Restore the pipe0 adddress, if exists
	if (pipe0_reading_address[0] > 0){
		nrf24l01_write_register_buff(RX_ADDR_P0, pipe0_reading_address, addr_width);
	}else{
		nrf24l01_close_reading_pipe(0);
	}

	// Flush buffers
	//flush_rx();
	if(nrf24l01_read_register(FEATURE) & (1<<EN_ACK_PAY)){
		nrf24l01_flush_tx();
	}
}

uint8_t nrf24l01_is_chip_connected() {
	uint8_t setup = nrf24l01_read_register(SETUP_AW);
	if(setup >= 1 && setup <= 3)
	{
		return 1;
	}

	return 0;
}


void nrf24l01_print_reg_addr(char *reg_name, uint8_t reg) {
	uart_puts(reg_name);
	uint8_t buff[5];
	nrf24l01_read_register_buff(reg++, buff, sizeof buff);
	uint8_t *buff_ptr = buff + sizeof buff;
	while (--buff_ptr >= buff) {
		uart_putint(*buff_ptr, 16);
	}
	uart_putsn("");
}

void nrf24l01_init_external_interrupt() {
	INT_PORT |= INT_PIN;
	EICRA |= (1<<ISC01);
}

void nrf24l01_on_received(void *rx_buff) {
	if (EIFR & (1<<INTF0)) {
		if (nrf24l01_available()) {
			uint8_t len = nrf24l01_get_dynamic_payload_size();
			nrf24l01_read(rx_buff, len);
			(*rx_callback)(rx_buff);
		}
		EIFR |= (1<<INTF0);
	}
}

void nrf24l01_set_on_receive_callback(void (*callback)(void *rx_buff)) {
	rx_callback = callback;
}

#if FAILURE_HANDLING
void nrf24l01_set_on_error_callback(void (*callback)(char *str)) {
	error_callback = callback;
}

void nrf24l01_error_notify() {
	error_callback("NRF24L01 HARDWARE FAIL");
}
#endif

#if USE_RX_LONG_PACKAGE || USE_TX_LONG_PACKAGE

void nrf24l01_set_start_char(char c) {
	start_frame_char = c;
}

void nrf24l01_set_end_char(char c) {
	end_frame_char = c;
}

#endif

#if USE_RX_LONG_PACKAGE

void nrf24l01_on_received_long_package(char *rx_buff) {
	static uint8_t i = 0;
	char buff[32];
	if (EIFR & (1<<INTF0)) {
		if (nrf24l01_available()) {
			uint8_t len = nrf24l01_get_dynamic_payload_size();
			nrf24l01_read(buff, len);
			uint8_t j = 0;
			if (buff[0] == start_frame_char) {
				*rx_buff = 0;
				i = 0;
			}
			for (j = 0; j < strlen(buff); j++) {
				if (buff[j] == start_frame_char) {
					continue;
				}
				if (buff[j] != end_frame_char) {
					rx_buff[i] = buff[j];
					i++;
				} else {
					i = 0;
					(*rx_long_pocket_callback)(rx_buff);
					*rx_buff = 0;
					break;
				}
			}

		}
		EIFR |= (1<<INTF0);
	}
}


void nrf24l01_set_on_receive_long_package_callback(void (*callback)(char *rx_buff)) {
	rx_long_pocket_callback = callback;
}

#endif

#if USE_TX_LONG_PACKAGE

void nrf24l01_set_separator(char c) {
	separator[0] = c;
}

void nrf24l01_write_long_package(char *str) {
	char tx_buff[200];
	char *tok, *saved;
	char format[200];
	uint8_t i,j = 0;
	uint8_t x = 0;
	sprintf(format, "%c%s%c", start_frame_char, str, end_frame_char);

	uint8_t len = strlen(format);

	for (i = 0; i < len; i++) {
		tx_buff[x] = format[i];
		if (!(j % 31) && j > 0) {
			tx_buff[x] = separator[0];
			tx_buff[x+1] = format[i];
			j = 0;
			x++;
		}
		j++;
		x++;
	}

	tx_buff[x] = '\0';



	for (tok = strtok_r(tx_buff, separator, &saved); tok; tok = strtok_r(NULL, separator, &saved)) {
		uint8_t l = strlen(tok);
		uint8_t status = nrf24l01_write_without_multicast(tok, l);
#if DEBUG
		uart_puts("Status: ");
		uart_putint(status, 10);
		uart_putsn("");
#endif
		if (!status) return;
	}
}

#endif


