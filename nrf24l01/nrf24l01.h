/*
 * nrf24l01.h
 *
 *  Created on: 8 maj 2020
 *      Author: MIKE
 */

/*
 * BASED ON RF24 library https://github.com/nRF24/RF24
 */

#ifndef NRF24L01_NRF24L01_H_
#define NRF24L01_NRF24L01_H_

#include <avr/io.h>
#include "nrf24l01_config.h"

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum {
	RF24_PA_MIN = 0,
	RF24_PA_LOW,
	RF24_PA_HIGH,
	RF24_PA_MAX,
	RF24_PA_ERROR
} rf24_pa_dbm_e;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum {
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS
} rf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum {
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
} rf24_crclength_e;



//initialization of module (if is ok should return 1)
uint8_t nrf24l01_init(void);

void nrf24l01_power_up(void);

void nrf24l01_power_down(void);

void nrf24l01_open_reading_pipe(uint8_t child, uint64_t address);

void nrf24l01_open_reading_pipe_by_pointer(uint8_t child, const uint8_t *address);

void nrf24l01_open_writing_pipe_by_pointer(const uint8_t *address);

void nrf24l01_open_wrting_pipe(uint64_t value);

uint8_t nrf24l01_is_chip_connected(void);

uint8_t nrf24l01_available(void);

void nrf24l01_read(void *buff, uint8_t len);

uint8_t nrf24l01_write_without_multicast(const void *buff, uint8_t len);

uint8_t nrf24l01_available_by_pipe(uint8_t *pipe_num);


uint8_t nrf24l01_fast_write(const void* buf, uint8_t len, uint8_t multicast);

//* Empty the transmit buffer. This is generally not required in standard operation.
// * May be required in specific cases after stopListening() , if operating at 250KBPS data rate.
uint8_t  nrf24l01_flush_tx();


//* This function should be called as soon as transmission is finished to
//   * drop the radio back to STANDBY-I mode. If not issued, the radio will
//   * remain in STANDBY-II mode which, per the data sheet, is not a recommended
//   * operating mode.
uint8_t nrf24l01_tx_standby_with_timeout(uint32_t timeout, uint8_t start_tx);

//* Stop listening for incoming messages, and switch to transmit mode.
void nrf24l01_stop_listening(void);

//* Test whether there was a carrier on the line for the
//* previous listening period.
uint8_t nrf24l01_test_carrier(void);

//* Test whether a signal (carrier or otherwise) greater than
//* or equal to -64dBm is present on the channel. Valid only
//* on nRF24L01P (+) hardware. On nRF24L01, use testCarrier().
uint8_t nrf24l01_test_rpd(void);

//* Retrieve the current status of the chip
uint8_t nrf24l01_get_status();

//* Non-blocking write to the open writing pipe used for buffered writes
//   *
//   * @note Optimization: This function now leaves the CE pin high, so the radio
//   * will remain in TX or STANDBY-II Mode until a txStandBy() command is issued. Can be used as an alternative to startWrite()
//   * if writing multiple payloads at once.
//   * @warning It is important to never keep the nRF24L01 in TX mode with FIFO full for more than 4ms at a time. If the auto
//   * retransmit/autoAck is enabled, the nRF24L01 is never in TX mode long enough to disobey this rule. Allow the FIFO
//   * to clear by issuing txStandBy() or ensure appropriate time between transmissions.
void nrf24l01_start_fast_write(const void* buf, uint8_t len, const uint8_t multicast, uint8_t start_tx);

void nrf24l01_mask_irq(uint8_t tx, uint8_t fail, uint8_t rx);

//* Enable dynamic ACKs (single write multicast or unicast) for chosen messages
void nrf24l01_enable_dynamic_ack_payload(void);

void nrf24l01_print_reg_addr(char *reg_name, uint8_t reg);

/**
 * Set Power Amplifier (PA) level to one of four levels:
 * RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
 *
 * The power levels correspond to the following output levels respectively:
 * NRF24L01: -18dBm, -12dBm,-6dBM, and 0dBm
 *
 * SI24R1: -6dBm, 0dBm, 3dBM, and 7dBm.*/
void nrf24l01_set_pa_level(uint8_t level);

/**
 * Set the transmission data rate
 *
 * @warning setting RF24_250KBPS will fail for non-plus units
 *
 * @param speed RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
 * @return true if the change was successful
 */
uint8_t nrf24l01_set_data_rate(rf24_datarate_e speed);

//* Get Dynamic Payload Size
//  *
//  * For dynamic payloads, this pulls the size of the payload off
//  * the chip
uint8_t nrf24l01_get_dynamic_payload_size(void);

//* Start listening on the pipes opened for reading.
void nrf24l01_start_listening();

/**
 * Enable dynamically-sized payloads
 *
 * This way you don't always have to send large packets just to send them
 * once in a while.  This enables dynamic payloads on ALL pipes.
 *
 */
void nrf24l01_enable_dynamic_payloads(void);
/**
 * This function should be called as soon as transmission is finished to
 * drop the radio back to STANDBY-I mode. If not issued, the radio will
 * remain in STANDBY-II mode which, per the data sheet, is not a recommended
 * operating mode.
 *
 * @note When transmitting data in rapid succession, it is still recommended by
 * the manufacturer to drop the radio out of TX or STANDBY-II mode if there is
 * time enough between sends for the FIFOs to empty. This is not required if auto-ack
 * is enabled.*/
uint8_t nrf24l01_tx_standby(void);

/**
 * Check if the radio needs to be read. Can be used to prevent data loss
 * @return True if all three 32-byte radio buffers are full
 */
uint8_t nrf24l01_is_fifo_full(void);

void nrf24l01_init_external_interrupt();

void nrf24l01_on_received(void *rx_buff);

void nrf24l01_on_received_long_package(char *rx_buff);

void nrf24l01_set_on_receive_callback(void (*callback)(void *rx_buff));

#if USE_TX_LONG_PACKAGE
	void nrf24l01_set_separator(char c);

	void nrf24l01_write_long_package(char *str);
#endif

#if USE_TX_LONG_PACKAGE || USE_RX_LONG_PACKAGE
	void nrf24l01_set_start_char(char c);

	void nrf24l01_set_end_char(char c);
#endif

#if FAILURE_HANDLING
	void nrf24l01_error_notify();
	void nrf24l01_set_on_error_callback(void (*callback)(char *str));
#endif

#if USE_RX_LONG_PACKAGE

	void nrf24l01_set_on_receive_long_package_callback(void (*callback)(char *rx_buff));

#endif




uint8_t nrf24l01_read_register(uint8_t reg);
uint8_t nrf24l01_write_register(uint8_t reg, uint8_t value);
#endif /* NRF24L01_NRF24L01_H_ */
