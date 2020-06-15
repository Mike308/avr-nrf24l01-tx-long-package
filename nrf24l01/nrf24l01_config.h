/*
 * nrf24l01_config.h
 *
 *  Created on: 8 maj 2020
 *      Author: MIKE
 */

/*
 * BASED ON RF24 library https://github.com/nRF24/RF24
 */

#ifndef NRF24L01_NRF24L01_CONFIG_H_
#define NRF24L01_NRF24L01_CONFIG_H_

#define CE_PORT PORTB
#define CE_PIN (1<<1)
#define CE_DDR DDRB

#define INT_PORT PORTD
#define INT_PIN (1<<PD2);

#define USE_TX_LONG_PACKAGE 1
#define USE_RX_LONG_PACKAGE 0

#define FAILURE_HANDLING 1

#define DEBUG 1

#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)

#endif /* NRF24L01_NRF24L01_CONFIG_H_ */
