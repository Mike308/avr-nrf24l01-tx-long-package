/*
 * timeout.c
 *
 *  Created on: 9 maj 2020
 *      Author: MIKE
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "timeout.h"

volatile unsigned long millis;

void timeout_init(void) {
	// 8-bit TimerX config
	TCCR0B |= (1<<CS00)|(1<<CS01);  // set prescaler = 64 ---> please check proper values in PDF;
	TCCR0A |= (1<<WGM01); // set CTC mode ---> please check proper values in PDF
	OCR0A  = 249;  // every  [ 1 ms ]
	TIMSK0  |= (1<<OCIE0A);     // enable CompareX interrupt
}

unsigned long timeout_millis() {
	return millis;
}

void timeout_delay_us(uint32_t delay) {
	while (delay--) {
		_delay_us(1);
	}
}

ISR( TIMER0_COMPA_vect ) {
	millis++;
}
