/*
 * timeout.h
 *
 *  Created on: 9 maj 2020
 *      Author: MIKE
 */

#ifndef TIMEOUT_TIMEOUT_H_
#define TIMEOUT_TIMEOUT_H_

void timeout_init();
unsigned long timeout_millis();
void timeout_delay_us(uint32_t delay);

#endif /* TIMEOUT_TIMEOUT_H_ */
