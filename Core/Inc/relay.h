/*
 * relay.h
 *
 *  Created on: Mar 4, 2025
 *      Author: ogahe
 */

#ifndef SRC_RELAY_H_
#define SRC_RELAY_H_

#define RELAY_SWITCH_DELAY_MS 5

typedef enum __switching_resistor_t { // this will be in mOhm!!
	RESISTOR0 = 100600,
	RESISTOR1 = 1000000,
	RESISTOR2 = 10000000,
	RESISTOR3 = 100000000
} switching_resistor_t;

void Set_Resistor_Hardware(switching_resistor_t res);

#endif /* SRC_RELAY_H_ */
