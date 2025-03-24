/*
 * relay.h
 *
 *  Created on: Mar 4, 2025
 *      Author: ogahe
 */

#ifndef SRC_RELAY_H_
#define SRC_RELAY_H_

#define RELAY_SWITCH_DELAY_MS 5

#define NUM_RESISTORS 4
#define RELAY_FREQ_MIN 1000
#define RELAY_FREQ_MAX 100000
#define RELAY_PPDECADE 5
#define RELAY_NDECADE (FREQ_NDECADE - 1) // 2
#define RELAY_NFREQUENCIES (RELAY_NDECADE * RELAY_PPDECADE)
#define RES_QTOLERANCE 0.05
#define RELAY_CENTER_FREQ 1000

typedef enum __switching_resistor_t { // this will be in mOhm!!
	RESISTOR0 = 100600,
	RESISTOR1 = 1000000,
	RESISTOR2 = 10000000,
	RESISTOR3 = 100000000
} switching_resistor_t;

void Set_Resistor_Hardware(switching_resistor_t res);

void Get_Neighbors(switching_resistor_t res, switching_resistor_t *neighbors, int *num_neighbors);

double Measure_Midrange_Distance();

void Choose_Switching_Resistor();

#endif /* SRC_RELAY_H_ */
