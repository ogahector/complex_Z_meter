/*
 * sig_gen.h
 *
 *  Created on: Jan 30, 2025
 *      Author: ogahe
 */

#ifndef SIG_GEN_H
#define SIG_GEN_H

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdint.h>

#define F_TIMER_CLOCK 45000000  // APB1 clock for Timer6
#define DAC_RESOLUTION_BITS 12  // 12-bit DAC
#define DAC_LUT_SIZE 100     // Size of the sine wave lookup table


extern uint16_t sine_wave_buffer[DAC_LUT_SIZE];  // Lookup table storage
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

// Function to compute sine wave lookup table
void Calculate_Sine_Wave(uint16_t buffer[], int size);

void Fill_Sine_Buffer();

// Function to enable sine wave generation via DAC + DMA
void Sig_Gen_Enable();

// Function to disable sine wave generation
void Sig_Gen_Disable();

// Function to set Timer6 frequency
uint32_t Set_Timer6_Frequency(uint32_t f_desired);

// Function to compute logarithmically spaced frequency points
void Calculate_Frequencies(long fstart, long fstop, uint8_t points_per_decade, int total_points, uint32_t frequencies[]);

#endif /* SIG_GEN_H */
