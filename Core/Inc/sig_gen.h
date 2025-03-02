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
#define FREQ_MAX 100e3
#define FREQ_MIN 100
#define FREQ_PPDECADE 50
#define FREQ_NDECADE 3

extern volatile uint32_t sine_wave_buffer[DAC_LUT_SIZE];  // Lookup table storage
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

// Function to compute sine wave lookup table
void Calculate_Sine_Wave(volatile uint32_t buffer[], int size);

void Fill_Sine_Buffer();

// Function to enable sine wave generation via DAC + DMA
void Enable_Sine_Gen();

// Function to disable sine wave generation
void Disable_Sine_Gen();

// Function to set Timer6 frequency
uint32_t Set_Timer2_Frequency(uint32_t f_desired);

// Function to compute logarithmically spaced frequency points
void Calculate_Frequencies(long fstart, long fstop, uint8_t points_per_decade, int total_points, uint32_t frequencies[]);

#endif /* SIG_GEN_H */
