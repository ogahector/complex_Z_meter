/*
 * sig_gen.h
 *
 *  Created on: Jan 30, 2025
 *      Author: ogahe
 */

#ifndef INC_SIG_GEN_H_
#define INC_SIG_GEN_H_

// INCLUDES
#include "math.h"
#include "stm32f4xx_hal.h"

/* MACROS */
#define SINE_LUT_SIZE 100
#define DAC_RESOLUTION_BITS 12

// EXPORT CONSTANTS

/* EXPORT FUNCTIONS */
void Calculate_Sine_Wave(uint32_t* buffer, int size);
//static void Start_Input_Sine_Wave(DAC_HandleTypeDef* hdac, int frequency, TIM_HandleTypeDef* htim);
void Set_Timer_Frequency(TIM_HandleTypeDef* htim, int frequency);
//static void Start_Input_Sine_Wave(TIM_HandleTypeDef* htim);
void Stop_Input_Sine_Wave(DAC_HandleTypeDef* hdac);

#endif
