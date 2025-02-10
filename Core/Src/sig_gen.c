/*
 * sig_gen.c
 *
 *  Created on: Jan 30, 2025
 *      Author: ogahe
 */


#include "sig_gen.h"

//TIM_HandleTypeDef htim6; // Linked with Main Input SWave 1
//TIM_HandleTypeDef htim7; // Linked with Main Input SWave 1

static uint32_t sine_wave_buffer[SINE_LUT_SIZE]; // Sine Wave Stored

void Calculate_Sine_Wave(uint32_t* buffer, int size)
{
    for (int i = 0; i < size; i++) {
        float angle = (2.0f * M_PI * i) / SINE_LUT_SIZE; // Convert index to angle (radians)
        buffer[i] = (uint32_t)((2 << (DAC_RESOLUTION_BITS-1)) * (1.0f + sin(angle))); // Scale to 0-4095
    }
    return 0;
}

static void Set_Timer_Frequency(TIM_HandleTypeDef* htim, int frequency)
{
	return;
}
//Calculate_Sine_Wave(void);


//static void Start_Input_Sine_Wave(DAC_HandleTypeDef* hdac, int frequency, TIM_HandleTypeDef* htim)
//{
//	HAL_TIM_Base_Start_DMA(htim, sine_wave_buffer, SINE_LUT_SIZE)(htim);
//
////	HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, sine_wave_buffer, SINE_LUT_SIZE, DAC_ALIGN_12B_R);
//}
//
//static void Start_Input_Sine_Wave(TIM_HandleTypeDef* htim, int frequency)
//{
//	HAL_TIM_Base_Stop_DMA(htim);
//}

static void Stop_Input_Sine_Wave(DAC_HandleTypeDef* hdac)
{

}
