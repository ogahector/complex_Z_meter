/*
 * sig_gen.c
 *
 *  Created on: Jan 30, 2025
 *      Author: ogahe
 */


#include "sig_gen.h"

//extern uint32_t sine_wave_buffer[]; // Sine Wave Stored

void Calculate_Sine_Wave(volatile uint32_t buffer[], int size)
{
    for (size_t i = 0; i < size; i++) {
        float angle = (2.0f * M_PI * i) / size; // Convert index to angle (radians)
        buffer[i] = (uint32_t) (2047 * sin(angle) + 2048); // Scale to 0-4095
    }
}

void Fill_Sine_Buffer()
{
	Calculate_Sine_Wave(sine_wave_buffer, SINE_LUT_SIZE);
}

void Enable_Sine_Gen()
{
	if(HAL_DAC_GetState(&hdac) != HAL_DAC_STATE_READY)
		return;
	// Start TIM6 (triggers DAC)
	HAL_TIM_Base_Start(&htim6);

	// Start DAC1 with DMA (assumes DAC1 Channel 1)
	// Also loads data into the DMA
//	const uint32_t buffer[] = sine_wave_buffer;
//	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_wave_buffer, SINE_LUT_SIZE, DAC_ALIGN_12B_R);
}

void Disable_Sine_Gen()
{
	if(__HAL_TIM_GET_COUNTER(&htim6) == 0)
		return;
	// Stop DAC
	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

	// Stop Timer
	HAL_TIM_Base_Stop(&htim6);

	// Set DAC output to 0V or mid-range
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
}

uint32_t Set_Timer6_Frequency(uint32_t f_desired)
{
    // Calculate the required total division factor:
    // (PSC+1) * (ARR+1) should ideally equal F_TIMER_CLOCK / f_desired.
    uint32_t idealDiv = F_TIMER_CLOCK / f_desired;
    uint32_t bestPSC = 0;
    uint32_t bestARR = 0;
    uint32_t bestError = 0xFFFFFFFF;
    uint32_t actualFreq = F_TIMER_CLOCK;

    // Iterate over possible prescaler values.
    // For a 16-bit timer, PSC and ARR must be <= 65535.
    for (uint32_t psc = 0; psc < 1024; psc++)  // limiting search range for speed
    {
        uint32_t divider = psc + 1;
        uint32_t arr_plus1 = idealDiv / divider;
        if (arr_plus1 == 0 || arr_plus1 > 65536)
            continue; // invalid ARR value for this PSC

        uint32_t arr = arr_plus1 - 1;
        actualFreq = F_TIMER_CLOCK / ((psc + 1) * (arr + 1));
        uint32_t error = (f_desired > actualFreq) ? (f_desired - actualFreq) : (actualFreq - f_desired);
        if (error < bestError)
        {
            bestError = error;
            bestPSC = psc;
            bestARR = arr;
        }
        if (error == 0)
            break; // perfect match found
    }

    // Set the calculated PSC and ARR values to Timer6.
    __HAL_TIM_SET_PRESCALER(&htim6, bestPSC);
    __HAL_TIM_SET_AUTORELOAD(&htim6, bestARR);
    HAL_TIM_GenerateEvent(&htim6, TIM_EVENTSOURCE_UPDATE); // Force update event to load new values

    return actualFreq;
}

void Calculate_Frequencies(long fstart, long fstop, uint8_t points_per_decade, int total_points, uint32_t frequencies[])
{
    double f_min = fstart;     // start frequency (Hz)
    double f_max = fstop;      // end frequency (Hz)
    // 50 points per decade => step exponent = 1/50
    double decadeStep = 1.0 / points_per_decade;

//    int total_points = points_per_decade * (log10(fstop / fstart));

    // Loop from f_min to f_max in logarithmic steps.
    for (int i = 0; i < total_points; i++)
    {
        // Calculate the desired frequency:
        double exponent = log10(f_min) + i * decadeStep;
        uint32_t f_desired = (uint32_t)round(pow(10.0, exponent));

        // Set Timer6 frequency:
//        Set_Timer6_Frequency(f_desired);
        frequencies[i] = f_desired;
    }
}
