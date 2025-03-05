/*
 * sig_gen.c
 *
 *  Created on: Jan 30, 2025
 *      Author: ogahe
 */


#include "sig_gen.h"

//extern uint16_t sine_wave_buffer[]; // Sine Wave Stored

void Calculate_Sine_Wave(uint16_t buffer[], int size)
{
    for (size_t i = 0; i < size; i++) {
        double sinangle = sin((2.0f * M_PI * i) / size); // Convert index to angle (radians)
        double scaled = 2047*sinangle;
        buffer[i] = (uint16_t) ((int) (scaled + 2048)); // Scale to 0-4095
    }
}

void Fill_Sine_Buffer()
{
	Calculate_Sine_Wave(sine_wave_buffer, DAC_LUT_SIZE);
}

void Sig_Gen_Enable()
{
	HAL_TIM_Base_Start(&htim6);
//	HAL_Delay(10);
}

void Sig_Gen_Disable()
{
	HAL_TIM_Base_Stop(&htim6);
//	HAL_Delay(10);
}

uint32_t Get_Signal_Frequency()
{
	uint32_t psc = __HAL_TIM_GET_ICPRESCALER(&htim6, TIM6_BASE);
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim6);


	// Compute update frequency: f_update = timer_clk / ((PSC+1) * (ARR+1))
	uint32_t f_update = F_SIG_TIMER / ((psc + 1) * (arr + 1));

	return (uint32_t) f_update / DAC_LUT_SIZE;
}

uint32_t Set_Timer6_Frequency(uint32_t f_sine)
{
    // Calculate the desired timer update frequency.
    uint32_t f_update = f_sine;

    // Calculate the ideal division factor.
    uint32_t idealDiv = F_SIG_TIMER / f_update;

    uint32_t bestPSC = 0, bestARR = 0, bestError = 0xFFFFFFFF, actualFreq = 0;

    // Iterate over possible prescaler values.
    // Here we search PSC values from 0 to a reasonable limit (e.g., 1024) to find valid values.
    for (uint32_t psc = 0; psc < 1024; psc++)
    {
        uint32_t divider = psc + 1;
        uint32_t arr_plus1 = idealDiv / divider;
        if (arr_plus1 == 0 || arr_plus1 > 65536)  // ARR must be 0..65535 (i.e. ARR+1 <= 65536)
            continue;

        uint32_t arr = arr_plus1 - 1;
        // Calculate the actual update frequency for this configuration.
        actualFreq = F_SIG_TIMER / ((psc + 1) * (arr + 1));
        // Compute the absolute error.
        uint32_t error = (f_update > actualFreq) ? (f_update - actualFreq) : (actualFreq - f_update);
        if (error < bestError)
        {
            bestError = error;
            bestPSC = psc;
            bestARR = arr;
        }
        if (error == 0)
            break;  // Perfect match found.
    }

    // Set the timer registers:
    __HAL_TIM_SET_PRESCALER(&htim6, bestPSC);
    __HAL_TIM_SET_AUTORELOAD(&htim6, bestARR);
    // Force an update event so that new PSC/ARR values are loaded immediately.
    HAL_TIM_GenerateEvent(&htim6, TIM_EVENTSOURCE_UPDATE);

    // Return the actual timer update frequency (f_update_actual = F_TIMER_CLOCK / ((PSC+1)*(ARR+1)) )
    return F_SIG_TIMER / ((bestPSC + 1) * (bestARR + 1));
}

uint32_t Set_Signal_Frequency(uint32_t freq)
{
	return (uint32_t) Set_Timer6_Frequency( (uint32_t) DAC_LUT_SIZE * freq) / DAC_LUT_SIZE;
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

//    TransmitUInt32Buffer(frequencies, total_points);
}
