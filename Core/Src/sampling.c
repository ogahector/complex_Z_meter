/*
 * sampling_iq.c
 *
 *  Created on: Mar 11, 2025
 *      Author: ogahe
 */


#include <sampling.h>

void Sampling_Enable(void)
{
	HAL_TIM_Base_Start(&htim2);
//	HAL_Delay(10);
}

void Sampling_Disable(void)
{
	HAL_TIM_Base_Stop(&htim2);
//	HAL_Delay(10);
}


uint32_t Get_Sampling_Frequency(void)
{
	uint32_t psc = __HAL_TIM_GET_ICPRESCALER(&htim2, TIM2_BASE);
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);


	// Compute update frequency: f_update = timer_clk / ((PSC+1) * (ARR+1))
	uint32_t f_update = F_SAMPLE_TIMER / ((psc + 1) * (arr + 1));
	return f_update;
}


uint32_t Set_Sampling_Frequency(uint32_t f_sine)
{
	// Calculate the desired timer update frequency.
	uint32_t f_update = f_sine;

    // Calculate the ideal division factor.
	uint32_t idealDiv = F_SAMPLE_TIMER / f_update;

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
	    actualFreq = F_SAMPLE_TIMER / ((psc + 1) * (arr + 1));
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
	__HAL_TIM_SET_PRESCALER(&htim2, bestPSC);
	__HAL_TIM_SET_AUTORELOAD(&htim2, bestARR);
	// Force an update event so that new PSC/ARR values are loaded immediately.
	HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);

	// Return the actual timer update frequency (f_update_actual = F_TIMER_CLOCK / ((PSC+1)*(ARR+1)) )
	return F_SAMPLE_TIMER / ((bestPSC + 1) * (bestARR + 1));
}
