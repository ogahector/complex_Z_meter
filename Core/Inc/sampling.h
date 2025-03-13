/*
 * sampling_iq.h
 *
 *  Created on: Mar 11, 2025
 *      Author: ogahe
 */

#ifndef INC_SAMPLING_H_
#define INC_SAMPLING_H_

#include "main.h"
#include "dsp.h"


/**
 * @brief Enables sampling by starting TIM2.
 */
void Sampling_Enable(void);

/**
 * @brief Disables sampling by stopping TIM2.
 */
void Sampling_Disable(void);

/**
 * @brief Gets the effective ADC sampling frequency based on TIM2's settings.
 * @return The sampling frequency in Hz.
 */
uint32_t Get_Sampling_Frequency(void);

uint32_t Set_Sampling_Frequency(uint32_t f_sine);

#endif /* INC_SAMPLING_H_ */
