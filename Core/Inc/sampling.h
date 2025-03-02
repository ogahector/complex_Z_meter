/*
 * sampling.h
 *
 *  Created on: Feb 28, 2025
 *      Author: ogahe
 */

#ifndef SRC_SAMPLING_H_
#define SRC_SAMPLING_H_

#include "main.h"

#define ADC_SAMPLES_PER_CHANNEL (10000)
//#define ADC_BUFFER_SIZE (3*ADC_SAMPLES_PER_CHANNEL)
#define ADC_BUFFER_SIZE 3

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern uint16_t vmeas_buffer[ADC_BUFFER_SIZE];


#endif /* SRC_SAMPLING_H_ */
