/*
 * dsp.h
 *
 *  Created on: Feb 28, 2025
 *      Author: ogahe
 */

#ifndef SRC_SAMPLING_H_
#define SRC_SAMPLING_H_

#include "main.h"

#define ADC_SAMPLES_PER_CHANNEL (10000)
#define ADC_BUFFER_SIZE (3*ADC_SAMPLES_PER_CHANNEL)
//#define ADC_BUFFER_SIZE 3

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern uint16_t vmeas_buffer[ADC_BUFFER_SIZE];


typedef struct __phasor_t {
    double magnitude;
    double phaserad;
} phasor_t;

typedef struct __freq_t {
	phasor_t phasor;

	uint16_t input[ADC_SAMPLES_PER_CHANNEL];
	uint16_t output[ADC_SAMPLES_PER_CHANNEL];

	float Rref;

} freq_t;


phasor_t Get_Phasor_1Sig(uint32_t sig[], size_t len, uint32_t f0, uint32_t fs);


phasor_t Get_Phasor_2Sig(uint32_t sig[], uint32_t ref[], size_t lensig, size_t lenref, uint32_t f0, uint32_t fs);


void ADC_Separate_Channels(uint16_t buffADC[], uint16_t buffA[], uint16_t buffB[], uint16_t buffC[]);


void Sampling_Enable();


void Sampling_Disable();


void Get_Steady_State(uint32_t f0, uint16_t buffADC[], uint16_t vmeas0[], uint16_t vmeas1[], uint16_t vmeas2[]);


#endif /* SRC_SAMPLING_H_ */
