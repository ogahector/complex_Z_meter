/*
 * sampling.c
 *
 *  Created on: Feb 28, 2025
 *      Author: ogahe
 */

#include "sampling.h"

void ADC_Separate_Channels(uint32_t buffADC[], uint32_t buffA[], uint32_t buffB[], uint32_t buffC[])
{
	for(size_t i = 0; i < ADC_SAMPLES_PER_CHANNEL; i += 3)
	{
		buffA[i] = buffADC[ 3*i ];
		buffB[i] = buffADC[ 3*(i+1) ];
		buffC[i] = buffADC[ 3*(i+2) ];

	}
}

