/*
 * iqmethod.c
 *
 *  Created on: Feb 28, 2025
 *      Author: ogahe
 */

#include "iqmethod.h"
#include "math.h"

void Align_Beginning(uint32_t data[], size_t len)
{
	int32_t buff[len];
	for(size_t i = 0; i < len; i++)
	{
		buff[i] = (int32_t) data[i];
	}

	// Remove Offsets
	for(size_t i = 0; i < len; i++)
	{
		buff[i] -= 2048;
	}
}

phasor_t Get_Phasor(uint32_t sig[], size_t len, uint32_t f0, uint32_t fs)
{
	double I, Q;

	for(size_t i = 0; i < len; i++)
	{
		double ang_step = 2.0 * M_PI * f0 * i / fs;
		double ref_cos = cosf(ang_step);
		double ref_sin = sinf(ang_step);

		I += sig[i] * ref_cos;
		Q += sig[i] * ref_sin;
	}

	I /= len;
	Q /= len;

	return (phasor_t) { sqrt(I*I + Q*Q), atan2(Q, I) };
}
