/*
 * iqmethod.c
 *
 *  Created on: Feb 28, 2025
 *      Author: ogahe
 */

#include "iqmethod.h"
#include "math.h"



phasor_t Get_Phasor_1Sig(uint32_t sig[], size_t len, uint32_t f0, uint32_t fs)
{
	/*
	 * Be VERY Careful using this function:
	 * it returns a phasor_t with a MISALIGNED phase.
	 * For an actual phasor, use Get_Output_Phasor
	 * to get a phasor with respect to a reference.
	 */
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

	return (phasor_t) {
		sqrt(I*I + Q*Q),
		atan2(Q, I)
	};
}



phasor_t Get_Phasor_2Sig(uint32_t sig[], uint32_t ref[], size_t lensig, size_t lenref, uint32_t f0, uint32_t fs)
{
	// Think about optimising this bc this calculates
	// the ref_cos and ref_sin twice
	phasor_t input = Get_Phasor_1Sig(ref, lenref, f0, fs);
	phasor_t output = Get_Phasor_1Sig(sig, lensig, f0, fs);

	return (phasor_t) {
		output.magnitude / input.magnitude,
		output.phaserad - input.phaserad
	};
}


