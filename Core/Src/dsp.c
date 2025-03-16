/*
 * dsp.c
 *
 *  Created on: Feb 28, 2025
 *      Author: ogahe
 */

#include <dsp.h>
#include "math.h"
#include "sig_gen.h"
#include "string.h"
#include "transmits.h"

volatile uint8_t adcDmaTransferComplete;
uint8_t adcDmaHalfTransfer;
uint16_t vmeas_buffer_copy[ADC_BUFFER_SIZE];
uint16_t vmeas0_copy[ADC_SAMPLES_PER_CHANNEL];
uint16_t vmeas1_copy[ADC_SAMPLES_PER_CHANNEL];
uint16_t vmeas2_copy[ADC_SAMPLES_PER_CHANNEL];

void HAL_ADC_HalfConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		adcDmaHalfTransfer = 1;
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		adcDmaTransferComplete = 1;
		Sampling_Disable();
	}
}

void ADC_SampleSingleShot(void)
{
	adcDmaTransferComplete = 0;
	adcDmaHalfTransfer = 0;

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) vmeas_buffer, ADC_BUFFER_SIZE) != HAL_OK)
    {
    	while(1) TransmitStringLn("BAD DMA START");
    }

    uint32_t timeout = 1000 * ADC_BUFFER_SIZE / Get_Sampling_Frequency() + HAL_GetTick();

    while(adcDmaTransferComplete == 0)
    {
//    	__WFI();
    	if(HAL_GetTick() > timeout)
    	{
    		break; // In case the interrupt is missed which ONLY happens at VHF anyways
    	}
    	// Wait BLOCKING to allow for full single shot DMA transfer
    }

    // ideally should be an atomic load / store of atomic ints but oh wells
	memcpy(vmeas_buffer_copy, vmeas_buffer, ADC_BUFFER_SIZE * sizeof(uint16_t));
}


void ADC_Separate_Channels(uint16_t buffADC[], uint16_t buffA[], uint16_t buffB[], uint16_t buffC[])
{
	/*
	 * NOTE:
	 * IRL the order should be vmeas2, vmeas1, vmeas0
	 * because of the way the ADC buffer fill is set up
	 */
    for(size_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        switch(i % 3) {
            case 0:  // Channel 0
                buffA[i/3] = buffADC[i];
                break;
            case 1:  // Channel 1
                buffB[i/3] = buffADC[i];
                break;
            case 2:  // Channel 2
                buffC[i/3] = buffADC[i];
                break;
        }
    }
}

uint32_t Sample_Steady_State(uint32_t f0, uint16_t vmeas0[], uint16_t vmeas1[], uint16_t vmeas2[])
{
	Sampling_Disable();
	Sig_Gen_Disable();
	uint32_t actualFreq = Set_Signal_Frequency(f0);

	Sampling_Enable();
	Sig_Gen_Enable();


	ADC_SampleSingleShot();


	ADC_Separate_Channels(vmeas_buffer_copy, vmeas2, vmeas1, vmeas0);

	Sampling_Disable();
	Sig_Gen_Disable();

	return actualFreq;
}

uint32_t Sample_Steady_State_Phasors(uint32_t f0, phasor_t* input, phasor_t* output)
{
	uint16_t vmeas0[ADC_SAMPLES_PER_CHANNEL];
	uint16_t vmeas1[ADC_SAMPLES_PER_CHANNEL];
	uint16_t vmeas2[ADC_SAMPLES_PER_CHANNEL];
	uint32_t actualFreq = Sample_Steady_State(f0, vmeas0, vmeas1, vmeas2);

	*input = (phasor_t) {1, 0};
	*output = Get_Phasor_2Sig(vmeas1, vmeas0, ADC_SAMPLES_PER_CHANNEL, ADC_SAMPLES_PER_CHANNEL,
			Get_Signal_Frequency(), Get_Sampling_Frequency());

	return actualFreq;
}

void Get_All_Raw_Phasors(phasor_t inputs[], phasor_t outputs[], float Rref)
{
	/*
	 * WARNING: This is NOT OPTIMISED!!
	 * I also don't know what to do with VMEAS2 for now..
	 */
	uint32_t frequencies[NFREQUENCIES];
	uint32_t frequencies_visited[NFREQUENCIES];
	Calculate_Frequencies(FREQ_MIN, FREQ_MAX, FREQ_PPDECADE, NFREQUENCIES, frequencies);

	// 1e6 WORKS-ish - use 500k is better
	// This may be the main bottleneck
	Set_Sampling_Frequency(1500000);
	// Initial sampling to give a baseline and fill the system caps
	// This is a good fix! Verified in practice
	ADC_SampleSingleShot();

	for(size_t i = 0; i < NFREQUENCIES; i++)
	{

		frequencies_visited[i] = Sample_Steady_State_Phasors(frequencies[i], &inputs[i], &outputs[i]);
	}
}


void Measurement_Routine_Zx(phasor_t Zx_buff[], phasor_t Zsm_buff[], phasor_t Zom_buff[], switching_resistor_t Rref, uint32_t frequencies_visited[])
{
	Choose_Switching_Resistor(Rref);
	uint32_t frequencies_wanted[NFREQUENCIES];
	Calculate_Frequencies(FREQ_MIN, FREQ_MAX, FREQ_PPDECADE, NFREQUENCIES, frequencies_wanted);

	phasor_t v1[NFREQUENCIES];
	phasor_t v2[NFREQUENCIES];

	for(size_t i = 0; i < NFREQUENCIES; i++)
	{
		frequencies_visited[i] = Sample_Steady_State_Phasors(frequencies_wanted[i], &v1[i], &v2[i]);
		Zx_buff[i] = Calculate_Zx_Calibrated(v1[i], v2[i], Rref, Zsm_buff[i], Zom_buff[i]);
	}
}

void Measurement_Routine_Voltage(phasor_t output[], phasor_t Zsm_buff[], phasor_t Zom_buff[], switching_resistor_t Rref, uint32_t frequencies_visited[])
{
	Choose_Switching_Resistor(Rref);
	uint32_t frequencies_wanted[NFREQUENCIES];
	Calculate_Frequencies(FREQ_MIN, FREQ_MAX, FREQ_PPDECADE, NFREQUENCIES, frequencies_wanted);

	phasor_t v1[NFREQUENCIES];

	for(size_t i = 0; i < NFREQUENCIES; i++)
	{
		frequencies_visited[i] = Sample_Steady_State_Phasors(frequencies_wanted[i], &v1[i], &output[i]);
	}
}



/*---------------------------------------------------------*/




phasor_t Get_Phasor_1Sig(uint16_t sig[], size_t len, uint32_t f0, uint32_t fs)
{
	/*
	 * Be VERY Careful using this function:
	 * it returns a phasor_t with a MISALIGNED phase.
	 * For an actual phasor, use Get_Output_Phasor
	 * to get a phasor with respect to a reference.
	 */
	double I = 0.0, Q = 0.0;

	for(size_t i = 0; i < len; i++)
	{
		double ang_step = 2.0 * M_PI * f0 * i / fs;
		double ref_cos = cos(ang_step);
		double ref_sin = sin(ang_step);

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

phasor_t Get_Phasor_2Sig(uint16_t sig[], uint16_t ref[], size_t lensig, size_t lenref, uint32_t f0, uint32_t fs)
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




phasor_t phasor_sub(phasor_t x1, phasor_t x2)
{
	// Check edgecases that simplify the maths
//	if(x1.phaserad == 0)
//	{
//		return (phasor_t) {
//			x1.magnitude * x2.magnitude,
//			- x2.phaserad
//		};
//	}
//	else if(x2.phaserad == 0)
//	{
//		return (phasor_t) {
//			x1.magnitude * x2.magnitude,
//			x1.phaserad
//		};
//	}

	double vdiff_real = x1.magnitude * cos(x1.phaserad) - x2.magnitude * cos(x2.phaserad);
	double vdiff_imag = x1.magnitude * sin(x1.phaserad) - x2.magnitude * sin(x2.phaserad);
	return (phasor_t) {
		sqrt(vdiff_real*vdiff_real + vdiff_imag*vdiff_imag),
		atan2(vdiff_imag, vdiff_real)
	};
}


phasor_t Calculate_Zx_Raw(phasor_t v1, phasor_t v2, switching_resistor_t Rref)
{
	phasor_t vdiff = phasor_sub(v1, v2);
	return (phasor_t) {
		((double) (Rref/1000)) * v2.magnitude / vdiff.magnitude, // divide my 1000 bc its in mOhms!!
		v2.phaserad - vdiff.phaserad
	};
}

phasor_t Calculate_Zx_Calibrated(phasor_t v1, phasor_t v2, switching_resistor_t Rref, phasor_t Zsm, phasor_t Zom)
{
	// I'm very sorry: I'm aware these operations are quite computationally intensive
	// This is also a formula taken from the Hioki Impedance Measurement Handbook

	phasor_t Zm = Calculate_Zx_Raw(v1, v2, Rref);

	phasor_t Z_temp1 = phasor_sub(Zom, Zsm);
	phasor_t Z_temp2 = phasor_sub(Zm, Zsm);
	phasor_t Z_temp3 = phasor_sub(Zom, Zm);
	 // edge case -> div by 0 => 0s will be ignored in dB
	if(Z_temp3.magnitude == 0) return (phasor_t) {0,0};

	return (phasor_t) {
		Z_temp1.magnitude * Z_temp2.magnitude / Z_temp3.magnitude,
		Z_temp1.phaserad + Z_temp2.phaserad - Z_temp3.phaserad
	};
}


