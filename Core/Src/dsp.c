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
uint16_t vmeas_buffer_copy[ADC_BUFFER_SIZE];


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		adcDmaTransferComplete = 1;
	}
}

void ADC_SampleSingleShot(void)
{

	adcDmaTransferComplete = 0;

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) vmeas_buffer, ADC_BUFFER_SIZE) != HAL_OK)
    {
    	while(1) TransmitStringLn("BAD DMA START");
    }

	Sig_Gen_Enable();

	HAL_Delay(10 * 1000 / Get_Signal_Frequency()); // 10 periods of the lowest freq
//	HAL_Delay( 3 * 806 );

	Sampling_Enable();


    uint32_t timeout = 1000 * ADC_BUFFER_SIZE / Get_Sampling_Frequency() + HAL_GetTick();

    while(adcDmaTransferComplete == 0)
    {
//    	__WFI();
    	 // In case the interrupt is missed which ONLY happens at VHF anyways
    	// at VHF OR only when sending a massive chunk of data while sampling
    	// amended: shouldnt happen now ;)
    	if(HAL_GetTick() > timeout)
    	{
    		break;
    	}
    	// Wait BLOCKING to allow for full single shot DMA transfer
    }

/*
    Sig_Gen_Disable();
    if(HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1) != HAL_OK)
    {
    	TransmitStringRaw("Unable to stop dac dma");
    	Error_Handler();
    }

    Sig_Gen_Enable();
    if(HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048) != HAL_OK)
    {
    	TransmitStringRaw("Unable to set dac dma val");
    	Error_Handler();
    }

    Sig_Gen_Disable();
    if(HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_wave_buffer, DAC_LUT_SIZE, DAC_ALIGN_12B_R) != HAL_OK)
    {
    	TransmitStringRaw("Unable to start dma again :(");
    	Error_Handler();
    }
*/
    // ideally should be an atomic load / store of atomic ints but oh wells
//	memcpy(vmeas_buffer_copy, vmeas_buffer, ADC_BUFFER_SIZE * sizeof(uint16_t));
}


void ADC_Separate_Channels(uint16_t buffADC[], uint16_t buffA[], uint16_t buffB[])
{
	/*
	 * NOTE:
	 * IRL the input order should be vmeas0, vmeas1
	 * because of the way the ADC buffer fill is set up
	 * Confirmed DO NOT TOUCH
	 */
    for(size_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        switch(i % 2) {
            case 0:  // Channel 0
                buffA[i/2] = buffADC[i];
                break;
            case 1:  // Channel 1
                buffB[i/2] = buffADC[i];
                break;
        }
    }
}

uint32_t Sample_Steady_State(uint32_t f0, uint16_t vmeas0[], uint16_t vmeas1[])
{
	Sampling_Disable();
	Sig_Gen_Disable();
	uint32_t actualFreq = Set_Signal_Frequency(f0);


	ADC_SampleSingleShot();

	Sampling_Disable();
	Sig_Gen_Disable();

	ADC_Separate_Channels(vmeas_buffer, vmeas0, vmeas1);

	return actualFreq;
}

uint32_t Sample_Steady_State_Phasors(uint32_t f0, phasor_t* input, phasor_t* output)
{
	uint16_t vmeas0[ADC_SAMPLES_PER_CHANNEL];
	uint16_t vmeas1[ADC_SAMPLES_PER_CHANNEL];
	uint32_t actualFreq = Sample_Steady_State(f0, vmeas0, vmeas1);

	*input = (phasor_t) {1, 0};
//	*output = Get_Phasor_2Sig(vmeas1, vmeas2, ADC_SAMPLES_PER_CHANNEL, ADC_SAMPLES_PER_CHANNEL,
//			Get_Signal_Frequency(), Get_Sampling_Frequency());
	*output = Get_Phasor_2Sig(vmeas1, vmeas0, ADC_SAMPLES_PER_CHANNEL, ADC_SAMPLES_PER_CHANNEL,
			actualFreq, Get_Sampling_Frequency());

//	 this should be very wrong
//	*input = Get_Phasor_1Sig(vmeas0, ADC_SAMPLES_PER_CHANNEL, actualFreq, Get_Sampling_Frequency());
//	*output = Get_Phasor_1Sig(vmeas1, ADC_SAMPLES_PER_CHANNEL, actualFreq, Get_Sampling_Frequency());
//
//	output->phaserad = output->phaserad - input->phaserad;
//	input->phaserad = 0;

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
	Set_Sampling_Frequency(F_SAMPLE);
	// Initial sampling to give a baseline and fill the system caps
	// This is a good fix! Verified in practice
	ADC_SampleSingleShot();

	for(size_t i = 0; i < NFREQUENCIES; i++)
	{
		frequencies_visited[i] = Sample_Steady_State_Phasors(frequencies[i], &inputs[i], &outputs[i]);
	}
}


void Measurement_Routine_Zx_Calibrated(phasor_t Zx_buff[], phasor_t Zsm_buff[], phasor_t Zom_buff[], switching_resistor_t Rref, uint32_t frequencies_visited[])
{
	Set_Resistor_Hardware(Rref);
	uint32_t frequencies_wanted[NFREQUENCIES];
	Calculate_Frequencies(FREQ_MIN, FREQ_MAX, FREQ_PPDECADE, NFREQUENCIES, frequencies_wanted);

	phasor_t v1[NFREQUENCIES];
	phasor_t v2[NFREQUENCIES];

	Set_Sampling_Frequency(F_SAMPLE);

	ADC_SampleSingleShot();

	uint32_t timeout_cnt = MEAS_EXEC_TIMEOUT;

	for(size_t i = 0; i < NFREQUENCIES; i++)
	{
		frequencies_visited[i] = Sample_Steady_State_Phasors(frequencies_wanted[i], &v1[i], &v2[i]);

		if(i == 0) continue; // accept value regardless -> see about this
		if( // nonsensical values, redo - sanity check
			v2[i].magnitude > 1
//			|| fabs(wrap2_2pi(Zx_buff[i].phaserad) - wrap2_2pi(Zx_buff[i-1].phaserad)) >= ACCEPTABLE_PHASE_DELTA )// ik it checks at index -1 but it's read only oh well
		)
		{
			TransmitStringRaw("Redoing Frequency Point: ");
			TransmitUInt32Raw(frequencies_visited[i]); TransmitStringRaw("\n");
			timeout_cnt--;

			if(timeout_cnt)	i--;
			else timeout_cnt = MEAS_EXEC_TIMEOUT;
		}
		else
		{
			timeout_cnt = MEAS_EXEC_TIMEOUT;
		}

		Zx_buff[i] = Calculate_Zx_Calibrated(v1[i], v2[i], Rref, Zsm_buff[i], Zom_buff[i]);
	}
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void Measurement_Routine_Zx_Full_Calibrated(phasor_t Zx_buff[], phasor_t Zsm_buff[], phasor_t Zom_buff[], phasor_t Zstdm_buff[], phasor_t Zstd[], switching_resistor_t Rref, uint32_t frequencies_visited[])
{
	Set_Resistor_Hardware(Rref);
	uint32_t frequencies_wanted[NFREQUENCIES];
	Calculate_Frequencies(FREQ_MIN, FREQ_MAX, FREQ_PPDECADE, NFREQUENCIES, frequencies_wanted);

	phasor_t v1[NFREQUENCIES];
	phasor_t v2[NFREQUENCIES];

	Set_Sampling_Frequency(F_SAMPLE);

	ADC_SampleSingleShot();

	uint32_t timeout_cnt = MEAS_EXEC_TIMEOUT;

	extern double capacitance_std;

	for(size_t i = 0; i < NFREQUENCIES; i++)
	{
		frequencies_visited[i] = Sample_Steady_State_Phasors(frequencies_wanted[i], &v1[i], &v2[i]);

		if( // nonsensical values, redo - sanity check
			v2[i].magnitude >= 1
//			|| fabs(wrap2_2pi(Zx_buff[i].phaserad) - wrap2_2pi(Zx_buff[i-1].phaserad)) >= ACCEPTABLE_PHASE_DELTA
		)// ik it checks at index -1 but it's read only oh well
		{
			TransmitStringRaw("Redoing Frequency Point: ");
			TransmitUInt32Raw(frequencies_visited[i]); TransmitStringRaw("\n");
			timeout_cnt--;

			if(timeout_cnt)	i--;
			else timeout_cnt = MEAS_EXEC_TIMEOUT;
		}
		else
		{
			timeout_cnt = MEAS_EXEC_TIMEOUT;
		}

		Zstd[i] = (phasor_t) { 1 / (2 * M_PI * frequencies_visited[i] * capacitance_std) , - M_PI / 2};

		Zx_buff[i] = Calculate_Zx_Full_Calibrated(v1[i], v2[i], Rref, Zsm_buff[i], Zom_buff[i], Zstdm_buff[i], Zstd[i]);

	}
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}


void Measurement_Routine_Zx_Raw(phasor_t Zx_buff[], switching_resistor_t Rref, uint32_t frequencies_visited[])
{
	Set_Resistor_Hardware(Rref);
	uint32_t frequencies_wanted[NFREQUENCIES];
	Calculate_Frequencies(FREQ_MIN, FREQ_MAX, FREQ_PPDECADE, NFREQUENCIES, frequencies_wanted);

	phasor_t v1[NFREQUENCIES];
	phasor_t v2[NFREQUENCIES];

	Set_Sampling_Frequency(F_SAMPLE);

	ADC_SampleSingleShot();

	uint32_t timeout_cnt = MEAS_EXEC_TIMEOUT;

	for(size_t i = 0; i < NFREQUENCIES; i++)
	{
		frequencies_visited[i] = Sample_Steady_State_Phasors(frequencies_wanted[i], &v1[i], &v2[i]);
		Zx_buff[i] = Calculate_Zx_Raw(v1[i], v2[i], Rref);
		if(i == 0) continue; // accept value regardless -> see about this
		if( // nonsensical values, redo - sanity check
			v2[i].magnitude > 1
//			|| fabs(wrap2_2pi(Zx_buff[i].phaserad) - wrap2_2pi(Zx_buff[i-1].phaserad)) >= ACCEPTABLE_PHASE_DELTA
			)// ik it checks at index -1 but it's read only oh well
		{
			TransmitStringRaw("Redoing Frequency Point: ");
			TransmitUInt32Raw(frequencies_visited[i]); TransmitStringRaw("\n");
			timeout_cnt--;

			if(timeout_cnt)	i--;
			else timeout_cnt = MEAS_EXEC_TIMEOUT;
		}
		else
		{
			timeout_cnt = MEAS_EXEC_TIMEOUT;
		}
	}
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void Measurement_Routine_Voltage(phasor_t output[], switching_resistor_t Rref, uint32_t frequencies_visited[])
{
	Set_Resistor_Hardware(Rref);
	uint32_t frequencies_wanted[NFREQUENCIES];
	Calculate_Frequencies(FREQ_MIN, FREQ_MAX, FREQ_PPDECADE, NFREQUENCIES, frequencies_wanted);

	phasor_t v1[NFREQUENCIES];

	Set_Sampling_Frequency(F_SAMPLE);

	for(size_t i = 0; i < NFREQUENCIES; i++)
	{
		frequencies_visited[i] = Sample_Steady_State_Phasors(frequencies_wanted[i], &v1[i], &output[i]);
		if(output[i].magnitude > 1){
			TransmitStringRaw("Redoing Frequency Point: ");
			TransmitUInt32Raw(frequencies_visited); TransmitStringRaw("\n");
			i--;
		}// nonsensical values, redo
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

//	if(isnan(Q/I))
//	{
//		while(1) TransmitStringRaw("IQ NAN!!");
//	}

	return (phasor_t) {
		sqrt(I*I + Q*Q),
#ifdef __WRAP2_2PI
		wrap2_2pi( atan2(Q, I) )
#else
		atan2(Q, I)
#endif
	};
}

phasor_t Get_Phasor_2Sig(uint16_t sig[], uint16_t ref[], size_t lensig, size_t lenref, uint32_t f0, uint32_t fs)
{
	// Think about optimising this bc this calculates
	// the ref_cos and ref_sin twice


#ifdef __USING_MOVING_AVERAGE
	// will now procede to filter!
	uint16_t sig_filtered[lensig];
	uint16_t ref_filtered[lenref];

	Moving_Average_Filter(sig, sig_filtered, lensig, (uint32_t) (f0 * sqrt(2)));
	Moving_Average_Filter(ref, ref_filtered, lenref, (uint32_t) (f0 * sqrt(2)));

	phasor_t input = Get_Phasor_1Sig(ref_filtered, lenref, f0, fs);
	phasor_t output = Get_Phasor_1Sig(sig_filtered, lensig, f0, fs);
#else
	phasor_t input = Get_Phasor_1Sig(ref, lenref, f0, fs);
	phasor_t output = Get_Phasor_1Sig(sig, lensig, f0, fs);
#endif
//	if(input.magnitude == 0)
//	{
//		while(1) TransmitStringRaw("2-phasor NAN!!");
//	}
#ifdef __INCLUDE_CONV_PHASE
	double conv_phase = 2*M_PI*Get_Signal_Frequency() * T_DELAY_SAMPLE;
#endif

	return (phasor_t) {
		output.magnitude / input.magnitude,
#ifdef __WRAP2_2PI
#ifdef __INCLUDE_CONV_PHASE
		wrap2_2pi( output.phaserad - input.phaserad + conv_phase ) // see if it's PLUS or MINUS
#else
		wrap2_2pi( output.phaserad - input.phaserad)
#endif /* INCLUDE CONV PHASE */
#else
#ifdef __INCLUDE_CONV_PHASE
		output.phaserad - input.phaserad + conv_phase
#else
		output.phaserad - input.phaserad
#endif /* INCLUDE CONV PHASE */
#endif /* WRAP 2 2PI*/
	};
}




phasor_t phasor_sub(phasor_t x1, phasor_t x2)
{
	double vdiff_real = x1.magnitude * cos(x1.phaserad) - x2.magnitude * cos(x2.phaserad);
	double vdiff_imag = x1.magnitude * sin(x1.phaserad) - x2.magnitude * sin(x2.phaserad);

    if (vdiff_real == 0.0 && vdiff_imag == 0.0)
    {
        return (phasor_t){0.0, 0.0};
    }

	return (phasor_t) {
		sqrt(vdiff_real*vdiff_real + vdiff_imag*vdiff_imag),
#ifdef __WRAP2_2PI
		wrap2_2pi( atan2(vdiff_imag, vdiff_real) )
#else
		atan2(vdiff_imag, vdiff_real)
#endif
	};
}


phasor_t Calculate_Zx_Raw(phasor_t v1, phasor_t v2, switching_resistor_t Rref)
{
	phasor_t vdiff = phasor_sub(v1, v2);

	return (phasor_t) {
		(Rref/1000) * v2.magnitude / vdiff.magnitude, // divide my 1000 bc its in mOhms!!
#ifdef __WRAP2_2PI
	    wrap2_2pi( v2.phaserad - vdiff.phaserad)
#else

		v2.phaserad - vdiff.phaserad
#endif
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
//	if(Z_temp3.magnitude == 0)
//	{
//		while(1) TransmitStringRaw("2-phasor NAN!!");
//	}
	return (phasor_t) {
		Z_temp1.magnitude * Z_temp2.magnitude / Z_temp3.magnitude,
#ifdef __WRAP2_2PI
		wrap2_2pi( Z_temp1.phaserad + Z_temp2.phaserad - Z_temp3.phaserad )
#else
		Z_temp1.phaserad + Z_temp2.phaserad - Z_temp3.phaserad
#endif
	};
}

phasor_t Calculate_Zx_Full_Calibrated(phasor_t v1, phasor_t v2, switching_resistor_t Rref, phasor_t Zsm, phasor_t Zom, phasor_t Zstdm, phasor_t Zstd)
{
	phasor_t Zm = Calculate_Zx_Raw(v1, v2, Rref);

	/*
	 * ZDUT = Ztd * (Zom - Zstdm) * (Zm - Zsm) / ( (Zstdm - Zsm) * (Zom - Zm) )
	* */

	phasor_t ZomZstdm = phasor_sub(Zom, Zstdm);
	phasor_t ZmZsm = phasor_sub(Zm, Zsm);
	phasor_t ZstdmZsm = phasor_sub(Zstdm, Zsm);
	phasor_t ZomZm = phasor_sub(Zom, Zm);
	 // edge case -> div by 0 => 0s will be ignored in dB
	if(ZomZm.magnitude == 0 || ZstdmZsm.magnitude == 0) return (phasor_t) {0,0};


	return (phasor_t) {
		Zstd.magnitude * ZomZstdm.magnitude * ZmZsm.magnitude / ( ZstdmZsm.magnitude * ZomZm.magnitude ),
#ifdef __WRAP2_2PI
		wrap2_2pi( Zstd.phaserad + ZomZstdm.phaserad + ZmZsm.phaserad - ZstdmZsm.phaserad - ZomZm.phaserad )
#else
		Zstd.phaserad + ZomZstdm.phaserad + ZmZsm.phaserad - ZstdmZsm.phaserad - ZomZm.phaserad
#endif
	};
}

// Applies a moving average filter to the input data.
// input: array of input samples.
// output: array where the filtered data will be stored (should be the same length as input).
// n: number of samples in the dataset.
// window_size: number of samples to include in the average.
void Moving_Average_Filter(const uint16_t *input, uint16_t *output, size_t size, uint32_t fc)
{
    double sum = 0.0f;

    double actual_window_size = ( (double) (Get_Sampling_Frequency() / (2 * M_PI * fc)) );
//    double actual_window_size = ( (double) (fc / (2 * M_PI * Get_Sampling_Frequency())) );

    if(actual_window_size <= 1)
    {
    	memcpy(output, input, size * sizeof(uint16_t));
    	return;
    }

    size_t window_size = actual_window_size > 1 ? (size_t) (actual_window_size/1) : 1;


    // For the first few samples, where we don't have a full window yet,
    // compute the average over the available samples.
    size_t i;
    for (i = 0; i < window_size && i < size; i++)
    {
        sum += input[i];
        output[i] = sum / (i + 1);
    }

    // For the rest, use a running sum to compute the moving average.
    for (i = window_size; i < size; i++)
    {
        // Add new sample and subtract the sample leaving the window.
        sum += input[i] - input[i - window_size];
        output[i] = sum / window_size;
    }
}

double wrap2_2pi(double phase)
{
    phase = fmod(phase, 2 * M_PI);

    if (phase < 0) {
        phase += 2 * M_PI;
    }

    return phase;
}
