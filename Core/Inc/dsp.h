/*
 * dsp.h
 *
 *  Created on: Feb 28, 2025
 *      Author: ogahe
 */

#ifndef SRC_DSP_H_
#define SRC_DSP_H_

#include "sig_gen.h"
#include <sampling.h>
#include "main.h"
#include "relay.h"

#define ADC_SAMPLES_PER_CHANNEL (3000)
#define ADC_BUFFER_SIZE (2*ADC_SAMPLES_PER_CHANNEL)
//#define ADC_BUFFER_SIZE 4

#define F_SAMPLE_TIMER (2 * HAL_RCC_GetPCLK1Freq())
#define F_SAMPLE 350000
#define T_DELAY_SAMPLE ((15 + 15) / (HAL_RCC_GetPCLK2Freq() / 4) )

#define ACCEPTABLE_PHASE_DELTA (M_PI / 4)
#define MEAS_EXEC_TIMEOUT 20

#define NCONVERSIONCYCLES 45
#define __USING_MOVING_AVERAGE
//#define __WRAP2_2PI
#define __INCLUDE_CONV_PHASE
#define __REMOVE_OFFSETS_IQ

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim2;
extern DAC_HandleTypeDef hdac;
extern uint16_t sine_wave_buffer[DAC_LUT_SIZE];
extern uint16_t vmeas_buffer[ADC_BUFFER_SIZE];

typedef struct __phasor_t {
    double magnitude;
    double phaserad;
} phasor_t;

void HAL_ADC_HalfConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

void ADC_SampleSingleShot(void);

/**
 * @brief Separates a buffer of ADC samples into three channel-specific buffers.
 * @param buffADC: Input buffer containing interleaved ADC samples.
 * @param buffA: Output buffer for channel A samples.
 * @param buffB: Output buffer for channel B samples.
 * @param buffC: Output buffer for channel C samples.
 */
void ADC_Separate_Channels(uint16_t buffADC[], uint16_t buffA[], uint16_t buffB[]);

/**
 * @brief Calculates the phasor of a single signal using the IQ method.
 * @param sig: Input signal buffer (ADC samples).
 * @param len: Number of samples in the signal buffer.
 * @param f0: Frequency of the signal of interest (Hz).
 * @param fs: Sampling frequency (Hz).
 * @return The phasor (magnitude and phase in radians) corresponding to the input signal.
 */
phasor_t Get_Phasor_1Sig(uint16_t sig[], size_t len, uint32_t f0, uint32_t fs);

/**
 * @brief Calculates the phasor difference between a signal and a reference signal.
 * @param sig: Signal buffer.
 * @param ref: Reference signal buffer.
 * @param lensig: Number of samples in the signal buffer.
 * @param lenref: Number of samples in the reference buffer.
 * @param f0: Frequency of the signals (Hz).
 * @param fs: Sampling frequency (Hz).
 * @return A phasor representing the ratio (magnitude and phase difference) between the signal and reference.
 */
phasor_t Get_Phasor_2Sig(uint16_t sig[], uint16_t ref[], size_t lensig, size_t lenref, uint32_t f0, uint32_t fs);

/**
 * @brief Subtracts two phasors.
 * @param x1: The first phasor.
 * @param x2: The second phasor.
 * @return The resulting phasor after subtraction.
 */
phasor_t phasor_sub(phasor_t x1, phasor_t x2);

/**
 * @brief Calculates the raw impedance (Zx) using the voltage measurements.
 * @param v1: Phasor corresponding to the source voltage.
 * @param v2: Phasor corresponding to the voltage across the DUT.
 * @param Rref: The reference resistance value.
 * @return The raw impedance phasor.
 */
phasor_t Calculate_Zx_Raw(phasor_t v1, phasor_t v2, switching_resistor_t Rref);

/**
 * @brief Calculates the calibrated DUT impedance using measured phasors.
 * @param v1: Phasor for the source voltage.
 * @param v2: Phasor for the DUT voltage.
 * @param Rref: The reference resistance value.
 * @param Zsm: The short-circuit impedance phasor from calibration.
 * @param Zom: The open-circuit impedance phasor from calibration.
 * @return The calibrated DUT impedance phasor.
 */
phasor_t Calculate_Zx_Full_Calibrated(phasor_t v1, phasor_t v2, switching_resistor_t Rref, phasor_t Zsm, phasor_t Zom, phasor_t Zstdm, phasor_t Zstd);

phasor_t Calculate_Zx_Calibrated(phasor_t v1, phasor_t v2, switching_resistor_t Rref, phasor_t Zsm, phasor_t Zom);

/**
 * @brief Samples a steady state and separates ADC channels into three buffers.
 * @param f0: The trigger frequency for the measurement.
 * @param buffADC: Input ADC buffer containing interleaved samples.
 * @param vmeas0: Output buffer for channel 0.
 * @param vmeas1: Output buffer for channel 1.
 * @param vmeas2: Output buffer for channel 2.
 * @return The actual frequency (Hz) at which the ADC is sampling.
 */
uint32_t Sample_Steady_State(uint32_t f0, uint16_t vmeas0[], uint16_t vmeas1[]);

/**
 * @brief Samples steady-state phasors from the ADC data.
 * @param f0: The trigger frequency for the measurement.
 * @param buffADC: ADC data buffer containing interleaved samples.
 * @param input: Pointer to a phasor_t to store the input phasor.
 * @param output: Pointer to a phasor_t to store the output phasor.
 * @return The actual frequency (Hz) at which the ADC is sampling.
 */
uint32_t Sample_Steady_State_Phasors(uint32_t f0, phasor_t* input, phasor_t* output);

/**
 * @brief Populates arrays of raw phasors for input and output signals.
 * @param inputs: Array to store input phasors.
 * @param outputs: Array to store output phasors.
 * @param Rref: The reference resistor value.
 */
void Get_All_Raw_Phasors(phasor_t inputs[], phasor_t outputs[], float Rref);

/**
 * @brief Runs the measurement routine, calculating calibrated impedance phasors.
 * @param Zx_buff: Array to store the measured DUT impedance phasors.
 * @param Zsm_buff: Array containing the short-circuit calibration impedance phasors.
 * @param Zom_buff: Array containing the open-circuit calibration impedance phasors.
 * @param Rref: The reference resistance value.
 * @param frequencies_visited: Array to store the frequencies at which measurements were made.
 */

void Measurement_Routine_Zx_Calibrated(phasor_t Zx_buff[], phasor_t Zsm_buff[], phasor_t Zom_buff[], switching_resistor_t Rref, uint32_t frequencies_visited[]);

void Measurement_Routine_Zx_Full_Calibrated(phasor_t Zx_buff[], phasor_t Zsm_buff[], phasor_t Zom_buff[], phasor_t Zstdm_buff[], phasor_t Zstd[], switching_resistor_t Rref, uint32_t frequencies_visited[]);

void Measurement_Routine_Zx_Raw(phasor_t Zx_buff[], switching_resistor_t Rref, uint32_t frequencies_visited[]);

void Measurement_Routine_Voltage(phasor_t output[], switching_resistor_t Rref, uint32_t frequencies_visited[]);

void Moving_Average_Filter(const uint16_t *input, uint16_t *output, size_t size, uint32_t fc);

double wrap2_2pi(double phase);

#endif /* SRC_DSP_H_ */
