/*
 * relay.c
 *
 *  Created on: Mar 4, 2025
 *      Author: ogahe
 */

#include "relay.h"
#include "main.h"
#include "stdbool.h"
#include "dsp.h"


extern switching_resistor_t current_resistor;
extern uint16_t vmeas_buffer[ADC_BUFFER_SIZE];


void Set_Resistor_Hardware(switching_resistor_t res)
{
	switch(res)
	{
	case RESISTOR0:
		HAL_GPIO_WritePin(RREF_SEL0_GPIO_Port, RREF_SEL0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RREF_SEL1_GPIO_Port, RREF_SEL1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RREF_SEL2_GPIO_Port, RREF_SEL2_Pin, GPIO_PIN_RESET);
		break;

	case RESISTOR1:
		HAL_GPIO_WritePin(RREF_SEL0_GPIO_Port, RREF_SEL0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RREF_SEL1_GPIO_Port, RREF_SEL1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RREF_SEL2_GPIO_Port, RREF_SEL2_Pin, GPIO_PIN_RESET);
		break;

	case RESISTOR2:
		HAL_GPIO_WritePin(RREF_SEL0_GPIO_Port, RREF_SEL0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RREF_SEL1_GPIO_Port, RREF_SEL1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RREF_SEL2_GPIO_Port, RREF_SEL2_Pin, GPIO_PIN_RESET);
		break;

	default: // RESISTOR3
		HAL_GPIO_WritePin(RREF_SEL0_GPIO_Port, RREF_SEL0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RREF_SEL1_GPIO_Port, RREF_SEL1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RREF_SEL2_GPIO_Port, RREF_SEL2_Pin, GPIO_PIN_SET);
		break;
	}

	HAL_Delay(RELAY_SWITCH_DELAY_MS);
}


// Helper function to get neighboring resistor indices for a given resistor.
void Get_Neighbors(switching_resistor_t res, switching_resistor_t *neighbors, int *num_neighbors)
{
    // For simplicity, assume a linear order: RESISTOR0, RESISTOR1, RESISTOR2, RESISTOR3.
    // Each resistor can have up to two neighbors.

	if (res == RESISTOR0)
    {
        neighbors[0] = RESISTOR1;
        *num_neighbors = 1;
    }
	else if (res == RESISTOR1)
	{
		neighbors[0] = RESISTOR0;
		neighbors[1] = RESISTOR2;
		*num_neighbors = 2;
	}
	else if (res == RESISTOR2)
	{
		neighbors[0] = RESISTOR1;
		neighbors[1] = RESISTOR3;
		*num_neighbors = 2;
	}
    else
    {
        neighbors[0] = RESISTOR2;
        *num_neighbors = 1;
    }
}


double Measure_Midrange_Distance()
{
	Sig_Gen_Disable();
	Sampling_Disable();

	uint16_t relay_frequencies_wanted[RELAY_NFREQUENCIES] = {RELAY_CENTER_FREQ/2, RELAY_CENTER_FREQ, RELAY_CENTER_FREQ*2};

	phasor_t restest[RELAY_NFREQUENCIES];
	phasor_t temp;

	double midrange_dist;

	for(size_t i = 0; i < RELAY_NFREQUENCIES; i++)
	{
		// actual frequency doesn't matter
		Sample_Steady_State_Phasors(relay_frequencies_wanted[i], &temp, &restest[i]);
		if(restest[i].magnitude >= 1)
		{
			i--;
			continue;
		}
		// normalise
		midrange_dist += fabs(restest[i].magnitude - 0.5);
	}

	midrange_dist /= RELAY_NFREQUENCIES;

	TransmitStringRaw("Quality Measured for Resistor: ");
	TransmitUInt32Raw(current_resistor);
	TransmitStringRaw(" | Quality: ");
	TransmitNumRaw((float) midrange_dist);
	TransmitStringRaw("\r\n");

	return midrange_dist;
}

/*
// Main function to choose the resistor based on quality measurements.
void Choose_Switching_Resistor()
{
    static const switching_resistor_t resistorOptions[NUM_RESISTORS] = {RESISTOR0, RESISTOR1, RESISTOR2, RESISTOR3};
    double amplitudeResults[NUM_RESISTORS];

    // 1. Try all resistors and store their quality measurements (atan2 at 1kHz)
    for (int i = 0; i < NUM_RESISTORS; i++)
    {
    	current_resistor = resistorOptions[i];
        Set_Resistor_Hardware(current_resistor);
        amplitudeResults[i] = Measure_Midrange_Distance();
    }

    // 2. Find the resistor with the relative maximum quality that also has a similar neighbor.
    double bestQuality = -INFINITY;
    switching_resistor_t bestResistor = RESISTOR0;

    for (int i = 0; i < NUM_RESISTORS; i++)
    {
        float currentQuality = amplitudeResults[i];

        // Get the neighbors for the current resistor.
        switching_resistor_t neighbors[2];
        int num_neighbors = 0;
        Get_Neighbors(resistorOptions[i], neighbors, &num_neighbors);
        TransmitStringLn("Neighbors Gotten");

        // Check if at least one neighbor has a similar quality (within TOLERANCE).
        bool similarNeighborFound = false;
        for (int j = 0; j < num_neighbors; j++)
        {
            // Calculate relative difference.
            float neighborQuality = amplitudeResults[neighbors[j]];
            float diff = fabs(neighborQuality - currentQuality) / currentQuality;
            if (diff < RES_QTOLERANCE)
            {
                similarNeighborFound = true;
                break;
            }
        }

        // Only consider this resistor if it has at least one similar neighbor.
        if (similarNeighborFound && currentQuality > bestQuality)
        {
            bestQuality = currentQuality;
            bestResistor = resistorOptions[i];
        }
    }

    TransmitStringRaw("Chose Resistor: ");
    TransmitUInt32Raw(bestResistor);
    TransmitStringRaw(" | Quality: ");
    TransmitNumRaw((float) bestQuality);
    TransmitStringRaw("\r\n");

    // 3. Set the final resistor selection.
    current_resistor = bestResistor;
    Set_Resistor_Hardware(current_resistor);
}

*/

void Choose_Switching_Resistor()
{
	static const switching_resistor_t available_resistors[NUM_RESISTORS] = {RESISTOR0, RESISTOR1, RESISTOR2, RESISTOR3};

	current_resistor = RESISTOR3;
	Set_Resistor_Hardware(current_resistor);

	uint32_t frequencies_wanted[RELAY_NFREQUENCIES];
	uint32_t frequencies_visited[RELAY_NFREQUENCIES];
	Calculate_Frequencies(RELAY_FREQ_MIN, RELAY_FREQ_MAX, RELAY_PPDECADE, RELAY_NFREQUENCIES, frequencies_wanted);

	phasor_t v1[RELAY_NFREQUENCIES];
	phasor_t v2[RELAY_NFREQUENCIES];
	phasor_t Zx_buff[RELAY_NFREQUENCIES];
	double max_magnitude = -1;

	Set_Sampling_Frequency(F_SAMPLE);

	for(size_t i = 0; i < RELAY_NFREQUENCIES; i++)
	{
		frequencies_visited[i] = Sample_Steady_State_Phasors(frequencies_wanted[i], &v1[i], &v2[i]);
		Zx_buff[i] = Calculate_Zx_Raw(v1[i], v2[i], current_resistor);


		if(v2[i].magnitude > 1) i--; // rejected
		else max_magnitude = Zx_buff[i].magnitude > max_magnitude ? Zx_buff[i].magnitude : max_magnitude; // accepted
	}

//	max_magnitude /= RELAY_NFREQUENCIES;

	TransmitStringRaw("Average Impedance Magnitude (Ohms): ");
	TransmitNumRaw((float) max_magnitude);
	TransmitStringRaw("\r\n");

	double best_error = 1e32;
	double cur_error;
	switching_resistor_t best_resistor = RESISTOR0;
	for(size_t i = 0; i < NUM_RESISTORS; i++)
	{
		cur_error = fabs(available_resistors[i] / 1000 - max_magnitude);
		if(cur_error < best_error)
		{
			best_error = cur_error;
			best_resistor = available_resistors[i];
			TransmitStringRaw("New Best Resistor: ");
			TransmitNumRaw(best_resistor / 1000);
			TransmitStringRaw("\r\n");
		}
	}

	TransmitStringRaw("Best Resistor is: ");
	TransmitNumRaw((float) best_resistor / 1000);
	TransmitStringRaw(" | Magnitude Error: ");
	TransmitUInt32Raw(best_error);
	TransmitStringRaw("\r\n");

	current_resistor = best_resistor;
	Set_Resistor_Hardware(current_resistor);


	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}
