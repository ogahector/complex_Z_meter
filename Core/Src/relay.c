/*
 * relay.c
 *
 *  Created on: Mar 4, 2025
 *      Author: ogahe
 */

#include "relay.h"
#include "main.h"

void Choose_Switching_Resistor(switching_resistor_t res)
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
    else if (res == RESISTOR3)
    {
        neighbors[0] = RESISTOR2;
        *num_neighbors = 1;
    }
    else // RESISTOR1 or RESISTOR2
    {
        neighbors[0] = (switching_resistor_t)(res - 1);
        neighbors[1] = (switching_resistor_t)(res + 1);
        *num_neighbors = 2;
    }
}

// Main function to choose the resistor based on quality measurements.
void Choose_Switching_Resistor()
{
    switching_resistor_t resistorOptions[NUM_RESISTORS] = {RESISTOR0, RESISTOR1, RESISTOR2, RESISTOR3};
    float qualityResults[NUM_RESISTORS];

    // 1. Try all resistors and store their quality measurements (atan2 at 1kHz)
    for (int i = 0; i < NUM_RESISTORS; i++)
    {
        Set_Resistor_Hardware(resistorOptions[i]);
        qualityResults[i] = Measure_Quality();
    }

    // 2. Find the resistor with the relative maximum quality that also has a similar neighbor.
    float bestQuality = -INFINITY;
    switching_resistor_t bestResistor = RESISTOR0;
    for (int i = 0; i < NUM_RESISTORS; i++)
    {
        float currentQuality = qualityResults[i];

        // Get the neighbors for the current resistor.
        switching_resistor_t neighbors[2];
        int num_neighbors = 0;
        Get_Neighbors(resistorOptions[i], neighbors, &num_neighbors);

        // Check if at least one neighbor has a similar quality (within TOLERANCE).
        bool similarNeighborFound = false;
        for (int j = 0; j < num_neighbors; j++)
        {
            // Calculate relative difference.
            float neighborQuality = qualityResults[neighbors[j]];
            float diff = fabs(neighborQuality - currentQuality) / currentQuality;
            if (diff < TOLERANCE)
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

    // 3. Set the final resistor selection.
    Set_Resistor_Hardware(bestResistor);
    Signal_Resistor_Selection_Complete(bestResistor);
}

