/*
 * relay.c
 *
 *  Created on: Mar 4, 2025
 *      Author: ogahe
 */

#include "relay.h"

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
		HAL_GPIO_WritePin(RREF_SEL1_GPIO_Port, RREF_SEL1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RREF_SEL2_GPIO_Port, RREF_SEL2_Pin, GPIO_PIN_SET);
		break;
	}
}
