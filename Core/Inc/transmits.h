/*
 * transmits.h
 *
 *  Created on: Mar 9, 2025
 *      Author: ogahe
 */

#ifndef SRC_TRANSMITS_H_
#define SRC_TRANSMITS_H_

#include "main.h"

extern UART_HandleTypeDef huart2;

HAL_StatusTypeDef TransmitString(char msg[]);
HAL_StatusTypeDef TransmitStringRaw(char msg[]);
HAL_StatusTypeDef TransmitStringLn(char msg[]);
HAL_StatusTypeDef TransmitIntBuffer(int buffer[], size_t size);
HAL_StatusTypeDef TransmitUInt32Buffer(uint32_t buffer[], size_t size);
HAL_StatusTypeDef TransmitUInt16Buffer(uint16_t buffer[], size_t size);
HAL_StatusTypeDef TransmitTwoUInt16Buffer(uint16_t buffer1[], uint16_t buffer2[], size_t size);
HAL_StatusTypeDef TransmitUInt8Buffer(uint8_t buffer[], size_t size);
HAL_StatusTypeDef TransmitUInt16BufferAsRaw(uint16_t buffer[], size_t size);
HAL_StatusTypeDef TransmitNum(float num);
HAL_StatusTypeDef TransmitNumRaw(float num);
HAL_StatusTypeDef TransmitNumLn(float num);

#endif /* SRC_TRANSMITS_H_ */
