/*
 * transmits.c
 *
 *  Created on: Mar 9, 2025
 *      Author: ogahe
 */


#include "transmits.h"





ui_command_t Receive_Command(void)
{
	char msg[RX_CMD_BYTE_NB];
	ReceiveMessage(msg, RX_CMD_BYTE_NB);

	return (ui_command_t) (msg[0] << 8) | (msg[1]); // nerver using par1 or par2
}


uint8_t Command_is_Available(void)
{
	if(cmd_available)
	{
		cmd_available = 0;
		return 1;
	}
	return 0;
}

void Echo_Message(void)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)"Received: $", 11, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*) &rx_buffer[rx_index], strlen((char*) &rx_buffer[rx_index]), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"#\r\n", 3, 100);
}


/*-------------------------------------------------------------------------*/



HAL_StatusTypeDef TransmitMessageUI(char msg[])
{

}
HAL_StatusTypeDef TransmitUInt32BufferUI(char msg[], uint32_t buffer[]);
HAL_StatusTypeDef TransmitUint32UI(char msg[], uint32_t num);


/*-------------------------------------------------------------------------*/

HAL_StatusTypeDef ReceiveMessage(char msg[], size_t len)
{
	return HAL_UART_Receive(&huart2, msg, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef TransmitString(char msg[])
{
	if(HAL_UART_Transmit(&huart2, (const unsigned char*) msg, (uint16_t) strlen(msg), 5) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_UART_Transmit(&huart2, (const unsigned char*) "\r", (uint16_t) strlen("\r"), 5);
}

HAL_StatusTypeDef TransmitStringRaw(char msg[])
{
//	return HAL_UART_Transmit(&huart2, msg, (uint16_t) strlen(msg), HAL_MAX_DELAY);
	return HAL_UART_Transmit(&huart2, msg, (uint16_t) strlen(msg), 5);
}

HAL_StatusTypeDef TransmitStringLn(char msg[])
{
	if (TransmitString(msg) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return TransmitString("\n");
}


HAL_StatusTypeDef TransmitIntBuffer(int buffer[], size_t size)
{
    char msg[32];  // Buffer to hold the formatted string

    TransmitString("\n");
    for (size_t i = 0; i < size; i++)
    {
        // Format the integer value into a string followed by a newline
        sprintf(msg, "%d\r\n", buffer[i]);

        // Transmit the formatted string over USART
        if (TransmitString(msg) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}


HAL_StatusTypeDef TransmitUInt32Buffer(uint32_t buffer[], size_t size)
{
    char msg[32];  // Buffer to hold the formatted string

    TransmitString("\n");
    for (size_t i = 0; i < size; i++)
    {
        // Format the integer value into a string followed by a newline
        sprintf(msg, "%lu\r\n", buffer[i]);

        // Transmit the formatted string over USART
        if (TransmitString(msg) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}


HAL_StatusTypeDef TransmitUInt16Buffer(uint16_t buffer[], size_t size)
{
    char msg[32];  // Buffer to hold the formatted string

    TransmitString("\n");
    for (size_t i = 0; i < size; i++)
    {
        // Format the integer value into a string followed by a newline
        sprintf(msg, "%u\r\n", buffer[i]);

        // Transmit the formatted string over USART
        if (TransmitString(msg) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef TransmitTwoUInt16Buffer(uint16_t buffer1[], uint16_t buffer2[], size_t size)
{
    char msg[32];  // Buffer to hold the formatted string

//    TransmitString("\n");
    for (size_t i = 0; i < size-1; i++)
    {
        // Format the integer value into a string followed by a newline
        sprintf(msg, "%u %u \r\n", buffer1[i], buffer2[i]);

        // Transmit the formatted string over USART
        if (TransmitStringRaw(msg) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    sprintf(msg, "%u %u \r\n", buffer1[size-1], buffer2[size-1]);

    // Transmit the formatted string over USART
    if (TransmitStringRaw(msg) != HAL_OK)
    {
    return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef TransmitUInt8Buffer(uint8_t buffer[], size_t size)
{
    char msg[32];  // Buffer to hold the formatted string

    TransmitString("\n");
    for (size_t i = 0; i < size; i++)
    {
        // Format the integer value into a string followed by a newline
        sprintf(msg, "%lu\r\n", buffer[i]);

        // Transmit the formatted string over USART
        if (TransmitString(msg) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef TransmitUInt16BufferAsRaw(uint16_t buffer[], size_t size)
{
	return HAL_UART_Transmit(&huart2, buffer, size, 10);
}

HAL_StatusTypeDef TransmitNum(float num)
{
	char msg[32];

	sprintf(msg, "%f", num);

	return TransmitString(msg);
}


HAL_StatusTypeDef TransmitNumRaw(float num)
{
	char msg[32];

	sprintf(msg, "%f", num);

	return TransmitStringRaw(msg);
}

HAL_StatusTypeDef TransmitUInt32Raw(uint32_t num)
{
	char msg[32];
	sprintf(msg, "%u", num);
	return TransmitStringRaw(msg);
}

HAL_StatusTypeDef TransmitNumLn(float num)
{
	char msg[32];
	sprintf(msg, "%f\r\n", num);
	return TransmitString(msg);
}
