/*
 * dual_spi_adc.h
 *
 *  Created on: Feb 25, 2025
 *      Author: ogahe
 */


#include "dual_spi_adc.h"
#include <stdint.h>
//#include <stdint-gcc.h>

void DualSPI_ADC_Read(uint16_t *adc_ch1, uint16_t *adc_ch2)
{
    // Define transmit and receive buffers.
    // Assuming the ADC sends 16 bits per channel.
//    uint8_t txBuffer[2] = {0x00, 0x00};  // If the ADC requires a dummy command, set it here.
//    uint8_t rxBuffer_SPI2[2] = {0};
//    uint8_t rxBuffer_SPI3[2] = {0};
//
//    // Assert the common NSS (active low)
//    HAL_GPIO_WritePin(ADC_NSS_GPIO_Port, ADC_NSS_Pin, GPIO_PIN_RESET);
//
//    // Perform SPI transaction on SPI2 (reads one channel)
//    if(HAL_SPI_TransmitReceive(&hspi2, txBuffer, rxBuffer_SPI2, 2, HAL_MAX_DELAY) != HAL_OK)
//    {
//        // Handle error (optional)
//    }
//
//    // Perform SPI transaction on SPI3 (reads the other channel)
//    if(HAL_SPI_TransmitReceive(&hspi3, txBuffer, rxBuffer_SPI3, 2, HAL_MAX_DELAY) != HAL_OK)
//    {
//        // Handle error (optional)
//    }
//
//    // Deassert NSS after both transactions
//    HAL_GPIO_WritePin(ADC_NSS_GPIO_Port, ADC_NSS_Pin, GPIO_PIN_SET);
//
//    // Combine the received bytes into 16-bit values
//    *adc_ch1 = ((uint16_t)rxBuffer_SPI2[0] << 8) | rxBuffer_SPI2[1];
//    *adc_ch2 = ((uint16_t)rxBuffer_SPI3[0] << 8) | rxBuffer_SPI3[1];
}
