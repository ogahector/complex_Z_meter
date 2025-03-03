/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sig_gen.h"
#include "dual_spi_adc.h"
#include "dsp.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define VMEAS0_Pin GPIO_PIN_0
#define VMEAS0_GPIO_Port GPIOC
#define VMEAS2_Pin GPIO_PIN_0
#define VMEAS2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ADC_CONVST_Pin GPIO_PIN_6
#define ADC_CONVST_GPIO_Port GPIOA
#define ADC_RD_Pin GPIO_PIN_7
#define ADC_RD_GPIO_Port GPIOA
#define VMEAS1_Pin GPIO_PIN_4
#define VMEAS1_GPIO_Port GPIOC
#define ADC_M0_Pin GPIO_PIN_7
#define ADC_M0_GPIO_Port GPIOC
#define ADC_M1_Pin GPIO_PIN_9
#define ADC_M1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SPI2_MISO_Pin GPIO_PIN_11
#define SPI2_MISO_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ADC_BUSY_Pin GPIO_PIN_6
#define ADC_BUSY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define FREQ_MAX 100000
#define FREQ_MIN 100
#define FREQ_PPDECADE 50
#define FREQ_NDECADE 3
#define NFREQUENCIES ( FREQ_PPDECADE * FREQ_NDECADE )

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
