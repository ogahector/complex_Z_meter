/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_common_tables.h"
#include "arm_const_structs.h"
#include "arm_math.h"
#include "stdio.h"
#include "math.h"
#include "stdbool.h"
#include "relay.h"
#include "sig_gen.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NORMAL_MODE
#define DEBUG_MODES
//#define ADC_DAC_DEBUG_MODE
//#define ONBOARD_FSM_MODE
//#define BOARD_DEBUG_MODE
//#define UART_DEBUG_MODE
//#define SWITCHING_RESISTOR_DEBUG_MODE


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Communication with the UI
volatile uint8_t cmd_available = 0;
volatile ui_command_t command = idle;
volatile uint16_t param1 = 0;
volatile uint16_t param2 = 0;
volatile uint8_t rx_buffer[RX_CMD_BYTE_NB];
volatile size_t rx_index = 0;

// Measuring and Signal Buffers
uint16_t sine_wave_buffer[DAC_LUT_SIZE];
uint16_t vmeas_buffer[ADC_BUFFER_SIZE];

// System Current Resistor
switching_resistor_t current_resistor;

// Calibration IMPEDANCE values
phasor_t OC_CAL[NFREQUENCIES];
phasor_t SC_CAL[NFREQUENCIES];
phasor_t LD_CAL[NFREQUENCIES];
phasor_t Zstd[NFREQUENCIES];
const double capacitance_std = 10e-9;
const double resistance_std = 3260.0f;
phasor_t phasorZero[NFREQUENCIES];

// Measured Impedance Values
phasor_t Zx_measured[NFREQUENCIES];
uint32_t frequencies_visited[NFREQUENCIES] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
bool buttonPress(void);
uint32_t GetTimXCurrentFrequency(TIM_HandleTypeDef* htim);

// should be in transmits.h but it's being fussy
char* data2str_formatted(phasor_t phasors[], uint32_t freqs[], size_t size);
HAL_StatusTypeDef TransmitPhasor(phasor_t phasor);
HAL_StatusTypeDef TransmitPhasorRaw(phasor_t phasor);
HAL_StatusTypeDef TransmitPhasorLn(phasor_t phasor);

HAL_StatusTypeDef TransmitPhasorDataframe(phasor_t phasors[], uint32_t frequencies_visited[], switching_resistor_t res);
HAL_StatusTypeDef TransmitPhasorSingleUI(uint32_t freq, phasor_t phasor);
HAL_StatusTypeDef TransmitPhasorBufferUI(uint32_t freqs[], phasor_t phasors[]);
HAL_StatusTypeDef TransmitPhasorDataframeUI(uint32_t freqs[], phasor_t phasors[], switching_resistor_t res);

void Process_Command(ui_command_t command_received);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  MX_USART2_UART_Init();
//  TransmitString("\n");

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  long long i = 0;

  // CALCULATE VALUES FOR SINE BUFFER DMA
  Fill_Sine_Buffer();

  // INITIALISE DAC DMA in the previous function

  // ADC DMA INIT in the previous function

#ifdef UART_DEBUG_MODE
  // GET DAC STATE ON STARTUP
  // Expected: BUSY  char msg1[32];
  sprintf(msg1, "Current DAC State: %d", HAL_DAC_GetState(&hdac));
  TransmitStringLn(msg1);

  // GET TIM STATE ON STARTUP
  // Expected: READY
  char msg2[32];
  sprintf(msg2, "Current TIM2 State: %d", HAL_TIMEx_GetChannelNState(&htim2, TIM2_BASE));
  TransmitStringLn(msg2);

  // GET CURRENT FREQUENCY
  char msg3[64];
  sprintf(msg3, "Current TIM2 Freq: %lu", GetTimXCurrentFrequency(&htim2));
  TransmitStringLn(msg3);
#endif

  Sampling_Enable();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);



  // assert 0 calibration phasors
  for(size_t i = 0; i < NFREQUENCIES; i++)
  {
	  SC_CAL[i] = (phasor_t) {0,0};
	  OC_CAL[i] = (phasor_t) {1e32,0};
	  LD_CAL[i] = (phasor_t) {0,0};
	  phasorZero[i] = (phasor_t) {0,0};
  }

  // calculate reference impedance
  uint32_t frequencies_wanted[NFREQUENCIES];

  current_resistor = RESISTOR1;
  Set_Resistor_Hardware(RESISTOR0);

  HAL_UART_Receive_IT(&huart2, rx_buffer, RX_CMD_BYTE_NB);

#ifdef BOARD_DEBUG_MODE
  Set_Signal_Frequency(100000);
  Set_Sampling_Frequency(F_SAMPLE);

  Sig_Gen_Enable();
  Set_Signal_Frequency(100e3);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  i++;
#ifdef NORMAL_MODE

	  if(Command_is_Available())
	  {
		  Process_Command(command);
	  }

#endif /*NORMAL MODE*/

#ifndef NORMAL_MODE
#ifdef ADC_DAC_DEBUG_MODE

	  if(buttonPress())
	  {
		  Sig_Gen_Enable();
		  ADC_SampleSingleShot();
		  Sig_Gen_Disable();
		  Sampling_Disable();

//		  TransmitTwoUInt16Buffer(vmeas_buffer3, vmeas_buffer2, ADC_BUFFER_SIZE);
		  TransmitUInt16Buffer(vmeas_buffer, ADC_BUFFER_SIZE);
//		  HAL_Delay(10);
	  }
	  continue;

#endif

#ifdef BOARD_DEBUG_MODE
	  if(buttonPress())
	  {
		  Sig_Gen_Enable();
	  }

//	  else
//	  {
//		  Sig_Gen_Disable();
//	  }
	  continue;

#endif

#ifdef SWITCHING_RESISTOR_DEBUG_MODE
	  if(buttonPress())
	  {
//		  Set_Resistor_Hardware(RESISTOR3);

		  Choose_Switching_Resistor();

		  TransmitStringLn("DONE!");

		  HAL_Delay(100);

		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  }
//	  else
//	  {
//		  Set_Resistor_Hardware(RESISTOR0);
//		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//	  }
#endif /* SWITCHING_RESISTOR_DEBUG_MODE */
#endif /* NORMAL_MODE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */
#ifdef UART_DEBUG_MODE
  TransmitStringLn("Beginning DAC DMA Init...");
#endif
  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  if(HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_wave_buffer, DAC_LUT_SIZE, DAC_ALIGN_12B_R) != HAL_OK)
  {
	  TransmitStringLn("ERROR SETTING UP DAC DMA");
  }
  else
  {
#ifdef UART_DEBUG_MODE
	  TransmitStringLn("DAC DMA SUCCESSFULLY INITIALISED");
#endif
  }

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 44;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
//    if(HAL_TIM_Base_Start(&htim2) != HAL_OK)
//    {
//  	  TransmitStringLn("TIM2 SUCCESSFULLY INITIALISED");
//    }
//    else
//    {
//  	  TransmitStringLn("ERROR INITIALISING TIM2");
//    }
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 44;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */


  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RREF_SEL0_Pin|RREF_SEL1_Pin|RREF_SEL2_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VMEAS2_Pin */
  GPIO_InitStruct.Pin = VMEAS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_CONVST_Pin ADC_RD_Pin ADC_M1_Pin */
  GPIO_InitStruct.Pin = ADC_CONVST_Pin|ADC_RD_Pin|ADC_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RREF_SEL0_Pin RREF_SEL1_Pin RREF_SEL2_Pin PB7 */
  GPIO_InitStruct.Pin = RREF_SEL0_Pin|RREF_SEL1_Pin|RREF_SEL2_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_M0_Pin */
  GPIO_InitStruct.Pin = ADC_M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_M0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_BUSY_Pin */
  GPIO_InitStruct.Pin = ADC_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_BUSY_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

//  GPIO_InitStruct.Pin = VMEAS0_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//  HAL_GPIO_Init(VMEAS0_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = VMEAS1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//  HAL_GPIO_Init(VMEAS1_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = VMEAS2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//  HAL_GPIO_Init(VMEAS2_GPIO_Port, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


bool buttonPress(void)
{
	return !HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
}


uint32_t GetTimXCurrentFrequency(TIM_HandleTypeDef* htim)
{
	uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();

	uint32_t psc = __HAL_TIM_GET_ICPRESCALER(htim, TIM6_BASE);
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);


	// Compute update frequency: f_update = timer_clk / ((PSC+1) * (ARR+1))
	uint32_t f_update = timer_clk / ((psc + 1) * (arr + 1));
	return f_update;
}


char* data2str_formatted(phasor_t phasors[], uint32_t freqs[], size_t size)
{
    // Calculate total length needed (start with "$[]#\0" = 5 bytes)
    size_t length = 5 + 4;
    TransmitStringRaw("1");
    TransmitStringRaw("freqs[0]");
    for (size_t i = 0; i < size * 3; i++) {
        char temp[64];  // Increased buffer size
        int num_len;
        const size_t index = i / 3;

        if (index >= size) break;  // Safety check

        if (i % 3 == 0) {
        	//            num_len = snprintf(temp, sizeof(temp), "%" PRIu32, freqs[index]);
        	num_len = snprintf(temp, sizeof(temp), "%lu", freqs[index]);
        } else if (i % 3 == 1) {
            num_len = snprintf(temp, sizeof(temp), "%.16g", phasors[index].magnitude);
        } else {
            num_len = snprintf(temp, sizeof(temp), "%.16g", phasors[index].phaserad);
        }
        length += num_len + (i > 0 ? 1 : 0);  // Add comma only after first element
    }

    TransmitStringRaw("2");

    // Allocate memory
    char* buffer = malloc(length);

    TransmitStringRaw("3");
    if (!buffer) return NULL;

    // Build the string
    char* ptr = buffer;
    ptr += sprintf(ptr, "fuck$[");


    TransmitStringRaw("4");

    for (size_t i = 0; i < size * 3; i++) {
        const size_t index = i / 3;
        if (index >= size) break;

        if (i > 0) {
            ptr += sprintf(ptr, ",");
        }

        if (i % 3 == 0) {
        	//            ptr += sprintf(ptr, "%" PRIu32, freqs[index]);
        	ptr += sprintf(ptr, "%lu", freqs[index]);
        } else if (i % 3 == 1) {
            ptr += sprintf(ptr, "%.16g", phasors[index].magnitude);
        } else {
            ptr += sprintf(ptr, "%.16g", phasors[index].phaserad);
        }
    }


    TransmitStringRaw("5");

    sprintf(ptr, "]#");

    TransmitStringRaw("6");
    return buffer;
}


HAL_StatusTypeDef TransmitPhasor(phasor_t phasor)
{
	char msg[32];
	sprintf(msg, "Phasor: %.3f, %.3f\r", phasor.magnitude, phasor.phaserad);
	return TransmitString(msg);
}

HAL_StatusTypeDef TransmitPhasorRaw(phasor_t phasor)
{
	char msg[32];
	sprintf(msg, "%e,%e", phasor.magnitude, phasor.phaserad);
	return TransmitStringRaw(msg);
}

HAL_StatusTypeDef TransmitPhasorLn(phasor_t phasor)
{
	char msg[32];
	sprintf(msg, "Phasor: %.3f, %.3f\r\n", phasor.magnitude, phasor.phaserad);
	return TransmitString(msg);
}

HAL_StatusTypeDef TransmitPhasorDataframe(phasor_t phasors[], uint32_t frequencies_visited[], switching_resistor_t res)
{
	for(size_t i = 0; i < NFREQUENCIES; i++)
	{
		TransmitStringRaw("Measurement:Freq "); TransmitUInt32Raw(frequencies_visited[i]); TransmitStringRaw(" ");
		TransmitUInt32Raw(Get_Sampling_Frequency()); TransmitStringRaw(" ");
		TransmitNumRaw((float) res/1000); TransmitStringRaw(" ");

		TransmitPhasorRaw(isnan(phasors[i].magnitude) ? (phasor_t) {0,phasors[i].phaserad} : phasors[i]);
		TransmitStringLn(" ");
		HAL_Delay(1);
	}
	return TransmitStringLn("DONE!");
}

HAL_StatusTypeDef TransmitPhasorSingleUI(uint32_t freq, phasor_t phasor)
{
	char msg[32 + 6 + 2 + 2 + 2 + 2*(6+2)];
	// message len + 6 freq digits + 2 brackets + 2 commas + 2 delimiter flags + 2 doubles + padding extra
	sprintf(msg, "Phasor: [%lu, %.6f, %.6f] $[%lu,%.6f,%.6f]#",
			freq, phasor.magnitude, phasor.phaserad,
			freq, phasor.magnitude, phasor.phaserad);
	return TransmitStringRaw(msg);
}

HAL_StatusTypeDef TransmitPhasorBufferUI(uint32_t freqs[], phasor_t phasors[])
{
	char msg[32 + 6 + 2 + 2 + 2 + 2*(6+2)];
	// message len + 6 freq digits + 2 brackets + 2 commas + 2 delimiter flags + 2 doubles + padding extra
	TransmitStringRaw("Sending Phasor DataFrame (please work) $[");
	for(size_t i = 0; i < NFREQUENCIES-1; i++)
	{
		sprintf(msg, "%lu,%.6f,%.6f,", freqs[i], phasors[i].magnitude, phasors[i].phaserad);
		TransmitStringRaw(msg);
	}
	sprintf(msg, "%lu,%.6f,%.6f", freqs[NFREQUENCIES - 1], phasors[NFREQUENCIES - 1].magnitude, phasors[NFREQUENCIES - 1].phaserad);
	TransmitStringRaw(msg);
	return TransmitStringRaw("]#");
}

HAL_StatusTypeDef TransmitPhasorDataframeUI(uint32_t freqs[], phasor_t phasors[], switching_resistor_t res)
{
	TransmitStringRaw("Sending Phasor DataFrame (please work) bla\n $[");
	for(int i = 0; i < NFREQUENCIES-1; i++)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		char msg[128];
		snprintf(msg, 128, "%lu,%.4e,%.4e,", freqs[i], phasors[i].magnitude, phasors[i].phaserad);
		TransmitStringRaw(msg);
	}
	char msg[128];
	sprintf(msg, "%lu,%.4e,%.4e", freqs[NFREQUENCIES - 1], phasors[NFREQUENCIES - 1].magnitude, phasors[NFREQUENCIES - 1].phaserad);
	TransmitStringRaw(msg);
	return TransmitStringRaw("]#");
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart->Instance == USART2)
	{
		cmd_available = 0x01;
		command = rx_buffer[0] << 8 | rx_buffer[1];
		param1 = rx_buffer[2] << 8 | rx_buffer[3];
		param2 = rx_buffer[4] << 8 | rx_buffer[5];

		HAL_UART_Receive_IT(huart, rx_buffer, RX_CMD_BYTE_NB);
	}
}

void Process_Command(ui_command_t command_received)
{
	char msg[128];

	switch(command_received)
	{
	case get_version:
	{
		sprintf(msg, "Current Version: %u.\n$%u#", DEVELOPMENT_VER, DEVELOPMENT_VER);
		TransmitStringRaw(msg);
		break;
	}

	case get_id:
	{
		sprintf(msg, "ID: %d \n$%d#", BOARD_ID, BOARD_ID);
		TransmitStringRaw(msg);
		break;
	}

	case get_clk_divpw:
	{
		uint32_t clk_mhz = (uint32_t) (HAL_RCC_GetHCLKFreq() / 1e6);
		sprintf(msg, "Clock Frequency: %lu \n$%lu#", clk_mhz, clk_mhz);
		TransmitStringRaw(msg);
		break;
	}

	case check_status_3v3:
	case check_status_neg12:
	case check_status_pos12:
	{
		sprintf(msg, "Power Supply Status %d \n$%d#", 1, 1);
		TransmitStringRaw(msg);
		break;
	}

	case rref_get_val:
	{
		double res_ohms = (double) (current_resistor / 1000);
		sprintf(msg, "Current Resistor Value: %f\n$%f#", res_ohms, res_ohms);
		TransmitStringRaw(msg);
	}

	case rref_set_val:
	{
		switching_resistor_t new_res;
		switch(param1) // determine correct switch cases
		{
		case 1:
		{
			new_res = RESISTOR1;
			break;
		}
		case 2:
		{
			new_res = RESISTOR2;
		}
		case 3:
		{
			new_res = RESISTOR3;
			break;
		}
		default:
		{
			new_res = RESISTOR0;
			break;
		}
		}
		current_resistor = new_res;
		Set_Resistor_Hardware(new_res);

		break;
	}

	case start_sc_calib:
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

		current_resistor = RESISTOR1;
//		Set_Resistor_Hardware(current_resistor);

		Measurement_Routine_Zx_Raw(SC_CAL, current_resistor, frequencies_visited);

		TransmitPhasorDataframeUI(frequencies_visited, SC_CAL, current_resistor); // ? idk if needed

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		break;
	}

	case start_oc_calib:
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

		current_resistor = RESISTOR1;
//		current_resistor = RESISTOR0; // bc we have no choice for now
//		Set_Resistor_Hardware(current_resistor);

		Measurement_Routine_Zx_Raw(OC_CAL, current_resistor, frequencies_visited);

		TransmitPhasorDataframeUI(frequencies_visited, OC_CAL, current_resistor);

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		break;
	}

	case start_ld_calib:
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

		current_resistor = RESISTOR1;
//		Set_Resistor_Hardware(current_resistor);

		Measurement_Routine_Zx_Raw(LD_CAL, current_resistor, frequencies_visited);

		TransmitPhasorDataframeUI(frequencies_visited, LD_CAL, current_resistor);

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		break;
	}

	case readout_meas:
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

//		Choose_Switching_Resistor();

//		Measurement_Routine_Zx_Calibrated(Zx_measured, SC_CAL, OC_CAL, current_resistor, frequencies_visited);
		Measurement_Routine_Zx_Full_Calibrated(Zx_measured, SC_CAL, OC_CAL, LD_CAL, Zstd, current_resistor, frequencies_visited);
//		Measurement_Routine_Zx_Raw(Zx_measured, current_resistor, frequencies_visited);
//		Measurement_Routine_Voltage(Zx_measured, current_resistor, frequencies_visited);

		TransmitPhasorDataframeUI(frequencies_visited, Zx_measured, current_resistor);

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		break;
	}

	case start_rlc_fit:
	{
		TransmitPhasorDataframeUI(frequencies_visited, Zx_measured, current_resistor);
		break;
	}

	case idle:
	{
		break;
	}
	default:
	{
		break;
	}

	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
      // Toggle an LED or do nothing so you can observe that a fault occurred
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(100);
      TransmitStringLn("Fatal ERROR! Handled by the Error_Handler...");
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
