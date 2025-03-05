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

#include "sig_gen.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum __INSTRUMENT_STATE
{
	IDLE,
	BUTTON_PRESS,
	CALIBRATING,
	MEASURING
} INSTRUMENT_STATE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

uint16_t sine_wave_buffer[DAC_LUT_SIZE];
uint16_t vmeas_buffer[ADC_BUFFER_SIZE];


//uint16_t vmeas1_buffer[1];
//uint16_t vmeas2_buffer[1];

//const uint32_t freq_max = 100e3;
//const uint32_t freq_min = 100;
//const uint8_t freq_ppdecade = 50;
//uint32_t frequencies[3 * freq_ppdecade];

INSTRUMENT_STATE STATE = IDLE;

phasor_t OC_CAL = (phasor_t) {0, 0};
phasor_t SC_CAL = (phasor_t) {0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
bool buttonPress(void);
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
HAL_StatusTypeDef TransmitNumLn(float num);
HAL_StatusTypeDef TransmitPhasor(phasor_t phasor);
HAL_StatusTypeDef TransmitPhasorLn(phasor_t phasor);
void ReceiveMessage(char msg[], size_t len);
uint32_t GetTimXCurrentFrequency(TIM_HandleTypeDef* htim);


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
  TransmitString("\n");

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  long long i = 0;

  // CALCULATE VALUES FOR SINE BUFFER DMA
  Fill_Sine_Buffer();

  // INITIALISE DAC DMA in the previous function

  // ADC DMA INIT in the previous function


  // GET DAC STATE ON STARTUP
  // Expected: BUSY
  char msg1[32];
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

  Sampling_Enable();

  uint32_t startTime;

//  phasor_t ZX_RAW[NFREQUENCIES];
  phasor_t inputs[NFREQUENCIES];
  phasor_t outputs[NFREQUENCIES];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  i++;
	  /*
	  switch(STATE)
	  {
	  case(IDLE):
		if(buttonPress())
		{
			startTime = HAL_GetTick();
			HAL_Delay(200);
			STATE = BUTTON_PRESS;
		}
		else STATE = IDLE;

	    break;

	  case(BUTTON_PRESS):
		// 2 second timeout
		if(HAL_GetTick() - startTime > 2000)
		{
			TransmitStringLn("Starting a Measurement! \n"
					"Please ensure your DUT is attached before beginning this\n");
			STATE = MEASURING;
		}
		else if(buttonPress())
		{
			TransmitStringLn("Beginning Calibration!\n"
					"Please follow the instructions closely...");
			STATE = CALIBRATING;
		}
		else STATE = BUTTON_PRESS;
		break;

	  case(CALIBRATING):
		TransmitStringLn("Beginning Calibration...");
	    TransmitStringLn("Please Connect a Short Circuit (S/C) DUT across the fixtures.");
	    TransmitStringLn("Once you have done so, please press the blue button");

	    while(!buttonPress()) // Poll at 50Hz
	    	HAL_Delay(20);


		STATE = IDLE;
		HAL_Delay(500);
		break;

	  case(MEASURING):
		TransmitStringLn("MEASURING...");
		STATE = IDLE;
	  	HAL_Delay(500);
		break;


	  default:
		  STATE = IDLE;
		  break;
	  }
	  */



//	  Sig_Gen_Enable();
//	  HAL_TIM_Base_Start(&htim6);

	  if(buttonPress())
	  {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		  Sig_Gen_Enable();
		  TransmitStringLn("Starting Measuring!");
//		  TransmitUInt16Buffer(sine_wave_buffer, DAC_LUT_SIZE);

		  Get_All_Raw_Phasors(inputs, outputs, 1000);
		  TransmitStringLn("DONE!");
		  HAL_Delay(1000);



		  for(int i = 0; i < NFREQUENCIES; i++)
		  {
			  TransmitPhasorLn(outputs[i]);
		  }
		  HAL_Delay(100);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  }


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
  TransmitStringLn("Beginning ADC DMA Init...");
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
  hadc1.Init.NbrOfConversion = 3;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */


//  if(HAL_ADC_Start_DMA(&hadc1, vmeas_buffer, ADC_BUFFER_SIZE) != HAL_OK)
//  {
//	  TransmitStringLn("ERROR SETTING UP ADC1 DMA");
//  }
//  else
//  {
//	  TransmitStringLn("ADC1 DMA SUCCESSFULLY INITIALISED");
//  }

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
  TransmitStringLn("Beginning DAC DMA Init...");
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
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
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
	  TransmitStringLn("DAC DMA SUCCESSFULLY INITIALISED");
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

  GPIO_InitStruct.Pin = VMEAS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(VMEAS0_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = VMEAS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(VMEAS1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = VMEAS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(VMEAS2_GPIO_Port, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


bool buttonPress(void)
{
	return !HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
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
	return HAL_UART_Transmit(&huart2, msg, (uint16_t) strlen(msg), HAL_MAX_DELAY);
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
        sprintf(msg, "%u %u ", buffer1[i], buffer2[i]);

        // Transmit the formatted string over USART
        if (TransmitStringRaw(msg) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    sprintf(msg, "%u %u", buffer1[size-1], buffer2[size-1]);

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

	sprintf(msg, "%f\r", num);

	return TransmitString(msg);
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

HAL_StatusTypeDef TransmitPhasor(phasor_t phasor)
{
	char msg[32];
	sprintf(msg, "Phasor: %.3f, %.3f\r", phasor.magnitude, phasor.phaserad);
	return TransmitString(msg);
}

HAL_StatusTypeDef TransmitPhasorLn(phasor_t phasor)
{
	char msg[32];
	sprintf(msg, "Phasor: %.3f, %.3f\r\n", phasor.magnitude, phasor.phaserad);
	return TransmitString(msg);
}

void ReceiveMessage(char msg[], size_t len)
{
	HAL_UART_Receive(&huart2, msg, len, 1);
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
