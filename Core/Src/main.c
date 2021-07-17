/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Phase Mappings
#define Phase_A_Ch TIM_CHANNEL_3
#define Phase_B_Ch TIM_CHANNEL_1
#define Phase_C_Ch TIM_CHANNEL_2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

// ADC
uint32_t ADC_3_Reading[3];						// Array for ADC 3 DMA requests
float	 Temp_Board_C, V_Bat, Phase_Cur_ABC[3];	// Board temp, V battery, phase currents (in order A, B, C)
// Encoder
float ENC_Ang = 0;			// Encoder angle
float ENC_Vel = 0;			// Encoder velocity
int16_t ENC_IIF_Count = 0;	// Encoder IIF count

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// For SWD debug port 0 printf()
int _write(int file, char *ptr, int len)
{
	int i=0;
	for(i=0; i<len;i++)
		ITM_SendChar((*ptr++));
	return len;
}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  printf(" Actuator Firmware Version: 1.0\n");
  HAL_Delay(10);

  /* Start ADCs */
  printf("Start ADC... ");
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start_DMA(&hadc3, ADC_3_Reading, 3);
  printf("Good\n");
  HAL_Delay(10);

  /* Startup PWM */
  printf("Start PWM... ");
  HAL_TIM_Base_Start_IT(&htim1);			// Start timer 1
  HAL_TIM_PWM_Start(&htim1, Phase_A_Ch);
  HAL_TIM_PWM_Start(&htim1, Phase_B_Ch);
  HAL_TIM_PWM_Start(&htim1, Phase_C_Ch);
  Set_PWM3(0,0,0);							// Set PWM channels to off
  printf("Good\n");
  HAL_Delay(10);

  /* Startup DRV chip */
  printf("Start DRV... ");
  int DRV_Err = DRV_Start();		// startup and write SPI registers
  if(DRV_Err){						// if errors occurs,
	  printf("Error: %i\n",DRV_Err);	// printf
	  //while(1);
  }
  printf("Good\n");
  HAL_Delay(10);

  /* Check Encoder talks */
  printf("Start ENC... ");
  int Enc_Err = Read_Encoder_SPI_Ang(&ENC_Ang);		// read one value from encoders
  if(Enc_Err){										// if errors occurs,
	  printf("Error: %i\n",Enc_Err);					// printf
	  //while(1);
  }
  ENC_IIF_Count = (int)(ENC_Ang /360.0 * 4095.0);	// Zero encoder
  printf("Good\n");
  HAL_Delay(10);

  printf("while(1) start\n");
  HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin)==0)
		  DRV_Error();

//	  HAL_GPIO_TogglePin(Y_LED_GPIO_Port, Y_LED_Pin);
	  HAL_GPIO_TogglePin(G_LED_GPIO_Port, G_LED_Pin);
	  HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 14;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4200-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 5;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Y_LED_Pin|G_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Y_LED_Pin G_LED_Pin */
  GPIO_InitStruct.Pin = Y_LED_Pin|G_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IF_A_Pin */
  GPIO_InitStruct.Pin = IF_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IF_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IF_B_Pin */
  GPIO_InitStruct.Pin = IF_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IF_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_EN_Pin */
  GPIO_InitStruct.Pin = DRV_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRV_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_FAULT_Pin */
  GPIO_InitStruct.Pin = DRV_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DRV_FAULT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

// DRV
int   DRV_SPI_Transmit_Check(uint16_t TX_Data, uint16_t RSVD_Mask)
{
	uint8_t SPI_Data[2];	// to transmit
	uint8_t SPI_Buff[2];	// recieve buffer

	// Transmit
	SPI_Data[0] = (TX_Data>>8)&0b01111111;	// first split data up into 8 bits and make it a write command
	SPI_Data[1] = TX_Data;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi2,(uint8_t*)&SPI_Data,2,1);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
	HAL_Delay(1);

	// Recieve
	SPI_Data[0] = (TX_Data>>8)|0b10000000;	// first split data up into 8 bits and make it a read command
	SPI_Data[1] = TX_Data;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t*)&SPI_Data,SPI_Buff,2,1);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
	HAL_Delay(1);

	//printf("%i%i%i  %i%i%i%i %i%i%i%i\n",(int)(SPI_Buff[0]>>2&1UL) ,(int)(SPI_Buff[0]>>1&1UL) ,(int)(SPI_Buff[0]>>0&1UL) ,(int)(SPI_Buff[1]>>7&1UL) ,(int)(SPI_Buff[1]>>6&1UL) ,(int)(SPI_Buff[1]>>5&1UL) ,(int)(SPI_Buff[1]>>4&1UL) ,(int)(SPI_Buff[1]>>3&1UL) ,(int)(SPI_Buff[1]>>2&1UL) ,(int)(SPI_Buff[1]>>1&1UL) ,(int)(SPI_Buff[1]>>0&1UL) );

	if((((SPI_Data[0]^SPI_Buff[0])&((int)(RSVD_Mask>>8)))==0) && (((SPI_Data[1]^SPI_Buff[1])&((int)RSVD_Mask))==0))	// XOR compare written to read data
		return 0;	// if they are the same, return 0
		else
		return 1;	// if they are not same, return 1
}
int   DRV_Start(void)
{
	HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 1);	// Set enable of drv chip high
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);

	//printf("\n");

	//printf("0x5 Register:  ");
	if(DRV_SPI_Transmit_Check(0b0010101101000100,0x03FF)) return 1;	// write 0x5 register : HS gate 1780ns peak source time, 60mA sink, 50mA source
	//printf("0x6 Register:  ");
	if(DRV_SPI_Transmit_Check(0b0011001101000100,0x03FF)) return 2;	// write 0x6 register : LS gate 1780ns peak source time, 60mA sink, 50mA source
	//printf("0x7 Register:  ");
	if(DRV_SPI_Transmit_Check(0b0011101010010110,0x03FF)) return 3;	// write 0x7 register : Active freewheeling, 3 channel PWM, 52ns dead time, 1.75us Vds sense, 3.5us Vds deglitch
	//printf("0x9 Register:  ");
	if(DRV_SPI_Transmit_Check(0b0100110010100000,0x07FF)) return 3;	// write 0x9 register : Clamp sense output to 3.3V, faults all enabled
	//printf("0xA Register:  ");
	if(DRV_SPI_Transmit_Check(0b0101000010101010,0x07FF)) return 4;	// write 0xA register : Normal operation, 2.5us amp blanking time, 40 gain
	//printf("0xB Register:  ");
	if(DRV_SPI_Transmit_Check(0b0101100100001010,0x031F)) return 5;	// write 0xB register : k=2, 10us Vreg power down down delay, UVLO at Vreg*0.7
	//printf("0xC Register:  ");
	if(DRV_SPI_Transmit_Check(0b0110000000000000,0x00FF)) return 6;	// write 0xC register : Vds threshold=60mV, Vds overcurrent latch shut down

  //if(DRV_SPI_Transmit_Check(0b0101011110101010,0x00FF)) return 7;	// write 0xC register : DC calibration mode, 2.5us amp blanking time, 40 gain

	return 0;
}
void  DRV_Error(void)
{
	printf("DRV Error\n");

	// Read errors
	uint8_t SPI_Data[2];
	uint8_t SPI_Buff[2];

	for(int i=1; i<=4; i++)
	{
		SPI_Data[0] = 0b10000000 | (i<<11);	// Create read command

		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
		HAL_SPI_TransmitReceive(&hspi2,(uint8_t*)&SPI_Data,SPI_Buff,2,1);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
		//HAL_Delay(1);

		printf("0x%x Register:  %i%i%i  %i%i%i%i %i%i%i%i\n",i,(int)(SPI_Buff[0]>>2&1UL) ,(int)(SPI_Buff[0]>>1&1UL) ,(int)(SPI_Buff[0]>>0&1UL) ,(int)(SPI_Buff[1]>>7&1UL) ,(int)(SPI_Buff[1]>>6&1UL) ,(int)(SPI_Buff[1]>>5&1UL) ,(int)(SPI_Buff[1]>>4&1UL) ,(int)(SPI_Buff[1]>>3&1UL) ,(int)(SPI_Buff[1]>>2&1UL) ,(int)(SPI_Buff[1]>>1&1UL) ,(int)(SPI_Buff[1]>>0&1UL) );
	}
}
// Read ADCs
void  Read_ADCs(float*Cur_Phase_A, float*Cur_Phase_B, float*Cur_Phase_C, float*V_Bat, float*Temp_Board_C)
{
	// LM60: V_o = (6.25mV * T/C) + 424mV
	#define Temp_V_Offset 	0.424
	#define Temp_Slope 		0.00625

	// V1: V_o = Vin * R2 / (R1+R2)
	#define V_bat_R_Top 	75.0
	#define V_bat_R_Bot 	5.1

	// 1V on the amp output = 25A
	#define V_to_Amps_Const	25

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);

	*Cur_Phase_A 	= ((float)HAL_ADC_GetValue(&hadc1))*3.3/4095.0*V_to_Amps_Const;
	*Cur_Phase_B 	= (float)HAL_ADC_GetValue(&hadc2)*3.3/4095.0*V_to_Amps_Const;
	*Cur_Phase_C 	= (float)ADC_3_Reading[0]*3.3/4095.0*V_to_Amps_Const;
	*V_Bat			= (float)ADC_3_Reading[1]*3.3/4095.0 / V_bat_R_Bot * (V_bat_R_Bot+V_bat_R_Top);
    *Temp_Board_C	= ((((float)ADC_3_Reading[2])*3.3/4095.0)-Temp_V_Offset)/Temp_Slope;
}
// Encoder
int   Read_Encoder_SPI_Ang(float*Angle)
{
	const uint8_t ENC_ASK_POS [2] = {0b10000000,0b00100000};	// Command for asking position
	uint8_t ENC_SPI_Buffer[4];

	//HAL_SPI_Transmit(&hspi1, (uint8_t*)&ENC_ASK_POS,   2, 1);
	//HAL_SPI_Receive (&hspi1, (uint8_t*)ENC_SPI_Buffer, 3, 1);

	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)&ENC_ASK_POS,   2, 1)) return 1;	// Ask for data
	if(HAL_SPI_Receive (&hspi1, (uint8_t*)ENC_SPI_Buffer, 3, 1)) return 2;	// Recieve 2 bytes of data

	int16_t SPI_ANG = (ENC_SPI_Buffer[1] << 8 | ENC_SPI_Buffer[2]);		// make 16 bit
	int16_t ANG_VAL = (0b0011111111111111 & SPI_ANG);					// keep last 14 bits
	ANG_VAL -= (((SPI_ANG)&(1UL<<(14)))>>(14))*(-16384);
	*Angle = 360.0/32768.0 * ANG_VAL;

	return 0;
}
// FOC
void  Set_PWM3(uint16_t ARR_1, uint16_t ARR_2, uint16_t ARR_3)
{
	__HAL_TIM_SET_COMPARE(&htim1,Phase_A_Ch,ARR_1);	// Set PWM channels
	__HAL_TIM_SET_COMPARE(&htim1,Phase_B_Ch,ARR_2);
	__HAL_TIM_SET_COMPARE(&htim1,Phase_C_Ch,ARR_3);
}
float _SIN(float theta)
{
	return 0;
}
// Interrupts
void  FOC_Interrupt(void)
{
	/* LED on */
	HAL_GPIO_WritePin(Y_LED_GPIO_Port, Y_LED_Pin, 1);

	ENC_Ang = (float)(ENC_IIF_Count/4095.0*360.0);

	/* Read ADCs */
	Read_ADCs(&Phase_Cur_ABC[0],&Phase_Cur_ABC[1],&Phase_Cur_ABC[2], &V_Bat, &Temp_Board_C);

	/* FOC Maths */

	/* Set PWM Compare values */

	Set_PWM3(PWM_Max_Count*0.1,PWM_Max_Count*0.2,PWM_Max_Count*0.3);

	/* LED off */
	HAL_GPIO_WritePin(Y_LED_GPIO_Port, Y_LED_Pin, 0);
}
void  IF_B_Int(void)
{
	if(HAL_GPIO_ReadPin(IF_A_GPIO_Port, IF_A_Pin))
		if(ENC_IIF_Count>=4095)
			ENC_IIF_Count = 0;
		else
			ENC_IIF_Count++;
	else
		if(ENC_IIF_Count<=0)
			ENC_IIF_Count = 4095;
		else
			ENC_IIF_Count--;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
