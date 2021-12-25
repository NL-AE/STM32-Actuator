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
#include <stdint.h>
#include <math.h>
#include "sin_lookup.h"
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define REV 1	// code revision

// Phase Mappings
#define Phase_A_Ch TIM_CHANNEL_3
#define Phase_B_Ch TIM_CHANNEL_1
#define Phase_C_Ch TIM_CHANNEL_2

// Enable DRV chip?
#define DRV_EN 0

// CAN ID (actuator)
#define CAN_TX_ID 1		// Transmits as this ID
#define CAN_RX_ID 1		// Recieves messages from this ID


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

// Encoder
typedef struct
{
	float SPI_theta;		// IIF from SPI 0-360deg used to zero

	int16_t  IIF_Counter;		// The counter variable that is interrupt driven so dont use it in calculations
	int64_t  IIF_Revolutions;	// Number of full revolutions taken
	uint16_t IIF_Raw;			// Current angle 0-4095
} ENC_Struct;
// ADC
typedef struct
{
	float VDDA;
	uint32_t DMA_Buff[3];						// Array for ADC 3 DMA requests
	float PVDD, V_bat_R_Bot, V_bat_R_Top;		// Temp_V_Offset/V 	and Resistor divider for PVDD	V1: V_o = Vin * R2 / (R1+R2)
	float Temp, Temp_V_Offset, Temp_Slope;		// Board temp 		and thermocouple properties		LM60: V_o = (6.25mV * T/C) + 424mV

	int16_t i_a_Raw, i_b_Raw, PVDD_Raw, Temp_Raw;	// Raw ADC readings
	int16_t i_a_Fil, i_b_Fil, PVDD_Fil, Temp_Fil;	// Filtered ADC readings

	float R_Shunt_Res;					// Shunt resistor resistance /ohms
	int SO_Gain;						// Gain of sense amp
	int16_t SO_A_Offset, SO_B_Offset;	// Raw offset of sense amp
} ADC_Struct;
// FOC
typedef struct
{
	// Proeprties
	int Pole_Pairs;				// number of pole pairs
	float dt;					// delta T of FOC response		/seconds

	// Angles
	float m_theta, m_dtheta;	// mechanical theta and dtheta	/deg		/degs-1
	float e_theta, e_dtheta;	// electrical theta and dtheta	/deg		/degs-1

	// FOC currents
	float i_a, i_b, i_c;		// Phase currents				/amps
	float i_alph, i_beta;		// Alpha/Beta currents			/amps
	float i_d, i_q;				// Direct/Quadrature currents	/amps

	// Current target
	float DC_I;					// 0 to 1 of the duty cycle controlling current

	// FOC sector control
	float alpha;				// Angle from start of sector to current position
	int sector;					// which sector currently in

	// Duty cycles
	float DC_1, DC_2, DC_0;		// Duty cycle of vector start, end and unforced

	uint16_t PWM_Reg_Max;		// PWM register max
	float PWM_A, PWM_C, PWM_B;	// Duty cycle
} FOC_Struct;
// Filter
typedef struct
{
	// Butterworth from: https://www.meme.net.au/butterworth.html
	// At 6.667KHz smapling
	// y(i) = k1*x(i) + k1*x(i-1) + k2*y(i-1)

	// Filters i_a, i_b, PVDD, Temp, IIF count, IIF vel

	float i_k[2];		// Filter coefficients for current filters
	float Misc_k[2];	// Filter coefficients for misc filter

	int16_t i_a_Pre, i_a_Pre_Fil, i_b_Pre, i_b_Pre_Fil;			// Previous values for current
	int16_t PVDD_Pre, PVDD_Pre_Fil, Temp_Pre, Temp_Pre_Fil;		// Previous values for PVDD and temp
} FIL_Struct;
// CAN
typedef struct
{
	uint32_t timeout;		// timeout/ms

	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[6];

	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[6];

	CAN_FilterTypeDef filter;
} CAN_Struct;
// Controller
typedef struct
{
	float Torque;
	float Velocity;


} CON_Struct;

ENC_Struct enc;
ADC_Struct adc;
FOC_Struct foc;
FIL_Struct fil;
CAN_Struct can;
CON_Struct con;

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
	for(i=0; i<len; i++)
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
  HAL_Delay(10);
  printf("Actuator Firmware Version: %i\n",REV);
  HAL_Delay(10);

  /* Start ADCs */
  printf("Start ADC... ");
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)adc.DMA_Buff,3);
  printf("Good\n");
  HAL_Delay(10);

  /* Startup Timers */
  printf("Start Timers... ");
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, Phase_A_Ch);
  HAL_TIM_PWM_Start(&htim1, Phase_B_Ch);
  HAL_TIM_PWM_Start(&htim1, Phase_C_Ch);
  Set_PWM3(0,0,0);							// Set PWM channels to off
  printf("Good\n");
  HAL_Delay(10);

  /* Startup DRV chip */
  if(DRV_EN){
	  printf("Start DRV... ");
	  int DRV_Err = DRV_Start();			// startup and write SPI registers
	  if(DRV_Err){							// if errors occurs,
	   printf("DRV Error: %i\n",DRV_Err);
	   Error_Handler();							// enter hardfault handler
	  }
	  DRV_Zero_SO();						// Zero sense amps
	  printf("Good\n");
	  HAL_Delay(10);
  }else{
	  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 0);	// Set enable of DRV chip low
  }

  /* Check Encoder talks */
  printf("Start ENC... ");
  int Enc_Err = Read_Encoder_SPI_Ang(&enc.SPI_theta);	// read one value from encoders
  if(Enc_Err){								// if errors occurs,
	  printf("ENC Error: %i\n",Enc_Err);
	  Error_Handler();							// enter hardfault handler
  }
  enc.IIF_Counter = (int)(enc.SPI_theta /360.0f * 4095.0f);	// Zero encoder
  printf("Good\n");
  HAL_Delay(10);

  /* CAN setup */
  printf("Start CAN... ");
  // Config RX Filter
  can.filter.FilterActivation		= ENABLE;					// use filter or not
  can.filter.FilterBank				= 0;						// specifies which filter to be initialised
  can.filter.FilterFIFOAssignment	= CAN_RX_FIFO0;				// specifies which FIFO this is for
  can.filter.FilterIdHigh			= CAN_RX_ID<<5;				// MSB for 32 bit / first  16 bit
  can.filter.FilterIdLow			= 0x0000;					// LSB for 32 bit / second 16 bit
  can.filter.FilterMaskIdHigh		= CAN_RX_ID<<5;				// MSB for 32 bit / first  16 bit
  can.filter.FilterMaskIdLow		= 0x0000;					// LSB for 32 bit / second 16 bit
  can.filter.FilterMode				= CAN_FILTERMODE_IDMASK;
  can.filter.FilterScale			= CAN_FILTERSCALE_32BIT;
  can.filter.SlaveStartFilterBank	= 14;						// select filter for slave CAN to use

  HAL_CAN_ConfigFilter(&hcan1, &can.filter);
  HAL_CAN_Start(&hcan1);

  // Config TX
  can.tx_header.DLC 				= 6;			// Data length/bytes
  can.tx_header.IDE					= CAN_ID_STD;	// standard ID
  can.tx_header.RTR					= CAN_RTR_DATA;	// set data type to transmission
  can.tx_header.StdId				= CAN_TX_ID;	// recipient CAN ID
  can.tx_header.ExtId				= CAN_TX_ID;   	// recipient extended CAN ID
  can.tx_header.TransmitGlobalTime	= DISABLE;

  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // Start can interrupt
  printf("Good\n");
  HAL_Delay(10);

  /* Setup ADC structure */
  adc.VDDA = 3.30f;
  adc.V_bat_R_Top = 75.0f;
  adc.V_bat_R_Bot = 5.1f;
  adc.Temp_V_Offset = 0.424f;
  adc.Temp_Slope = 0.00625f;
  adc.R_Shunt_Res = 0.001f;
  adc.SO_Gain = 40.0f;

  /* Setup FOC structure*/
  foc.Pole_Pairs = 21.0f;
  foc.dt = (float)(1.0f/(168.0f*1000000.0f/(htim1.Init.Period+1)/(htim1.Init.RepetitionCounter+1)));
  foc.PWM_Reg_Max = htim1.Init.Period;

  /* Setup Filter structure */
  fil.i_k[0]    = 0.421f;	fil.i_k[1]    = 0.158f;
  fil.Misc_k[0] = 0.421f;	fil.Misc_k[1] = 0.158f;

  printf("FOC Start\n");
  HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Heartbeat */
	  HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, 1);

	  // Read if there is an error
	  if(DRV_EN)
		  if(HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin)==0)
			  DRV_Error();

	  ADC_Filter_Misc(adc.PVDD_Raw,adc.Temp_Raw,&adc.PVDD_Fil,&adc.Temp_Fil);	// Filter raw ADC PVDD and temp
	  ADC_Norm_Misc(adc.PVDD_Fil,adc.Temp_Fil,&adc.PVDD,&adc.Temp);				// Normalise PVDD and temp

	  printf("ENC IIF: %i\n",enc.IIF_Raw);

	  HAL_Delay(50);

	  HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, 0);

	  HAL_Delay(50);
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
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
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
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
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 2;
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
  hcan1.Init.Prescaler = 7;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  htim1.Init.RepetitionCounter = 3;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
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
int   DRV_Start  (void)
{
	HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, 1);	// Set enable of drv chip high
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);

	if(DRV_SPI_Transmit_Check(0b0010101101000100,0x03FF)) return 1;	// write 0x5 register : HS gate 1780ns peak source time, 60mA sink, 50mA source
	if(DRV_SPI_Transmit_Check(0b0011001101000100,0x03FF)) return 2;	// write 0x6 register : LS gate 1780ns peak source time, 60mA sink, 50mA source
	if(DRV_SPI_Transmit_Check(0b0011101010010110,0x03FF)) return 3;	// write 0x7 register : Active freewheeling, 3 channel PWM, 52ns dead time, 1.75us Vds sense, 3.5us Vds deglitch
	if(DRV_SPI_Transmit_Check(0b0100110010100000,0x07FF)) return 3;	// write 0x9 register : Clamp sense output to 3.3V, faults all enabled
	if(DRV_SPI_Transmit_Check(0b0101000010101010,0x07FF)) return 4;	// write 0xA register : Normal operation, 2.5us amp blanking time, 40 gain
	if(DRV_SPI_Transmit_Check(0b0101100100001010,0x031F)) return 5;	// write 0xB register : k=2, 10us Vreg power down down delay, UVLO at Vreg*0.7
	if(DRV_SPI_Transmit_Check(0b0110000000000000,0x00FF)) return 6;	// write 0xC register : Vds threshold=60mV, Vds overcurrent latch shut down

	return 0;
}
void  DRV_Error  (void)
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
void  DRV_Zero_SO(void)
{
	// Turn PWM off
	Set_PWM3(0,0,0);

	// Set to DC calibration mode
	DRV_SPI_Transmit_Check(0b0101011110101010,0x00FF);	// write 0xC register : DC calibration mode, 2.5us amp blanking time, 40 gain

	HAL_Delay(1);

	// Take 5 readings
	int16_t temp_A[5], temp_B[5], temp_PVDD[5], temp_Temp[5];
	int16_t outp_A[5], outp_B[5];

	for(int i=0; i<5; i++)
	{
		ADC_Get_Raw(&temp_A[i],&temp_B[i],&temp_PVDD[i],&temp_Temp[i]);
		HAL_Delay(1);
	}

	// Sort arrays in ascending order
	Array_Sort(temp_A,outp_A,5);
	Array_Sort(temp_B,outp_B,5);

	// Take average of three readings in the middle
	adc.SO_A_Offset = (outp_A[1]+outp_A[2]+outp_A[3]) / 3;
	adc.SO_B_Offset = (outp_B[1]+outp_B[2]+outp_B[3]) / 3;

	// Set to normal operation again
	DRV_SPI_Transmit_Check(0b0101000010101010,0x07FF);	// write 0xA register : Normal operation, 2.5us amp blanking time, 40 gain
}
void  Array_Sort (int16_t input[], int16_t output[], int n)
{
	for(int i=0; i<n; i++)	// For each element in the input aray,
	{
		int count=0;

		for(int k=0; k<n; k++)		// Count how many numbers it is larger than
		{
			if(input[i]>input[k])
			{
				count++;
			}
		}

		while(output[count]==input[i])	// If duplicates, increase index by 1
		{
			count = count+1;
		}

		output[count] = input[i];		// Set the output[count] as input[i]
	}
}
// Read ADCs
void  ADC_Get_Raw    (int16_t*i_a_Raw, int16_t*i_b_Raw, int16_t*PVDD_Raw, int16_t*Temp_Raw)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);

	*i_a_Raw	= HAL_ADC_GetValue(&hadc1);
	*i_b_Raw	= HAL_ADC_GetValue(&hadc2);
	*PVDD_Raw	= adc.DMA_Buff[0];
	*Temp_Raw	= adc.DMA_Buff[1];
}
void  ADC_Filter_Curr(int16_t i_a_Raw, int16_t i_b_Raw, int16_t*i_a_Fil, int16_t*i_b_Fil)
{
	// Filter
	*i_a_Fil = fil.i_k[0]*i_a_Raw + fil.i_k[0]*fil.i_a_Pre + fil.i_k[1]*fil.i_a_Pre_Fil;
	*i_b_Fil = fil.i_k[0]*i_b_Raw + fil.i_k[0]*fil.i_b_Pre + fil.i_k[1]*fil.i_b_Pre_Fil;

	// Now store current values as previous values
	fil.i_a_Pre = i_a_Raw;
	fil.i_b_Pre = i_b_Raw;

	fil.i_a_Pre_Fil = *i_a_Fil;
	fil.i_b_Pre_Fil = *i_b_Fil;
}
void  ADC_Norm_Curr  (int16_t i_a_Fil, int16_t i_b_Fil, float*i_a, float*i_b)
{
	*i_a = (((float)(i_a_Fil-adc.SO_A_Offset))*adc.VDDA/4095.0f)/adc.SO_Gain/adc.R_Shunt_Res;
	*i_b = (((float)(i_b_Fil-adc.SO_B_Offset))*adc.VDDA/4095.0f)/adc.SO_Gain/adc.R_Shunt_Res;
}
void  ADC_Filter_Misc(int16_t PVDD_Raw, int16_t Temp_Raw, int16_t*PVDD_Fil, int16_t*Temp_Fil)
{
	// Filter
	*PVDD_Fil = fil.Misc_k[0]*PVDD_Raw + fil.Misc_k[0]*fil.PVDD_Pre + fil.Misc_k[1]*fil.PVDD_Pre_Fil;
	*Temp_Fil = fil.Misc_k[0]*Temp_Raw + fil.Misc_k[0]*fil.Temp_Pre + fil.Misc_k[1]*fil.Temp_Pre_Fil;

	// Now store current values as previous values
	fil.PVDD_Pre = PVDD_Raw;
	fil.Temp_Pre = Temp_Raw;

	fil.PVDD_Pre_Fil = *PVDD_Fil;
	fil.Temp_Pre_Fil = *Temp_Fil;
}
void  ADC_Norm_Misc  (int16_t PVDD_Fil, int16_t Temp_Fil, float*PVDD, float*Temp)
{
	*PVDD = (float)PVDD_Fil*adc.VDDA/4095.0f / adc.V_bat_R_Bot * (adc.V_bat_R_Bot+adc.V_bat_R_Top);
	*Temp = (((float)Temp_Fil*adc.VDDA/4095.0f)-adc.Temp_V_Offset)/adc.Temp_Slope;
}
// Encoder
int   Read_Encoder_SPI_Ang(float*Angle)
{
	const uint8_t ENC_ASK_POS [2] = {0b10000000,0b00100000};	// Command for asking position
	uint8_t ENC_SPI_Buffer[4];

	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)&ENC_ASK_POS,   2, 1)) return 1;	// Ask for data
	if(HAL_SPI_Receive (&hspi1, (uint8_t*)ENC_SPI_Buffer, 3, 1)) return 2;	// Recieve 2 bytes of data

	int16_t SPI_ANG = (ENC_SPI_Buffer[1] << 8 | ENC_SPI_Buffer[2]);		// make 16 bit
	int16_t ANG_VAL = (0b0011111111111111 & SPI_ANG);					// keep last 14 bits
	ANG_VAL -= (((SPI_ANG)&(1UL<<(14)))>>(14))*(-16384);
	*Angle = 360.0/32768.0 * ANG_VAL;

	return 0;
}
void  IF_B_Int(void)
{
	if(HAL_GPIO_ReadPin(IF_A_GPIO_Port, IF_A_Pin))
		enc.IIF_Counter++;		// If high, increment
	else
		enc.IIF_Counter--;		// If low , decrement

	if(enc.IIF_Counter>=4096)	// If overflow
	{
		enc.IIF_Counter = 0;		// Set to 0
		enc.IIF_Revolutions++;		// Increment revolutions counter
	}

	if(enc.IIF_Counter<0)		// If underflow
	{
		enc.IIF_Counter = 4095;		// Set to 4095
		enc.IIF_Revolutions--;		// Decrement revolutions counter
	}
}
// FOC stuff
void  Set_PWM3(uint16_t ARR_1, uint16_t ARR_2, uint16_t ARR_3)
{
	__HAL_TIM_SET_COMPARE(&htim1,Phase_A_Ch,ARR_1);	// Set PWM channels
	__HAL_TIM_SET_COMPARE(&htim1,Phase_B_Ch,ARR_2);
	__HAL_TIM_SET_COMPARE(&htim1,Phase_C_Ch,ARR_3);
}
float _sin(float theta)
{
	return sin_lookup[(int)floor(theta)];
}
float _cos(float theta)
{
	return sin_lookup[(int)floor(fmodf(theta+270.0f,360.0f))];
}
// Timer Interrupts
void  FOC_Interrupt(void)
{
	/* LED on */
	HAL_GPIO_WritePin(Y_LED_GPIO_Port, Y_LED_Pin, 1);

	/* FOC sample */
	ADC_Get_Raw(&adc.i_a_Raw,&adc.i_b_Raw, &adc.PVDD_Raw, &adc.Temp_Raw);	// Read raw ADC
	enc.IIF_Raw = enc.IIF_Counter;											// Get encoder angle

	/* Filter and normalise readings */
	ADC_Filter_Curr(adc.i_a_Raw,adc.i_b_Raw,&adc.i_a_Fil,&adc.i_b_Fil);		// Filter raw ADC currents
	ADC_Norm_Curr  (adc.i_a_Fil,adc.i_b_Fil,&foc.i_a,&foc.i_b);				// Normalise currents
	foc.m_theta = (float)enc.IIF_Raw / 4095.0f * 360.0f;					// Normalise angle to 0-360deg

	/* FOC maths */
	// Get electrical angles correct
	foc.e_theta = fmodf(foc.m_theta*foc.Pole_Pairs,360.0f);	// get electrical angle and constrain in 360 deg

	// Clarke -> alpha/beta
	foc.i_alph = foc.i_a;
	foc.i_beta = SQRT1_3 * (2.0f*foc.i_b - foc.i_a);

	// Park -> direct/quadrature
	float sin_Ang = _sin(foc.e_theta);
	float cos_Ang = _cos(foc.e_theta);
	foc.i_d = cos_Ang*foc.i_alph + sin_Ang*foc.i_beta;
	foc.i_q = cos_Ang*foc.i_beta - sin_Ang*foc.i_alph;

	/* Regulate currents */
	foc.DC_I = 0.1f;				// Current duty cycle

	/* Set PWM Compare values */
	foc.alpha = fmodf(foc.e_theta,60.0f);	// calculate alpha

	foc.DC_1 = foc.DC_I*_sin(60.0f - foc.alpha);
	foc.DC_2 = foc.DC_I*_sin(foc.alpha);
	foc.DC_0 = 1.0f - foc.DC_1 - foc.DC_2;

	foc.sector = (int)floor(foc.e_theta/60.0f);

	switch (foc.sector)
	{
		case 0:
			foc.PWM_A = 0.5*foc.DC_0;
			foc.PWM_B = 0.5*foc.DC_0 + foc.DC_1;
			foc.PWM_C = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
			break;
		case 1:
			foc.PWM_A = 0.5*foc.DC_0 + foc.DC_2;
			foc.PWM_B = 0.5*foc.DC_0;
			foc.PWM_C = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
			break;
		case 2:
			foc.PWM_A = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
			foc.PWM_B = 0.5*foc.DC_0;
			foc.PWM_C = 0.5*foc.DC_0 + foc.DC_1;
			break;
		case 3:
			foc.PWM_A = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
			foc.PWM_B = 0.5*foc.DC_0 + foc.DC_2;
			foc.PWM_C = 0.5*foc.DC_0;
			break;
		case 4:
			foc.PWM_A = 0.5*foc.DC_0 + foc.DC_1;
			foc.PWM_B = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
			foc.PWM_C = 0.5*foc.DC_0;
			break;
		case 5:
			foc.PWM_A = 0.5*foc.DC_0;
			foc.PWM_B = 0.5*foc.DC_0 + foc.DC_1 + foc.DC_2;
			foc.PWM_C = 0.5*foc.DC_0 + foc.DC_2;
			break;
	}

	Set_PWM3(foc.PWM_Reg_Max*(1.0f-foc.PWM_A),foc.PWM_Reg_Max*(1.0f-foc.PWM_B),foc.PWM_Reg_Max*(1.0f-foc.PWM_C));

	/* LED off */
	HAL_GPIO_WritePin(Y_LED_GPIO_Port, Y_LED_Pin, 0);
}
void  CAN_Interrupt(void)
{
	// Get CAN message
		// 4b  Unused
		// 12b Position
		// 16b Velocity
		// 16b Torque
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can.rx_header, can.rx_data);

	// Create CAN response message
		// 8b  temp
		// 12b position
		// 14b velocity
		// 14b torque
	uint32_t TxMailbox;
	uint16_t temp_vel = 0b0011111111111111;
	uint16_t temp_tor = 0b0011111111111111;

	can.tx_data[0] = (uint8_t) (((float)adc.Temp_Fil*adc.VDDA/4095.0f)-adc.Temp_V_Offset)/adc.Temp_Slope;
	can.tx_data[1] = (uint8_t) (enc.IIF_Raw>>4);
	can.tx_data[2] = (uint8_t) ((enc.IIF_Raw<<4) | ((temp_vel>>10) & 0b00001111));
	can.tx_data[3] = (uint8_t) (temp_vel>>2);
	can.tx_data[4] = (uint8_t) ((temp_vel<<6) | ((temp_tor>>8) & 0b00001111));
	can.tx_data[5] = (uint8_t) (temp_tor);

	// Send CAN message
	HAL_CAN_AddTxMessage(&hcan1, &can.tx_header, can.tx_data, &TxMailbox);

	// if special commands, do function else unpack rx message
	if((can.rx_data[0]==0xFF) & (can.rx_data[1]==0xFF) & (can.rx_data[2]==0xFF) & (can.rx_data[3]==0xFF) & (can.rx_data[4]==0xFF))
	{
		switch (can.rx_data[5])
		{
		case 0:
			break;
		case 1:
			break;
		}
	}
	else
	{
		// unpack and update target values
	}

	can.timeout = 0;	// reset timeout timer
}

// Misc
void LED_Blink(int N, uint32_t delay, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN)
{
	for (int i = 0; i < N; ++i)
	{
		HAL_GPIO_WritePin(GPIOx,GPIO_PIN,1);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(GPIOx,GPIO_PIN,0);
		HAL_Delay(delay);
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

  Set_PWM3(0, 0, 0);	// turn off PWM

  __disable_irq();

  while (1)
  {
	  LED_Blink(3,200,Y_LED_GPIO_Port,Y_LED_Pin);	HAL_Delay(200);	// S
	  LED_Blink(3,400,Y_LED_GPIO_Port,Y_LED_Pin);					// O
	  LED_Blink(3,200,Y_LED_GPIO_Port,Y_LED_Pin);	HAL_Delay(800);	// S
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
