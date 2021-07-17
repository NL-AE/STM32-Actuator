/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// FOC related
#define SQRT3 		1.73205080757f
#define SQRT3_2		0.86602540378f
#define SQRT1_3		0.57735026919f
#define PI_F 		3.14159274101f
#define PWM_Max_Count htim1.Init.Period

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

// DRV Chip
int DRV_SPI_Transmit_Check(uint16_t,uint16_t);				// custom SPI transmit and check
int DRV_Start(void);										// sends over SPI commands to setup
//void DRV_Error(void);										// when NFAULT error occurs
// ADC
void Read_ADCs(float*, float*, float*, float*, float*);		// reads ADCs and returns in those pointers
// Encoder
int Read_Encoder_SPI_Ang(float*);					// ask for encoder angle over SPI
// FOC stuff
void Set_PWM3(uint16_t, uint16_t, uint16_t);		// set pwm values for channels A,B,C
float _SIN(float);									// sin(theta)
// Interrupts
void FOC_Interrupt(void);							// FOC interrupt
void IF_B_Int(void);		// Phase B interrupt for encoder

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEMP_Pin GPIO_PIN_2
#define TEMP_GPIO_Port GPIOC
#define SO3_Pin GPIO_PIN_0
#define SO3_GPIO_Port GPIOA
#define Y_LED_Pin GPIO_PIN_1
#define Y_LED_GPIO_Port GPIOA
#define G_LED_Pin GPIO_PIN_2
#define G_LED_GPIO_Port GPIOA
#define PVDD_Pin GPIO_PIN_3
#define PVDD_GPIO_Port GPIOA
#define SO2_Pin GPIO_PIN_5
#define SO2_GPIO_Port GPIOC
#define SO1_Pin GPIO_PIN_0
#define SO1_GPIO_Port GPIOB
#define IF_A_Pin GPIO_PIN_10
#define IF_A_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define IF_B_Pin GPIO_PIN_6
#define IF_B_GPIO_Port GPIOC
#define IF_B_EXTI_IRQn EXTI9_5_IRQn
#define Phase_B_Pin GPIO_PIN_8
#define Phase_B_GPIO_Port GPIOA
#define Phase_C_Pin GPIO_PIN_9
#define Phase_C_GPIO_Port GPIOA
#define Phase_A_Pin GPIO_PIN_10
#define Phase_A_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DRV_EN_Pin GPIO_PIN_11
#define DRV_EN_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
