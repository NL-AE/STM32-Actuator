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

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC1_Pin GPIO_PIN_0
#define OSC1_GPIO_Port GPIOH
#define OSC2_Pin GPIO_PIN_1
#define OSC2_GPIO_Port GPIOH
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
