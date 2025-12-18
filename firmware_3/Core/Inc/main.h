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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CHRLY3_Pin GPIO_PIN_13
#define CHRLY3_GPIO_Port GPIOC
#define CHRLY4_Pin GPIO_PIN_14
#define CHRLY4_GPIO_Port GPIOC
#define CHRLY5_Pin GPIO_PIN_15
#define CHRLY5_GPIO_Port GPIOC
#define CHRLY6_Pin GPIO_PIN_0
#define CHRLY6_GPIO_Port GPIOD
#define CHRLY7_Pin GPIO_PIN_1
#define CHRLY7_GPIO_Port GPIOD
#define CHRLY8_Pin GPIO_PIN_0
#define CHRLY8_GPIO_Port GPIOA
#define CHRLY9_Pin GPIO_PIN_1
#define CHRLY9_GPIO_Port GPIOA
#define CHRLY10_Pin GPIO_PIN_2
#define CHRLY10_GPIO_Port GPIOA
#define CHRLY11_Pin GPIO_PIN_3
#define CHRLY11_GPIO_Port GPIOA
#define CHRLY12_Pin GPIO_PIN_4
#define CHRLY12_GPIO_Port GPIOA
#define CHRLY13_Pin GPIO_PIN_5
#define CHRLY13_GPIO_Port GPIOA
#define CHRLY14_Pin GPIO_PIN_6
#define CHRLY14_GPIO_Port GPIOA
#define CHRLY15_Pin GPIO_PIN_7
#define CHRLY15_GPIO_Port GPIOA
#define CHRLY16_Pin GPIO_PIN_0
#define CHRLY16_GPIO_Port GPIOB
#define CHRLY17_Pin GPIO_PIN_1
#define CHRLY17_GPIO_Port GPIOB
#define CHRLY18_Pin GPIO_PIN_2
#define CHRLY18_GPIO_Port GPIOB
#define CHRLY19_Pin GPIO_PIN_10
#define CHRLY19_GPIO_Port GPIOB
#define CHRLY20_Pin GPIO_PIN_11
#define CHRLY20_GPIO_Port GPIOB
#define CHRLY1_Pin GPIO_PIN_8
#define CHRLY1_GPIO_Port GPIOB
#define CHRLY2_Pin GPIO_PIN_9
#define CHRLY2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
