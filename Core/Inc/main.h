/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define X_DIR_Pin GPIO_PIN_0
#define X_DIR_GPIO_Port GPIOC
#define Y_DIR_Pin GPIO_PIN_1
#define Y_DIR_GPIO_Port GPIOC
#define T_DIR_Pin GPIO_PIN_2
#define T_DIR_GPIO_Port GPIOC
#define X_EN_Pin GPIO_PIN_3
#define X_EN_GPIO_Port GPIOC
#define BAT_ADC_Pin GPIO_PIN_4
#define BAT_ADC_GPIO_Port GPIOA
#define X_STEP_Pin GPIO_PIN_6
#define X_STEP_GPIO_Port GPIOA
#define Y_STEP_Pin GPIO_PIN_7
#define Y_STEP_GPIO_Port GPIOA
#define Y_EN_Pin GPIO_PIN_4
#define Y_EN_GPIO_Port GPIOC
#define T_EN_Pin GPIO_PIN_5
#define T_EN_GPIO_Port GPIOC
#define X_MIN_Pin GPIO_PIN_8
#define X_MIN_GPIO_Port GPIOD
#define X_MAX_Pin GPIO_PIN_9
#define X_MAX_GPIO_Port GPIOD
#define Y_MIN_Pin GPIO_PIN_10
#define Y_MIN_GPIO_Port GPIOD
#define Y_MAX_Pin GPIO_PIN_11
#define Y_MAX_GPIO_Port GPIOD
#define T_HOME_Pin GPIO_PIN_12
#define T_HOME_GPIO_Port GPIOD
#define EMERGENCY_STOP_Pin GPIO_PIN_13
#define EMERGENCY_STOP_GPIO_Port GPIOD
#define GRIP_SENSOR_Pin GPIO_PIN_14
#define GRIP_SENSOR_GPIO_Port GPIOD
#define T_STEP_Pin GPIO_PIN_6
#define T_STEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
