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
/* 步进电机控制信号：每个轴由 DIR、EN 和 STEP 三根线组成。 */
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
#define T_HOME_Pin GPIO_PIN_12
#define T_HOME_GPIO_Port GPIOD
#define EMERGENCY_STOP_Pin GPIO_PIN_13
#define EMERGENCY_STOP_GPIO_Port GPIOD
#define GRIP_SENSOR_Pin GPIO_PIN_14
#define GRIP_SENSOR_GPIO_Port GPIOD
#define T_STEP_Pin GPIO_PIN_6
#define T_STEP_GPIO_Port GPIOB
#define Z_SERVO_PWM_Pin GPIO_PIN_1
#define Z_SERVO_PWM_GPIO_Port GPIOA
#define SERVO_PWM_Pin GPIO_PIN_8
#define SERVO_PWM_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_MISO_Pin GPIO_PIN_14
#define LCD_MISO_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_15
#define LCD_MOSI_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_7
#define LCD_DC_GPIO_Port GPIOE
#define LCD_RST_Pin GPIO_PIN_8
#define LCD_RST_GPIO_Port GPIOE
#define LCD_BLK_Pin GPIO_PIN_9
#define LCD_BLK_GPIO_Port GPIOE

/* 舵机 PWM：
 * Z_SERVO_PWM 为 Z 轴（PA1 / TIM2_CH2，硬件 PWM）；
 * SERVO_PWM 为夹爪（PB8，普通 GPIO）。
 * PA1 的板载 LED 已拆除，PA1 可正常作 TIM2_CH2 复用输出。
 * PB8 只作普通 GPIO，夹爪复用 TIM1_CH2 的内部比较中断
 * （不配置 PA9 引脚）产生 50 Hz 软件 PWM：
 * TIM1 更新时先写入 CCR2，再拉高 PB8；CC2 比较匹配时 PB8 拉低。
 * PA9 虽然是 TIM1_CH2 的硬件引脚，但 PCB 未引出，禁止再使用。
 * PD8 已释放为扩展口，PD9～PD11 仍为新增模块的优先扩展口。
 * 限位功能后续单独考虑。 */
/* 显示屏：SPI2 + ST7789V2，SPI 引脚在 spi.c 中配置为复用功能，
 * CS/DC/RST/BLK 为普通 GPIO，由 lcd_st7789.c 控制。 */
/* 传感器输入均在 gpio.c 中配置为上拉，低电平表示触发。 */
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
