/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* 方向/使能/步进和夹爪舵机信号为推挽输出；
 * 急停和夹爪传感器为上拉输入。PD9～PD11 作为新增模块的优先扩展口，
 * 当前固件不配置；限位功能后续单独考虑。 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, X_DIR_Pin|Y_DIR_Pin|T_DIR_Pin|X_EN_Pin
                          |Y_EN_Pin|T_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, X_STEP_Pin|Y_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(T_STEP_GPIO_Port, T_STEP_Pin, GPIO_PIN_RESET);

  /* 夹爪舵机信号 PB8 初始为低，避免上电出现长时间高电平。
   * 实际 50 Hz 脉冲由 servo_gripper.c 在 TIM1 中断中软件生成。 */
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SERVO_PWM_GPIO_Port, SERVO_PWM_Pin, GPIO_PIN_RESET);

  /* LCD_CS 空闲为高；LCD_RST/LCD_DC 初始为高（RST 高表示不复位）；
   * LCD_BLK 初始为低，等初始化完成后再由驱动打开背光。 */
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_DC_Pin|LCD_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : X_DIR_Pin Y_DIR_Pin T_DIR_Pin X_EN_Pin
                           Y_EN_Pin T_EN_Pin */
  GPIO_InitStruct.Pin = X_DIR_Pin|Y_DIR_Pin|T_DIR_Pin|X_EN_Pin
                          |Y_EN_Pin|T_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : X_STEP_Pin Y_STEP_Pin */
  GPIO_InitStruct.Pin = X_STEP_Pin|Y_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : T_HOME_Pin EMERGENCY_STOP_Pin GRIP_SENSOR_Pin */
  GPIO_InitStruct.Pin = T_HOME_Pin|EMERGENCY_STOP_Pin|GRIP_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 (夹爪舵机软件 PWM 信号) */
  GPIO_InitStruct.Pin = SERVO_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SERVO_PWM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : T_STEP_Pin */
  GPIO_InitStruct.Pin = T_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(T_STEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin LCD_RST_Pin LCD_BLK_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_RST_Pin|LCD_BLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* 板载测试按键 KEY0/KEY2/KEY3/KEY4 分别接 PD0/PD2/PD3/PD4。
   * 当前正式固件不使用这些按键，保持默认输入状态；
   * 以后恢复台架测试时再按实际电平配置上拉/下拉，不要照搬 PA0 的高电平有效。 */

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
