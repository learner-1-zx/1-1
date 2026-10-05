#include "servo_gripper.h"
#include "tim.h"

/* 这些值是 TIM1 比较寄存器计数值，不是直接的微秒数。 */
#define SERVO_OPEN_PULSE   100U
#define SERVO_CLOSE_PULSE  200U

static uint8_t gripper_closed;

static void ServoGripper_SetPulse(uint16_t pulse)
{
  /* TIM1 已配置为 PWM，修改比较值即可改变舵机角度。 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

void ServoGripper_Init(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  ServoGripper_Open();
}

void ServoGripper_Open(void)
{
  ServoGripper_SetPulse(SERVO_OPEN_PULSE);
  gripper_closed = 0;
}

void ServoGripper_Close(void)
{
  ServoGripper_SetPulse(SERVO_CLOSE_PULSE);
  gripper_closed = 1;
}

uint8_t ServoGripper_IsClosed(void)
{
  return gripper_closed;
}
