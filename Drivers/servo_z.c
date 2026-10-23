#include "servo_z.h"
#include "tim.h"

/* 最近一次写入的比较值（单位 us），上电默认放在安全位置。 */
static uint16_t zservo_pulse_us = ZSERVO_PULSE_SAFE_US;

/* 首次调试只允许在 1000～2000 us 之间逐步试，
 * 不允许直接写极限角度，避免顶坏机械结构或堵转烧舵机。 */
static uint16_t ZServo_ClampPulse(uint16_t pulse)
{
  if (pulse < ZSERVO_PULSE_MIN_US)
  {
    return ZSERVO_PULSE_MIN_US;
  }
  if (pulse > ZSERVO_PULSE_MAX_US)
  {
    return ZSERVO_PULSE_MAX_US;
  }
  return pulse;
}

void ZServo_Init(void)
{
  /* 先写比较值再启动 PWM，确保第一个周期就是运输高度。 */
  ZServo_SetSafePosition();
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void ZServo_SetPulse(uint16_t pulse)
{
  zservo_pulse_us = ZServo_ClampPulse(pulse);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, zservo_pulse_us);
}

void ZServo_Travel(void)
{
  ZServo_SetPulse(ZSERVO_PULSE_TRAVEL_US);
}

void ZServo_Approach(void)
{
  ZServo_SetPulse(ZSERVO_PULSE_APPROACH_US);
}

void ZServo_Grasp(void)
{
  ZServo_SetPulse(ZSERVO_PULSE_GRASP_US);
}

void ZServo_Up(void)
{
  ZServo_Travel();
}

void ZServo_Down(void)
{
  ZServo_Grasp();
}

void ZServo_SetSafePosition(void)
{
  ZServo_Travel();
}

uint16_t ZServo_GetPulse(void)
{
  return zservo_pulse_us;
}
