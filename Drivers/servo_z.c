#include "servo_z.h"
#include "tim.h"

/* 最近一次写入的比较值（单位 us），上电默认放在安全位置。 */
static uint16_t zservo_pulse_us = ZSERVO_PULSE_SAFE_US;

/* 限幅窗口随 ZSERVO_CALIB_MODE 切换：
 *   正式 1000~2000 us，标定 800~2200 us。
 * 首次调试不允许直接写极限角度，避免顶坏机械结构或堵转烧舵机。 */
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
#if ZSERVO_CALIB_MODE
  /* 标定模式：上电停在窗口正中 1500 us，KEY2/KEY3 往两侧逐档扫描。
   * 不能停在 TRAVEL(1000 us)：那样往“脉宽减小”方向只剩 200 us 可试。 */
  ZServo_SetPulse(ZSERVO_CALIB_START_US);
#else
  /* 先写比较值再启动 PWM，确保第一个周期就是运输高度。 */
  ZServo_SetSafePosition();
#endif
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

void ZServo_StepPulse(int8_t direction)
{
  uint16_t pulse = zservo_pulse_us;

  if (direction > 0)
  {
    pulse = (pulse <= (uint16_t)(ZSERVO_PULSE_MAX_US - ZSERVO_STEP_US))
                ? (uint16_t)(pulse + ZSERVO_STEP_US)
                : ZSERVO_PULSE_MAX_US;
  }
  else if (direction < 0)
  {
    pulse = (pulse >= (uint16_t)(ZSERVO_PULSE_MIN_US + ZSERVO_STEP_US))
                ? (uint16_t)(pulse - ZSERVO_STEP_US)
                : ZSERVO_PULSE_MIN_US;
  }
  else
  {
    /* 0 不做任何动作，避免误调用把脉宽写成极限值。 */
    return;
  }

  ZServo_SetPulse(pulse);
}

void ZServo_JumpTo(uint16_t pulse_us)
{
  /* 台架一键到位：一次写 CCR1，跳过逐档点动；脉宽仍按当前模式限幅，
   * 所以即使误传超界值也不会越过 ZSERVO_PULSE_MIN/MAX_US。 */
  ZServo_SetPulse(pulse_us);
}

void ZServo_Travel(void)
{
#if ZSERVO_CALIB_MODE
  /* 标定模式：不改脉宽。上电回零（SORT_SYS_INIT / T 回零 / READY）都会
   * 调用 Travel()，若照常写 1000 us，按键找到的位置会被状态机拽走。 */
  return;
#else
  ZServo_SetPulse(ZSERVO_PULSE_TRAVEL_US);
#endif
}

void ZServo_Approach(void)
{
#if ZSERVO_CALIB_MODE
  /* 标定模式：同 Travel()，只保留接口。 */
  return;
#else
  ZServo_SetPulse(ZSERVO_PULSE_APPROACH_US);
#endif
}

void ZServo_Grasp(void)
{
#if ZSERVO_CALIB_MODE
  /* 标定模式：同 Travel()，只保留接口。 */
  return;
#else
  ZServo_SetPulse(ZSERVO_PULSE_GRASP_US);
#endif
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
