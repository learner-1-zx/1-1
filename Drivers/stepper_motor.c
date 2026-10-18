#include "stepper_motor.h"

typedef struct
{
  /* 每个轴的 STEP/DIR/EN 引脚及软件位置。 */
  GPIO_TypeDef *step_port;
  uint16_t step_pin;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_TypeDef *en_port;
  uint16_t en_pin;
  int32_t position;
  int32_t target;
  uint8_t busy;
  uint16_t step_interval;
  uint16_t tick_count;
  uint8_t pulse_high;
  uint8_t step_active_low;
} StepperMotor_t;

static StepperMotor_t motors[STEPPER_COUNT] =
{
  {X_STEP_GPIO_Port, X_STEP_Pin, X_DIR_GPIO_Port, X_DIR_Pin,
   X_EN_GPIO_Port, X_EN_Pin, 0, 0, 0, 1, 0, 0, 0},
  {Y_STEP_GPIO_Port, Y_STEP_Pin, Y_DIR_GPIO_Port, Y_DIR_Pin,
   Y_EN_GPIO_Port, Y_EN_Pin, 0, 0, 0, 1, 0, 0, 0},
  {T_STEP_GPIO_Port, T_STEP_Pin, T_DIR_GPIO_Port, T_DIR_Pin,
   T_EN_GPIO_Port, T_EN_Pin, 0, 0, 0, 1, 0, 0, 1}
};

static void StepperMotor_SetEnable(StepperMotor_t *motor, uint8_t enable)
{
  /* A4988 通常为低电平使能；TB6600 的实际有效电平需在接线后确认。 */
  HAL_GPIO_WritePin(motor->en_port, motor->en_pin,
                    enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void StepperMotor_Init(void)
{
  uint8_t i;

  for (i = 0; i < STEPPER_COUNT; ++i)
  {
    motors[i].position = 0;
    motors[i].target = 0;
    motors[i].busy = 0;
    motors[i].step_interval = 1;
    motors[i].tick_count = 0;
    motors[i].pulse_high = 0;
    HAL_GPIO_WritePin(motors[i].step_port, motors[i].step_pin,
                      motors[i].step_active_low
                      ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* 上电默认关闭驱动，避免初始化期间电机误动作。 */
    StepperMotor_SetEnable(&motors[i], 0);
  }
}

HAL_StatusTypeDef StepperMotor_MoveTo(StepperAxis_t axis, int32_t position)
{
  if (axis >= STEPPER_COUNT)
  {
    return HAL_ERROR;
  }

  /* 目标是绝对步数，不是相对位移；坐标原点由回零流程定义。 */
  motors[axis].target = position;
  motors[axis].busy = (motors[axis].position != position) ? 1U : 0U;
  motors[axis].pulse_high = 0;
  if (motors[axis].busy)
  {
    HAL_GPIO_WritePin(motors[axis].dir_port, motors[axis].dir_pin,
                      (position > motors[axis].position)
                      ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  StepperMotor_SetEnable(&motors[axis], motors[axis].busy);
  return HAL_OK;
}

void StepperMotor_SetStepInterval(StepperAxis_t axis, uint16_t interval)
{
  if (axis >= STEPPER_COUNT)
  {
    return;
  }

  motors[axis].step_interval = (interval == 0U) ? 1U : interval;
  motors[axis].tick_count = 0;
}

void StepperMotor_Tick(void)
{
  uint8_t i;
  GPIO_PinState direction;

  for (i = 0; i < STEPPER_COUNT; ++i)
  {
    if (!motors[i].busy)
    {
      continue;
    }

    /* 保持有效脉冲一个完整 TIM6 周期，满足 TB6600 最小脉宽。 */
    if (motors[i].pulse_high)
    {
      HAL_GPIO_WritePin(motors[i].step_port, motors[i].step_pin,
                        motors[i].step_active_low
                        ? GPIO_PIN_SET : GPIO_PIN_RESET);
      motors[i].pulse_high = 0;
      if (motors[i].position == motors[i].target)
      {
        motors[i].busy = 0;
        StepperMotor_SetEnable(&motors[i], 0);
      }
      continue;
    }

    if (motors[i].tick_count + 1U < motors[i].step_interval)
    {
      ++motors[i].tick_count;
      continue;
    }
    motors[i].tick_count = 0;

    /* 当前实现每个节拍每轴最多走一步，速度由 TIM6 周期决定。 */
    direction = (motors[i].target > motors[i].position)
                  ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(motors[i].step_port, motors[i].step_pin,
                      motors[i].step_active_low
                      ? GPIO_PIN_RESET : GPIO_PIN_SET);
    motors[i].pulse_high = 1;

    motors[i].position += (direction == GPIO_PIN_SET) ? 1 : -1;
  }
}

void StepperMotor_StopAll(void)
{
  uint8_t i;

  for (i = 0; i < STEPPER_COUNT; ++i)
  {
    motors[i].target = motors[i].position;
    motors[i].busy = 0;
    motors[i].pulse_high = 0;
    StepperMotor_SetEnable(&motors[i], 0);
    HAL_GPIO_WritePin(motors[i].step_port, motors[i].step_pin,
                      motors[i].step_active_low
                      ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

uint8_t StepperMotor_IsBusy(StepperAxis_t axis)
{
  return (axis < STEPPER_COUNT) ? motors[axis].busy : 0;
}

int32_t StepperMotor_GetPosition(StepperAxis_t axis)
{
  return (axis < STEPPER_COUNT) ? motors[axis].position : 0;
}
