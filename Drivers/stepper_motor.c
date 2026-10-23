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

  /* 以下字段会被主循环和 TIM6 中断同时访问，必须用 volatile。 */
  volatile int32_t position;
  volatile int32_t target;
  volatile uint8_t busy;
  volatile uint16_t step_interval;
  volatile uint16_t tick_count;
  volatile uint8_t pulse_high;
  volatile int32_t total_steps;
  volatile uint16_t ramp_steps_current;
  volatile uint16_t interval_step;

  /* 配置字段，初始化或任务开始前设置。 */
  uint8_t step_active_low;
  uint8_t dir_invert;
  uint8_t en_active_low;
  uint8_t position_valid;
  uint16_t start_interval;
  uint16_t cruise_interval;
  uint16_t ramp_steps;
  uint32_t accel_steps_s2;
  int32_t min_step;
  int32_t max_step;
} StepperMotor_t;

/* C99 指定初始化，避免结构体字段增加后出现位置错位。
 * X/Y 的 STEP 为高有效，T 轴驱动器按低有效预留；实际接线后以实测为准。 */
static StepperMotor_t motors[STEPPER_COUNT] =
{
  {
    .step_port = X_STEP_GPIO_Port, .step_pin = X_STEP_Pin,
    .dir_port = X_DIR_GPIO_Port, .dir_pin = X_DIR_Pin,
    .en_port = X_EN_GPIO_Port, .en_pin = X_EN_Pin,
    .step_active_low = 0U, .dir_invert = 0U, .en_active_low = 1U
  },
  {
    .step_port = Y_STEP_GPIO_Port, .step_pin = Y_STEP_Pin,
    .dir_port = Y_DIR_GPIO_Port, .dir_pin = Y_DIR_Pin,
    .en_port = Y_EN_GPIO_Port, .en_pin = Y_EN_Pin,
    .step_active_low = 0U, .dir_invert = 0U, .en_active_low = 1U
  },
  {
    .step_port = T_STEP_GPIO_Port, .step_pin = T_STEP_Pin,
    .dir_port = T_DIR_GPIO_Port, .dir_pin = T_DIR_Pin,
    .en_port = T_EN_GPIO_Port, .en_pin = T_EN_Pin,
    .step_active_low = 1U, .dir_invert = 0U, .en_active_low = 1U
  }
};

static uint32_t StepperMotor_EnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void StepperMotor_ExitCritical(uint32_t primask)
{
  __set_PRIMASK(primask);
}

static void StepperMotor_WriteStepPin(StepperMotor_t *motor, uint8_t active)
{
  GPIO_PinState state = active ? GPIO_PIN_SET : GPIO_PIN_RESET;

  if (motor->step_active_low != 0U)
  {
    state = (state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }
  HAL_GPIO_WritePin(motor->step_port, motor->step_pin, state);
}

static void StepperMotor_WriteDir(StepperMotor_t *motor, uint8_t forward)
{
  if (motor->dir_invert != 0U)
  {
    forward = (forward != 0U) ? 0U : 1U;
  }
  HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                    (forward != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void StepperMotor_SetEnablePin(StepperMotor_t *motor, uint8_t enable)
{
  GPIO_PinState state = (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  /* A4988/TB6600 常见为低电平使能，有效电平集中在这里处理。 */
  if (motor->en_active_low != 0U)
  {
    state = (state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }
  HAL_GPIO_WritePin(motor->en_port, motor->en_pin, state);
}

/* 根据巡航速度和加速度估算加减速段步数。
 * 速度用“步/秒”，加速度用“步/秒^2”，计算只在参数变化时执行一次。 */
static void StepperMotor_RecalcRamp(StepperMotor_t *motor)
{
  uint32_t v_cruise;
  uint32_t v_start;
  uint32_t accel;
  uint32_t ramp;

  if (motor->cruise_interval == 0U)
  {
    motor->cruise_interval = 1U;
  }
  if (motor->start_interval <= motor->cruise_interval)
  {
    uint16_t next = (uint16_t)(motor->cruise_interval + 1U);
    motor->start_interval = (next == 0U) ? 1U : next;
  }
  accel = (motor->accel_steps_s2 == 0U) ? 1U : motor->accel_steps_s2;

  v_cruise = STEPPER_TICK_HZ / motor->cruise_interval;
  v_start = STEPPER_TICK_HZ / motor->start_interval;
  if (v_cruise <= v_start)
  {
    motor->ramp_steps = 0U;
    return;
  }

  ramp = ((v_cruise * v_cruise) - (v_start * v_start)) / (2U * accel);
  if (ramp < 8U)
  {
    ramp = 8U;
  }
  if (ramp > 4000U)
  {
    ramp = 4000U;
  }
  motor->ramp_steps = (uint16_t)ramp;
}

void StepperMotor_Init(void)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)STEPPER_COUNT; ++i)
  {
    StepperMotor_t *motor = &motors[i];

    motor->position = 0;
    motor->target = 0;
    motor->busy = 0U;
    motor->tick_count = 0U;
    motor->pulse_high = 0U;
    motor->position_valid = 0U;
    motor->min_step = INT32_MIN;
    motor->max_step = INT32_MAX;
    motor->total_steps = 0;
    motor->ramp_steps_current = 0U;
    motor->interval_step = 0U;

    /* 默认速度只用于上电安全状态，正式任务开始前由状态机重新设置。
     * T 轴带转盘惯量，默认比 X/Y 慢。 */
    if (i == (uint8_t)STEPPER_T)
    {
      motor->cruise_interval = 50U;   /* 400 步/秒 */
      motor->start_interval = 100U;   /* 200 步/秒 */
      motor->accel_steps_s2 = 4000U;
    }
    else
    {
      motor->cruise_interval = 25U;   /* 800 步/秒 */
      motor->start_interval = 50U;    /* 400 步/秒 */
      motor->accel_steps_s2 = 10000U;
    }
    motor->step_interval = motor->start_interval;
    StepperMotor_RecalcRamp(motor);

    StepperMotor_WriteStepPin(motor, 0U);
    /* 上电默认关闭驱动，避免初始化期间电机误动作。 */
    StepperMotor_SetEnablePin(motor, 0U);
  }
}

void StepperMotor_SetSpeed(StepperAxis_t axis, uint32_t steps_per_s)
{
  StepperMotor_t *motor;
  uint32_t interval;
  uint32_t start;
  uint32_t primask;

  if (axis >= STEPPER_COUNT)
  {
    return;
  }
  motor = &motors[axis];

  if (steps_per_s < 10U)
  {
    steps_per_s = 10U;
  }
  interval = STEPPER_TICK_HZ / steps_per_s;
  if (interval < 2U)
  {
    interval = 2U;
  }
  if (interval > 65535U)
  {
    interval = 65535U;
  }
  start = interval * 2U;
  if (start > 65535U)
  {
    start = 65535U;
  }
  if (start <= interval)
  {
    start = interval + 1U;
  }

  primask = StepperMotor_EnterCritical();
  motor->cruise_interval = (uint16_t)interval;
  motor->start_interval = (uint16_t)start;
  StepperMotor_RecalcRamp(motor);
  StepperMotor_ExitCritical(primask);
}

void StepperMotor_SetAcceleration(StepperAxis_t axis, uint32_t steps_per_s2)
{
  uint32_t primask;

  if (axis >= STEPPER_COUNT)
  {
    return;
  }

  primask = StepperMotor_EnterCritical();
  motors[axis].accel_steps_s2 = (steps_per_s2 == 0U) ? 1U : steps_per_s2;
  StepperMotor_RecalcRamp(&motors[axis]);
  StepperMotor_ExitCritical(primask);
}

void StepperMotor_SetStepInterval(StepperAxis_t axis, uint16_t interval)
{
  uint32_t primask;

  if (axis >= STEPPER_COUNT)
  {
    return;
  }
  if (interval < 2U)
  {
    interval = 2U;
  }

  primask = StepperMotor_EnterCritical();
  motors[axis].cruise_interval = interval;
  motors[axis].start_interval = interval;
  motors[axis].ramp_steps = 0U;
  if (motors[axis].busy == 0U)
  {
    motors[axis].step_interval = interval;
  }
  StepperMotor_ExitCritical(primask);
}

void StepperMotor_SetSoftLimit(StepperAxis_t axis, int32_t min_step,
                               int32_t max_step)
{
  uint32_t primask;

  if (axis >= STEPPER_COUNT)
  {
    return;
  }
  if (min_step > max_step)
  {
    int32_t temp = min_step;
    min_step = max_step;
    max_step = temp;
  }

  primask = StepperMotor_EnterCritical();
  motors[axis].min_step = min_step;
  motors[axis].max_step = max_step;
  StepperMotor_ExitCritical(primask);
}

uint8_t StepperMotor_IsTargetValid(StepperAxis_t axis, int32_t position)
{
  if (axis >= STEPPER_COUNT)
  {
    return 0U;
  }
  return ((position >= motors[axis].min_step) &&
          (position <= motors[axis].max_step)) ? 1U : 0U;
}

HAL_StatusTypeDef StepperMotor_MoveTo(StepperAxis_t axis, int32_t position)
{
  StepperMotor_t *motor;
  int64_t delta;
  uint32_t total;
  uint32_t half;
  uint16_t ramp;
  uint16_t diff;
  uint32_t primask;

  if (axis >= STEPPER_COUNT)
  {
    return HAL_ERROR;
  }
  motor = &motors[axis];
  if (position < motor->min_step || position > motor->max_step)
  {
    return HAL_ERROR;
  }

  /* 位置和方向必须在同一临界区里读取，避免 TIM6 中断在两行之间
   * 推进位置，造成距离和 DIR 电平不一致。 */
  primask = StepperMotor_EnterCritical();

  delta = (int64_t)position - (int64_t)motor->position;
  if (delta == 0)
  {
    motor->target = position;
    motor->busy = 0U;
    motor->pulse_high = 0U;
    motor->tick_count = 0U;
    StepperMotor_WriteStepPin(motor, 0U);
    /* 即使不需要走步，也保持使能，防止龙门被外力/重力推走。 */
    StepperMotor_SetEnablePin(motor, 1U);
    StepperMotor_ExitCritical(primask);
    return HAL_OK;
  }

  total = (uint32_t)((delta > 0) ? delta : -delta);
  if (total > 0x7FFFFFFFU)
  {
    total = 0x7FFFFFFFU;
  }

  /* 如果上一段运动尚未结束，先清掉当前有效脉冲，避免 STEP 停在有效电平。 */
  motor->pulse_high = 0U;
  StepperMotor_WriteStepPin(motor, 0U);

  motor->target = position;
  motor->busy = 1U;
  motor->tick_count = 0U;
  motor->total_steps = (int32_t)total;

  ramp = motor->ramp_steps;
  half = total / 2U;
  if ((uint32_t)ramp > half)
  {
    ramp = (uint16_t)half;
  }
  motor->ramp_steps_current = ramp;

  if ((ramp > 0U) && (motor->start_interval > motor->cruise_interval))
  {
    diff = (uint16_t)(motor->start_interval - motor->cruise_interval);
    motor->interval_step = (uint16_t)((diff + ramp - 1U) / ramp);
    if (motor->interval_step == 0U)
    {
      motor->interval_step = 1U;
    }
  }
  else
  {
    motor->interval_step = 0U;
  }
  motor->step_interval = motor->start_interval;

  StepperMotor_WriteDir(motor, (position > motor->position) ? 1U : 0U);
  StepperMotor_SetEnablePin(motor, 1U);

  StepperMotor_ExitCritical(primask);
  return HAL_OK;
}

HAL_StatusTypeDef StepperMotor_MoveRelative(StepperAxis_t axis, int32_t delta)
{
  int64_t target;

  if (axis >= STEPPER_COUNT)
  {
    return HAL_ERROR;
  }

  target = (int64_t)StepperMotor_GetPosition(axis) + (int64_t)delta;
  if (target > INT32_MAX)
  {
    target = INT32_MAX;
  }
  if (target < INT32_MIN)
  {
    target = INT32_MIN;
  }
  return StepperMotor_MoveTo(axis, (int32_t)target);
}

HAL_StatusTypeDef StepperMotor_JogStart(StepperAxis_t axis, int8_t direction)
{
  StepperMotor_t *motor;
  uint16_t diff;
  uint32_t primask;

  if ((axis >= STEPPER_COUNT) || (direction == 0))
  {
    return HAL_ERROR;
  }
  motor = &motors[axis];

  primask = StepperMotor_EnterCritical();
  motor->pulse_high = 0U;
  StepperMotor_WriteStepPin(motor, 0U);
  motor->target = (direction > 0) ? 0x7FFFFFF0 : -0x7FFFFFF0;
  motor->busy = 1U;
  motor->tick_count = 0U;

  /* 连续找开关时只保留加速段，不进入减速段：
   * 目标设在极远处，剩余步数远大于 ramp_steps，减速条件不会触发。
   * 这样 JogStart 也能从起步速度平滑加速到巡航速度。 */
  motor->total_steps = 0x7FFFFFFF;
  motor->ramp_steps_current = motor->ramp_steps;
  if ((motor->ramp_steps > 0U) &&
      (motor->start_interval > motor->cruise_interval))
  {
    diff = (uint16_t)(motor->start_interval - motor->cruise_interval);
    motor->interval_step =
        (uint16_t)((diff + motor->ramp_steps - 1U) / motor->ramp_steps);
    if (motor->interval_step == 0U)
    {
      motor->interval_step = 1U;
    }
  }
  else
  {
    motor->interval_step = 0U;
  }
  motor->step_interval = motor->start_interval;

  StepperMotor_WriteDir(motor, (direction > 0) ? 1U : 0U);
  StepperMotor_SetEnablePin(motor, 1U);
  StepperMotor_ExitCritical(primask);
  return HAL_OK;
}

void StepperMotor_Stop(StepperAxis_t axis)
{
  StepperMotor_t *motor;
  uint32_t primask;

  if (axis >= STEPPER_COUNT)
  {
    return;
  }
  motor = &motors[axis];

  primask = StepperMotor_EnterCritical();
  motor->target = motor->position;
  motor->busy = 0U;
  motor->pulse_high = 0U;
  motor->tick_count = 0U;
  StepperMotor_WriteStepPin(motor, 0U);
  StepperMotor_ExitCritical(primask);
}

void StepperMotor_StopAll(void)
{
  uint8_t i;
  uint32_t primask = StepperMotor_EnterCritical();

  for (i = 0U; i < (uint8_t)STEPPER_COUNT; ++i)
  {
    motors[i].target = motors[i].position;
    motors[i].busy = 0U;
    motors[i].pulse_high = 0U;
    motors[i].tick_count = 0U;
    StepperMotor_WriteStepPin(&motors[i], 0U);
    StepperMotor_SetEnablePin(&motors[i], 0U);
  }
  StepperMotor_ExitCritical(primask);
}

void StepperMotor_Enable(StepperAxis_t axis, uint8_t enable)
{
  uint32_t primask;

  if (axis >= STEPPER_COUNT)
  {
    return;
  }

  primask = StepperMotor_EnterCritical();
  StepperMotor_SetEnablePin(&motors[axis], enable);
  StepperMotor_ExitCritical(primask);
}

void StepperMotor_SetPosition(StepperAxis_t axis, int32_t position)
{
  StepperMotor_t *motor;
  uint32_t primask;

  if (axis >= STEPPER_COUNT)
  {
    return;
  }
  motor = &motors[axis];

  primask = StepperMotor_EnterCritical();
  motor->position = position;
  motor->target = position;
  motor->busy = 0U;
  motor->pulse_high = 0U;
  motor->tick_count = 0U;
  motor->position_valid = 1U;
  StepperMotor_WriteStepPin(motor, 0U);
  StepperMotor_ExitCritical(primask);
}

void StepperMotor_InvalidatePosition(StepperAxis_t axis)
{
  if (axis < STEPPER_COUNT)
  {
    motors[axis].position_valid = 0U;
  }
}

uint8_t StepperMotor_IsPositionValid(StepperAxis_t axis)
{
  return (axis < STEPPER_COUNT) ? motors[axis].position_valid : 0U;
}

void StepperMotor_Tick(void)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)STEPPER_COUNT; ++i)
  {
    StepperMotor_t *motor = &motors[i];
    int64_t delta;
    uint32_t remaining;
    uint32_t done;

    if (motor->busy == 0U)
    {
      continue;
    }

    /* 保证 STEP 有效电平至少保持一个 50 us 节拍，满足驱动器最小脉宽。 */
    if (motor->pulse_high != 0U)
    {
      StepperMotor_WriteStepPin(motor, 0U);
      motor->pulse_high = 0U;
      if (motor->position == motor->target)
      {
        motor->busy = 0U;
        continue;
      }
    }

    if (motor->tick_count > 0U)
    {
      --motor->tick_count;
      continue;
    }

    /* 每个节拍每轴最多走一步；速度由 step_interval 控制。 */
    StepperMotor_WriteStepPin(motor, 1U);
    motor->pulse_high = 1U;
    motor->position += (motor->target > motor->position) ? 1 : -1;
    motor->tick_count = (motor->step_interval > 1U)
                            ? (uint16_t)(motor->step_interval - 1U) : 0U;

    /* 线性加减速：前 ramp_steps 步逐步缩短间隔，最后 ramp_steps 步拉长。
     * 只对 MoveTo 生效；JogStart 的 ramp_steps_current 为 0，保持匀速。 */
    if ((motor->ramp_steps_current > 0U) && (motor->interval_step > 0U))
    {
      delta = (int64_t)motor->target - (int64_t)motor->position;
      remaining = (uint32_t)((delta >= 0) ? delta : -delta);
      done = (uint32_t)motor->total_steps - remaining;

      if (done < motor->ramp_steps_current)
      {
        if ((uint32_t)motor->step_interval >
            ((uint32_t)motor->cruise_interval + motor->interval_step))
        {
          motor->step_interval -= motor->interval_step;
        }
        else
        {
          motor->step_interval = motor->cruise_interval;
        }
      }
      else if (remaining <= motor->ramp_steps_current)
      {
        if ((uint32_t)motor->step_interval <
            ((uint32_t)motor->start_interval - motor->interval_step))
        {
          motor->step_interval += motor->interval_step;
        }
        else
        {
          motor->step_interval = motor->start_interval;
        }
      }
    }
  }
}

uint8_t StepperMotor_IsBusy(StepperAxis_t axis)
{
  return (axis < STEPPER_COUNT) ? motors[axis].busy : 0U;
}

int32_t StepperMotor_GetPosition(StepperAxis_t axis)
{
  return (axis < STEPPER_COUNT) ? motors[axis].position : 0;
}

int32_t StepperMotor_GetRemaining(StepperAxis_t axis)
{
  int64_t delta;

  if ((axis >= STEPPER_COUNT) || (motors[axis].busy == 0U))
  {
    return 0;
  }

  delta = (int64_t)motors[axis].target - (int64_t)motors[axis].position;
  if (delta < 0)
  {
    delta = -delta;
  }
  if (delta > INT32_MAX)
  {
    return INT32_MAX;
  }
  return (int32_t)delta;
}
