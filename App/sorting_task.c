#include "sorting_task.h"
#include "sorting_config.h"
#include "main.h"
#include "hall_sensor.h"
#include "protocol.h"
#include "safety_manager.h"
#include "servo_gripper.h"
#include "servo_z.h"
#include "stepper_motor.h"

/* 状态机只在主循环中运行；电机脉冲由 TIM6 中断产生。
 * 所有等待都使用 HAL_GetTick() 的毫秒时基，避免依赖 TIM6 的 20 kHz 计数。 */

static SortMission_t current_mission;
static SortingState_t state;
static SortError_t error_code;
static uint8_t has_mission;

static uint32_t state_enter_ms;
static uint32_t mission_start_ms;
static uint32_t t_move_start_ms;

/* T 轴停稳计时：只有观察到 T 轴空闲后才开始累计稳定时间。 */
static uint32_t turret_idle_since_ms;
static uint8_t turret_idle_seen;

/* T_HOME 消抖用：低电平=霍尔触发（有磁铁），高电平=释放。 */
static uint8_t home_active_seen;
static uint32_t home_active_since_ms;
static uint8_t home_release_seen;
static uint32_t home_release_since_ms;

static void SortingTask_Enter(SortingState_t next)
{
  state = next;
  state_enter_ms = HAL_GetTick();
}

static uint8_t SortingTask_Elapsed(uint32_t ms)
{
  return ((uint32_t)(HAL_GetTick() - state_enter_ms) >= ms) ? 1U : 0U;
}

static uint8_t SortingTask_THomeActive(void)
{
  /* 实际霍尔接 PD12 / T_HOME（CubeMX 网络名沿用 T_HOME）。
   * 器件是开集电极、低有效：有磁铁输出低，gpio.c 配内部上拉，
   * hall_sensor.h 里 HALL_SENSOR_ACTIVE_HIGH=0，这里直接复用。 */
  return HallSensor_IsTriggered();
}

/* 连续低电平达到消抖时间才算真正触发，避免转盘振动造成误判。 */
static uint8_t SortingTask_THomeDebounced(void)
{
  uint32_t now;

  if (SortingTask_THomeActive() == 0U)
  {
    home_active_seen = 0U;
    return 0U;
  }

  now = HAL_GetTick();
  if (home_active_seen == 0U)
  {
    home_active_seen = 1U;
    home_active_since_ms = now;
    return 0U;
  }
  return ((uint32_t)(now - home_active_since_ms) >= SORT_T_HOME_DEBOUNCE_MS)
             ? 1U : 0U;
}

/* 连续高电平达到消抖时间才算真正释放。回零退让阶段用它判断
 * 是否已经完全退出磁铁有效宽度，不再依赖固定退让步数。 */
static uint8_t SortingTask_THomeReleasedDebounced(void)
{
  uint32_t now;

  if (SortingTask_THomeActive() != 0U)
  {
    home_release_seen = 0U;
    return 0U;
  }

  now = HAL_GetTick();
  if (home_release_seen == 0U)
  {
    home_release_seen = 1U;
    home_release_since_ms = now;
    return 0U;
  }
  return ((uint32_t)(now - home_release_since_ms) >= SORT_T_HOME_DEBOUNCE_MS)
             ? 1U : 0U;
}

#if SORT_GRIP_VERIFY_ENABLE
static uint8_t SortingTask_GripSensorActive(void)
{
  GPIO_PinState value = HAL_GPIO_ReadPin(GRIP_SENSOR_GPIO_Port,
                                         GRIP_SENSOR_Pin);
#if SORT_GRIP_SENSOR_ACTIVE_LOW
  return (value == GPIO_PIN_RESET) ? 1U : 0U;
#else
  return (value == GPIO_PIN_SET) ? 1U : 0U;
#endif
}
#endif

/* 统一的运动参数入口；换细分或更换机构后只改 sorting_config.h。 */
static void SortingTask_ApplyMotionConfig(void)
{
  StepperMotor_SetSpeed(STEPPER_X, SORT_XY_MAX_SPEED_STEPS_S);
  StepperMotor_SetAcceleration(STEPPER_X, SORT_XY_ACCEL_STEPS_S2);
  StepperMotor_SetSoftLimit(STEPPER_X, SORT_X_MIN_STEP, SORT_X_MAX_STEP);

  StepperMotor_SetSpeed(STEPPER_Y, SORT_XY_MAX_SPEED_STEPS_S);
  StepperMotor_SetAcceleration(STEPPER_Y, SORT_XY_ACCEL_STEPS_S2);
  StepperMotor_SetSoftLimit(STEPPER_Y, SORT_Y_MIN_STEP, SORT_Y_MAX_STEP);

  StepperMotor_SetSpeed(STEPPER_T, SORT_T_MAX_SPEED_STEPS_S);
  StepperMotor_SetAcceleration(STEPPER_T, SORT_T_ACCEL_STEPS_S2);
  StepperMotor_SetSoftLimit(STEPPER_T, SORT_T_MIN_STEP, SORT_T_MAX_STEP);

#if SORT_ASSUME_XY_ORIGIN
  /* 没有 X/Y 回零开关时，操作者必须先手动把机构推到原点。
   * 装上限位后把 SORT_ASSUME_XY_ORIGIN 改为 0，并补真正的回零流程。
   * 这里保持 X/Y 使能，避免 T 回零期间龙门被外力推走。 */
  StepperMotor_SetPosition(STEPPER_X, 0);
  StepperMotor_SetPosition(STEPPER_Y, 0);
  StepperMotor_Enable(STEPPER_X, 1U);
  StepperMotor_Enable(STEPPER_Y, 1U);
#endif
  /* T 轴必须靠 T_HOME 建立绝对位置。 */
  StepperMotor_InvalidatePosition(STEPPER_T);
}

static void SortingTask_Fail(SortError_t code)
{
  error_code = code;
  StepperMotor_StopAll();
  /* 故障时保持夹爪当前脉宽，不主动开爪，避免半空掉料。 */
  ServoGripper_Stop();

  if (has_mission != 0U)
  {
    Protocol_SendError(current_mission.task_id, (uint8_t)code);
  }
  has_mission = 0U;
  SortingTask_Enter(SORT_ERROR);
}

/* 计算目标仓并启动 T 轴；T 与 Z 抬升并行执行。
 * box_id 1..6 使用仓表；否则按配置回退到 drop_step。 */
static uint8_t SortingTask_StartTurret(void)
{
  int32_t current;
  int32_t desired;
  int32_t delta;
  int32_t target;
  int32_t bin_index = -1;

#if SORT_BOX_ID_ONE_BASED
  if ((current_mission.box_id >= 1U) &&
      (current_mission.box_id <= SORT_T_BOX_COUNT))
  {
    bin_index = (int32_t)current_mission.box_id - 1;
  }
#else
  if (current_mission.box_id < SORT_T_BOX_COUNT)
  {
    bin_index = (int32_t)current_mission.box_id;
  }
#endif

  if (bin_index >= 0)
  {
    desired = SORT_T_HOME_OFFSET_STEP + SORT_T_BIN_STEP(bin_index);
    current = StepperMotor_GetPosition(STEPPER_T);

#if SORT_T_USE_SHORTEST_PATH
    delta = (desired - current) % SORT_T_STEPS_PER_REV;
    if (delta < 0)
    {
      delta += SORT_T_STEPS_PER_REV;
    }
    if (delta > (SORT_T_STEPS_PER_REV / 2))
    {
      delta -= SORT_T_STEPS_PER_REV;
    }
    target = current + delta;
#else
    /* 单向转：始终按 SORT_T_ROTATE_SIGN 指定方向补到目标仓，
     * 避免线缆反向绕回。 */
#if SORT_T_ROTATE_SIGN > 0
    delta = (desired - current) % SORT_T_STEPS_PER_REV;
    if (delta < 0)
    {
      delta += SORT_T_STEPS_PER_REV;
    }
    target = current + delta;
#else
    delta = (current - desired) % SORT_T_STEPS_PER_REV;
    if (delta < 0)
    {
      delta += SORT_T_STEPS_PER_REV;
    }
    target = current - delta;
#endif
#endif
  }
  else if (SORT_T_ALLOW_DROP_STEP_FALLBACK != 0)
  {
    target = current_mission.drop_step;
  }
  else
  {
    return 0U;
  }

  if (StepperMotor_MoveTo(STEPPER_T, target) != HAL_OK)
  {
    return 0U;
  }
  t_move_start_ms = HAL_GetTick();
  turret_idle_seen = 0U;
  return 1U;
}

/* 把 T 轴累计位置归一化到 0..STEPS_PER_REV-1。
 * 单向转多任务后位置会持续增大，归一化只改软件坐标、不动电机，
 * 既避免 int32 长时间累积，也让软限位保持有意义。 */
static void SortingTask_NormalizeTurretPosition(void)
{
  int32_t pos;
  int32_t norm;

  pos = StepperMotor_GetPosition(STEPPER_T);
  norm = pos % SORT_T_STEPS_PER_REV;
  if (norm < 0)
  {
    norm += SORT_T_STEPS_PER_REV;
  }
  if (norm != pos)
  {
    StepperMotor_SetPosition(STEPPER_T, norm);
  }
}

static uint8_t SortingTask_StartPickMove(void)
{
  if ((StepperMotor_IsTargetValid(STEPPER_X, current_mission.pick_x_step) == 0U) ||
      (StepperMotor_IsTargetValid(STEPPER_Y, current_mission.pick_y_step) == 0U))
  {
    return 0U;
  }
  if (StepperMotor_MoveTo(STEPPER_X, current_mission.pick_x_step) != HAL_OK)
  {
    return 0U;
  }
  if (StepperMotor_MoveTo(STEPPER_Y, current_mission.pick_y_step) != HAL_OK)
  {
    StepperMotor_Stop(STEPPER_X);
    return 0U;
  }
  return 1U;
}

static uint8_t SortingTask_StartDropMove(void)
{
  if ((StepperMotor_IsTargetValid(STEPPER_X, SORT_DROP_X_STEP) == 0U) ||
      (StepperMotor_IsTargetValid(STEPPER_Y, SORT_DROP_Y_STEP) == 0U))
  {
    return 0U;
  }
  if (StepperMotor_MoveTo(STEPPER_X, SORT_DROP_X_STEP) != HAL_OK)
  {
    return 0U;
  }
  if (StepperMotor_MoveTo(STEPPER_Y, SORT_DROP_Y_STEP) != HAL_OK)
  {
    StepperMotor_Stop(STEPPER_X);
    return 0U;
  }
  return 1U;
}

#if SORT_RETURN_TO_PARK
static uint8_t SortingTask_StartParkMove(void)
{
  if ((StepperMotor_IsTargetValid(STEPPER_X, SORT_PARK_X_STEP) == 0U) ||
      (StepperMotor_IsTargetValid(STEPPER_Y, SORT_PARK_Y_STEP) == 0U))
  {
    return 0U;
  }
  if (StepperMotor_MoveTo(STEPPER_X, SORT_PARK_X_STEP) != HAL_OK)
  {
    return 0U;
  }
  if (StepperMotor_MoveTo(STEPPER_Y, SORT_PARK_Y_STEP) != HAL_OK)
  {
    StepperMotor_Stop(STEPPER_X);
    return 0U;
  }
  return 1U;
}
#endif

void SortingTask_Init(void)
{
  state = SORT_SYS_INIT;
  error_code = SORT_ERR_NONE;
  has_mission = 0U;
  state_enter_ms = HAL_GetTick();
  mission_start_ms = 0U;
  t_move_start_ms = 0U;
  turret_idle_since_ms = 0U;
  turret_idle_seen = 0U;
  home_active_seen = 0U;
  home_active_since_ms = 0U;
  home_release_seen = 0U;
  home_release_since_ms = 0U;

  SortingTask_ApplyMotionConfig();
}

uint8_t SortingTask_Submit(const SortMission_t *mission)
{
  if ((mission == 0) || (has_mission != 0U) ||
      (state != SORT_SYS_READY) || (SafetyManager_IsSafe() == 0U))
  {
    return 0U;
  }

  if ((StepperMotor_IsPositionValid(STEPPER_X) == 0U) ||
      (StepperMotor_IsPositionValid(STEPPER_Y) == 0U) ||
      (StepperMotor_IsPositionValid(STEPPER_T) == 0U))
  {
    error_code = SORT_ERR_NOT_READY;
    return 0U;
  }

  if ((StepperMotor_IsTargetValid(STEPPER_X, mission->pick_x_step) == 0U) ||
      (StepperMotor_IsTargetValid(STEPPER_Y, mission->pick_y_step) == 0U))
  {
    error_code = SORT_ERR_COORD;
    return 0U;
  }

#if SORT_BOX_ID_ONE_BASED
  if ((mission->box_id < 1U) || (mission->box_id > SORT_T_BOX_COUNT))
#else
  if (mission->box_id >= SORT_T_BOX_COUNT)
#endif
  {
    /* box_id 非法时只能回退到 drop_step；若两者都不可用，提前拒绝任务，
     * 不要等到 LIFT_AND_TURRET 才失败。 */
    if ((SORT_T_ALLOW_DROP_STEP_FALLBACK == 0) ||
        (StepperMotor_IsTargetValid(STEPPER_T,
                                    mission->drop_step) == 0U))
    {
      error_code = SORT_ERR_COORD;
      return 0U;
    }
  }

  current_mission = *mission;
  has_mission = 1U;
  error_code = SORT_ERR_NONE;
  mission_start_ms = HAL_GetTick();

  /* READY 状态下夹爪应已张开；这里再发一次，确保下降前处于张开状态。 */
  ServoGripper_Open();

  if (SortingTask_StartPickMove() == 0U)
  {
    StepperMotor_StopAll();
    has_mission = 0U;
    error_code = SORT_ERR_COORD;
    return 0U;
  }

  SortingTask_Enter(SORT_MOVE_TO_PICK_XY);
  return 1U;
}

void SortingTask_Stop(void)
{
  /* READY/ERROR/急停状态下重复发 STOP 不改变状态。 */
  if ((state == SORT_SYS_READY) || (state == SORT_ERROR) ||
      (state == SORT_EMERGENCY_STOP))
  {
    return;
  }

  /* 先停轴，再保持夹爪当前脉宽；不主动开爪，避免半空掉料。 */
  StepperMotor_StopAll();
  ServoGripper_Stop();

  if (has_mission != 0U)
  {
    Protocol_SendError(current_mission.task_id, (uint8_t)SORT_ERR_STOPPED);
  }
  has_mission = 0U;
  error_code = SORT_ERR_STOPPED;

  if (SafetyManager_IsSafe() != 0U)
  {
    SortingTask_Enter(SORT_ERROR);
  }
  else
  {
    SortingTask_Enter(SORT_EMERGENCY_STOP);
  }
}

uint8_t SortingTask_ResetFault(void)
{
  /* 急停松开后才允许清除锁存故障；软停止时该调用无副作用。 */
  if (SafetyManager_ResetFault() == 0U)
  {
    return 0U;
  }

  /* READY 已经是正常状态，重复复位不改变位置。 */
  if (state == SORT_SYS_READY)
  {
    return 1U;
  }
  if ((state != SORT_ERROR) && (state != SORT_EMERGENCY_STOP))
  {
    return 0U;
  }

  StepperMotor_StopAll();
  ServoGripper_Stop();
  has_mission = 0U;
  error_code = SORT_ERR_NONE;
  ZServo_Travel();
  SortingTask_Enter(SORT_SYS_HOMING);
  return 1U;
}

uint8_t SortingTask_RequestHoming(void)
{
  if ((SafetyManager_IsSafe() == 0U) || (state != SORT_SYS_READY) ||
      (has_mission != 0U))
  {
    return 0U;
  }

  StepperMotor_StopAll();
  ZServo_Travel();
  SortingTask_Enter(SORT_SYS_HOMING);
  return 1U;
}

void SortingTask_Process(void)
{
  /* 安全检查优先级最高；急停后锁存，必须走 ResetFault 重新回零。 */
  if (SafetyManager_IsSafe() == 0U)
  {
    if (state != SORT_EMERGENCY_STOP)
    {
      StepperMotor_StopAll();
      ServoGripper_Stop();
      if (has_mission != 0U)
      {
        Protocol_SendError(current_mission.task_id,
                           (uint8_t)SORT_ERR_EMERGENCY);
      }
      has_mission = 0U;
      error_code = SORT_ERR_EMERGENCY;
      SortingTask_Enter(SORT_EMERGENCY_STOP);
    }
    return;
  }

  /* 整任务兜底超时，防止某个传感器/驱动器异常导致状态机永久卡住。 */
  if ((has_mission != 0U) && (state != SORT_TASK_DONE) &&
      (state != SORT_ERROR) && (state != SORT_EMERGENCY_STOP) &&
      ((uint32_t)(HAL_GetTick() - mission_start_ms) >
       SORT_MISSION_TIMEOUT_MS))
  {
    SortingTask_Fail(SORT_ERR_MISSION_TIMEOUT);
    return;
  }

  switch (state)
  {
    case SORT_SYS_INIT:
      /* 上电先把 Z 抬到运输高度并张开夹爪，再开始 T 轴回零。 */
      ZServo_Travel();
      ServoGripper_Open();
      SortingTask_Enter(SORT_SYS_HOMING);
      break;

    case SORT_SYS_HOMING:
#if SORT_ASSUME_XY_ORIGIN
      /* 故障/软停时 StopAll 会关闭 X/Y 使能；回零期间重新保持，
       * 防止 T 找零时龙门被外力推走。 */
      StepperMotor_Enable(STEPPER_X, 1U);
      StepperMotor_Enable(STEPPER_Y, 1U);
#endif
      StepperMotor_InvalidatePosition(STEPPER_T);
      StepperMotor_SetAcceleration(STEPPER_T, SORT_T_ACCEL_STEPS_S2);
      StepperMotor_SetSpeed(STEPPER_T, SORT_T_HOME_SPEED_STEPS_S);
      home_active_seen = 0U;
      home_release_seen = 0U;
      if (StepperMotor_JogStart(STEPPER_T,
                                (int8_t)SORT_T_HOME_DIR) != HAL_OK)
      {
        SortingTask_Fail(SORT_ERR_HOMING);
        break;
      }
      ZServo_Travel();
      ServoGripper_Open();
      SortingTask_Enter(SORT_T_HOMING_SEARCH);
      break;

    case SORT_T_HOMING_SEARCH:
      if (SortingTask_THomeDebounced() != 0U)
      {
        StepperMotor_Stop(STEPPER_T);
        /* 反向慢速退出磁场，直到霍尔释放；不再依赖固定退让步数，
         * 磁铁宽/窄、气隙变化都不需要改参数。 */
        StepperMotor_SetSpeed(STEPPER_T,
                              SORT_T_HOME_BACKOFF_SPEED_STEPS_S);
        home_release_seen = 0U;
        if (StepperMotor_JogStart(
                STEPPER_T,
                (int8_t)(-SORT_T_HOME_DIR)) != HAL_OK)
        {
          SortingTask_Fail(SORT_ERR_HOMING);
          break;
        }
        SortingTask_Enter(SORT_T_HOMING_BACKOFF);
      }
      else if (SortingTask_Elapsed(SORT_T_HOME_TIMEOUT_MS) != 0U)
      {
        SortingTask_Fail(SORT_ERR_HOMING);
      }
      else if (StepperMotor_IsBusy(STEPPER_T) == 0U)
      {
        /* Jog 没有遇到传感器就停了，说明驱动/接线异常。 */
        SortingTask_Fail(SORT_ERR_HOMING);
      }
      break;

    case SORT_T_HOMING_BACKOFF:
      if (SortingTask_THomeReleasedDebounced() != 0U)
      {
        /* 已完全退出磁铁，再按回零方向慢速接近；这次触发点
         * 与第一次搜索的触发点是同一个磁场边沿，作为 T 轴 0 点。 */
        StepperMotor_Stop(STEPPER_T);
        StepperMotor_SetSpeed(STEPPER_T,
                              SORT_T_HOME_APPROACH_SPEED_STEPS_S);
        home_active_seen = 0U;
        if (StepperMotor_JogStart(STEPPER_T,
                                  (int8_t)SORT_T_HOME_DIR) != HAL_OK)
        {
          SortingTask_Fail(SORT_ERR_HOMING);
          break;
        }
        SortingTask_Enter(SORT_T_HOMING_APPROACH);
      }
      else if (SortingTask_Elapsed(SORT_T_HOME_TIMEOUT_MS) != 0U)
      {
        /* 反向走了 20 s 还没退出磁场：霍尔卡死/短路或磁铁占满整圈。 */
        SortingTask_Fail(SORT_ERR_HOMING);
      }
      else if (StepperMotor_IsBusy(STEPPER_T) == 0U)
      {
        /* 反向 Jog 不应自己停；停了说明驱动/接线异常。 */
        SortingTask_Fail(SORT_ERR_HOMING);
      }
      break;

    case SORT_T_HOMING_APPROACH:
      if (SortingTask_THomeDebounced() != 0U)
      {
        StepperMotor_Stop(STEPPER_T);
        /* 慢速二次触发的位置定义为 T 轴 0 点，重复精度高于高速触发。 */
        StepperMotor_SetPosition(STEPPER_T, 0);
        StepperMotor_SetSpeed(STEPPER_T, SORT_T_MAX_SPEED_STEPS_S);
        StepperMotor_SetAcceleration(STEPPER_T, SORT_T_ACCEL_STEPS_S2);
        ZServo_Travel();
        ServoGripper_Open();
        SortingTask_Enter(SORT_SYS_READY);
        /* 回零完成主动上报一次，视觉侧不必轮询。 */
        Protocol_SendStatus((uint8_t)SORT_SYS_READY,
                            (uint8_t)SORT_ERR_NONE);
      }
      else if (SortingTask_Elapsed(SORT_T_HOME_TIMEOUT_MS) != 0U)
      {
        SortingTask_Fail(SORT_ERR_HOMING);
      }
      else if (StepperMotor_IsBusy(STEPPER_T) == 0U)
      {
        SortingTask_Fail(SORT_ERR_HOMING);
      }
      break;

    case SORT_SYS_READY:
      break;

    case SORT_MOVE_TO_PICK_XY:
      if ((StepperMotor_IsBusy(STEPPER_X) == 0U) &&
          (StepperMotor_IsBusy(STEPPER_Y) == 0U))
      {
        ZServo_Approach();
        SortingTask_Enter(SORT_Z_APPROACH);
      }
      else if (SortingTask_Elapsed(SORT_XY_TIMEOUT_MS) != 0U)
      {
        SortingTask_Fail(SORT_ERR_XY_TIMEOUT);
      }
      break;

    case SORT_Z_APPROACH:
      if (SortingTask_Elapsed(SORT_Z_APPROACH_MS) != 0U)
      {
        ServoGripper_Open();
        SortingTask_Enter(SORT_GRIP_OPEN);
      }
      break;

    case SORT_GRIP_OPEN:
      if (SortingTask_Elapsed(SORT_GRIP_OPEN_MS) != 0U)
      {
        ZServo_Grasp();
        SortingTask_Enter(SORT_Z_DESCEND);
      }
      break;

    case SORT_Z_DESCEND:
      if (SortingTask_Elapsed(SORT_Z_DESCEND_MS) != 0U)
      {
        ServoGripper_Close();
        SortingTask_Enter(SORT_GRIP_CLOSE);
      }
      break;

    case SORT_GRIP_CLOSE:
      if (SortingTask_Elapsed(SORT_GRIP_CLOSE_MS) != 0U)
      {
#if SORT_GRIP_VERIFY_ENABLE
        SortingTask_Enter(SORT_GRIP_VERIFY);
#else
        /* 夹紧稳定后立刻启动 T 轴，与 Z 抬升并行，缩短节拍。 */
        if (SortingTask_StartTurret() == 0U)
        {
          SortingTask_Fail(SORT_ERR_COORD);
          break;
        }
        ZServo_Travel();
        SortingTask_Enter(SORT_LIFT_AND_TURRET);
#endif
      }
      break;

    case SORT_GRIP_VERIFY:
#if SORT_GRIP_VERIFY_ENABLE
      if (SortingTask_Elapsed(SORT_GRIP_VERIFY_MS) != 0U)
      {
        if (SortingTask_GripSensorActive() != 0U)
        {
          if (SortingTask_StartTurret() == 0U)
          {
            SortingTask_Fail(SORT_ERR_COORD);
            break;
          }
          ZServo_Travel();
          SortingTask_Enter(SORT_LIFT_AND_TURRET);
        }
        else
        {
          /* 夹爪闭合但没有物块：保持闭合等待人工处理 */
          SortingTask_Fail(SORT_ERR_GRIP);
        }
      }
#endif
      break;

    case SORT_LIFT_AND_TURRET:
      if (SortingTask_Elapsed(SORT_Z_LIFT_MS) != 0U)
      {
        /* 抬到运输高度后才可以横向去投放口。 */
        if (SortingTask_StartDropMove() == 0U)
        {
          SortingTask_Fail(SORT_ERR_COORD);
          break;
        }
        SortingTask_Enter(SORT_MOVE_TO_DROP_XY);
      }
      break;

    case SORT_MOVE_TO_DROP_XY:
      if ((StepperMotor_IsBusy(STEPPER_X) == 0U) &&
          (StepperMotor_IsBusy(STEPPER_Y) == 0U))
      {
        SortingTask_Enter(SORT_TURRET_WAIT);
      }
      else if ((StepperMotor_IsBusy(STEPPER_T) != 0U) &&
               ((uint32_t)(HAL_GetTick() - t_move_start_ms) >
                SORT_T_TIMEOUT_MS))
      {
        /* T 轴与 XY 并行运动，XY 还没到位时也要检查 T 的超时。 */
        SortingTask_Fail(SORT_ERR_T_TIMEOUT);
      }
      else if (SortingTask_Elapsed(SORT_XY_TIMEOUT_MS) != 0U)
      {
        SortingTask_Fail(SORT_ERR_XY_TIMEOUT);
      }
      break;

    case SORT_TURRET_WAIT:
      /* T 轴停稳后再等稳定时间，防止转盘惯性/物块晃动导致投放偏位。
       * 稳定计时从“观察到 T 轴空闲”开始，而不是从进入本状态开始；
       * 否则 T 比 XY 晚停时，会在 T 刚停甚至还在转时就开爪。 */
      if (StepperMotor_IsBusy(STEPPER_T) == 0U)
      {
        if (turret_idle_seen == 0U)
        {
          turret_idle_seen = 1U;
          turret_idle_since_ms = HAL_GetTick();
        }
        if ((uint32_t)(HAL_GetTick() - turret_idle_since_ms) >=
            SORT_T_SETTLE_MS)
        {
          SortingTask_NormalizeTurretPosition();
          ServoGripper_Open();
          SortingTask_Enter(SORT_DROP_OPEN);
        }
      }
      else
      {
        turret_idle_seen = 0U;
        if ((uint32_t)(HAL_GetTick() - t_move_start_ms) >
            SORT_T_TIMEOUT_MS)
        {
          SortingTask_Fail(SORT_ERR_T_TIMEOUT);
        }
      }
      break;

    case SORT_DROP_OPEN:
      if (SortingTask_Elapsed(SORT_DROP_OPEN_MS) != 0U)
      {
#if SORT_RETURN_TO_PARK
        if (SortingTask_StartParkMove() == 0U)
        {
          SortingTask_Fail(SORT_ERR_COORD);
          break;
        }
#endif
        SortingTask_Enter(SORT_RETURN_SAFE);
      }
      break;

    case SORT_RETURN_SAFE:
      if ((SortingTask_Elapsed(SORT_RETURN_MS) != 0U) &&
          (StepperMotor_IsBusy(STEPPER_X) == 0U) &&
          (StepperMotor_IsBusy(STEPPER_Y) == 0U))
      {
        ZServo_Travel();
        ServoGripper_Open();
        SortingTask_Enter(SORT_TASK_DONE);
      }
      else if (SortingTask_Elapsed(SORT_RETURN_MS +
                                   SORT_XY_TIMEOUT_MS) != 0U)
      {
        SortingTask_Fail(SORT_ERR_XY_TIMEOUT);
      }
      break;

    case SORT_TASK_DONE:
      /* 0x82 表示任务完成；task_id 用于让上位机匹配原任务。 */
      Protocol_SendEvent(PROTOCOL_EVENT_DONE, current_mission.task_id);
      has_mission = 0U;
      ZServo_Travel();
      ServoGripper_Open();
      SortingTask_Enter(SORT_SYS_READY);
      break;

    case SORT_ERROR:
    case SORT_EMERGENCY_STOP:
    default:
      break;
  }
}
SortingState_t SortingTask_GetState(void)
{
  return state;
}

SortError_t SortingTask_GetError(void)
{
  return error_code;
}

uint8_t SortingTask_HasMission(void)
{
  return has_mission;
}
