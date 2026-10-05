#include "sorting_task.h"
#include "main.h"
#include "protocol.h"
#include "safety_manager.h"
#include "servo_gripper.h"
#include "stepper_motor.h"

/* 本模块只在主循环中读写这些变量；state_tick 由 TIM6 中断递增，
 * 因此它使用无符号溢出安全的饱和计数方式。 */
static SortMission_t current_mission;
static SortingState_t state;
static volatile uint32_t state_tick;
static uint8_t has_mission;

void SortingTask_Init(void)
{
  /* 上电后先打开夹爪，再等待回零流程完成。 */
  state = SORT_SYS_INIT;
  state_tick = 0;
  has_mission = 0;
}

void SortingTask_Tick(void)
{
  if (state_tick < UINT32_MAX)
  {
    ++state_tick;
  }
}

uint8_t SortingTask_Submit(const SortMission_t *mission)
{
  /* 不覆盖正在执行的任务，避免上位机重复下发导致目标位置改变。 */
  if (mission == 0 || has_mission || state != SORT_SYS_READY)
  {
    return 0;
  }
  current_mission = *mission;
  has_mission = 1;
  state = SORT_MOVE_TO_PICK;
  state_tick = 0;
  return 1;
}

void SortingTask_Stop(void)
{
  /* STOP 是主动停止：无论当前处于哪个动作阶段，都先停止所有轴。 */
  has_mission = 0;
  state = SafetyManager_IsSafe() ? SORT_SYS_READY : SORT_EMERGENCY_STOP;
  state_tick = 0;
  ServoGripper_Open();
}

void SortingTask_Process(void)
{
  /* 安全检查优先级最高，不能被普通状态机动作覆盖。 */
  if (!SafetyManager_IsSafe())
  {
    state = SORT_EMERGENCY_STOP;
    StepperMotor_StopAll();
    return;
  }

  switch (state)
  {
    case SORT_SYS_INIT:
      ServoGripper_Open();
      state = SORT_SYS_HOMING;
      state_tick = 0;
      break;
    case SORT_SYS_HOMING:
      /* TODO: 根据 T_HOME 及 X/Y 限位输入实现真实回零和位置清零。
       * 当前实现用计时占位，不能替代最终的机械回零流程。 */
      if (state_tick > 100U)
      {
        state = SORT_SYS_READY;
        state_tick = 0;
      }
      break;
    case SORT_SYS_READY:
      break;
    case SORT_MOVE_TO_PICK:
      /* MoveTo 只设置目标；实际脉冲由 TIM6 -> StepperMotor_Tick 产生。 */
      if (StepperMotor_MoveTo(STEPPER_X, current_mission.pick_x_step) != HAL_OK ||
          StepperMotor_MoveTo(STEPPER_Y, current_mission.pick_y_step) != HAL_OK)
      {
        state = SORT_ERROR;
        break;
      }
      state = SORT_GRIP_CLOSE;
      state_tick = 0;
      break;
    case SORT_GRIP_CLOSE:
      /* 两个平移轴到位后才夹取，避免夹爪在运动中碰撞工件。 */
      if (!StepperMotor_IsBusy(STEPPER_X) &&
          !StepperMotor_IsBusy(STEPPER_Y))
      {
        ServoGripper_Close();
        state = SORT_GRIP_STABILIZE;
        state_tick = 0;
      }
      break;
    case SORT_GRIP_STABILIZE:
      /* 给舵机和工件留出稳定时间，再移动旋转/下料轴。 */
      if (state_tick > 200U)
      {
        state = SORT_MOVE_TO_DROP;
        state_tick = 0;
      }
      break;
    case SORT_MOVE_TO_DROP:
      /* T 轴完成目标位置后打开夹爪释放工件。 */
      if (StepperMotor_MoveTo(STEPPER_T, current_mission.drop_step) != HAL_OK)
      {
        state = SORT_ERROR;
        break;
      }
      if (!StepperMotor_IsBusy(STEPPER_T))
      {
        ServoGripper_Open();
        state = SORT_RETURN_SAFE;
        state_tick = 0;
      }
      break;
    case SORT_RETURN_SAFE:
      if (state_tick > 100U)
      {
        state = SORT_TASK_DONE;
      }
      break;
    case SORT_TASK_DONE:
      /* 0x82 表示任务完成；task_id 用于让上位机匹配原任务。 */
      Protocol_SendEvent(0x82, current_mission.task_id);
      has_mission = 0;
      state = SORT_SYS_READY;
      state_tick = 0;
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
