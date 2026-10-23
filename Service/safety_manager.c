#include "safety_manager.h"
#include "main.h"
#include "stepper_motor.h"

static SafetyFault_t fault;

void SafetyManager_Init(void)
{
  /* 故障状态采用锁存语义，避免输入抖动导致系统自动恢复运动。 */
  fault = SAFETY_FAULT_NONE;
}

void SafetyManager_Process(void)
{
  /* 当前输入采用上拉，急停按低电平有效处理。
   * 夹爪使用 PA8，PD8～PD10 作为新增模块的优先扩展口，PD11 上拉备用，
   * PD12 为转盘零位霍尔；限位功能后续单独考虑。 */
  if (HAL_GPIO_ReadPin(EMERGENCY_STOP_GPIO_Port, EMERGENCY_STOP_Pin)
      == GPIO_PIN_RESET)
  {
    fault = SAFETY_FAULT_EMERGENCY;
    StepperMotor_StopAll();
  }
}

uint8_t SafetyManager_IsSafe(void)
{
  return fault == SAFETY_FAULT_NONE;
}

SafetyFault_t SafetyManager_GetFault(void)
{
  return fault;
}

uint8_t SafetyManager_ResetFault(void)
{
  /* 只有急停输入已经松开（上拉为高）才允许清除，避免按住急停时
   * 通过串口复位后系统立刻恢复运动。 */
  if (HAL_GPIO_ReadPin(EMERGENCY_STOP_GPIO_Port, EMERGENCY_STOP_Pin)
      == GPIO_PIN_SET)
  {
    fault = SAFETY_FAULT_NONE;
    return 1U;
  }
  return 0U;
}
