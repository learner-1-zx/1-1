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
  /* 当前输入采用上拉，急停和限位均按低电平有效处理。 */
  if (HAL_GPIO_ReadPin(EMERGENCY_STOP_GPIO_Port, EMERGENCY_STOP_Pin)
      == GPIO_PIN_RESET)
  {
    fault = SAFETY_FAULT_EMERGENCY;
    StepperMotor_StopAll();
    return;
  }

  /* 限位输入当前只监控 X/Y；T_HOME 由后续回零流程使用。 */
  if ((HAL_GPIO_ReadPin(X_MIN_GPIO_Port, X_MIN_Pin) == GPIO_PIN_RESET) ||
      (HAL_GPIO_ReadPin(X_MAX_GPIO_Port, X_MAX_Pin) == GPIO_PIN_RESET) ||
      (HAL_GPIO_ReadPin(Y_MIN_GPIO_Port, Y_MIN_Pin) == GPIO_PIN_RESET) ||
      (HAL_GPIO_ReadPin(Y_MAX_GPIO_Port, Y_MAX_Pin) == GPIO_PIN_RESET))
  {
    fault = SAFETY_FAULT_LIMIT;
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
