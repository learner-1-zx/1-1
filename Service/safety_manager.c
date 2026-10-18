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
   * PD9～PD11 作为新增模块的优先扩展口（PB8 已分配给夹爪舵机，PD8 已释放）；
   * 限位功能后续单独考虑。 */
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
