#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "main.h"
#include <stdint.h>

/* 三个逻辑轴：X/Y 负责取放位置，T 负责旋转或下料位置。 */
typedef enum
{
  STEPPER_X = 0,
  STEPPER_Y,
  STEPPER_T,
  STEPPER_COUNT
} StepperAxis_t;

/* 初始化 GPIO 输出、电机软件位置和使能状态。 */
void StepperMotor_Init(void);
/* 产生一个步进脉冲；应由固定周期的定时器中断调用。 */
void StepperMotor_Tick(void);
/* 立即停止所有轴，并保持当前位置作为新的目标位置。 */
void StepperMotor_StopAll(void);
/* 设置绝对步数目标，函数本身不等待运动完成。 */
HAL_StatusTypeDef StepperMotor_MoveTo(StepperAxis_t axis, int32_t position);
/* 查询轴是否仍有未完成的目标。 */
uint8_t StepperMotor_IsBusy(StepperAxis_t axis);
/* 获取软件维护的当前位置；真实机械位置需先完成回零校准。 */
int32_t StepperMotor_GetPosition(StepperAxis_t axis);

#endif
