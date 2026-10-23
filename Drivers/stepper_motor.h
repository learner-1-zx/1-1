#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "main.h"
#include <stdint.h>

/* TIM6 运动节拍频率：Prescaler 83、Period 49 -> 20 kHz，周期 50 us。 */
#define STEPPER_TICK_HZ 20000U

/* 三个逻辑轴：X/Y 负责取放位置，T 负责转盘仓位。 */
typedef enum
{
  STEPPER_X = 0,
  STEPPER_Y,
  STEPPER_T,
  STEPPER_COUNT
} StepperAxis_t;

/* 初始化 GPIO 输出、电机软件位置和默认速度。
 * 上电默认关闭驱动器使能，避免初始化期间电机误动作。 */
void StepperMotor_Init(void);
/* 产生一个步进脉冲；由固定 20 kHz 的 TIM6 中断调用。 */
void StepperMotor_Tick(void);

/* 设置巡航速度（步/秒），内部会换算成 TIM6 的脉冲间隔。
 * 速度上限受 20 kHz 节拍限制：实际最大 10000 步/秒。 */
void StepperMotor_SetSpeed(StepperAxis_t axis, uint32_t steps_per_s);
/* 设置加速度（步/秒^2），内部换算成加减速所需步数。
 * 数值越大，起停越猛；X/Y 负载和 T 轴惯量不同，必须分别调。 */
void StepperMotor_SetAcceleration(StepperAxis_t axis, uint32_t steps_per_s2);
/* 兼容旧接口：直接指定每隔多少个 TIM6 节拍走一步。
 * interval 会同时作为起步和巡航间隔，等于关闭加减速。 */
void StepperMotor_SetStepInterval(StepperAxis_t axis, uint16_t interval);

/* 设置软限位（绝对步数）。MoveTo 越界时返回 HAL_ERROR。 */
void StepperMotor_SetSoftLimit(StepperAxis_t axis, int32_t min_step,
                               int32_t max_step);
/* 判断目标是否在软限位内。 */
uint8_t StepperMotor_IsTargetValid(StepperAxis_t axis, int32_t position);

/* 设置绝对步数目标，函数本身不等待运动完成。
 * 运动过程中保持驱动器使能，到位后也不自动释放。 */
HAL_StatusTypeDef StepperMotor_MoveTo(StepperAxis_t axis, int32_t position);
/* 相对当前位置移动指定步数；用于回零退让等小行程动作。 */
HAL_StatusTypeDef StepperMotor_MoveRelative(StepperAxis_t axis, int32_t delta);

/* 回零/找传感器用：朝指定方向连续走，直到调用 StepperMotor_Stop。
 * direction 只接受 +1 / -1。此接口不检查软限位，调用方必须自己限时。 */
HAL_StatusTypeDef StepperMotor_JogStart(StepperAxis_t axis, int8_t direction);
/* 停止单轴：目标改为当前位置，清除忙标志和 STEP 电平。 */
void StepperMotor_Stop(StepperAxis_t axis);
/* 立即停止所有轴，并关闭全部驱动器使能（急停/故障用）。 */
void StepperMotor_StopAll(void);

/* 单独控制驱动器使能；MoveTo 会自动使能，StopAll 会关闭。 */
void StepperMotor_Enable(StepperAxis_t axis, uint8_t enable);

/* 回零完成后把软件位置清零。 */
void StepperMotor_SetPosition(StepperAxis_t axis, int32_t position);
/* 位置不可信时调用；急停后应重新回零。 */
void StepperMotor_InvalidatePosition(StepperAxis_t axis);
/* 返回该轴是否已经建立可信的绝对位置。 */
uint8_t StepperMotor_IsPositionValid(StepperAxis_t axis);

/* 查询轴是否仍有未完成的目标。 */
uint8_t StepperMotor_IsBusy(StepperAxis_t axis);
/* 获取软件维护的当前位置；真实机械位置需先完成回零校准。 */
int32_t StepperMotor_GetPosition(StepperAxis_t axis);
/* 剩余步数；不忙时返回 0。 */
int32_t StepperMotor_GetRemaining(StepperAxis_t axis);

#endif
