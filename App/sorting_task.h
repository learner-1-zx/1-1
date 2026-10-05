#ifndef __SORTING_TASK_H
#define __SORTING_TASK_H

#include "protocol.h"
#include <stdint.h>

/* 分拣任务状态机：
 * 初始化/回零完成后进入 READY，收到任务后依次执行取料、夹取、
 * 旋转/下料和回安全位。急停和普通错误状态需要外部复位。 */
typedef enum
{
  SORT_SYS_INIT = 0,
  SORT_SYS_HOMING,
  SORT_SYS_READY,
  SORT_MOVE_TO_PICK,
  SORT_GRIP_CLOSE,
  SORT_GRIP_STABILIZE,
  SORT_MOVE_TO_DROP,
  SORT_GRIP_OPEN,
  SORT_RETURN_SAFE,
  SORT_TASK_DONE,
  SORT_ERROR,
  SORT_EMERGENCY_STOP
} SortingState_t;

/* 在系统启动时清空任务并进入初始化状态。 */
void SortingTask_Init(void);
/* 在主循环中驱动非阻塞状态机。 */
void SortingTask_Process(void);
/* 由 TIM6 周期中断调用，维护状态机超时计数。 */
void SortingTask_Tick(void);
/* 仅在 READY 且没有未完成任务时接收任务，成功返回 1。 */
uint8_t SortingTask_Submit(const SortMission_t *mission);
/* 中止当前任务，打开夹爪，并根据安全状态回到 READY 或急停态。 */
void SortingTask_Stop(void);
/* 读取当前状态，供上位机或调试代码使用。 */
SortingState_t SortingTask_GetState(void);

#endif
