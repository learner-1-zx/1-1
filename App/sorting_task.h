#ifndef __SORTING_TASK_H
#define __SORTING_TASK_H

#include "protocol.h"
#include <stdint.h>

/* 分拣任务状态机：
 * 初始化后按 X 回零 -> Y 回零（SORT_ENABLE_Y_AXIS=1 时）-> T 轴回零，
 * 完成后进入 READY；收到任务后依次执行
 * XY 取料 -> Z 接近 -> 开爪 -> Z 下降 -> 闭爪 -> Z 抬升 + T 转动
 * -> XY 投放 -> 等 T 到位 -> 开爪 -> 回安全位。
 * 急停和普通错误状态需要外部复位，复位后重新回零。 */
typedef enum
{
  SORT_SYS_INIT = 0,
  SORT_SYS_HOMING,
  SORT_T_HOMING_SEARCH,
  SORT_T_HOMING_BACKOFF,
  SORT_T_HOMING_APPROACH,
  SORT_SYS_READY,
  SORT_MOVE_TO_PICK_XY,
  SORT_Z_APPROACH,
  SORT_GRIP_OPEN,
  SORT_Z_DESCEND,
  SORT_GRIP_CLOSE,
  SORT_GRIP_VERIFY,
  SORT_LIFT_AND_TURRET,
  SORT_MOVE_TO_DROP_XY,
  SORT_TURRET_WAIT,
  SORT_DROP_OPEN,
  SORT_RETURN_SAFE,
  SORT_TASK_DONE,
  SORT_ERROR,
  SORT_EMERGENCY_STOP,
  /* 2026-10-26 新增：X/Y 零点开关回零。追加在末尾，保证前面状态的
   * 枚举值不变（0x84 状态事件直接发这个值，上位机可能有映射）。 */
  SORT_XY_HOMING_SEARCH,
  SORT_XY_HOMING_BACKOFF,
  SORT_XY_HOMING_APPROACH
} SortingState_t;

/* 错误码与协议 ERROR 事件（0x83）的 detail 字段对应。 */
typedef enum
{
  SORT_ERR_NONE = 0,
  SORT_ERR_COORD = 1,          /* 坐标越界/非法任务 */
  SORT_ERR_XY_TIMEOUT = 2,     /* XY 运动超时 */
  SORT_ERR_T_TIMEOUT = 3,      /* T 轴未在时限内到位 */
  SORT_ERR_GRIP = 4,           /* 夹爪未检测到物块 */
  SORT_ERR_HOMING = 5,         /* 回零失败 */
  SORT_ERR_EMERGENCY = 6,      /* 急停 */
  SORT_ERR_PROTOCOL = 7,       /* 协议错误 */
  SORT_ERR_STOPPED = 8,        /* 软停止，需复位/回零 */
  SORT_ERR_NOT_READY = 9,      /* 系统未就绪 */
  SORT_ERR_MISSION_TIMEOUT = 10 /* 整任务超时 */
} SortError_t;

/* 在系统启动时清空任务并进入初始化状态。 */
void SortingTask_Init(void);
/* 在主循环中驱动非阻塞状态机；计时使用 HAL_GetTick()，单位 ms。 */
void SortingTask_Process(void);
/* 仅在 READY、安全且没有未完成任务时接收任务，成功返回 1。 */
uint8_t SortingTask_Submit(const SortMission_t *mission);
/* 软停止：停止所有轴并保持夹爪当前状态，进入 ERROR 等待复位。
 * 不会主动开爪，避免半空掉落物块。 */
void SortingTask_Stop(void);
/* 急停释放后复位故障并重新回零；成功返回 1。 */
uint8_t SortingTask_ResetFault(void);
/* 在 READY 时请求重新回零；成功返回 1。 */
uint8_t SortingTask_RequestHoming(void);
/* 读取当前状态，供上位机或调试代码使用。 */
SortingState_t SortingTask_GetState(void);
/* 读取最近一次错误码。 */
SortError_t SortingTask_GetError(void);
/* 是否还有正在执行（或已提交等待执行）的分拣任务。
 * 台架按键用它判断“不抢自动流程”：任务执行期间返回 1。 */
uint8_t SortingTask_HasMission(void);

/* 投放口（drop）坐标由 STM32 自己保存，不走上位机：
 * 初值取 sorting_config.h 的 SORT_DROP_X_STEP / SORT_DROP_Y_STEP，
 * 标定模式下可以用按键把当前轴位置写进来（RAM，掉电丢失）。
 * 保存失败（轴未回零、正在运动或超出软限位）返回 0。 */
uint8_t SortingTask_SaveDropX(void);
uint8_t SortingTask_SaveDropY(void);
int32_t SortingTask_GetDropX(void);
int32_t SortingTask_GetDropY(void);

#endif
