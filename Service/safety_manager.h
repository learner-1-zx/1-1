#ifndef __SAFETY_MANAGER_H
#define __SAFETY_MANAGER_H

#include <stdint.h>

/* 故障一旦锁存，需按产品策略增加复位流程后才能恢复正常运行。 */
typedef enum
{
  SAFETY_FAULT_NONE = 0,
  SAFETY_FAULT_EMERGENCY,
  /* 保留给以后的限位方案；当前不会产生该故障。 */
  SAFETY_FAULT_LIMIT
} SafetyFault_t;

/* 初始化故障状态。 */
void SafetyManager_Init(void);
/* 轮询急停输入，发现故障立即停止电机。限位功能后续单独接入。 */
void SafetyManager_Process(void);
/* 返回当前是否允许执行运动任务。 */
uint8_t SafetyManager_IsSafe(void);
/* 返回具体故障类型，便于状态上报和调试。 */
SafetyFault_t SafetyManager_GetFault(void);

#endif
