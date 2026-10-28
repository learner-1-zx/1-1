#ifndef __AXIS_HOME_H
#define __AXIS_HOME_H

#include "main.h"
#include <stdint.h>

/* X/Y 零点限位开关：普通机械限位开关，NO 接 IO、C 接 GND。
 * gpio.c 把 X_HOME(PD6)、Y_HOME(PD5) 都配成内部上拉输入：
 *   - 未压到开关（NO 断开）：上拉高电平（约 3.3 V）；
 *   - 压到零点开关（NO 闭合到 GND）：低电平（约 0 V）。
 * 所以 AXIS_HOME_ACTIVE_HIGH = 0（低有效）。
 * 断线也会读到高电平，表现是“回零超时”，不会误触发。
 *
 * 本文件只负责电平读取和消抖；回零状态机在 sorting_task.c。 */

typedef enum
{
  AXIS_HOME_X = 0,
  AXIS_HOME_Y,
  AXIS_HOME_COUNT
} AxisHomeId_t;

/* 1 = 高电平触发；普通限位开关 NO->IO、C->GND 时保持 0。 */
#define AXIS_HOME_ACTIVE_HIGH 0

/* 读取原始电平，不换算极性。 */
GPIO_PinState AxisHome_ReadRaw(AxisHomeId_t id);
/* 返回 1 表示开关已压到（按 AXIS_HOME_ACTIVE_HIGH 判定）。 */
uint8_t AxisHome_IsTriggered(AxisHomeId_t id);

/* 清空两轴的消抖计时；每次开始回零或切换回零阶段前调用。 */
void AxisHome_ResetDebounce(void);
/* 连续触发达到 debounce_ms 才返回 1，用于回零搜索。 */
uint8_t AxisHome_TriggeredDebounced(AxisHomeId_t id, uint32_t debounce_ms);
/* 连续释放达到 debounce_ms 才返回 1，用于退让后确认已离开开关。 */
uint8_t AxisHome_ReleasedDebounced(AxisHomeId_t id, uint32_t debounce_ms);

#endif /* __AXIS_HOME_H */
