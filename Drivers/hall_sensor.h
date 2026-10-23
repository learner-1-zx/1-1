#ifndef __HALL_SENSOR_H
#define __HALL_SENSOR_H

#include "main.h"

/* 转盘零位霍尔：实际接 PD12（CubeMX 网络名 T_HOME），不是 PD11。
 *
 * 器件 datasheet：4.5～24 V 供电、开集电极输出、低有效：
 *   - 无磁铁 / 断线（内部上拉）：PD12 高电平（约 3.3 V）；
 *   - 磁铁靠近（B ≥ BOP，7～23 mT）：PD12 被拉低（约 0 V）。
 * 因此 gpio.c 把 PD12 配成 GPIO_PULLUP，本文件
 * HALL_SENSOR_ACTIVE_HIGH = 0。10 ms 软件消抖足够。
 *
 * 注意：磁铁的 N/S 极是机械问题，装反了传感器不会动作，
 * 需要把磁铁翻面，不是改这里的电平极性。
 * PD11 已改名 SPARE_PD11，留作后续扩展，不再当作霍尔输入。 */
#define HALL_SENSOR_GPIO_Port   T_HOME_GPIO_Port
#define HALL_SENSOR_Pin         T_HOME_Pin
#define HALL_SENSOR_ACTIVE_HIGH 0

/* 返回 PD12/T_HOME 原始电平，不换算极性。 */
GPIO_PinState HallSensor_ReadRaw(void);

/* 返回 1 表示磁铁靠近（按 HALL_SENSOR_ACTIVE_HIGH 判定），0 表示未触发。 */
uint8_t HallSensor_IsTriggered(void);

#endif /* __HALL_SENSOR_H */
