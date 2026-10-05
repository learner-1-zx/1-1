#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

/* USART2 使用的帧命令。当前业务主要使用 SORT_MISSION 和 STOP，
 * STATUS/HOME/RESET_FAULT 保留给后续协议扩展。 */
typedef enum
{
  PROTOCOL_CMD_STATUS = 0x01,
  PROTOCOL_CMD_HOME = 0x02,
  PROTOCOL_CMD_SORT_MISSION = 0x10,
  PROTOCOL_CMD_STOP = 0x13,
  PROTOCOL_CMD_RESET_FAULT = 0x30
} ProtocolCommand_t;

/* 分拣任务的线缆协议载荷；发送端和接收端需保持字节布局一致。 */
typedef struct
{
  uint16_t task_id;
  uint8_t class_id;
  uint8_t box_id;
  int32_t pick_x_step;
  int32_t pick_y_step;
  int32_t drop_step;
  uint8_t state;
} SortMission_t;

/* 初始化 USART2 单字节接收中断和环形缓冲区。 */
void Protocol_Init(void);
/* 从环形缓冲区解析完整帧，并分发有效命令。 */
void Protocol_Process(void);
/* 发送带 task_id 的事件通知帧。 */
void Protocol_SendEvent(uint8_t event, uint16_t task_id);
/* 对指定数据计算 Modbus 风格 CRC16。 */
uint16_t Protocol_Crc16(const uint8_t *data, uint16_t length);

#endif
