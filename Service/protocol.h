#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

/* USART2 使用的帧命令：分拣任务、停止、状态查询、回零和故障复位。
 * 帧版本仍为 1；应答事件使用 0x80～0x86。 */
typedef enum
{
  PROTOCOL_CMD_STATUS = 0x01,
  PROTOCOL_CMD_HOME = 0x02,
  PROTOCOL_CMD_SORT_MISSION = 0x10,
  PROTOCOL_CMD_STOP = 0x13,
  PROTOCOL_CMD_RESET_FAULT = 0x30
} ProtocolCommand_t;

/* 事件帧命令字。 */
#define PROTOCOL_EVENT_ACK          0x80U
#define PROTOCOL_EVENT_BUSY         0x81U
#define PROTOCOL_EVENT_DONE         0x82U
#define PROTOCOL_EVENT_ERROR        0x83U
#define PROTOCOL_EVENT_STATUS       0x84U
#define PROTOCOL_EVENT_RESET_ACK    0x85U
#define PROTOCOL_EVENT_HOME_ACK     0x86U

/* 分拣任务的线缆协议载荷；发送端和接收端需保持字节布局一致。
 *
 * 注意：结构体在 ARM 上会因 4 字节对齐产生尾部填充（sizeof 可能为 20），
 * 而线缆上只传下面 17 个有效字节。接收端按固定偏移解析，不直接
 * memcpy sizeof，以免上位机按紧凑结构发送 17 字节时长度不匹配。
 *
 * 偏移：task_id(0,2) class_id(2) box_id(3)
 *       pick_x_step(4,4) pick_y_step(8,4) drop_step(12,4) state(16) */
#define SORT_MISSION_WIRE_SIZE 17U

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
/* 发送带 task_id 的事件通知帧（2 字节载荷）。 */
void Protocol_SendEvent(uint8_t event, uint16_t task_id);
/* 发送错误事件：0xAA 55 ver 0x83 len=3 task_id(2) error(1) CRC16。 */
void Protocol_SendError(uint16_t task_id, uint8_t error);
/* 发送状态事件：载荷为 state + error_code。 */
void Protocol_SendStatus(uint8_t state, uint8_t error);
/* 对指定数据计算 Modbus 风格 CRC16。 */
uint16_t Protocol_Crc16(const uint8_t *data, uint16_t length);

#endif
