#include "protocol.h"
#include "main.h"
#include "usart.h"
#include "sorting_task.h"
#include "safety_manager.h"

#define PROTOCOL_MAX_PAYLOAD 32U
#define PROTOCOL_RX_BUFFER_SIZE 128U
#define PROTOCOL_VERSION 1U

/* 接收采用“中断写入、主循环读取”的单生产者环形缓冲区。 */
static uint8_t rx_byte;
static uint8_t rx_buffer[PROTOCOL_RX_BUFFER_SIZE];
static volatile uint16_t rx_write;
static volatile uint16_t rx_read;

static uint16_t Protocol_Crc16Update(uint16_t crc, uint8_t value)
{
  uint8_t i;
  crc ^= value;
  for (i = 0; i < 8; ++i)
  {
    crc = (crc & 1U) ? (crc >> 1U) ^ 0xA001U : crc >> 1U;
  }
  return crc;
}

uint16_t Protocol_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  while (length-- > 0U)
  {
    crc = Protocol_Crc16Update(crc, *data++);
  }
  return crc;
}

void Protocol_Init(void)
{
  rx_write = 0;
  rx_read = 0;
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t next;

  if (huart->Instance != USART2)
  {
    return;
  }

  next = (uint16_t)((rx_write + 1U) % PROTOCOL_RX_BUFFER_SIZE);
  /* 缓冲区满时丢弃新字节，保留已有数据，避免覆盖未解析帧。 */
  if (next != rx_read)
  {
    rx_buffer[rx_write] = rx_byte;
    rx_write = next;
  }
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

static uint8_t Protocol_ReadByte(uint8_t *value)
{
  if (rx_read == rx_write)
  {
    return 0;
  }
  *value = rx_buffer[rx_read];
  rx_read = (uint16_t)((rx_read + 1U) % PROTOCOL_RX_BUFFER_SIZE);
  return 1;
}

static uint16_t Protocol_ReadU16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int32_t Protocol_ReadI32(const uint8_t *data)
{
  uint32_t value = (uint32_t)data[0] |
                   ((uint32_t)data[1] << 8) |
                   ((uint32_t)data[2] << 16) |
                   ((uint32_t)data[3] << 24);
  return (int32_t)value;
}

static void Protocol_HandleFrame(uint8_t command, const uint8_t *payload,
                                 uint8_t length)
{
  SortMission_t mission;
  SortError_t error;

  if (command == PROTOCOL_CMD_SORT_MISSION)
  {
    /* 只要求前 17 个有效字节；上位机若按结构体 padding 发 20 字节，
     * 多出的尾部字节直接忽略。 */
    if (length < SORT_MISSION_WIRE_SIZE)
    {
      return;
    }
    mission.task_id = Protocol_ReadU16(&payload[0]);
    mission.class_id = payload[2];
    mission.box_id = payload[3];
    mission.pick_x_step = Protocol_ReadI32(&payload[4]);
    mission.pick_y_step = Protocol_ReadI32(&payload[8]);
    mission.drop_step = Protocol_ReadI32(&payload[12]);
    mission.state = payload[16];

    if (SafetyManager_IsSafe() == 0U)
    {
      Protocol_SendError(mission.task_id, (uint8_t)SORT_ERR_EMERGENCY);
      return;
    }

    if (SortingTask_Submit(&mission) != 0U)
    {
      Protocol_SendEvent(PROTOCOL_EVENT_ACK, mission.task_id);
    }
    else
    {
      error = SortingTask_GetError();
      if (error == SORT_ERR_NONE)
      {
        Protocol_SendEvent(PROTOCOL_EVENT_BUSY, mission.task_id);
      }
      else
      {
        Protocol_SendError(mission.task_id, (uint8_t)error);
      }
    }
    return;
  }

  if (command == PROTOCOL_CMD_STOP)
  {
    /* 停轴、保持夹爪并进入 ERROR，需 RESET_FAULT 后重新回零。 */
    SortingTask_Stop();
    return;
  }

  if (command == PROTOCOL_CMD_RESET_FAULT)
  {
    if (SortingTask_ResetFault() != 0U)
    {
      Protocol_SendEvent(PROTOCOL_EVENT_RESET_ACK, 0U);
    }
    return;
  }

  if (command == PROTOCOL_CMD_HOME)
  {
    if (SortingTask_RequestHoming() != 0U)
    {
      Protocol_SendEvent(PROTOCOL_EVENT_HOME_ACK, 0U);
    }
    return;
  }

  if (command == PROTOCOL_CMD_STATUS)
  {
    Protocol_SendStatus((uint8_t)SortingTask_GetState(),
                        (uint8_t)SortingTask_GetError());
  }
}

void Protocol_Process(void)
{
  /* 帧格式：AA 55 | version | command | length | payload | CRC16(低字节在前)。 */
  static uint8_t frame[2U + 1U + 1U + 1U + PROTOCOL_MAX_PAYLOAD + 2U];
  static uint8_t frame_length;
  static uint8_t payload_length;
  uint8_t value;
  uint16_t expected_crc;
  uint16_t actual_crc;

  while (Protocol_ReadByte(&value))
  {
    if (frame_length == 0U && value != 0xAAU)
    {
      continue;
    }
    frame[frame_length++] = value;
    if (frame_length == 2U && value != 0x55U)
    {
      frame_length = 0U;
    }
    if (frame_length == 5U)
    {
      payload_length = frame[4];
      if (payload_length > PROTOCOL_MAX_PAYLOAD)
      {
        frame_length = 0U;
      }
    }
    if (frame_length >= 5U &&
        frame_length == (uint8_t)(7U + payload_length))
    {
      expected_crc = (uint16_t)frame[5U + payload_length] |
                     ((uint16_t)frame[6U + payload_length] << 8U);
      actual_crc = Protocol_Crc16(&frame[2], (uint16_t)(3U + payload_length));
      if (frame[2] == PROTOCOL_VERSION && expected_crc == actual_crc)
      {
        Protocol_HandleFrame(frame[3], &frame[5], payload_length);
      }
      frame_length = 0U;
    }
  }
}

void Protocol_SendEvent(uint8_t event, uint16_t task_id)
{
  uint8_t frame[9];
  uint16_t crc;

  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = PROTOCOL_VERSION;
  frame[3] = event;
  frame[4] = 2;
  frame[5] = (uint8_t)task_id;
  frame[6] = (uint8_t)(task_id >> 8);
  crc = Protocol_Crc16(&frame[2], 5);
  frame[7] = (uint8_t)crc;
  frame[8] = (uint8_t)(crc >> 8);
  HAL_UART_Transmit(&huart2, frame, sizeof(frame), 20);
}

void Protocol_SendError(uint16_t task_id, uint8_t error)
{
  uint8_t frame[10];
  uint16_t crc;

  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = PROTOCOL_VERSION;
  frame[3] = PROTOCOL_EVENT_ERROR;
  frame[4] = 3U;
  frame[5] = (uint8_t)task_id;
  frame[6] = (uint8_t)(task_id >> 8);
  frame[7] = error;
  crc = Protocol_Crc16(&frame[2], 6U);
  frame[8] = (uint8_t)crc;
  frame[9] = (uint8_t)(crc >> 8);
  HAL_UART_Transmit(&huart2, frame, (uint16_t)sizeof(frame), 20U);
}

void Protocol_SendStatus(uint8_t state, uint8_t error)
{
  uint8_t frame[9];
  uint16_t crc;

  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = PROTOCOL_VERSION;
  frame[3] = PROTOCOL_EVENT_STATUS;
  frame[4] = 2U;
  frame[5] = state;
  frame[6] = error;
  crc = Protocol_Crc16(&frame[2], 5U);
  frame[7] = (uint8_t)crc;
  frame[8] = (uint8_t)(crc >> 8);
  HAL_UART_Transmit(&huart2, frame, (uint16_t)sizeof(frame), 20U);
}
