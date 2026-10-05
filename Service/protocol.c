#include "protocol.h"
#include "main.h"
#include "usart.h"
#include "sorting_task.h"
#include "safety_manager.h"
#include "stepper_motor.h"
#include <string.h>

#define PROTOCOL_MAX_PAYLOAD 32U
#define PROTOCOL_RX_BUFFER_SIZE 128U
#define PROTOCOL_VERSION 1U

/* 接收采用“中断写入、主循环读取”的单生产者环形缓冲区。 */
static uint8_t rx_byte;
static uint8_t rx_buffer[PROTOCOL_RX_BUFFER_SIZE];
static volatile uint16_t rx_write;
static uint16_t rx_read;

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

static void Protocol_HandleFrame(uint8_t command, const uint8_t *payload,
                                 uint8_t length)
{
  SortMission_t mission;

  /* 任务载荷长度必须严格匹配结构体，防止截断或越界解释。 */
  if (command == PROTOCOL_CMD_SORT_MISSION &&
      length == sizeof(SortMission_t))
  {
    memcpy(&mission, payload, sizeof(mission));
    if (SafetyManager_IsSafe() && SortingTask_Submit(&mission))
    {
      Protocol_SendEvent(0x80, mission.task_id);
    }
    return;
  }

  if (command == PROTOCOL_CMD_STOP)
  {
    StepperMotor_StopAll();
    SortingTask_Stop();
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
