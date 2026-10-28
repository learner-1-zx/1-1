#include "debug_uart.h"
#include "usart.h"
#include <string.h>

/* 调试串口：huart3 已在 main.c 的 MX_USART3_UART_Init() 里初始化，
 * 这里只做阻塞发送。115200 下每行十几个字节约 1~2 ms，
 * 只在回零完成/标定按键这种一次性事件里调用，不影响主循环节拍。 */
#define DEBUG_UART_TIMEOUT_MS 50U

static void DebugUart_Send(const uint8_t *data, uint16_t length)
{
  if ((data == 0) || (length == 0U))
  {
    return;
  }
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, length,
                          DEBUG_UART_TIMEOUT_MS);
}

void DebugUart_Write(const char *text)
{
  size_t length;

  if (text == 0)
  {
    return;
  }
  length = strlen(text);
  if (length > 0xFFFFU)
  {
    length = 0xFFFFU;
  }
  DebugUart_Send((const uint8_t *)text, (uint16_t)length);
}

void DebugUart_WriteInt(const char *label, int32_t value)
{
  char line[24];
  char digits[11];
  uint32_t magnitude;
  uint16_t index = 0U;
  uint8_t count = 0U;

  if (label != 0)
  {
    DebugUart_Write(label);
  }

  /* INT32_MIN 直接取负会溢出，用无符号减法得到绝对值。 */
  if (value < 0)
  {
    line[index++] = '-';
    magnitude = 0U - (uint32_t)value;
  }
  else
  {
    magnitude = (uint32_t)value;
  }

  do
  {
    digits[count++] = (char)('0' + (magnitude % 10U));
    magnitude /= 10U;
  } while ((magnitude > 0U) && (count < (uint8_t)sizeof(digits)));

  while (count > 0U)
  {
    line[index++] = digits[--count];
  }
  line[index++] = '\r';
  line[index++] = '\n';
  DebugUart_Send((const uint8_t *)line, index);
}
