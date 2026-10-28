#ifndef __DEBUG_UART_H
#define __DEBUG_UART_H

#include <stdint.h>

/* USART3（PB10/PB11，115200 8N1）调试输出。
 * 只提供“字符串 + 十进制整数”两种最简接口，不引入 printf / 浮点，
 * 避免为台架标定把整个 stdio 拉进固件。
 * 调用点由 sorting_config.h 的 SORT_DEBUG_UART 控制，正式比赛置 0 后
 * 这些代码不会被编译进去。 */
void DebugUart_Write(const char *text);
void DebugUart_WriteInt(const char *label, int32_t value);

#endif /* __DEBUG_UART_H */
