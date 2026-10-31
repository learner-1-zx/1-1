#ifndef __BENCH_CONSOLE_H
#define __BENCH_CONSOLE_H

#include <stdint.h>

/* USART3（PB10/PB11，115200 8N1）台架文字控制台。
 *
 * 目的：板载只有 KEY0~KEY3 四个按键，却要覆盖夹爪、Z 轴、X 轴点动和
 * 投放口标定；接线/标定时很难同时按两个键或精确按住 1 s。控制台把
 * 电脑串口助手当成“虚拟按键”：
 *   - 收到 0~3 就模拟 KEY0~KEY3 的“按下 → 释放”；
 *   - 收到 A~D 模拟对应的“长按”（默认 1500 ms，可触发 KEY0 存投放口）；
 * 虚拟按键只提供“原始电平”，后面的 20 ms 消抖、安全/无任务门控、
 * 按住连发和 KEY0 长按判据全部复用 main.c 的原有逻辑，所以串口按键
 * 和真实按键走同一条代码路径，不会出现“测的是另一套逻辑”。
 *
 * 编译开关：sorting_config.h 的 SORT_DEBUG_CONSOLE（正式比赛置 0）。
 * 接收：huart3 由 MX_USART3_UART_Init() 初始化；单字节接收中断在
 * protocol.c 的 HAL_UART_RxCpltCallback 里分发到 BenchConsole_RxComplete()，
 * 主循环调用 BenchConsole_Process() 解析命令（不在中断里做事）。
 *
 * 命令（单字符，直接用串口助手“发送”即可，不用加回车）：
 *   0 1 2 3  短按 KEY0~KEY3（100 ms，够 20 ms 消抖、短于 1 s 长按）
 *   A B C D  长按 KEY0~KEY3（1500 ms，> 1 s，可测投放口长按保存）
 *   x        立即释放全部虚拟按键
 *   s        打印状态、轴位置、Z/夹爪脉宽、四个按键的原始电平
 *   h 或 ?   打印帮助
 * 按键动作仍受“安全 + 无任务”门控：任务执行期间按 0~3 不会动作，
 * 与真实按键完全一致（这时可以按 s 看 STATE 判断卡在哪一步）。 */

#define BENCH_CONSOLE_KEY_COUNT    4U
/* 短按保持时间：必须大于 SORT_BENCH_KEY_DEBOUNCE_MS（20 ms），
 * 同时明显小于 SORT_DROP_KEY_LONG_PRESS_MS（1000 ms）。 */
#define BENCH_CONSOLE_TAP_MS       100U
/* 长按保持时间：必须大于 SORT_DROP_KEY_LONG_PRESS_MS（1000 ms）。 */
#define BENCH_CONSOLE_HOLD_MS      1500U

/* 主机仿真（tools/hostsim）只链接驱动层，不链接 sorting_task / stepper_motor /
 * servo_* 等应用模块，编译时加 -DBENCH_CONSOLE_NO_STATUS=1 裁掉 's' 状态打印，
 * 这样 test 12 仍能验证命令解析和虚拟按键时序。固件里保持 0。 */
#ifndef BENCH_CONSOLE_NO_STATUS
#define BENCH_CONSOLE_NO_STATUS    0
#endif

/* 清空缓冲、释放虚拟按键并挂上 USART3 单字节接收中断。 */
void BenchConsole_Init(void);
/* 主循环调用：解析已收到的字符并执行虚拟按键/状态打印。 */
void BenchConsole_Process(void);
/* USART3 接收完成（由 protocol.c 的 HAL_UART_RxCpltCallback 调用）。 */
void BenchConsole_RxComplete(void);
/* 直接注入一个命令字符：给主机仿真用，固件里不要调用。 */
void BenchConsole_FeedByte(uint8_t byte);
/* 查询虚拟按键：返回 1 表示当前是控制台注入的电平，
 * *pressed = 1 表示“逻辑按下”（对应 GPIO 低电平）。
 * index 越界或没有注入时返回 0，调用方应回退到 HAL_GPIO_ReadPin()。 */
uint8_t BenchConsole_GetVirtualKey(uint8_t index, uint8_t *pressed);

#endif /* __BENCH_CONSOLE_H */
