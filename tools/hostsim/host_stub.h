/* 主机仿真桩：把 HAL 的 GPIO / SPI / 延时 / PWM 调用记录下来，
 * 用于在没有硬件的情况下复核 lcd_st7789.c、servo_z.c 和
 * servo_gripper.c 的行为。 */
#ifndef HOST_STUB_H
#define HOST_STUB_H

#include "main.h"
#include <stdint.h>

#define HOST_TRACE_MAX 400000U
#define HOST_XFER_MAX  8192U

typedef struct
{
  uint8_t  dc;     /* 本次传输时 DC 引脚电平：0 = 命令，1 = 数据 */
  uint32_t start;  /* 在 host_bytes 中的起始下标 */
  uint16_t len;    /* 传输字节数 */
} host_xfer_t;

extern uint8_t      host_bytes[HOST_TRACE_MAX];
extern uint32_t     host_byte_count;
extern host_xfer_t  host_xfers[HOST_XFER_MAX];
extern uint32_t     host_xfer_count;
extern uint32_t     host_cs_low;
extern uint32_t     host_cs_high;
extern uint32_t     host_rst_low;
extern uint32_t     host_rst_high;
extern uint8_t      host_dc_level;
extern uint8_t      host_blk_level;
extern uint32_t     host_delay_ms;
extern uint32_t     host_tick_ms;          /* HAL_GetTick() 返回值 */
extern uint32_t     host_error_handler_calls;
extern uint32_t     host_pwm_z_start;      /* TIM2_CH2：Z 轴舵机（PA1） */
extern uint32_t     host_pwm_grip_start;   /* TIM1_CH1：夹爪舵机（PA8） */
extern uint32_t     host_pwm_start_it;     /* HAL_TIM_PWM_Start_IT 调用次数（应为 0） */
extern uint8_t      host_hall_level;       /* 仿真用 PD12/T_HOME 电平：0 = 低（有磁铁），1 = 高 */
extern uint8_t      host_home_level[2];    /* 仿真用零点开关电平：[0]=X/PD6，[1]=Y/PD5；0 = 低（压到），1 = 高 */
extern uint32_t     host_gpio_init_calls;  /* HAL_GPIO_Init 调用次数 */
extern uint32_t     host_gpio_pa8_mode;    /* PA8 最近一次 HAL_GPIO_Init 的 Mode */
extern uint32_t     host_gpio_pa8_pull;    /* PA8 最近一次 HAL_GPIO_Init 的 Pull */
extern uint32_t     host_gpio_pa8_af;      /* PA8 最近一次 HAL_GPIO_Init 的 Alternate */
extern uint8_t      host_gpio_pa8_level;   /* PA8 最近一次 HAL_GPIO_WritePin 的电平 */
extern uint32_t     host_gpio_pa8_mode_at_pwm_start; /* 启动 TIM1_CH1 瞬间的 PA8 模式 */

/* USART 桩：记录 USART3 发出的调试/控制台文本，以及 HAL_UART_Receive_IT
 * 的挂载次数（验证 BenchConsole_Init 有没有把接收中断挂上）。 */
#define HOST_UART_TX_MAX 4096U
extern uint8_t      host_uart_tx[HOST_UART_TX_MAX];
extern uint32_t     host_uart_tx_count;
extern uint32_t     host_uart_rx_arm_calls;

/* 台架运动指令（bench_motion.c）只关心“有没有自动任务在执行”。
 * 主机仿真不链接 sorting_task.c，用这个变量模拟；
 * 置 1 等价于 SortingTask_HasMission() == 1。 */
extern uint8_t      host_mission_active;

/* 外设句柄实体：仿真时用普通内存代替寄存器。 */
extern TIM_TypeDef host_tim1_regs;
extern TIM_TypeDef host_tim2_regs;
extern TIM_TypeDef host_tim6_regs;
extern SPI_TypeDef host_spi2_regs;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* 清空所有记录。 */
void host_reset(void);
/* 返回第 index 次传输中第 offset 个字节，越界返回 0。 */
uint8_t host_xfer_byte(uint32_t index, uint16_t offset);
/* USART3 已发送文本（NUL 结尾），用于断言控制台/调试输出。 */
const char *host_uart_text(void);

#endif
