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
extern uint32_t     host_pwm_start_ch2;  /* TIM1_CH2 内部比较：夹爪软件 PWM（PD8） */
extern uint32_t     host_pwm_start_ch1;  /* TIM1_CH1：Z 轴舵机（PA8） */
extern uint8_t      host_servo_level;    /* 夹爪软件 PWM 信号 PD8 的电平 */

/* 外设句柄实体：仿真时用普通内存代替寄存器。 */
extern TIM_TypeDef host_tim1_regs;
extern TIM_TypeDef host_tim2_regs;
extern TIM_TypeDef host_tim6_regs;
extern SPI_TypeDef host_spi2_regs;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern SPI_HandleTypeDef hspi2;

/* 清空所有记录。 */
void host_reset(void);
/* 返回第 index 次传输中第 offset 个字节，越界返回 0。 */
uint8_t host_xfer_byte(uint32_t index, uint16_t offset);

#endif
