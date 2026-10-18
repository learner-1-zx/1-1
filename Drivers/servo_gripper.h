#ifndef __SERVO_GRIPPER_H
#define __SERVO_GRIPPER_H

#include "main.h"
#include <stdint.h>

/* 夹爪舵机信号：PB8（普通 GPIO，软件 PWM）。
 * PA9 虽可复用为 TIM1_CH2，但 PCB 未引出，代码中不再使用该引脚。
 *
 * PB8 在 STM32F407 上没有用作定时器输出，因此借用 TIM1_CH2 作为
 * “无引脚输出”的内部比较通道：
 *   - TIM1 更新中断（50 Hz）先把当前脉宽写入 CCR2，再把 PB8 拉高；
 *   - TIM1_CH2 比较中断把 PB8 拉低。
 * TIM1 为 50 Hz、1 us 分辨率，所以下面的脉宽值就是 CCR2 数值（单位 us）。
 *
 * 最终选型：BDS300 数字舵机（7.4 V、10 kg·cm、0.13 s/60°）。
 * 重要：BDS300 的 PWM 频率和脉宽范围尚未实测；下面的 1170/1870 us
 * 仍是 SG90 试装时的临时值，仅用于占位，不能作为 BDS300 的最终标定。
 * 上电不再执行舵机自检；完成 BDS300 空载标定前，不要直接接 BDS300。
 *
 * 机械爪齿轮 20 齿、模数 0.8、分度圆 16 mm，齿距 2.513 mm；
 * 闭合到张开约 3.5 个齿距 = 8.8 mm，对应齿轮转角
 * 3.5 / 20 * 360 = 63°。63° 是机械行程，与舵机型号无关；
 * 对应的脉宽差要按 BDS300 实测结果重新计算。
 * 实机方向相反时把 OPEN / CLOSE 两个宏对调即可。 */
#define SERVO_GRIPPER_CLOSE_PULSE_US  1170U
#define SERVO_GRIPPER_OPEN_PULSE_US   1870U
#define SERVO_GRIPPER_MIN_PULSE_US     500U
#define SERVO_GRIPPER_MAX_PULSE_US    2500U

/* 启动 TIM1_CH2 内部比较中断、TIM1 更新中断和 PB8 软件 PWM；
 * 上电默认输出张开脉宽，不阻塞、不自动往返；开合由状态机控制。
 * 开合脉宽需结合实际舵机重新标定。 */
void ServoGripper_Init(void);
void ServoGripper_Open(void);
void ServoGripper_Close(void);
uint8_t ServoGripper_IsClosed(void);
/* 返回最近一次设置的脉宽，便于调试和主机仿真核对。 */
uint16_t ServoGripper_GetPulse(void);
/* 由 TIM1 更新中断调用：把当前脉宽写入 CCR2，再把 PB8 拉高。 */
void ServoGripper_TimerUpdateIsr(void);

#endif
