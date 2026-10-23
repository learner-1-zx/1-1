#ifndef __SERVO_GRIPPER_H
#define __SERVO_GRIPPER_H

#include "main.h"
#include <stdint.h>

/* 夹爪舵机信号：PA8 / TIM1_CH1 硬件 PWM。
 * TIM1 为 50 Hz、1 us 分辨率，所以下面的脉宽值就是 CCR1 数值（单位 us）。
 *
 * 最终选型：BDS300 数字舵机（7.4 V、10 kg·cm、0.13 s/60°）。
 * 2026-10-20 台架实测（从 1500 us 起，每次按键 10 us）：
 *   KEY1 11 次到闭合临界 = 1390 us；
 *   KEY0 30 次到张开临界 = 1800 us；
 *   总行程 410 us。若 BDS300 按 300°/2000 us 估算约 61.5°，
 *   与下面机构计算的 63° 基本吻合，说明齿条啮合和安装角正常。
 * 正式值各留 20 us 余量，避免自动流程长时间顶在机械死点：
 *   CLOSE = 1390 + 20 = 1410 us
 *   OPEN  = 1800 - 20 = 1780 us
 * 限幅收紧到实测机械临界 1390~1800 us，手动按键最多到临界、不会越过。
 *
 * 机械爪齿轮 20 齿、模数 0.8、分度圆 16 mm，齿距 2.513 mm；
 * 闭合到张开约 3.5 个齿距 = 8.8 mm，对应齿轮转角
 * 3.5 / 20 * 360 = 63°。
 * 实机方向相反时把 OPEN / CLOSE 两个宏对调即可。 */
#define SERVO_GRIPPER_CLOSE_PULSE_US  1410U
#define SERVO_GRIPPER_OPEN_PULSE_US   1780U
#define SERVO_GRIPPER_STEP_US           10U
#define SERVO_GRIPPER_MIN_PULSE_US    1390U
#define SERVO_GRIPPER_MAX_PULSE_US    1800U

/* 上电行为：ServoGripper_Init() 只复位驱动状态，不写 CCR1、不启动 TIM1_CH1，
 * 所以上电时 PA8 没有脉冲、舵机不上力。第一次 Open()/Close()/JumpTo()
 * 才由 ServoGripper_EnsurePwm() 启动 PWM 并直接输出目标脉宽；
 * 正常流程由 SORT_SYS_INIT 调 ServoGripper_Open() 把夹爪张开。
 * 当前 1410/1780 us、限幅 1390~1800 us 已按 2026-10-20 实测写入；
 * 标定过程见 SORTING_FLOW.md 第 8 节，换舵机后必须重新标定。 */
void ServoGripper_Init(void);
void ServoGripper_Open(void);
void ServoGripper_Close(void);
/* 台架手动控制：以小步长改变目标脉宽，用于逐步找机械临界。 */
void ServoGripper_StartOpening(void);
void ServoGripper_StartClosing(void);
void ServoGripper_StepOpening(void);
void ServoGripper_StepClosing(void);
/* 台架调试：立即跳到指定脉宽（直接写 CCR1，不做缓动），脉宽按限幅窗口裁剪。
 * closed 用于更新 ServoGripper_IsClosed() 的返回值。 */
void ServoGripper_JumpTo(uint16_t pulse_us, uint8_t closed);
void ServoGripper_Stop(void);
void ServoGripper_Process(void);
uint8_t ServoGripper_IsClosed(void);
/* 返回最近一次设置的脉宽，便于调试和主机仿真核对。 */
uint16_t ServoGripper_GetPulse(void);

#endif
