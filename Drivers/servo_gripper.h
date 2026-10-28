#ifndef __SERVO_GRIPPER_H
#define __SERVO_GRIPPER_H

#include "main.h"
#include <stdint.h>

/* 夹爪舵机：BDS300（7.4 V、10 kg·cm、0.13 s/60°），信号 PA8 / TIM1_CH1，
 * 50 Hz、1 us 分辨率，所以下面的脉宽值就是 CCR1 数值（单位 us）。
 *
 * 2026-10-25 重新标定（上电 1500 us，每步 50 us）：
 *   KEY0 5 次到张开临界 = 1750 us；KEY1 2 次到闭合临界 = 1400 us；行程 350 us。
 * 正式值各留 20 us 余量，避免自动流程长时间顶在机械死点：
 *   CLOSE = 1400 + 20 = 1420 us
 *   OPEN  = 1750 - 20 = 1730 us
 * 限幅收紧到实测机械临界 1400~1750 us，手动按键最多到临界、不会越过。
 * ！！CLOSE 绝不能小于 1400：曾误改成 1340，闭合时直接顶死在机械限位上，
 *   舵机堵转嗡嗡响、机构被顶变形，随后整个夹爪行程都乱了。
 * 实机方向相反时把 OPEN / CLOSE 两个宏对调即可。
 *
 * 机械爪齿轮 20 齿、模数 0.8、分度圆 16 mm；闭合到张开约 3.5 个齿距
 * = 8.8 mm，对应齿轮转角 3.5 / 20 * 360 = 63°。 */

/* ---------------- 标定模式开关（默认 0 = 正式） ----------------
 * 1 = 标定：上电就停在 START_PULSE 并保持，Open()/Close() 只改标志不动脉宽
 *     （状态机抢不走夹爪），限幅放宽到 CALIB_MIN~CALIB_MAX，KEY0/KEY1 每次
 *     ±STEP_US 手动找机械两端。重新标定时把它改回 1，标定完必须改回 0，
 *     否则自动流程“夹爪完全不动”。
 * 0 = 正式：上电不上力，第一次 Open()/Close() 才启动 PWM 并缓动到目标。
 * 下面的标定分支代码保留，不删除。 */
#ifndef SERVO_GRIPPER_CALIB_MODE
#define SERVO_GRIPPER_CALIB_MODE        0
#endif

/* 标定窗口：舵机标准行程约 500~2500 us，这里取 800~2200。 */
#define SERVO_GRIPPER_CALIB_MIN_US     800U
#define SERVO_GRIPPER_CALIB_MAX_US    2200U

/* 断力调试开关：1 = 完全不输出脉冲（PA8 保持低电平，舵机无扭矩），用于手动
 * 检查齿条是否卡死/滑齿，或与“拔信号线”对照；排查完必须改回 0。 */
#ifndef SERVO_GRIPPER_DISABLE_OUTPUT
#define SERVO_GRIPPER_DISABLE_OUTPUT    0
#endif

/* 正式值：2026-10-25 实测闭合临界 1400 us、张开临界 1750 us，各留 20 us 余量。 */
#define SERVO_GRIPPER_CLOSE_PULSE_US  1420U
#define SERVO_GRIPPER_OPEN_PULSE_US   1730U
/* 台架点动步长（KEY0 +、KEY1 -）；想要更高精度可临时改 10。 */
#define SERVO_GRIPPER_STEP_US           50U

#if SERVO_GRIPPER_CALIB_MODE
/* 标定模式放宽到标定窗口，按键才能触到真正的机械临界。 */
#define SERVO_GRIPPER_MIN_PULSE_US    SERVO_GRIPPER_CALIB_MIN_US
#define SERVO_GRIPPER_MAX_PULSE_US    SERVO_GRIPPER_CALIB_MAX_US
#else
/* 正式限幅：实测机械临界 1400~1750 us，按键最多到临界、不会越过。 */
#define SERVO_GRIPPER_MIN_PULSE_US    1400U
#define SERVO_GRIPPER_MAX_PULSE_US    1750U
#endif

/* 软启动：START 是上电/首次启动的第一帧脉宽，取窗口中间，不落在两端；
 * Process() 每 SLEW_PERIOD_MS 走 SLEW_US，350 us 行程约 7 拍 = 140 ms，
 * 接近 BDS300 空载 0.13 s/60°，不会撞端点。
 * ！！SLEW_US 不要调到 10：全行程会变成 700 ms，超过状态机的
 *   SORT_GRIP_OPEN_MS / SORT_GRIP_CLOSE_MS（400 ms），夹爪没到位就动 Z 轴。 */
#define SERVO_GRIPPER_START_PULSE_US  1500U
#define SERVO_GRIPPER_SLEW_US          50U
#define SERVO_GRIPPER_SLEW_PERIOD_MS    20U

/* 上电行为：
 *  1. Init() 只复位状态（脉宽 = START_PULSE），不写 CCR1、不启动 TIM1_CH1，
 *     并把 PA8 拉成推挽低电平（TIM1 未使能时 AF 引脚高阻，浮空会被噪声驱动）；
 *  2. 第一次 Open()/Close() 由 EnsurePwm() 切回 AF1 并启动 PWM；
 *  3. Process() 缓动到目标；JumpTo() 一次写到位，只给台架按键/标定用。
 * 标定过程见 SORTING_FLOW.md 第 8 节，换舵机或重装舵机臂后必须重新标定。 */
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
/* 									返回最近一次设置的脉宽，便于调试和主机仿真核对。 */
uint16_t ServoGripper_GetPulse(void);

#endif
