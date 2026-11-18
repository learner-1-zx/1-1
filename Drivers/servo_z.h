#ifndef __SERVO_Z_H
#define __SERVO_Z_H

#include "main.h"
#include <stdint.h>

/* Z 轴舵机使用 PA1 / TIM2_CH2 输出 PWM。
 * TIM2 位于 APB1，定时器时钟 84 MHz：
 *   Prescaler 83 -> 84 MHz / (83 + 1) = 1 MHz，计数分辨率 1 us；
 *   Period 19999  -> 1 MHz / (19999 + 1) = 50 Hz。
 * 因此比较寄存器数值就是脉宽微秒数，可直接按 us 写入。
 * 注意：PA1 的板载 LED 已拆除，可以正常作 PWM 输出。 */

/* ---------------- Z 轴模式（2026-11-08 修改） ----------------
 * 用户实测：Z 轴最低工作位为 2150 us；不再使用 1850 us 的 zmid。
 * 上电直接到 ztop（1200 us）。
 * 0 = 正式（当前）：上电到 1200 us，Travel/Approach/Grasp 正常控制 Z；
 * 1 = 台架手动：上电停 ZSERVO_CALIB_START_US，Travel/Approach/Grasp 不改脉宽。
 * 视觉/自动流程联调时必须是 0。 */
#ifndef ZSERVO_CALIB_MODE
#define ZSERVO_CALIB_MODE              0
#endif

/* 工作窗口：最上面 1200 us，最下面 2150 us（用户 2026-11-08 标定）。
 * 脉宽小 = 上，脉宽大 = 下。 */
#define ZSERVO_CALIB_MIN_US    1200U
#define ZSERVO_CALIB_MAX_US    2150U
/* 台架模式上电位置：直接放最上面 1200 us（不再用 1850 中点）。
 * 正式模式由 ZServo_Init() 走 ZSERVO_PULSE_TRAVEL_US=1200。 */
#define ZSERVO_CALIB_START_US  1200U
/* 逐档模式下 KEY2/KEY3 每次改变的脉宽；想更细可临时改成 10。 */
#define ZSERVO_STEP_US           50U

/* ---------------- 一键到位目标（2026-11-08） ----------------
 *   KEY3 / ztop → 1200 us：最上面；
 *   KEY2 / zbottom → 2150 us：最低工作位（不再用 2500）。
 * 脉宽小 = 上，脉宽大 = 下。 */
#define ZSERVO_PULSE_TOP_US      1200U
#define ZSERVO_PULSE_BOTTOM_US   2150U

/* ---------------- 台架按键分工 ----------------
 * 只在 SORT_BENCH_KEY_Z_SERVO=1（sorting_config.h）且安全、无任务时生效：
 *   KEY2：1（当前）= 按下一次到最低位 ZSERVO_PULSE_BOTTOM_US（2150 us），不连发；
 *         0 = 每次 +ZSERVO_STEP_US，按住连发；
 *   KEY3：1（当前）= 按下一次到最上面 ZSERVO_PULSE_TOP_US（1200 us），不连发；
 *         0 = 每次 -ZSERVO_STEP_US，按住连发。 */
#ifndef ZSERVO_BENCH_KEY2_JUMP
#define ZSERVO_BENCH_KEY2_JUMP    1
#endif
#ifndef ZSERVO_BENCH_KEY3_JUMP
#define ZSERVO_BENCH_KEY3_JUMP    1
#endif

#if ZSERVO_CALIB_MODE
/* 标定模式放宽到标定窗口。 */
#define ZSERVO_PULSE_MIN_US    ZSERVO_CALIB_MIN_US
#define ZSERVO_PULSE_MAX_US    ZSERVO_CALIB_MAX_US
#else
/* 正式限幅 = 1200~2150 us（2026-11-08 标定）。 */
#define ZSERVO_PULSE_MIN_US   1200U
#define ZSERVO_PULSE_MAX_US   2150U
#endif

/* 三个工作高度（2026-11-08）：
 *   TRAVEL   - 上电/运输高度 = 最上面 1200 us；
 *   APPROACH - 不再使用中间高度，等同 TRAVEL=1200；
 *   GRASP    - 最低抓取高度 = 2150 us。
 * 脉宽小 = 上、脉宽大 = 下。 */
#define ZSERVO_PULSE_TRAVEL_US    1200U
#define ZSERVO_PULSE_APPROACH_US  1200U
#define ZSERVO_PULSE_GRASP_US     2150U

/* 兼容旧接口：safe/up 都指运输高度，down 指抓取高度。 */
#define ZSERVO_PULSE_SAFE_US      ZSERVO_PULSE_TRAVEL_US
#define ZSERVO_PULSE_UP_US        ZSERVO_PULSE_TRAVEL_US
#define ZSERVO_PULSE_DOWN_US      ZSERVO_PULSE_GRASP_US

/* 启动 TIM2_CH2 的 PWM 并把舵机放到运输高度。 */
void ZServo_Init(void);
/* 设置脉宽；超出 MIN/MAX 范围时自动限幅，避免直接给到极限角度。 */
void ZServo_SetPulse(uint16_t pulse);
/* 台架逐档点动（SORT_BENCH_KEY_Z_SERVO=1 且 ZSERVO_BENCH_KEY2/3_JUMP=0 时用）：
 * direction > 0 脉宽每调用一次增大 ZSERVO_STEP_US，< 0 则减小，
 * 到标定窗口边界后停住；不做缓动，方便一档一档对齿。 */
void ZServo_StepPulse(int8_t direction);
/* 台架一键到位（SORT_BENCH_KEY_Z_SERVO=1 且 ZSERVO_BENCH_KEY2/3_JUMP=1 时用）：
 * 直接写 CCR1，不做缓动；脉宽仍按当前模式的 MIN/MAX 限幅。
 * KEY2 传 ZSERVO_PULSE_BOTTOM_US（最下面），KEY3 传 ZSERVO_PULSE_TOP_US（最上面）。 */
void ZServo_JumpTo(uint16_t pulse_us);
/* 三个工作高度。 */
void ZServo_Travel(void);
void ZServo_Approach(void);
void ZServo_Grasp(void);
/* 兼容旧接口。 */
void ZServo_Up(void);
void ZServo_Down(void);
/* 回到运输高度，上电和任务结束后使用。 */
void ZServo_SetSafePosition(void);
/* 返回最近一次写入的脉宽，便于在 Keil 观察窗口核对。 */
uint16_t ZServo_GetPulse(void);

#endif
