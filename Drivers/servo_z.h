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

/* ---------------- 齿条标定模式（台架按键，2026-10-27） ----------------
 * 齿轮和齿条要在整个行程内都能啮合。用 KEY2/KEY3 把脉宽一档一档扫过
 * 整段行程，逐点确认没有跳齿、卡齿、顶死，并记录机械两端。
 * 1 = 标定：上电停在 ZSERVO_CALIB_START_US（1850 us，行程中点）并保持；
 *     Travel/Approach/Grasp 只保留接口、不改脉宽，状态机抢不走 Z；
 *     限幅用 ZSERVO_CALIB_MIN/MAX_US（= 实测行程 1200~2500）；
 *     KEY2/KEY3 按 ZSERVO_BENCH_KEY2/3_JUMP 逐档或一键到端点。
 * 0 = 正式（当前）：上电到运输位，限幅 1200~2500 us，
 *     KEY2/KEY3 恢复 X 轴点动。
 * ！！重新标定时把这里改 1，标定完必须改回 0。 */
#ifndef ZSERVO_CALIB_MODE
#define ZSERVO_CALIB_MODE              0
#endif

/* 标定窗口 = 2026-10-27 实测机械行程：
 *   最上面 1200 us（KEY3 逐档确认）、最下面 2500 us（KEY2 逐档确认）。
 * 脉宽小 = 上，脉宽大 = 下，行程 1300 us。 */
#define ZSERVO_CALIB_MIN_US    1200U
#define ZSERVO_CALIB_MAX_US    2500U
/* 标定上电位置：行程中点 1850 us。
 * 不能用 1200（那是上端点）或 2200（下端点），否则上电就顶在机械端堵转。 */
#define ZSERVO_CALIB_START_US  1850U
/* 逐档模式下 KEY2/KEY3 每次改变的脉宽；想更细可临时改成 10。 */
#define ZSERVO_STEP_US           50U

/* ---------------- 台架一键到位目标（2026-10-27 实测） ----------------
 *   KEY3 → 1200 us：机械爪最上面；
 *   KEY2 → 2500 us：机械爪最下面。
 * 脉宽小 = 上，脉宽大 = 下，行程 1300 us。
 * 两个都是机械临界：长时间停在端点会顶住齿条/舵机堵转，按键测试短停没问题；
 * 自动流程用下面的 TRAVEL/GRASP（各留 20 us 余量）。 */
#define ZSERVO_PULSE_TOP_US      1200U
#define ZSERVO_PULSE_BOTTOM_US   2500U

/* ---------------- 台架按键分工（2026-10-27） ----------------
 * 只在 ZSERVO_CALIB_MODE=1 时生效：
 *   KEY2：0（当前）= 每次 +ZSERVO_STEP_US、按住连发（逐档测试）；
 *         1 = 按下一次到最下面 ZSERVO_PULSE_BOTTOM_US（2200 us），不连发；
 *   KEY3：0（当前）= 每次 -ZSERVO_STEP_US、按住连发（逐档测试）；
 *         1 = 按下一次到最上面 ZSERVO_PULSE_TOP_US（1200 us）。
 * 两个端点都已实测，需要一键到位时把对应开关改成 1。 */
#ifndef ZSERVO_BENCH_KEY2_JUMP
#define ZSERVO_BENCH_KEY2_JUMP    0
#endif
#ifndef ZSERVO_BENCH_KEY3_JUMP
#define ZSERVO_BENCH_KEY3_JUMP    0
#endif

#if ZSERVO_CALIB_MODE
/* 标定模式放宽到标定窗口，按键才能扫到正式行程两端之外。 */
#define ZSERVO_PULSE_MIN_US    ZSERVO_CALIB_MIN_US
#define ZSERVO_PULSE_MAX_US    ZSERVO_CALIB_MAX_US
#else
/* 正式限幅 = 实测机械行程 1200~2500 us。
 * （原 1000~2000/1200~2200 都是未标定时的占位限制。） */
#define ZSERVO_PULSE_MIN_US   1200U
#define ZSERVO_PULSE_MAX_US   2500U
#endif

/* 三个工作高度（端点已实测：1200 最上、2500 最下；工作值待按物块标定）：
 *   TRAVEL   - 运输/初始高度，XY 平移前必须回到这里（距上端点留 20 us）；
 *   APPROACH - 物块上方接近高度，先开爪再下降到抓取高度（暂取行程中间）；
 *   GRASP    - 物块底部抓取高度（距下端点留 20 us）。
 * 脉宽小 = 上、脉宽大 = 下；实测方向相反时把三个宏对调即可。 */
#define ZSERVO_PULSE_TRAVEL_US    1220U
#define ZSERVO_PULSE_APPROACH_US  1850U
#define ZSERVO_PULSE_GRASP_US     2480U

/* 兼容旧接口：safe/up 都指运输高度，down 指抓取高度。 */
#define ZSERVO_PULSE_SAFE_US      ZSERVO_PULSE_TRAVEL_US
#define ZSERVO_PULSE_UP_US        ZSERVO_PULSE_TRAVEL_US
#define ZSERVO_PULSE_DOWN_US      ZSERVO_PULSE_GRASP_US

/* 启动 TIM2_CH2 的 PWM 并把舵机放到运输高度。 */
void ZServo_Init(void);
/* 设置脉宽；超出 MIN/MAX 范围时自动限幅，避免直接给到极限角度。 */
void ZServo_SetPulse(uint16_t pulse);
/* 台架手动点动（ZSERVO_CALIB_MODE=1 且 ZSERVO_BENCH_KEY_JUMP=0 时用）：
 * direction > 0 脉宽每调用一次增大 ZSERVO_STEP_US，< 0 则减小，
 * 到标定窗口边界后停住；不做缓动，方便一档一档对齿。 */
void ZServo_StepPulse(int8_t direction);
/* 台架一键到位（ZSERVO_CALIB_MODE=1 且 ZSERVO_BENCH_KEY_JUMP=1 时用）：
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
