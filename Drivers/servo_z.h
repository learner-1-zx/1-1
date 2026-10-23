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
#define ZSERVO_PULSE_MIN_US   1000U
#define ZSERVO_PULSE_MAX_US   2000U

/* 三个工作高度（待实机标定）：
 *   TRAVEL   - 运输/初始高度，XY 平移前必须回到这里；
 *   APPROACH - 物块上方接近高度，先开爪再下降到抓取高度；
 *   GRASP    - 物块底部抓取高度。
 * 1000 us 为上、2000 us 为下是临时假设；实测方向相反时，
 * 直接把三个宏的数值对调，不需要改状态机。 */
#define ZSERVO_PULSE_TRAVEL_US    1000U
#define ZSERVO_PULSE_APPROACH_US  1450U
#define ZSERVO_PULSE_GRASP_US     1750U

/* 兼容旧接口：safe/up 都指运输高度，down 指抓取高度。 */
#define ZSERVO_PULSE_SAFE_US      ZSERVO_PULSE_TRAVEL_US
#define ZSERVO_PULSE_UP_US        ZSERVO_PULSE_TRAVEL_US
#define ZSERVO_PULSE_DOWN_US      ZSERVO_PULSE_GRASP_US

/* 启动 TIM2_CH2 的 PWM 并把舵机放到运输高度。 */
void ZServo_Init(void);
/* 设置脉宽；超出 MIN/MAX 范围时自动限幅，避免直接给到极限角度。 */
void ZServo_SetPulse(uint16_t pulse);
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
