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
#define ZSERVO_PULSE_SAFE_US  1500U

/* 上/下方向的脉宽对应关系尚未按实机标定：
 * 这里先假定 1000 us 为上、2000 us 为下，
 * 空载实测确认方向后，如相反只需把下面两个宏对调。 */
#define ZSERVO_PULSE_UP_US    1000U
#define ZSERVO_PULSE_DOWN_US  2000U

/* 启动 TIM1_CH1 的 PWM 并把舵机放到安全位置。 */
void ZServo_Init(void);
/* 设置脉宽；超出 MIN/MAX 范围时自动限幅，避免直接给到极限角度。 */
void ZServo_SetPulse(uint16_t pulse);
void ZServo_Up(void);
void ZServo_Down(void);
/* 回到安全位置（约中位），上电和急停后使用。 */
void ZServo_SetSafePosition(void);
/* 返回最近一次写入的脉宽，便于在 Keil 观察窗口核对。 */
uint16_t ZServo_GetPulse(void);

#endif
