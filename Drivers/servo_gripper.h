#ifndef __SERVO_GRIPPER_H
#define __SERVO_GRIPPER_H

#include "main.h"
#include <stdint.h>

/* TIM1_CH1 输出舵机 PWM；开合脉宽需结合实际舵机重新标定。 */
void ServoGripper_Init(void);
void ServoGripper_Open(void);
void ServoGripper_Close(void);
uint8_t ServoGripper_IsClosed(void);

#endif
