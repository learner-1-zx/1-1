#include "servo_gripper.h"
#include "tim.h"

/* 夹爪舵机信号接 PA8，由 TIM1_CH1 硬件 PWM 输出。
 *
 * 2026-10-20 台架实测：闭合临界 1390 us、张开临界 1800 us；
 * 正式值各留 20 us 余量（CLOSE 1410 / OPEN 1780），限幅 1390~1800。
 * 上电不执行往返自检，也**不启动 PWM**：位置完全由状态机决定，
 * 避免先摆到 1500 us 中间位再张开。 */

static uint8_t gripper_closed;
/* PWM 是否已启动。ServoGripper_Init() 有意保持 0，不输出脉冲；
 * 第一次 Open/Close/JumpTo 写脉宽时由 ServoGripper_EnsurePwm() 启动。 */
static uint8_t gripper_pwm_started;
/* 由主循环写入 CCR1；16 位访问在 Cortex-M4 上是原子的。 */
static volatile uint16_t gripper_pulse_us = SERVO_GRIPPER_OPEN_PULSE_US;
static uint16_t gripper_target_pulse_us = SERVO_GRIPPER_OPEN_PULSE_US;
static uint32_t gripper_last_step_tick;

/* 限幅到实测机械临界 1390~1800 us，避免误写把舵机顶过死点。 */
static uint16_t ServoGripper_ClampPulse(uint16_t pulse)
{
  if (pulse < SERVO_GRIPPER_MIN_PULSE_US)
  {
    return SERVO_GRIPPER_MIN_PULSE_US;
  }
  if (pulse > SERVO_GRIPPER_MAX_PULSE_US)
  {
    return SERVO_GRIPPER_MAX_PULSE_US;
  }
  return pulse;
}

/* 启动保护：若 PWM 尚未启动，先写当前脉宽再启动 TIM1_CH1。
 * ServoGripper_Init() 有意不启动 PWM，所以上电后第一次
 * Open()/Close()/JumpTo() 一定会走到这里，第一帧就是目标脉宽。 */
static void ServoGripper_EnsurePwm(void)
{
  if (gripper_pwm_started != 0U)
  {
    return;
  }
  /* 先写入当前软件脉宽，再启动 PWM，避免第一周期输出未定义占空比。 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, gripper_pulse_us);
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  gripper_pwm_started = 1U;
}

static void ServoGripper_SetPulse(uint16_t pulse)
{
  /* 先记录新脉宽再启动 PWM：EnsurePwm() 内部会写 CCR1，
   * 这样第一帧输出就是目标值，不会先闪一帧旧位置。 */
  gripper_pulse_us = ServoGripper_ClampPulse(pulse);
  ServoGripper_EnsurePwm();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, gripper_pulse_us);
}

static void ServoGripper_SetTarget(uint16_t pulse)
{
  gripper_target_pulse_us = ServoGripper_ClampPulse(pulse);
}

void ServoGripper_Init(void)
{
  /* 上电不上力：不写 CCR1、不启动 TIM1_CH1。
   * 夹爪位置完全由分拣状态机决定（SORT_SYS_INIT 会调用 ServoGripper_Open()）。
   * 以前这里写 1500 us 并立即启动 PWM，而紧接着的 LCD_Init() 要阻塞
   * 约 430 ms，舵机会先往中间位走、随后才被 Open() 拉开，表现为
   * “上电先合一下再张开”。第一次写脉宽时由 EnsurePwm() 启动 PWM。 */
  gripper_pulse_us = SERVO_GRIPPER_OPEN_PULSE_US;
  gripper_target_pulse_us = SERVO_GRIPPER_OPEN_PULSE_US;
  gripper_last_step_tick = HAL_GetTick();
  gripper_closed = 0;
  gripper_pwm_started = 0;
}

void ServoGripper_Open(void)
{
  ServoGripper_SetPulse(SERVO_GRIPPER_OPEN_PULSE_US);
  ServoGripper_SetTarget(SERVO_GRIPPER_OPEN_PULSE_US);
  gripper_closed = 0;
}

void ServoGripper_Close(void)
{
  ServoGripper_SetPulse(SERVO_GRIPPER_CLOSE_PULSE_US);
  ServoGripper_SetTarget(SERVO_GRIPPER_CLOSE_PULSE_US);
  gripper_closed = 1;
}

void ServoGripper_StartOpening(void)
{
  ServoGripper_SetTarget(SERVO_GRIPPER_OPEN_PULSE_US);
  gripper_closed = 0;
}

void ServoGripper_StartClosing(void)
{
  ServoGripper_SetTarget(SERVO_GRIPPER_CLOSE_PULSE_US);
  gripper_closed = 1;
}

void ServoGripper_StepOpening(void)
{
  uint16_t target = gripper_target_pulse_us;

  if (target <= (SERVO_GRIPPER_MAX_PULSE_US - SERVO_GRIPPER_STEP_US))
  {
    target = (uint16_t)(target + SERVO_GRIPPER_STEP_US);
  }
  else
  {
    target = SERVO_GRIPPER_MAX_PULSE_US;
  }
  ServoGripper_SetTarget(target);
  gripper_closed = 0;
}

void ServoGripper_StepClosing(void)
{
  uint16_t target = gripper_target_pulse_us;

  if (target >= (SERVO_GRIPPER_MIN_PULSE_US + SERVO_GRIPPER_STEP_US))
  {
    target = (uint16_t)(target - SERVO_GRIPPER_STEP_US);
  }
  else
  {
    target = SERVO_GRIPPER_MIN_PULSE_US;
  }
  ServoGripper_SetTarget(target);
  gripper_closed = 1;
}

void ServoGripper_JumpTo(uint16_t pulse_us, uint8_t closed)
{
  /* 台架调试用：直接写 CCR1，跳过 5 us/20 ms 缓动，
   * 一次按键就模拟出开/合到位状态。脉宽仍按实测窗口限幅。 */
  ServoGripper_SetPulse(pulse_us);
  ServoGripper_SetTarget(pulse_us);
  gripper_closed = (closed != 0U) ? 1U : 0U;
}

void ServoGripper_Stop(void)
{
  /* 保持当前已输出脉宽；舵机不会继续收到新的位置目标。 */
  gripper_target_pulse_us = gripper_pulse_us;
}

void ServoGripper_Process(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - gripper_last_step_tick) < 20U)
  {
    return;
  }
  gripper_last_step_tick = now;

  if (gripper_pulse_us < gripper_target_pulse_us)
  {
    uint16_t next = (uint16_t)(gripper_pulse_us + 5U);
    ServoGripper_SetPulse(next > gripper_target_pulse_us
                              ? gripper_target_pulse_us : next);
  }
  else if (gripper_pulse_us > gripper_target_pulse_us)
  {
    uint16_t next = gripper_pulse_us > 5U
                        ? (uint16_t)(gripper_pulse_us - 5U) : 0U;
    ServoGripper_SetPulse(next < gripper_target_pulse_us
                              ? gripper_target_pulse_us : next);
  }
}

uint8_t ServoGripper_IsClosed(void)
{
  return gripper_closed;
}

uint16_t ServoGripper_GetPulse(void)
{
  return gripper_pulse_us;
}
