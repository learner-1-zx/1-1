#include "servo_gripper.h"
#include "tim.h"

/* 夹爪舵机：PA8 / TIM1_CH1 硬件 PWM，50 Hz、1 us 分辨率。
 *
 * 2026-10-25 重新标定：闭合临界 1400 us、张开临界 1750 us，
 * 正式值各留 20 us 余量（CLOSE 1420 / OPEN 1730），限幅 1400~1750。
 * 上电不执行往返自检、也不启动 PWM，位置完全由状态机决定：
 *   1. 脉宽复位到安全中位 1500 us；
 *   2. 第一次 Open()/Close() 才启动 PWM，第一帧输出 1500 us；
 *   3. Process() 按 50 us / 20 ms 缓动到目标（350 us ≈ 140 ms），
 *      不会一帧跳完整个行程撞端点；JumpTo() 才一次写到位（台架/标定用）。
 */

static uint8_t gripper_closed;
/* PWM 是否已启动。ServoGripper_Init() 有意保持 0，不输出脉冲；
 * 第一次 Open/Close/JumpTo 写脉宽时由 ServoGripper_EnsurePwm() 启动。 */
static uint8_t gripper_pwm_started;
/* 由主循环写入 CCR1；16 位访问在 Cortex-M4 上是原子的。
 * 初始值取安全中位，避免上电第一帧就是某个机械端点。 */
static volatile uint16_t gripper_pulse_us = SERVO_GRIPPER_START_PULSE_US;
static uint16_t gripper_target_pulse_us = SERVO_GRIPPER_START_PULSE_US;
static uint32_t gripper_last_step_tick;

/* 限幅到实测机械临界 1400~1750 us，避免误写把舵机顶过死点。 */
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

/* 上电待机态：把 PA8 主动拉成推挽低电平。
 * TIM1 的 MOE=0、OSSI=0 时 AF 引脚是高阻，从 MX_TIM1_Init() 到第一次
 * HAL_TIM_PWM_Start() 之间（中间还夹着 LCD_Init() 约 430 ms）信号线浮空，
 * 会拾取噪声让舵机抽一下，而且这种毛刺不经过软件限幅。拉低等于“信号线没插”。 */
static void ServoGripper_ParkSignal(void)
{
  GPIO_InitTypeDef gpio = {0};

  HAL_GPIO_WritePin(SERVO_PWM_GPIO_Port, SERVO_PWM_Pin, GPIO_PIN_RESET);
  gpio.Pin = SERVO_PWM_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SERVO_PWM_GPIO_Port, &gpio);
}

/* 把 PA8 交还给 TIM1_CH1。HAL_GPIO_Init 会先写 AFR 再写 MODER，
 * 所以不会出现“AF 模式 + AF0”的中间态毛刺。 */
static void ServoGripper_AttachSignal(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = SERVO_PWM_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(SERVO_PWM_GPIO_Port, &gpio);
}

/* 启动保护：若 PWM 尚未启动，先切回 AF1、写当前脉宽，再启动 TIM1_CH1。
 * Init() 有意不启动 PWM，所以上电后第一次 Open()/Close()/JumpTo() 一定会
 * 走到这里。上电时当前脉宽是安全中位 1500 us，第一帧不会落在 1400/1750 端点上。 */
static void ServoGripper_EnsurePwm(void)
{
#if SERVO_GRIPPER_DISABLE_OUTPUT
  /* 断力调试：不接管 PA8、不启动 TIM1_CH1，PA8 保持 Init 里拉低的推挽输出，
   * 舵机收不到任何脉冲 → 无扭矩。用来做机械检查和“拔信号线”对照。 */
  return;
#endif
  if (gripper_pwm_started != 0U)
  {
    return;
  }
  /* 先接回定时器输出，再写 CCR1，最后启动 PWM：
   * 启动瞬间 CNT=0、CCR1=当前脉宽，第一个周期就是完整的一帧。 */
  ServoGripper_AttachSignal();
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
   * 保证启动的第一帧就是这里记录的当前脉宽，不会闪一帧旧位置。 */
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
  /* 上电不上力：不写 CCR1、不启动 TIM1_CH1，位置完全由状态机决定
   * （SORT_SYS_INIT 会调 ServoGripper_Open()）。以前这里写 1500 us 就启动
   * PWM，LCD_Init() 的 430 ms 阻塞里舵机会先走到中间位，表现为“上电先合
   * 一下再张开”。pulse/target 复位到安全中位而非端点，第一次 Open/Close
   * 由 EnsurePwm() 先输出 1500 us 再缓动，不会一上电就顶死。 */
  gripper_pulse_us = SERVO_GRIPPER_START_PULSE_US;
  gripper_target_pulse_us = SERVO_GRIPPER_START_PULSE_US;
  gripper_last_step_tick = HAL_GetTick();
  gripper_closed = 0;
  gripper_pwm_started = 0;
  ServoGripper_ParkSignal();
#if SERVO_GRIPPER_CALIB_MODE
  /* 标定模式：上电即停在零点（START_PULSE），方便看清中间位置后
   * 用 KEY0/KEY1 逐步点动找两端；Open()/Close() 本模式不改脉宽。 */
  ServoGripper_EnsurePwm();
#endif
}

void ServoGripper_Open(void)
{
#if SERVO_GRIPPER_CALIB_MODE
  /* 标定模式：只更新逻辑标志、不动脉宽，否则状态机会把夹爪拽到 OPEN 位。 */
  ServoGripper_EnsurePwm();
  gripper_closed = 0;
#else
  /* 只改目标，由 Process() 缓动过去；EnsurePwm() 保证第一次调用时
   * 以当前脉宽（上电为安全中位 1500 us）启动 PWM。 */
  ServoGripper_SetTarget(SERVO_GRIPPER_OPEN_PULSE_US);
  ServoGripper_EnsurePwm();
  gripper_closed = 0;
#endif
}

void ServoGripper_Close(void)
{
#if SERVO_GRIPPER_CALIB_MODE
  /* 标定模式：同 Open()，不动脉宽。 */
  ServoGripper_EnsurePwm();
  gripper_closed = 1;
#else
  ServoGripper_SetTarget(SERVO_GRIPPER_CLOSE_PULSE_US);
  ServoGripper_EnsurePwm();
  gripper_closed = 1;
#endif
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
  /* 台架调试用：直接写 CCR1，跳过 50 us/20 ms 缓动，
   * 一次按键就模拟出开/合到位状态。脉宽仍按实测窗口限幅。
   * 标定找临界时用它最直观，但正常流程（Open/Close）不要走这条路。 */
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

  if ((uint32_t)(now - gripper_last_step_tick) < SERVO_GRIPPER_SLEW_PERIOD_MS)
  {
    return;
  }
  gripper_last_step_tick = now;

  if (gripper_pulse_us < gripper_target_pulse_us)
  {
    uint16_t next = (uint16_t)(gripper_pulse_us + SERVO_GRIPPER_SLEW_US);
    ServoGripper_SetPulse(next > gripper_target_pulse_us
                              ? gripper_target_pulse_us : next);
  }
  else if (gripper_pulse_us > gripper_target_pulse_us)
  {
    uint16_t next = gripper_pulse_us > SERVO_GRIPPER_SLEW_US
                        ? (uint16_t)(gripper_pulse_us - SERVO_GRIPPER_SLEW_US)
                        : 0U;
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
