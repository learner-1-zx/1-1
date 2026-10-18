#include "servo_gripper.h"
#include "tim.h"

/* 夹爪舵机信号接 PB8。PB8 只作普通 GPIO，
 * 因此把 TIM1_CH2 配置成“无引脚输出”的内部比较通道：
 *   - TIM1 更新中断（50 Hz）到来时，先把当前脉宽写入 CCR2，再把 PB8 拉高；
 *   - TIM1_CH2 比较匹配时，PB8 拉低。
 * 这样不影响 TIM1 的其他通道，也不需要额外定时器。
 * PA9 虽然是 TIM1_CH2 的硬件引脚，但 PCB 未引出，代码中不再配置。
 *
 * 注意：BDS300 的 PWM 频率和脉宽范围尚未实测，当前 1170/1870 us
 * 仍是 SG90 试装的临时值，仅作占位；标定前不要直接接 BDS300 上电。
 * 上电不再执行往返自检，避免临时脉宽把未标定舵机顶到机械极限。 */

static uint8_t gripper_closed;
/* 由主循环写入、TIM1 中断读取，16 位访问在 Cortex-M4 上是原子的。 */
static volatile uint16_t gripper_pulse_us = SERVO_GRIPPER_CLOSE_PULSE_US;

/* 限幅到 0.5~2.5 ms，避免误写把舵机顶到机械极限。 */
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

static void ServoGripper_SetPulse(uint16_t pulse)
{
  /* 只更新目标脉宽；下一个 TIM1 更新中断会把它写入 CCR2。 */
  gripper_pulse_us = ServoGripper_ClampPulse(pulse);
}

void ServoGripper_Init(void)
{
  /* 上电先把信号脚拉低，避免出现不确定的长高电平。 */
  HAL_GPIO_WritePin(SERVO_PWM_GPIO_Port, SERVO_PWM_Pin, GPIO_PIN_RESET);

  /* 首个周期直接按张开脉宽输出，避免先出现中位脉冲。 */
  gripper_pulse_us = SERVO_GRIPPER_OPEN_PULSE_US;
  gripper_closed = 0;

  /* CH2 只用于软件 PWM：关闭比较预装载，
   * 使更新中断写入的 CCR2 在当前周期立即生效（CH1 不受影响）。 */
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, gripper_pulse_us);

  /* CH2 只用作内部比较，不连接 GPIO。
   * 注意：HAL_TIM_PWM_Start_IT() 只打开 CH2 比较中断，
   * 更新中断必须单独使能，否则 PB8 没有上升沿。 */
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
  /* 清掉可能残留的更新标志，确保第一次上升沿出现在真正的周期起点，
   * 避免在计数器已经超过 CCR2 时拉高 PB8 而形成超长脉冲。 */
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

void ServoGripper_Open(void)
{
  ServoGripper_SetPulse(SERVO_GRIPPER_OPEN_PULSE_US);
  gripper_closed = 0;
}

void ServoGripper_Close(void)
{
  ServoGripper_SetPulse(SERVO_GRIPPER_CLOSE_PULSE_US);
  gripper_closed = 1;
}

uint8_t ServoGripper_IsClosed(void)
{
  return gripper_closed;
}

uint16_t ServoGripper_GetPulse(void)
{
  return gripper_pulse_us;
}

void ServoGripper_TimerUpdateIsr(void)
{
  /* 先写比较值，再拉高：本周期从计数器 0 开始，
   * 只要脉宽大于中断延迟，就不会错过本次 CC2 匹配。 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, gripper_pulse_us);
  HAL_GPIO_WritePin(SERVO_PWM_GPIO_Port, SERVO_PWM_Pin, GPIO_PIN_SET);
}

/* TIM1_CH2 比较匹配：软件 PWM 的下降沿。
 * HAL 会在调用前把 htim->Channel 置为 HAL_TIM_ACTIVE_CHANNEL_2。
 * 当前工程只有 TIM1_CH2 使能了比较中断，因此按通道判断即可；
 * 这样主机仿真用内存句柄代替寄存器时也能正常工作。 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    HAL_GPIO_WritePin(SERVO_PWM_GPIO_Port, SERVO_PWM_Pin, GPIO_PIN_RESET);
  }
}
