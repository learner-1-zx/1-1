#include "sorting_config.h"

/* SORT_DEBUG_CONSOLE=0 时整个控制台不参与编译（本文件仍在 Keil 工程里，
 * 但只剩一个空编译单元），正式比赛不会为它多花 Flash/中断。 */
#if SORT_DEBUG_CONSOLE

#include "bench_console.h"
#if SORT_BENCH_MOTION
#include "bench_motion.h"
#endif
#include "debug_uart.h"
#include "usart.h"
#include "main.h"
#include "safety_manager.h"
#include "sorting_task.h"
#include "stepper_motor.h"
#include "servo_gripper.h"
#include "servo_z.h"

/* 接收采用“中断写、主循环读”的环形缓冲区，和 protocol.c 的 USART2
 * 一样只保留单生产者/单消费者语义；缓冲区满时丢弃新字节，
 * 避免覆盖还没解析的命令。 */
#define BENCH_CONSOLE_RX_SIZE 32U

static uint8_t rx_byte;
static uint8_t rx_buffer[BENCH_CONSOLE_RX_SIZE];
static volatile uint16_t rx_write;
static volatile uint16_t rx_read;

/* 虚拟按键：active 置位期间向主循环输出“按下”电平，
 * 到期自动失效、回落到真实 GPIO 电平。 */
static uint8_t virtual_active[BENCH_CONSOLE_KEY_COUNT];
static uint32_t virtual_until[BENCH_CONSOLE_KEY_COUNT];

/* 板载按键实际引脚：PD0/PD1/PD2/PD3，上拉、低电平按下。 */
#if !BENCH_CONSOLE_NO_STATUS
static GPIO_TypeDef *const key_ports[BENCH_CONSOLE_KEY_COUNT] = {
    KEY0_GPIO_Port, KEY1_GPIO_Port, KEY2_GPIO_Port, KEY3_GPIO_Port};
static const uint16_t key_pins[BENCH_CONSOLE_KEY_COUNT] = {
    KEY0_Pin, KEY1_Pin, KEY2_Pin, KEY3_Pin};
#endif

static uint8_t Console_ReadByte(uint8_t *value)
{
  if (rx_read == rx_write)
  {
    return 0;
  }
  *value = rx_buffer[rx_read];
  rx_read = (uint16_t)((rx_read + 1U) % BENCH_CONSOLE_RX_SIZE);
  return 1;
}

/* 中断和主机仿真共用入队逻辑。 */
static void Console_PushByte(uint8_t byte)
{
  uint16_t next = (uint16_t)((rx_write + 1U) % BENCH_CONSOLE_RX_SIZE);

  if (next != rx_read)
  {
    rx_buffer[rx_write] = byte;
    rx_write = next;
  }
}

static void Console_ReleaseAll(void)
{
  uint32_t i;

  for (i = 0U; i < BENCH_CONSOLE_KEY_COUNT; ++i)
  {
    virtual_active[i] = 0U;
    virtual_until[i] = 0U;
  }
}

static void Console_PrintHelp(void)
{
  DebugUart_Write("\r\n--- BENCH CONSOLE (USART3 115200 8N1) ---\r\n");
  DebugUart_Write("0..3  tap  KEY0..KEY3 (100 ms)\r\n");
  DebugUart_Write("A..D  hold KEY0..KEY3 (1500 ms, KEY0 >1 s saves DROP_X)\r\n");
  DebugUart_Write("x     release virtual keys\r\n");
  DebugUart_Write("s     state / positions / pulses / key levels\r\n");
  DebugUart_Write("h     this help\r\n");
#if SORT_BENCH_MOTION
  DebugUart_Write("-- motion commands (see tools/pc/README.md) --\r\n");
  BenchMotion_PrintHelp();
#endif
  DebugUart_Write("----------------------------------------\r\n");
}

#if !BENCH_CONSOLE_NO_STATUS
/* 打印状态：卡在哪个状态、位置是否可信、舵机当前脉宽。
 * 只用十进制整数，和 debug_uart 的极简输出保持一致。 */
static void Console_PrintStatus(void)
{
  uint32_t i;

  DebugUart_WriteInt("STATE=", (int32_t)SortingTask_GetState());
  DebugUart_WriteInt("ERR=", (int32_t)SortingTask_GetError());
  DebugUart_WriteInt("SAFE=", (int32_t)SafetyManager_IsSafe());
  DebugUart_WriteInt("MISSION=", (int32_t)SortingTask_HasMission());
  DebugUart_WriteInt("X_POS=", StepperMotor_GetPosition(STEPPER_X));
  DebugUart_WriteInt("X_POS_OK=",
                     (int32_t)StepperMotor_IsPositionValid(STEPPER_X));
  DebugUart_WriteInt("X_BUSY=", (int32_t)StepperMotor_IsBusy(STEPPER_X));
  DebugUart_WriteInt("Y_POS=", StepperMotor_GetPosition(STEPPER_Y));
  DebugUart_WriteInt("T_POS=", StepperMotor_GetPosition(STEPPER_T));
  DebugUart_WriteInt("Z_US=", (int32_t)ZServo_GetPulse());
  DebugUart_WriteInt("GRIP_US=", (int32_t)ServoGripper_GetPulse());
  DebugUart_WriteInt("GRIP_CLOSED=", (int32_t)ServoGripper_IsClosed());
  DebugUart_WriteInt("DROP_X=", SortingTask_GetDropX());
  DebugUart_WriteInt("DROP_Y=", SortingTask_GetDropY());
#if SORT_BENCH_MOTION
  DebugUart_WriteInt("TEST_X=", BenchMotion_GetLastX());
  DebugUart_WriteInt("TEST_BIN=", (int32_t)BenchMotion_GetLastBin());
  DebugUart_WriteInt("SEQ_BUSY=", (int32_t)BenchMotion_IsBusy());
#endif
  for (i = 0U; i < BENCH_CONSOLE_KEY_COUNT; ++i)
  {
    /* 原始电平：0 = 低（按下），1 = 高（松开）；上拉输入。 */
    char label[4];

    label[0] = 'K';
    label[1] = (char)('0' + i);
    label[2] = '=';
    label[3] = '\0';
    DebugUart_WriteInt(label,
                       (HAL_GPIO_ReadPin(key_ports[i], key_pins[i]) ==
                        GPIO_PIN_RESET) ? 0 : 1);
  }
}
#endif /* !BENCH_CONSOLE_NO_STATUS */

/* 注入一次“按下-释放”：hold_ms 后自动失效。 */
static void Console_Press(uint8_t index, uint32_t hold_ms)
{
  uint32_t now;

  if (index >= BENCH_CONSOLE_KEY_COUNT)
  {
    return;
  }
  now = HAL_GetTick();
  virtual_active[index] = 1U;
  virtual_until[index] = now + hold_ms;
  DebugUart_Write((hold_ms > BENCH_CONSOLE_TAP_MS) ? "HOLD K" : "TAP K");
  DebugUart_WriteInt("", (int32_t)index);
}

static void Console_Execute(uint8_t command)
{
  if ((command >= (uint8_t)'0') &&
      (command < (uint8_t)('0' + BENCH_CONSOLE_KEY_COUNT)))
  {
    Console_Press((uint8_t)(command - (uint8_t)'0'), BENCH_CONSOLE_TAP_MS);
    return;
  }
  if ((command >= (uint8_t)'A') &&
      (command < (uint8_t)('A' + BENCH_CONSOLE_KEY_COUNT)))
  {
    Console_Press((uint8_t)(command - (uint8_t)'A'), BENCH_CONSOLE_HOLD_MS);
    return;
  }

  switch (command)
  {
    case (uint8_t)'x':
    case (uint8_t)'X':
      Console_ReleaseAll();
      DebugUart_Write("RELEASE ALL\r\n");
      break;
    case (uint8_t)'s':
    case (uint8_t)'S':
#if !BENCH_CONSOLE_NO_STATUS
      Console_PrintStatus();
#endif
      break;
    case (uint8_t)'h':
    case (uint8_t)'H':
    case (uint8_t)'?':
      Console_PrintHelp();
      break;
    /* 串口助手常把回车/换行一起发出来，直接忽略。 */
    case (uint8_t)'\r':
    case (uint8_t)'\n':
    case (uint8_t)' ':
    case (uint8_t)'\t':
      break;
    default:
      DebugUart_Write("UNKNOWN CMD (h = help)\r\n");
      break;
  }
}

void BenchConsole_Init(void)
{
  rx_write = 0U;
  rx_read = 0U;
  Console_ReleaseAll();
#if SORT_BENCH_MOTION
  BenchMotion_Init();
#endif
  (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  DebugUart_Write("\r\nBENCH CONSOLE READY (h = help)\r\n");
}

void BenchConsole_RxComplete(void)
{
  Console_PushByte(rx_byte);
  (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void BenchConsole_FeedByte(uint8_t byte)
{
  Console_PushByte(byte);
}

void BenchConsole_Process(void)
{
  uint8_t value;

  while (Console_ReadByte(&value))
  {
#if SORT_BENCH_MOTION
    /* 大写字母开头的运动指令先解析（X/Y/T/Z/G/R）；
     * 不认识的字符（0~3 按键、A~D 长按、s/h 等）交回按键控制台。 */
    if (BenchMotion_HandleByte(value) != 0U)
    {
      continue;
    }
#endif
    Console_Execute(value);
  }
#if SORT_BENCH_MOTION
  /* 每轮都推进一次序列（没有新字符时也要跑），非阻塞。 */
  BenchMotion_Process();
#endif
}

uint8_t BenchConsole_GetVirtualKey(uint8_t index, uint8_t *pressed)
{
  uint32_t now;

  if ((index >= BENCH_CONSOLE_KEY_COUNT) || (pressed == 0))
  {
    return 0U;
  }
  if (virtual_active[index] == 0U)
  {
    return 0U;
  }
  now = HAL_GetTick();
  /* 用有符号差值比较，HAL_GetTick() 回绕时仍然正确。 */
  if ((int32_t)(virtual_until[index] - now) > 0)
  {
    *pressed = 1U;
    return 1U;
  }
  virtual_active[index] = 0U;
  return 0U;
}

/* 溢出/噪声会让 HAL 的接收中断停摆；只对 USART3 重新挂上单字节接收，
 * 不改变 USART2（视觉协议）原有的错误处理行为。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != 0) && (huart->Instance == USART3))
  {
    (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  }
}

#else /* SORT_DEBUG_CONSOLE */
/* SORT_DEBUG_CONSOLE=0：保留一个空编译单元，避免“文件为空”的编译告警。 */
typedef int BenchConsole_Disabled_t;
#endif /* SORT_DEBUG_CONSOLE */
