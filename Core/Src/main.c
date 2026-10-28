/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "protocol.h"
#include "safety_manager.h"
#include "sorting_task.h"
#include "sorting_config.h"
#include "stepper_motor.h"
#include "servo_gripper.h"
#include "servo_z.h"
#include "lcd_st7789.h"
#if SORT_DEBUG_UART
#include "debug_uart.h"
#endif

/* 是否有台架按键需要“按住连发”：
 *   - 夹爪标定（SORT_BENCH_KEY_JUMP=0）：KEY0/KEY1；
 *   - Z 轴逐档标定（ZSERVO_BENCH_KEY2/3_JUMP=0）：对应的 KEY2/KEY3。
 * 一次到位模式（JUMP=1）按下一次就跳到目标，不需要连发。 */
#if (SORT_BENCH_KEY_JUMP == 0) || \
    ((ZSERVO_CALIB_MODE != 0) && \
     ((ZSERVO_BENCH_KEY2_JUMP == 0) || (ZSERVO_BENCH_KEY3_JUMP == 0)))
#define BENCH_KEY_REPEAT_ENABLED        1
#else
#define BENCH_KEY_REPEAT_ENABLED        0
#endif

/* 正式分拣逻辑 + 台架按键（SORT_ENABLE_BENCH_KEYS = 1）：
 * KEY0/KEY1 直接跳到张开位 1730 / 闭合位 1420 us（SORT_BENCH_KEY_JUMP = 1）；
 * 夹爪标定（JUMP = 0 且 SERVO_GRIPPER_CALIB_MODE = 1）时改为每次 ±50 us，
 * 按住可连发。
 * KEY2/KEY3：ZSERVO_CALIB_MODE = 1（当前）时控制 Z 轴舵机高度：
 *   当前两个键都是逐档测试（KEY2 +50 us、KEY3 -50 us，按住连发），
 *   每次从 USART3 打印 Z_PULSE=。上端点 1200、下端点 2200 已实测，
 *   需要一键到位时把 ZSERVO_BENCH_KEY2/3_JUMP 改成 1。
 *   两种情况都在 ZSERVO_CALIB_MODE = 0 后恢复 X 轴点动（正/反，整圈）。
 * SORT_XY_CALIB_MODE = 1（当前）时 KEY0 改为“按下即开爪 + 长按 1 s 存投放口 X”，
 * KEY1 在 Y 未启用时仍闭合夹爪；X 轴点动改为慢速 SORT_CALIB_JOG_STEPS
 * （被 Z 轴标定模式占用时不生效）。
 * 正式比赛把 SORT_ENABLE_BENCH_KEYS 置 0 即可完全关闭手动入口。 */

/*
 * 工程分层：
 *   Core  - STM32 HAL、时钟、GPIO、定时器、串口和中断入口；
 *   Drivers - 步进电机和舵机等硬件抽象；
 *   Service - 通信协议和安全输入管理；
 *   App   - 分拣任务状态机。
 *
 * 运行模型：
 *   主循环负责协议解析、安全轮询和任务状态迁移；
 *   TIM6 中断只负责固定节拍的步进脉冲，任务计时用 HAL_GetTick()；
 *   USART2 接收中断只搬运字节，不在中断中执行耗时业务。
 */
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
#if SORT_ENABLE_BENCH_KEYS
static uint8_t BenchKey_Allowed(void);
static void BenchKey_Process(void);
#endif
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 台架按键只控制夹爪，不改变步进轴任务状态。 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* 初始化顺序：底层 HAL/外设先完成，再启动应用模块和周期中断。 */
  MX_GPIO_Init();
  MX_TIM1_Init();
  /* Z 轴舵机使用 PA1 / TIM2_CH2 硬件 PWM（板载 LED 已拆除）；
   * 夹爪舵机信号接 PA8，由 TIM1_CH1 硬件 PWM 输出。 */
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  StepperMotor_Init();
  /* 先启动 Z 轴和夹爪 PWM；夹爪初始位置与开合由分拣状态机控制，
   * 上电不再执行会阻塞 1.5 s 的舵机自检。 */
  ZServo_Init();
  ServoGripper_Init();
  /* 显示屏初始化：SPI2 + ST7789V2，初始化完成后打开背光。 */
  LCD_Init();
  /* 上电自检：整屏刷黑，确认 SPI 通路、片选和复位时序正常。 */
  LCD_FillColor(LCD_COLOR_BLACK);
  SafetyManager_Init();
  SortingTask_Init();
  Protocol_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 所有函数都应保持非阻塞，步进脉冲由 TIM6 中断统一调度。 */
    SafetyManager_Process();
    ServoGripper_Process();
#if SORT_ENABLE_BENCH_KEYS
    if (BenchKey_Allowed() != 0U)
    {
      BenchKey_Process();
    }
#endif
    Protocol_Process();
    SortingTask_Process();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /* 板上 Y2 为 8 MHz HSE；PLLM 必须与实际晶振匹配。 */
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

#if SORT_ENABLE_BENCH_KEYS
/* 台架按键门控：
 *  - 安全输入正常，且没有正在执行的分拣任务时允许按键；
 *    上电回零期间、READY、回零失败后的 ERROR 都能立即响应，便于只接
 *    夹爪舵机、没接 T_HOME 的台架测试（否则要等满回零超时时间）；
 *  - 任务执行期间（has_mission != 0）不响应，避免中途改夹爪；
 *  - 急停按下（SafetyManager_IsSafe()==0）在任何状态都不允许。
 * 注意：回零完成进入 READY 时状态机会再调一次 ServoGripper_Open()，
 * 手动在回零期间闭合的状态会被这一次“就绪开爪”覆盖。 */
static uint8_t BenchKey_Allowed(void)
{
  if (SafetyManager_IsSafe() == 0U)
  {
    return 0U;
  }
  return (SortingTask_HasMission() == 0U) ? 1U : 0U;
}

/* X 轴点动：KEY2 正向、KEY3 反向。回零后位置是相对零点的绝对步数，
 * 软限位会拦住越界；门控只要求 X 空闲（安全和“无任务”已在
 * BenchKey_Allowed 里保证），避免前一次点动没走完就叠加新目标。
 *
 * SORT_XY_CALIB_MODE=1 时改为“标定步长 + 慢速”，方便对准投放口：
 * 起步前把速度降到 SORT_CALIB_JOG_SPEED_STEPS_S，走完由
 * BenchKey_Process 开头恢复正式速度。
 *
 * ZSERVO_CALIB_MODE=1（Z 轴齿条啮合标定）时 KEY2/KEY3 归 Z 轴脉宽用，
 * 这一整段 X 轴点动代码不编译，避免“定义了却没调用”的告警。 */
#if !ZSERVO_CALIB_MODE
#if SORT_XY_CALIB_MODE
static uint8_t calib_jog_slow;
#endif

static void BenchX_Jog(int8_t direction)
{
  int32_t steps;

  if (StepperMotor_IsBusy(STEPPER_X) != 0U)
  {
    return;
  }

#if SORT_XY_CALIB_MODE
  steps = SORT_CALIB_JOG_STEPS;
  StepperMotor_SetSpeed(STEPPER_X, SORT_CALIB_JOG_SPEED_STEPS_S);
  StepperMotor_SetAcceleration(STEPPER_X, SORT_XY_ACCEL_STEPS_S2);
  calib_jog_slow = 1U;
#else
  steps = SORT_BENCH_X_JOG_STEPS;
#endif

  (void)StepperMotor_MoveRelative(STEPPER_X, (int32_t)direction * steps);
}
#endif /* !ZSERVO_CALIB_MODE */

/* 台架“张开夹爪”动作：KEY0 专用（一次到位/步进两种模式共用一份实现）。
 * SORT_BENCH_KEY_JUMP=1 时直接跳到张开位（默认 1730 us，SORT_BENCH_KEY_AT_LIMIT=1
 * 时用机械临界 1750 us）；=0 时按 SERVO_GRIPPER_STEP_US 步进张开。 */
#if SORT_BENCH_KEY_JUMP
static void BenchGripper_OpenAction(void)
{
#if SORT_BENCH_KEY_AT_LIMIT
  ServoGripper_JumpTo(SERVO_GRIPPER_MAX_PULSE_US, 0U);
#else
  ServoGripper_JumpTo(SERVO_GRIPPER_OPEN_PULSE_US, 0U);
#endif
}
#else
static void BenchGripper_OpenAction(void)
{
  ServoGripper_StepOpening();
}
#endif

static void BenchKey_Process(void)
{
  static GPIO_PinState stable[4] = {
      GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET};
  static GPIO_PinState last_raw[4] = {
      GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET};
  static uint32_t changed_tick[4] = {0U, 0U, 0U, 0U};
#if BENCH_KEY_REPEAT_ENABLED
  /* 上一次执行动作的时刻：KEY1 夹爪标定（JUMP=0）、Z 轴步进标定（KEY2/KEY3）
   * 按住不放时按这个节拍连发。KEY0 在投放口标定模式下不连发，见下面的条件。
   * 没有按键需要连发时整个变量一起裁掉，避免“赋值未使用”告警。 */
  static uint32_t last_action_tick[4] = {0U, 0U, 0U, 0U};
#endif
#if SORT_XY_CALIB_MODE
  /* KEY0 短按/长按：按下即开爪并开始计时；按住满
   * SORT_DROP_KEY_LONG_PRESS_MS 才执行一次“存投放口 X”，每次按住只存一次。
   * 任务抢占按键（门控关闭）后 stable[0] 会回到释放态，下一轮顺带清状态。 */
  static uint8_t key0_hold_active;
  static uint8_t key0_long_fired;
  static uint32_t key0_hold_tick;
#endif
  uint8_t pressed[4] = {0U, 0U, 0U, 0U};
  static uint8_t initialized;
  static uint32_t last_call_tick;
  static GPIO_TypeDef *const ports[4] = {
      KEY0_GPIO_Port, KEY1_GPIO_Port, KEY2_GPIO_Port, KEY3_GPIO_Port};
  static const uint16_t pins[4] = {KEY0_Pin, KEY1_Pin, KEY2_Pin, KEY3_Pin};
  uint32_t now = HAL_GetTick();
  uint32_t i;

#if SORT_XY_CALIB_MODE && !ZSERVO_CALIB_MODE
  /* 标定慢速点动已经走完：恢复正式速度，避免下一次自动任务也慢速。 */
  if ((calib_jog_slow != 0U) && (StepperMotor_IsBusy(STEPPER_X) == 0U))
  {
    StepperMotor_SetSpeed(STEPPER_X, SORT_XY_MAX_SPEED_STEPS_S);
    StepperMotor_SetAcceleration(STEPPER_X, SORT_XY_ACCEL_STEPS_S2);
    calib_jog_slow = 0U;
  }
#endif

  /*
   * 先同步一次上电电平，避免按键在复位时已经按下却被误判为
   * 一次不完整的边沿。后续动作在稳定按下沿触发。
   */
  if (!initialized)
  {
    for (i = 0U; i < 4U; ++i)
    {
      stable[i] = HAL_GPIO_ReadPin(ports[i], pins[i]);
      last_raw[i] = stable[i];
      changed_tick[i] = now;
#if BENCH_KEY_REPEAT_ENABLED
      last_action_tick[i] = now;
#endif
    }
    initialized = 1U;
  }

  /*
   * 门控中断后恢复：任务执行期间主循环不调用本函数，stable/last_raw 可能
   * 已经过期（例如按住 KEY0 时来了任务，任务结束后才恢复）。这里把四个键
   * 重新同步成当前电平并清掉 KEY0 长按状态，避免恢复瞬间误开爪/误存投放口。
   * 正常运行时主循环每轮都调用，间隔 < 1 ms，不会进入这个分支。
   */
  if ((last_call_tick != 0U) &&
      ((uint32_t)(now - last_call_tick) > (SORT_BENCH_KEY_DEBOUNCE_MS * 5U)))
  {
    for (i = 0U; i < 4U; ++i)
    {
      stable[i] = HAL_GPIO_ReadPin(ports[i], pins[i]);
      last_raw[i] = stable[i];
      changed_tick[i] = now;
#if BENCH_KEY_REPEAT_ENABLED
      last_action_tick[i] = now;
#endif
    }
#if SORT_XY_CALIB_MODE
    key0_hold_active = 0U;
    key0_long_fired = 0U;
#endif
  }
  last_call_tick = now;

  for (i = 0U; i < 4U; ++i)
  {
    GPIO_PinState raw = HAL_GPIO_ReadPin(ports[i], pins[i]);

    if (raw != last_raw[i])
    {
      last_raw[i] = raw;
      changed_tick[i] = now;
    }
    if (raw != stable[i] &&
        (uint32_t)(now - changed_tick[i]) >= SORT_BENCH_KEY_DEBOUNCE_MS)
    {
      /* 只在“释放 -> 按下”的稳定边沿记录一次动作；
       * 按住不放不会连发，松开后再次按下才会再触发。 */
      if ((stable[i] == GPIO_PIN_SET) && (raw == GPIO_PIN_RESET))
      {
        pressed[i] = 1U;
      }
      stable[i] = raw;
    }
  }

  /*
   * 按键动作：一次到位模式按一下动一次；步进标定模式首次按下走一步，
   * 按住不放则每 SORT_BENCH_KEY_REPEAT_MS 再走一步，省得为几步反复点击：
   *   - 夹爪标定（SORT_BENCH_KEY_JUMP=0）：KEY0/KEY1 连发；
   *     但投放口标定开启（SORT_XY_CALIB_MODE=1）时 KEY0 要留给长按计时，不连发；
   *   - Z 轴步进标定（ZSERVO_BENCH_KEY_JUMP=0）：KEY2/KEY3 连发。
   * Z 轴两个键当前都是逐档（ZSERVO_BENCH_KEY2/3_JUMP=0），按住连发；
   * 改成 1 时对应的键只在按下沿一键到端点。
   * 其余情况下 KEY2/KEY3（X 轴点动）也只在按下沿触发一次。
   */
  for (i = 0U; i < 4U; ++i)
  {
    uint8_t act = pressed[i];

#if BENCH_KEY_REPEAT_ENABLED
    if ((act == 0U) && (stable[i] == GPIO_PIN_RESET) &&
        ((uint32_t)(now - last_action_tick[i]) >= SORT_BENCH_KEY_REPEAT_MS))
    {
      uint8_t repeatable = 0U;

#if (SORT_BENCH_KEY_JUMP == 0)
      /* 投放口标定开启（SORT_XY_CALIB_MODE=1）时 KEY0 走“短按开爪/长按存投放口”，
       * 不参与连发，否则按住 1 s 会被连发不断重置计时，长按永远触发不了。 */
      if ((i == 1U) || ((i == 0U) && (SORT_XY_CALIB_MODE == 0)))
      {
        repeatable = 1U;
      }
#endif
#if (ZSERVO_CALIB_MODE != 0)
      /* 只有“逐档”那一侧的键需要连发；一键到位只在按下沿跳一次。 */
      if (((i == 2U) && (ZSERVO_BENCH_KEY2_JUMP == 0)) ||
          ((i == 3U) && (ZSERVO_BENCH_KEY3_JUMP == 0)))
      {
        repeatable = 1U;
      }
#endif
      if (repeatable != 0U)
      {
        act = 1U;
      }
    }
#endif

    if (act != 0U)
    {
#if BENCH_KEY_REPEAT_ENABLED
      last_action_tick[i] = now;
#endif
#if SORT_XY_CALIB_MODE
      /* 投放口标定：KEY0 按下即开爪并开始长按计时，长按满
       * SORT_DROP_KEY_LONG_PRESS_MS 才存投放口 X（见函数末尾）；
       * KEY1 在 Y 启用（SORT_ENABLE_Y_AXIS=1）时存当前 Y。 */
      if (i == 0U)
      {
        BenchGripper_OpenAction();
        key0_hold_active = 1U;
        key0_long_fired = 0U;
        key0_hold_tick = now;
        continue;
      }
#if SORT_ENABLE_Y_AXIS
      if (i == 1U)
      {
        (void)SortingTask_SaveDropY();
        continue;
      }
#endif
#endif
#if SORT_BENCH_KEY_JUMP
      /* 一次到位：KEY0 直接到张开位，KEY1 直接到闭合位。
       * 视觉未就绪时用它手动模拟夹爪状态，不必按几十次。 */
      if (i == 0U)
      {
        BenchGripper_OpenAction();
      }
      else if (i == 1U)
      {
#if SORT_BENCH_KEY_AT_LIMIT
        ServoGripper_JumpTo(SERVO_GRIPPER_MIN_PULSE_US, 1U);
#else
        ServoGripper_JumpTo(SERVO_GRIPPER_CLOSE_PULSE_US, 1U);
#endif
      }
#else
      /* 标定模式：每次 SERVO_GRIPPER_STEP_US（50 us，KEY0 加、KEY1 减），按住可连发。 */
      if (i == 0U)
      {
        BenchGripper_OpenAction();
      }
      else if (i == 1U)
      {
        ServoGripper_StepClosing();
      }
#endif
      /* KEY2/KEY3：
       *   ZSERVO_CALIB_MODE=1（Z 轴标定）：
       *     KEY2 由 ZSERVO_BENCH_KEY2_JUMP 决定：0（当前）= 每次 +ZSERVO_STEP_US
       *       （按住连发）；1 = 一键到最下面 ZSERVO_PULSE_BOTTOM_US（2200 us）；
       *     KEY3 由 ZSERVO_BENCH_KEY3_JUMP 决定：0（当前）= 每次 -ZSERVO_STEP_US
       *       （按住连发）；1 = 一键到最上面 ZSERVO_PULSE_TOP_US（1200 us）；
       *   每次动作都从 USART3 打印 Z_PULSE=。
       *   否则恢复 X 轴点动：KEY2 正向、KEY3 反向。回零后是绝对坐标，
       *   MoveRelative 仍按当前位置累加；软限位由 stepper_motor.c 的
       *   MoveTo 统一检查。 */
#if ZSERVO_CALIB_MODE
      if (i == 2U)
      {
#if ZSERVO_BENCH_KEY2_JUMP
        ZServo_JumpTo(ZSERVO_PULSE_BOTTOM_US);
#else
        ZServo_StepPulse(+1);
#endif
#if SORT_DEBUG_UART
        DebugUart_WriteInt("Z_PULSE=", (int32_t)ZServo_GetPulse());
#endif
      }
      else if (i == 3U)
      {
#if ZSERVO_BENCH_KEY3_JUMP
        ZServo_JumpTo(ZSERVO_PULSE_TOP_US);
#else
        ZServo_StepPulse(-1);
#endif
#if SORT_DEBUG_UART
        DebugUart_WriteInt("Z_PULSE=", (int32_t)ZServo_GetPulse());
#endif
      }
#else
      if (i == 2U)
      {
        BenchX_Jog(+1);
      }
      else if (i == 3U)
      {
        BenchX_Jog(-1);
      }
#endif
    }
  }

#if SORT_XY_CALIB_MODE
  /* KEY0 长按判据：按住满 SORT_DROP_KEY_LONG_PRESS_MS 才存一次投放口。
   * 按下那一轮刚把 key0_hold_tick 置成 now，所以这里不会立刻触发；
   * key0_long_fired 保证“一次按住只存一次”，继续按住不刷串口。
   * stable[0] 回到释放态（松手）时清状态，下次按下重新计时。 */
  if (key0_hold_active != 0U)
  {
    /* 必须“消抖后仍是按下”且“当前原始电平也是按下”才算继续长按：
     * 松开时原始电平立刻变高，这样 980~1000 ms 松手不会被误判成长按。 */
    if ((stable[0] == GPIO_PIN_RESET) && (last_raw[0] == GPIO_PIN_RESET))
    {
      if ((key0_long_fired == 0U) &&
          ((uint32_t)(now - key0_hold_tick) >= SORT_DROP_KEY_LONG_PRESS_MS))
      {
        key0_long_fired = 1U;
        (void)SortingTask_SaveDropX();
      }
    }
    else
    {
      key0_hold_active = 0U;
      key0_long_fired = 0U;
    }
  }
#endif
}

#endif /* SORT_ENABLE_BENCH_KEYS */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
