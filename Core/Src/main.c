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

/* 当前固件保留正式分拣逻辑，并提供台架按键：
 * KEY0 松开后直接到张开位、KEY1 松开后直接到闭合位，
 * KEY2/KEY3 在 READY 时单仓点动 T 轴（正/反，一次 60°），
 * 用于视觉未就绪时手动模拟夹爪和检查 6 仓对位；正式比赛把
 * SORT_ENABLE_BENCH_KEYS 置 0 即可完全关闭。 */

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

/* T 轴点动：KEY2 正向、KEY3 反向，一次走一个仓距（≈60°）。
 * 只在 READY 且 T 空闲时响应；点动后 T 位置标志被清除，
 * 必须重新回零（发 HOME 或复位）才能接收自动任务。
 * 这样可以在台架上逐个仓检查“仓口是否正对投放口”，
 * 又不会让标定后的错误坐标进入正式流程。 */
static void BenchTurret_Jog(int8_t direction)
{
  if ((SortingTask_GetState() != SORT_SYS_READY) ||
      (StepperMotor_IsBusy(STEPPER_T) != 0U))
  {
    return;
  }

  StepperMotor_InvalidatePosition(STEPPER_T);
  (void)StepperMotor_MoveRelative(
      STEPPER_T,
      (int32_t)direction * (int32_t)SORT_BENCH_T_JOG_STEPS);
}

static void BenchKey_Process(void)
{
  static GPIO_PinState stable[4] = {
      GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET};
  static GPIO_PinState last_raw[4] = {
      GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET};
  static uint32_t changed_tick[4] = {0U, 0U, 0U, 0U};
  uint8_t pressed[4] = {0U, 0U, 0U, 0U};
  static uint8_t initialized;
  static GPIO_TypeDef *const ports[4] = {
      KEY0_GPIO_Port, KEY1_GPIO_Port, KEY2_GPIO_Port, KEY3_GPIO_Port};
  static const uint16_t pins[4] = {KEY0_Pin, KEY1_Pin, KEY2_Pin, KEY3_Pin};
  uint32_t now = HAL_GetTick();
  uint32_t i;

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
    }
    initialized = 1U;
  }

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
   * 按键按下并稳定后执行一次动作：一次到位模式让手感接近真正的
   * “按一下夹爪就动”，不需要等松开；标定模式同样是按一下走一步。
   */
  for (i = 0U; i < 4U; ++i)
  {
    if (pressed[i] != 0U)
    {
#if SORT_BENCH_KEY_JUMP
      /* 一次到位：KEY0 直接到张开位，KEY1 直接到闭合位。
       * 视觉未就绪时用它手动模拟夹爪状态，不必按几十次。 */
      if (i == 0U)
      {
#if SORT_BENCH_KEY_AT_LIMIT
        ServoGripper_JumpTo(SERVO_GRIPPER_MAX_PULSE_US, 0U);
#else
        ServoGripper_JumpTo(SERVO_GRIPPER_OPEN_PULSE_US, 0U);
#endif
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
      /* 标定模式：每次 10 us，用于逐步找机械临界。 */
      if (i == 0U)
      {
        ServoGripper_StepOpening();
      }
      else if (i == 1U)
      {
        ServoGripper_StepClosing();
      }
#endif
      /* KEY2/KEY3 用于 T 轴单仓点动；只在 READY 且 T 空闲时生效。
       * 与上面的夹爪按键门控不同，这里额外要求状态机已回零完成，
       * 避免和上电回零的 Jog 抢同一根轴。 */
      if (i == 2U)
      {
        BenchTurret_Jog(+1);
      }
      else if (i == 3U)
      {
        BenchTurret_Jog(-1);
      }
    }
  }
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
