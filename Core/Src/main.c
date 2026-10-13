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
#include "gpio.h"
#include "protocol.h"
#include "safety_manager.h"
#include "sorting_task.h"
#include "stepper_motor.h"
#include "servo_gripper.h"

#define TURNTABLE_TEST_MODE 1

/*
 * 工程分层：
 *   Core  - STM32 HAL、时钟、GPIO、定时器、串口和中断入口；
 *   Drivers - 步进电机和舵机等硬件抽象；
 *   Service - 通信协议和安全输入管理；
 *   App   - 分拣任务状态机。
 *
 * 运行模型：
 *   主循环负责协议解析、安全轮询和任务状态迁移；
 *   TIM6 中断负责固定节拍的步进脉冲和任务计时；
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
static void TurntableTest_Init(void);
static void TurntableTest_Process(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if TURNTABLE_TEST_MODE
/* T 轴测试速度：TIM6 基础节拍 = 84 MHz / (83 + 1) / (49 + 1) = 20 kHz（每 tick 50 us）。
 * 目标步进速率越高，分频值越小，关系为 STEP_INTERVAL = TICK_HZ / SPEED_HZ。
 * 每个 STEP 有效电平固定保持 50 us，满足 TB6600 最小脉宽，与速率无关。 */
#define TURNTABLE_TEST_TICK_HZ 20000U

/* 加速段：上电后从起步速率逐步提升到最高速率，避免静止直接跳到高速造成丢步。
 * 2026-10-13 提速：起步 200 steps/s，最高 1200 steps/s（原为固定 250 steps/s）。
 * 寻找最佳速度时，可在 Keil 观察窗口监视 turntable_test_speed_hz。
 * 若在某个速率开始丢步或异响，把 SPEED_MAX_HZ 降到该速率以下即可。 */
#define TURNTABLE_TEST_SPEED_START_HZ 200U
#define TURNTABLE_TEST_SPEED_MAX_HZ 1200U

/* 加速节奏：每 20 ms 提升 25 steps/s，从 200 提升到 1200 约需 800 ms。 */
#define TURNTABLE_TEST_RAMP_INTERVAL_MS 20U
#define TURNTABLE_TEST_RAMP_STEP_HZ 25U

/* 单次连续运行的步数上限，保证观察期间不会中途走完而自行停下。 */
#define TURNTABLE_TEST_RUN_STEPS 2000000L

/* 停止键去抖时间：从按下到真正停机之间会多走约 FILTER_MS 对应的步数。 */
#define TURNTABLE_TEST_KEY_FILTER_MS 20U

static uint8_t turntable_test_running;
static uint8_t turntable_test_finished;
static uint32_t turntable_test_key_low_since;
static uint32_t turntable_test_ramp_last_ms;
static uint16_t turntable_test_speed_hz;

/* 停机瞬间的软件步数与速率，便于在 Keil 观察窗口核对停止位置和当时速度。 */
static int32_t turntable_test_stop_position;
static uint16_t turntable_test_stop_speed_hz;

/* 由目标速率换算出 TIM6 分频值，并写入 T 轴。 */
static void TurntableTest_SetSpeed(uint16_t speed_hz)
{
  if (speed_hz == 0U)
  {
    speed_hz = 1U;
  }
  StepperMotor_SetStepInterval(STEPPER_T,
                               (uint16_t)(TURNTABLE_TEST_TICK_HZ / speed_hz));
}

static void TurntableTest_Init(void)
{
  turntable_test_running = 0;
  turntable_test_finished = 0;
  turntable_test_key_low_since = 0;
  turntable_test_ramp_last_ms = 0;
  turntable_test_speed_hz = TURNTABLE_TEST_SPEED_START_HZ;
  turntable_test_stop_position = 0;
  turntable_test_stop_speed_hz = 0;
  TurntableTest_SetSpeed(TURNTABLE_TEST_SPEED_START_HZ);
}

static void TurntableTest_Process(void)
{
  GPIO_PinState key_state;
  uint16_t next_speed_hz;

  if (!SafetyManager_IsSafe())
  {
    /* 急停已由安全模块锁存并关闭使能，这里只结束测试并记录停机位置。 */
    turntable_test_stop_position = StepperMotor_GetPosition(STEPPER_T);
    turntable_test_stop_speed_hz = turntable_test_speed_hz;
    turntable_test_running = 0;
    turntable_test_finished = 1;
    return;
  }

  key_state = HAL_GPIO_ReadPin(TURNTABLE_TEST_KEY_GPIO_Port,
                               TURNTABLE_TEST_KEY_Pin);

  if (!turntable_test_running)
  {
    if (turntable_test_finished)
    {
      return;
    }

    /* 上电时若按键已经按下，先等待释放，防止误启动。 */
    if (key_state == GPIO_PIN_SET)
    {
      turntable_test_speed_hz = TURNTABLE_TEST_SPEED_START_HZ;
      turntable_test_ramp_last_ms = HAL_GetTick();
      TurntableTest_SetSpeed(turntable_test_speed_hz);
      /* 一次性设定足够大的连续行程，全程保持使能，
       * 避免原分段方式在段间反复开关使能造成高速抖动。 */
      StepperMotor_MoveTo(STEPPER_T, TURNTABLE_TEST_RUN_STEPS);
      turntable_test_running = 1;
    }
    return;
  }

  /* 加速段：按固定时间间隔提升一步速率，直到达到最高速率。 */
  if (turntable_test_speed_hz < TURNTABLE_TEST_SPEED_MAX_HZ &&
      (HAL_GetTick() - turntable_test_ramp_last_ms)
       >= TURNTABLE_TEST_RAMP_INTERVAL_MS)
  {
    turntable_test_ramp_last_ms = HAL_GetTick();
    next_speed_hz = (uint16_t)(turntable_test_speed_hz
                               + TURNTABLE_TEST_RAMP_STEP_HZ);
    if (next_speed_hz > TURNTABLE_TEST_SPEED_MAX_HZ)
    {
      next_speed_hz = TURNTABLE_TEST_SPEED_MAX_HZ;
    }
    turntable_test_speed_hz = next_speed_hz;
    TurntableTest_SetSpeed(next_speed_hz);
  }

  /* 停止键：低电平去抖稳定后立即停止脉冲输出。
   * 保留驱动器使能，用保持力矩防止停机后滑移，便于核对停止位置。 */
  if (key_state == GPIO_PIN_RESET)
  {
    if (turntable_test_key_low_since == 0U)
    {
      turntable_test_key_low_since = HAL_GetTick();
    }
    else if ((HAL_GetTick() - turntable_test_key_low_since)
             >= TURNTABLE_TEST_KEY_FILTER_MS)
    {
      turntable_test_stop_position = StepperMotor_GetPosition(STEPPER_T);
      turntable_test_stop_speed_hz = turntable_test_speed_hz;
      StepperMotor_HoldStop(STEPPER_T);
      turntable_test_running = 0;
      turntable_test_finished = 1;
      turntable_test_key_low_since = 0;
    }
    return;
  }

  turntable_test_key_low_since = 0;
}
#endif

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  StepperMotor_Init();
  ServoGripper_Init();
  SafetyManager_Init();
#if TURNTABLE_TEST_MODE
  TurntableTest_Init();
#else
  SortingTask_Init();
  Protocol_Init();
#endif
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 所有函数都应保持非阻塞，步进脉冲由 TIM6 中断统一调度。 */
#if TURNTABLE_TEST_MODE
    SafetyManager_ProcessTurntableTest();
#else
    SafetyManager_Process();
    Protocol_Process();
    SortingTask_Process();
#endif
#if TURNTABLE_TEST_MODE
    TurntableTest_Process();
#endif
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
  RCC_OscInitStruct.PLL.PLLM = 25;
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
