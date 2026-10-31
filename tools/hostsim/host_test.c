/* 主机仿真测试：不接硬件，复核 ST7789V2 驱动和 Z 轴舵机驱动的行为。
 * 用真实 HAL 头文件编译，链接 host_stub.c 提供的 HAL 桩函数。 */
#include "host_stub.h"
#include "tim.h"
#include "lcd_st7789.h"
#include "servo_z.h"
#include "servo_gripper.h"
#include "hall_sensor.h"
#include "axis_home.h"
#include "bench_console.h"
#include "bench_motion.h"
#include "safety_manager.h"
#include "sorting_config.h"
#include "stepper_motor.h"
#include <stdio.h>
#include <string.h>

static int failures;
static int checks;

#define CHECK(cond, msg)                                                \
  do                                                                    \
  {                                                                     \
    checks++;                                                           \
    if (!(cond))                                                        \
    {                                                                   \
      failures++;                                                       \
      printf("  [FAIL] line %d: %s\n", __LINE__, msg);                  \
    }                                                                   \
  } while (0)

static void dump_xfers(const char *title)
{
  uint32_t i;
  uint16_t j;

  printf("  trace of %s: xfers=%u cs_low=%u cs_high=%u\n",
         title, (unsigned)host_xfer_count,
         (unsigned)host_cs_low, (unsigned)host_cs_high);
  for (i = 0U; (i < host_xfer_count) && (i < 8U); i++)
  {
    printf("    #%u %s len=%u :", (unsigned)i,
           (host_xfers[i].dc == 0U) ? "CMD" : "DAT",
           (unsigned)host_xfers[i].len);
    for (j = 0U; (j < host_xfers[i].len) && (j < 8U); j++)
    {
      printf(" %02X", host_xfer_byte(i, j));
    }
    printf("\n");
  }
}

/* 收集所有 DC=低 的字节（命令字节）。 */
static uint32_t collect_commands(uint8_t *out, uint32_t out_max)
{
  uint32_t i;
  uint32_t n = 0U;

  for (i = 0U; i < host_xfer_count; i++)
  {
    if ((host_xfers[i].dc == 0U) && (n < out_max))
    {
      out[n] = host_xfer_byte(i, 0U);
      n++;
    }
  }
  return n;
}

static void test_fill_color(void)
{
  uint32_t i;
  uint32_t data_bytes = 0U;

  printf("test 1: LCD_FillColor(RED) full screen 240x320\n");
  host_reset();
  LCD_FillColor(LCD_COLOR_RED);

  CHECK(host_cs_low == 1U, "CS is asserted exactly once during one fill");
  CHECK(host_cs_high == 1U, "CS is released exactly once during one fill");
  CHECK(host_xfer_count >= 5U, "window + RAMWR + data transfers present");

  CHECK((host_xfers[0].dc == 0U) && (host_xfers[0].len == 1U) &&
        (host_xfer_byte(0U, 0U) == 0x2AU), "first byte is CASET(0x2A)");
  CHECK((host_xfers[1].dc == 1U) && (host_xfers[1].len == 4U) &&
        (host_xfer_byte(1U, 0U) == 0x00U) && (host_xfer_byte(1U, 1U) == 0x00U) &&
        (host_xfer_byte(1U, 2U) == 0x00U) && (host_xfer_byte(1U, 3U) == 0xEFU),
        "CASET window is x = 0..239");

  CHECK((host_xfers[2].dc == 0U) && (host_xfer_byte(2U, 0U) == 0x2BU),
        "second command is RASET(0x2B)");
  CHECK((host_xfers[3].dc == 1U) && (host_xfers[3].len == 4U) &&
        (host_xfer_byte(3U, 0U) == 0x00U) && (host_xfer_byte(3U, 1U) == 0x00U) &&
        (host_xfer_byte(3U, 2U) == 0x01U) && (host_xfer_byte(3U, 3U) == 0x3FU),
        "RASET window is y = 0..319");

  CHECK((host_xfers[4].dc == 0U) && (host_xfer_byte(4U, 0U) == 0x2CU),
        "third command is RAMWR(0x2C)");

  for (i = 5U; i < host_xfer_count; i++)
  {
    CHECK(host_xfers[i].dc == 1U, "every pixel transfer uses DC = data");
    CHECK(host_xfers[i].len == 512U, "pixel chunks are 512 bytes");
    data_bytes += (uint32_t)host_xfers[i].len;
  }
  CHECK(data_bytes == (240U * 320U * 2U),
        "pixel payload is exactly 240*320*2 = 153600 bytes");
  CHECK(host_xfer_count == 5U + 300U, "153600 / 512 = 300 chunks");
  CHECK((host_xfer_byte(5U, 0U) == 0xF8U) && (host_xfer_byte(5U, 1U) == 0x00U),
        "RGB565 red 0xF800 is sent big endian first");
  CHECK((host_xfer_byte(host_xfer_count - 1U, 510U) == 0xF8U) &&
        (host_xfer_byte(host_xfer_count - 1U, 511U) == 0x00U),
        "last chunk keeps big endian order");

  if (failures != 0)
  {
    dump_xfers("FillColor");
  }
}

static const uint16_t test_image[4] = {0x1234U, 0x5678U, 0x9ABCU, 0xDEF0U};

static void test_draw_image(void)
{
  printf("test 2: LCD_DrawImage(10,20,2,2) window and byte order\n");
  host_reset();
  LCD_DrawImage(10U, 20U, 2U, 2U, test_image);

  CHECK(host_cs_low == 1U, "CS is asserted exactly once per picture");
  CHECK(host_xfer_count == 6U, "CASET, RASET, RAMWR and pixel transfer");

  CHECK(host_xfer_byte(0U, 0U) == 0x2AU, "CASET first");
  CHECK((host_xfer_byte(1U, 0U) == 0x00U) && (host_xfer_byte(1U, 1U) == 0x0AU) &&
        (host_xfer_byte(1U, 2U) == 0x00U) && (host_xfer_byte(1U, 3U) == 0x0BU),
        "CASET covers x = 10..11");
  CHECK(host_xfer_byte(2U, 0U) == 0x2BU, "RASET second");
  CHECK((host_xfer_byte(3U, 0U) == 0x00U) && (host_xfer_byte(3U, 1U) == 0x14U) &&
        (host_xfer_byte(3U, 2U) == 0x00U) && (host_xfer_byte(3U, 3U) == 0x15U),
        "RASET covers y = 20..21");
  CHECK(host_xfer_byte(4U, 0U) == 0x2CU, "RAMWR third");

  CHECK((host_xfers[5].dc == 1U) && (host_xfers[5].len == 8U),
        "4 pixels are sent as 8 data bytes");
  CHECK((host_xfer_byte(5U, 0U) == 0x12U) && (host_xfer_byte(5U, 1U) == 0x34U) &&
        (host_xfer_byte(5U, 2U) == 0x56U) && (host_xfer_byte(5U, 3U) == 0x78U) &&
        (host_xfer_byte(5U, 4U) == 0x9AU) && (host_xfer_byte(5U, 5U) == 0xBCU) &&
        (host_xfer_byte(5U, 6U) == 0xDEU) && (host_xfer_byte(5U, 7U) == 0xF0U),
        "pixel bytes keep row major big endian order");
}

static void test_bad_arguments(void)
{
  printf("test 3: out of range or null arguments must not touch the bus\n");
  host_reset();
  LCD_DrawImage(239U, 0U, 2U, 1U, test_image);
  LCD_DrawImage(0U, 319U, 1U, 2U, test_image);
  LCD_DrawImage(0U, 0U, 0U, 1U, test_image);
  LCD_DrawImage(0U, 0U, 1U, 1U, NULL);
  LCD_WriteData(NULL, 3U);
  CHECK(host_xfer_count == 0U, "no SPI transfer for rejected arguments");
  CHECK(host_cs_low == 0U, "CS stays idle for rejected arguments");
}

static void test_write_command_and_data(void)
{
  uint8_t payload[3] = {0x01U, 0x02U, 0x03U};

  printf("test 4: LCD_WriteCommand / LCD_WriteData transaction framing\n");
  host_reset();
  LCD_WriteCommand(0x01U);
  CHECK(host_cs_low == 1U, "WriteCommand asserts CS once");
  CHECK(host_cs_high == 1U, "WriteCommand releases CS once");
  CHECK((host_xfers[0].dc == 0U) && (host_xfers[0].len == 1U) &&
        (host_xfer_byte(0U, 0U) == 0x01U), "command byte goes out with DC low");
  CHECK(host_dc_level == 1U, "DC returns to data level after a command");

  host_reset();
  LCD_WriteData(payload, 3U);
  CHECK(host_cs_low == 1U, "WriteData asserts CS once");
  CHECK((host_xfers[0].dc == 1U) && (host_xfers[0].len == 3U),
        "data goes out with DC high");
  CHECK((host_xfer_byte(0U, 0U) == 0x01U) &&
        (host_xfer_byte(0U, 2U) == 0x03U), "data bytes are not reordered");
}

static void test_address_window(void)
{
  printf("test 5: LCD_SetAddressWindow keeps command and parameters together\n");
  host_reset();
  LCD_SetAddressWindow(0U, 0U, 239U, 319U);
  CHECK(host_cs_low == 1U, "both CASET and RASET run under one CS cycle");
  CHECK(host_xfer_count == 4U, "CASET + 4 argv + RASET + 4 argv");
  CHECK((host_xfers[0].dc == 0U) && (host_xfers[1].dc == 1U) &&
        (host_xfers[2].dc == 0U) && (host_xfers[3].dc == 1U),
        "DC toggles around each parameter block");
}

static void test_init_sequence(void)
{
  static const uint8_t expected[] =
  {
    0x01U, 0x11U, 0x3AU, 0x36U, 0xB2U, 0xB7U, 0xBBU, 0xC0U, 0xC2U,
    0xC3U, 0xC4U, 0xC6U, 0xD0U, 0xE0U, 0xE1U, 0x21U, 0x13U, 0x29U
  };
  uint8_t commands[64];
  uint32_t count;
  uint32_t expected_count = (uint32_t)(sizeof(expected) / sizeof(expected[0]));
  uint32_t i;

  printf("test 6: LCD_Init reset timing, command list and backlight\n");
  host_reset();
  LCD_Init();

  CHECK(host_rst_low == 1U, "RST is driven low once");
  CHECK(host_rst_high == 1U, "RST is released once");
  CHECK(host_delay_ms >= 310U, "reset and sleep out timing at least 310 ms");
  /* LCD_Reset 开头会先把 CS 拉高一次，所以 CS 高电平次数比低电平多 1。 */
  CHECK(host_cs_high == (host_cs_low + 1U),
        "every CS assertion is released again");

  count = collect_commands(commands, 64U);
  CHECK(count == expected_count,
        "init sends exactly the expected number of commands");
  for (i = 0U; (i < count) && (i < expected_count); i++)
  {
    checks++;
    if (commands[i] != expected[i])
    {
      failures++;
      printf("  [FAIL] command #%u is 0x%02X, expected 0x%02X\n",
             (unsigned)i, commands[i], expected[i]);
    }
  }

  CHECK(host_xfer_byte(3U, 0U) == 0x55U,
        "COLMOD(0x3A) selects 16 bit RGB565");
  CHECK(host_xfer_byte(5U, 0U) == (uint8_t)LCD_MADCTL_VALUE,
        "MADCTL(0x36) uses LCD_MADCTL_VALUE");
  CHECK(host_blk_level == 1U, "backlight is switched on at the end of init");

  if (failures != 0)
  {
    dump_xfers("LCD_Init");
  }
}

static void test_z_servo(void)
{
  printf("test 7: Z servo TIM2_CH2 (PA1) pulse clamping in microseconds\n");
  host_reset();
  htim2.Instance = &host_tim2_regs;

  ZServo_Init();
  CHECK(host_pwm_z_start == 1U, "ZServo_Init starts TIM2 channel 2");
  CHECK(host_pwm_grip_start == 0U, "ZServo_Init does not touch TIM1");
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_TRAVEL_US,
        "power on pulse is the travel height");
  CHECK(ZServo_GetPulse() == ZSERVO_PULSE_TRAVEL_US,
        "reported pulse is the travel height");

  ZServo_SetPulse(500U);
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_MIN_US,
        "below range is clamped to 1200 us");
  CHECK(ZServo_GetPulse() == ZSERVO_PULSE_MIN_US, "reported pulse follows clamp");

  ZServo_SetPulse(3000U);
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_MAX_US,
        "above range is clamped to 2500 us");

  ZServo_SetPulse(1750U);
  CHECK(host_tim2_regs.CCR2 == 1750U, "in range pulse is written unchanged");

  ZServo_Approach();
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_APPROACH_US,
        "Approach uses the APPROACH macro");
  ZServo_Grasp();
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_GRASP_US,
        "Grasp uses the GRASP macro");
  ZServo_Travel();
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_TRAVEL_US,
        "Travel uses the TRAVEL macro");
  ZServo_SetSafePosition();
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_TRAVEL_US,
        "safe position is the travel height");

  /* 台架点动接口（2026-10-27 为 Z 轴齿条啮合标定新增）：
   * 每调用一次改变 ZSERVO_STEP_US，到达限幅边界后停住；
   * 正式模式窗口 1000~2000 us，标定模式 800~2200 us，这里都用宏比较。 */
  ZServo_SetPulse(ZSERVO_PULSE_TRAVEL_US);
  ZServo_StepPulse(+1);
  CHECK(host_tim2_regs.CCR2 == (ZSERVO_PULSE_TRAVEL_US + ZSERVO_STEP_US),
        "StepPulse(+1) adds one ZSERVO_STEP_US");
  ZServo_StepPulse(-1);
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_TRAVEL_US,
        "StepPulse(-1) subtracts one ZSERVO_STEP_US");
  ZServo_SetPulse(ZSERVO_PULSE_MAX_US);
  ZServo_StepPulse(+1);
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_MAX_US,
        "StepPulse(+1) stops at the upper limit");
  ZServo_SetPulse(ZSERVO_PULSE_MIN_US);
  ZServo_StepPulse(-1);
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_MIN_US,
        "StepPulse(-1) stops at the lower limit");
  ZServo_StepPulse(0);
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_MIN_US,
        "StepPulse(0) leaves the pulse unchanged");

  /* 一键到位（2026-10-27 实测：最上 1200 us / 最下 2500 us）。
   * 台架按键 ZSERVO_BENCH_KEY2/3_JUMP=1 时 KEY2 到最下面、KEY3 到最上面。 */
  ZServo_JumpTo(ZSERVO_PULSE_BOTTOM_US);
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_BOTTOM_US,
        "JumpTo(BOTTOM) writes 2500 us directly");
  ZServo_JumpTo(ZSERVO_PULSE_TOP_US);
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_TOP_US,
        "JumpTo(TOP) writes 1200 us directly");
  CHECK(ZSERVO_PULSE_TOP_US == 1200U, "measured TOP endpoint is 1200 us");
  CHECK(ZSERVO_PULSE_BOTTOM_US == 2500U, "measured BOTTOM endpoint is 2500 us");
  CHECK(ZSERVO_PULSE_TOP_US < ZSERVO_PULSE_BOTTOM_US,
        "TOP pulse is smaller than BOTTOM pulse (smaller pulse = higher)");

  CHECK((ZSERVO_PULSE_TRAVEL_US < ZSERVO_PULSE_APPROACH_US) &&
        (ZSERVO_PULSE_APPROACH_US < ZSERVO_PULSE_GRASP_US),
        "height order is TRAVEL < APPROACH < GRASP");
}

static void test_gripper_servo(void)
{
  uint32_t i;
  uint32_t starts;

  /* BDS300 2026-10-25 重新标定：闭合临界 1400 us、张开临界 1750 us，
   * 正式值各留 20 us 余量（1420/1730）；换舵机后同步修改这里。
   * 2026-10-23 软启动：Init() 不启动 PWM、不写 CCR1，软件脉宽从安全中位
   * 1500 us 开始；第一次 Open()/Close() 先以 1500 us 启动 PWM，再由
   * Process() 按 50 us / 20 ms 缓动到目标，避免一帧跳 310 us 撞进端点。
   * 当前夹爪用 PA8 / TIM1_CH1 硬件 PWM，不再需要更新/比较中断。 */
  printf("test 8: gripper servo TIM1_CH1 (PA8) hardware PWM + soft start\n");
  host_reset();
  htim1.Instance = &host_tim1_regs;

  CHECK(SERVO_PWM_Pin == GPIO_PIN_8, "gripper PWM pin must be PA8");
  CHECK(SERVO_PWM_GPIO_Port == GPIOA, "gripper PWM port must be GPIOA");
  CHECK((SERVO_GRIPPER_START_PULSE_US > SERVO_GRIPPER_MIN_PULSE_US) &&
        (SERVO_GRIPPER_START_PULSE_US < SERVO_GRIPPER_MAX_PULSE_US),
        "safe start pulse sits strictly inside the 1400~1750 us window");

  ServoGripper_Init();
  CHECK(host_pwm_grip_start == 0U,
        "ServoGripper_Init does not start TIM1 channel 1");
  CHECK(host_pwm_z_start == 0U, "gripper init does not touch TIM2");
  CHECK(host_pwm_start_it == 0U,
        "gripper uses hardware PWM, not update/compare interrupts");
  CHECK(host_delay_ms == 0U, "ServoGripper_Init no longer blocks");
  CHECK(host_tim1_regs.CCR1 == 0U,
        "power-on leaves CCR1 untouched (no output while LCD_Init runs)");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_START_PULSE_US,
        "driver state starts from the safe mid pulse, not the end stop");
  CHECK(ServoGripper_IsClosed() == 0U, "gripper reports open after init");
  CHECK(host_gpio_pa8_mode == GPIO_MODE_OUTPUT_PP,
        "Init parks PA8 as a push-pull output (no floating servo signal)");
  CHECK(host_gpio_pa8_pull == GPIO_PULLDOWN,
        "parked PA8 has an internal pull-down");
  CHECK(host_gpio_pa8_level == 0U,
        "parked PA8 is driven low, same as an unplugged signal wire");

  /* 上电后第一次 Open()：启动 PWM，但第一帧必须是安全中位 1500 us，
   * 不能像以前那样直接写 1730 us（一帧跳完 310 us 行程）。 */
  ServoGripper_Open();
  CHECK(host_pwm_grip_start == 1U,
        "first Open starts TIM1 channel 1 when PWM is not running");
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_START_PULSE_US,
        "first Open energises at the safe mid pulse, not at the open end stop");
  CHECK(host_gpio_pa8_mode == GPIO_MODE_AF_PP,
        "first Open hands PA8 back to alternate function mode");
  CHECK(host_gpio_pa8_af == GPIO_AF1_TIM1,
        "PA8 alternate function is TIM1_CH1");
  CHECK(host_gpio_pa8_mode_at_pwm_start == GPIO_MODE_AF_PP,
        "PA8 is already AF1 when the PWM starts (no floating first frame)");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_START_PULSE_US,
        "Open only retargets, the slew loop moves the pulse");
  CHECK(ServoGripper_IsClosed() == 0U, "Open reports open");

  /* 缓动节拍：20 ms 未到不动，到了才走 SERVO_GRIPPER_SLEW_US。 */
  host_tick_ms += (SERVO_GRIPPER_SLEW_PERIOD_MS - 1U);
  ServoGripper_Process();
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_START_PULSE_US,
        "no slew step before the 20 ms period elapses");
  host_tick_ms += 1U;
  ServoGripper_Process();
  CHECK(ServoGripper_GetPulse() ==
            (uint16_t)(SERVO_GRIPPER_START_PULSE_US + SERVO_GRIPPER_SLEW_US),
        "one slew step changes the pulse by SERVO_GRIPPER_SLEW_US");

  for (i = 0U; i < 16U; ++i)
  {
    host_tick_ms += SERVO_GRIPPER_SLEW_PERIOD_MS;
    ServoGripper_Process();
  }
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_OPEN_PULSE_US,
        "slew reaches the open pulse and stops there");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_OPEN_PULSE_US,
        "reported pulse follows the slew");
  CHECK(host_pwm_grip_start == 1U,
        "Open reuses the already running PWM (no double start)");
  CHECK(ServoGripper_IsClosed() == 0U, "gripper reports open");

  /* 闭合同样是缓动：先不跳变，到位后 CCR1 才是 1420 us。 */
  ServoGripper_Close();
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_OPEN_PULSE_US,
        "Close retargets instead of jumping to the close pulse");
  CHECK(ServoGripper_IsClosed() == 1U,
        "Close updates the closed flag immediately");
  for (i = 0U; i < 16U; ++i)
  {
    host_tick_ms += SERVO_GRIPPER_SLEW_PERIOD_MS;
    ServoGripper_Process();
  }
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_CLOSE_PULSE_US,
        "slew reaches the close pulse and stops there");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_CLOSE_PULSE_US,
        "reported pulse follows Close");

  /* 台架按键一次到位：JumpTo 仍直接写 CCR1，不经过缓动。 */
  ServoGripper_JumpTo(SERVO_GRIPPER_MAX_PULSE_US, 0U);
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_MAX_PULSE_US,
        "JumpTo writes the requested pulse to CCR1 immediately");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_MAX_PULSE_US,
        "reported pulse follows JumpTo");
  CHECK(ServoGripper_IsClosed() == 0U,
        "JumpTo updates the closed flag (open)");

  ServoGripper_JumpTo(1000U, 1U);
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_MIN_PULSE_US,
        "JumpTo clamps below the measured 1400 us mechanical window");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_MIN_PULSE_US,
        "reported pulse follows the JumpTo clamp");
  CHECK(ServoGripper_IsClosed() == 1U,
        "JumpTo updates the closed flag (closed)");

  /* 台架场景：上电后（PWM 未启动）第一次按键直接 JumpTo，也必须自动补启动。 */
  starts = host_pwm_grip_start;
  ServoGripper_Init();
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_START_PULSE_US,
        "re-Init returns the driver state to the safe mid pulse");
  ServoGripper_JumpTo(SERVO_GRIPPER_CLOSE_PULSE_US, 1U);
  CHECK(host_pwm_grip_start == (starts + 1U),
        "first JumpTo after power-on starts TIM1 channel 1");
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_CLOSE_PULSE_US,
        "first JumpTo writes the requested pulse to CCR1");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_CLOSE_PULSE_US,
        "first JumpTo updates the reported pulse");
  CHECK(ServoGripper_IsClosed() == 1U,
        "first JumpTo updates the closed flag");

  CHECK((SERVO_GRIPPER_OPEN_PULSE_US - SERVO_GRIPPER_CLOSE_PULSE_US) == 310U,
        "open-close travel is 310 us (350 us measured, 20 us margin each side)");
  CHECK((SERVO_GRIPPER_CLOSE_PULSE_US >= SERVO_GRIPPER_MIN_PULSE_US) &&
        (SERVO_GRIPPER_OPEN_PULSE_US <= SERVO_GRIPPER_MAX_PULSE_US),
        "both positions stay inside the measured 1400~1750 us window");
}

/* 霍尔开关接 PD12/T_HOME：内部上拉，低有效（有磁铁 = 低）。
 * 仿真只复核引脚映射、读取和极性换算，真实电平原机用万用表
 * 在 PD12 上确认。 */
static void test_hall_sensor(void)
{
  printf("test 9: hall sensor PD12/T_HOME read and active-low mapping\n");

  host_reset();

  CHECK((HALL_SENSOR_GPIO_Port == GPIOD) && (HALL_SENSOR_Pin == GPIO_PIN_12),
        "hall sensor is mapped to PD12 (T_HOME)");
  CHECK(HALL_SENSOR_ACTIVE_HIGH == 0,
        "hall sensor is active low (open collector)");

  host_hall_level = 1U;
  CHECK(HallSensor_ReadRaw() == GPIO_PIN_SET,
        "no magnet: PD12 is pulled high (3.3 V)");
  CHECK(HallSensor_IsTriggered() == 0U,
        "no magnet: sensor reports not triggered");

  host_hall_level = 0U;
  CHECK(HallSensor_ReadRaw() == GPIO_PIN_RESET,
        "magnet near: PD12 is pulled low (0 V)");
  CHECK(HallSensor_IsTriggered() == 1U,
        "magnet near: sensor reports triggered");
}

/* T 轴仓距：16640 不能被 6 整除，必须四舍五入分配，
 * 六段之和严格等于一圈，否则每次 6 仓循环会累积 2 步误差。 */
static void test_turret_bin_math(void)
{
  uint32_t i;
  uint32_t total = 0U;

  printf("test 10: turret bin steps are exact (no 2-step/rev drift)\n");

  CHECK(SORT_T_STEPS_PER_REV == 16640U,
        "200 full steps * 16 microstep * 5.2 = 16640 steps/rev");
  CHECK(SORT_T_STEPS_PER_BIN == 2773U, "nominal steps per bin is 2773");

  CHECK(SORT_T_BIN_STEP(0) == 0, "bin 1 is at the home zero");
  CHECK(SORT_T_BIN_STEP(1) == 2773, "bin 2 offset");
  CHECK(SORT_T_BIN_STEP(2) == 5547, "bin 3 offset");
  CHECK(SORT_T_BIN_STEP(3) == 8320, "bin 4 offset");
  CHECK(SORT_T_BIN_STEP(4) == 11093, "bin 5 offset");
  CHECK(SORT_T_BIN_STEP(5) == 13867, "bin 6 offset");
  CHECK(SORT_T_BIN_STEP(SORT_T_BOX_COUNT) ==
            (int32_t)SORT_T_STEPS_PER_REV,
        "one past the last bin is exactly one revolution");

  for (i = 0U; i < (uint32_t)SORT_T_BOX_COUNT; ++i)
  {
    int32_t next = (i + 1U < (uint32_t)SORT_T_BOX_COUNT)
                       ? SORT_T_BIN_STEP(i + 1U)
                       : (int32_t)SORT_T_STEPS_PER_REV;
    int32_t pitch = next - SORT_T_BIN_STEP(i);

    CHECK(pitch > 0, "bin pitches are positive");
    total += (uint32_t)pitch;
  }
  CHECK(total == SORT_T_STEPS_PER_REV,
        "six bin pitches sum to exactly one revolution");
}

/* X/Y 零点限位开关：X=PD6、Y=PD5，NO→IO、C→GND，内部上拉、低有效。
 * 仿真只验证引脚映射、极性换算和消抖计时；真实电平必须上电后用
 * 万用表在 PD6/PD5 上量（未压到约 3.3 V，压到约 0 V）。 */
static void test_axis_home(void)
{
  printf("test 11: axis home switches PD6(X)/PD5(Y), active low + debounce\n");

  host_reset();
  AxisHome_ResetDebounce();

  CHECK((X_HOME_GPIO_Port == GPIOD) && (X_HOME_Pin == GPIO_PIN_6),
        "X home switch is mapped to PD6");
  CHECK((Y_HOME_GPIO_Port == GPIOD) && (Y_HOME_Pin == GPIO_PIN_5),
        "Y home switch is mapped to PD5");
  CHECK(AXIS_HOME_ACTIVE_HIGH == 0,
        "home switches are active low (NO to GND)");

  CHECK(AxisHome_ReadRaw(AXIS_HOME_X) == GPIO_PIN_SET,
        "X idle: PD6 is pulled high");
  CHECK(AxisHome_IsTriggered(AXIS_HOME_X) == 0U,
        "X idle: reports not triggered");

  host_home_level[0] = 0U;
  CHECK(AxisHome_ReadRaw(AXIS_HOME_X) == GPIO_PIN_RESET,
        "X pressed: PD6 is pulled low");
  CHECK(AxisHome_IsTriggered(AXIS_HOME_X) == 1U,
        "X pressed: reports triggered");

  /* 消抖：刚变低不算触发，连续 15 ms 才算。 */
  AxisHome_ResetDebounce();
  host_tick_ms = 1000U;
  CHECK(AxisHome_TriggeredDebounced(AXIS_HOME_X,
                                    SORT_XY_HOME_DEBOUNCE_MS) == 0U,
        "first sample only opens the debounce window");
  host_tick_ms = 1014U;
  CHECK(AxisHome_TriggeredDebounced(AXIS_HOME_X,
                                    SORT_XY_HOME_DEBOUNCE_MS) == 0U,
        "14 ms is shorter than the 15 ms debounce");
  host_tick_ms = 1015U;
  CHECK(AxisHome_TriggeredDebounced(AXIS_HOME_X,
                                    SORT_XY_HOME_DEBOUNCE_MS) == 1U,
        "15 ms of continuous low is a valid trigger");

  /* 中途抖回高电平：计时必须重新开始。 */
  host_home_level[0] = 1U;
  CHECK(AxisHome_TriggeredDebounced(AXIS_HOME_X,
                                    SORT_XY_HOME_DEBOUNCE_MS) == 0U,
        "release clears the trigger debounce");
  host_home_level[0] = 0U;
  host_tick_ms = 1020U;
  CHECK(AxisHome_TriggeredDebounced(AXIS_HOME_X,
                                    SORT_XY_HOME_DEBOUNCE_MS) == 0U,
        "bounce restarts the debounce window");
  host_tick_ms = 1035U;
  CHECK(AxisHome_TriggeredDebounced(AXIS_HOME_X,
                                    SORT_XY_HOME_DEBOUNCE_MS) == 1U,
        "clean 15 ms window is accepted again");

  /* Y 轴独立计时：X 的消抖状态不能影响 Y。 */
  host_home_level[1] = 0U;
  AxisHome_ResetDebounce();
  host_tick_ms = 2000U;
  CHECK(AxisHome_TriggeredDebounced(AXIS_HOME_Y,
                                    SORT_XY_HOME_DEBOUNCE_MS) == 0U,
        "Y debounce starts independently");
  host_tick_ms = 2015U;
  CHECK(AxisHome_TriggeredDebounced(AXIS_HOME_Y,
                                    SORT_XY_HOME_DEBOUNCE_MS) == 1U,
        "Y trigger accepted after 15 ms");

  /* 释放消抖：回零退让阶段用它确认已经离开开关。 */
  host_home_level[0] = 1U;
  AxisHome_ResetDebounce();
  host_tick_ms = 3000U;
  CHECK(AxisHome_ReleasedDebounced(AXIS_HOME_X,
                                   SORT_XY_HOME_DEBOUNCE_MS) == 0U,
        "release debounce starts");
  host_tick_ms = 3015U;
  CHECK(AxisHome_ReleasedDebounced(AXIS_HOME_X,
                                   SORT_XY_HOME_DEBOUNCE_MS) == 1U,
        "15 ms of continuous high is a valid release");

  /* 非法轴编号必须安全返回，不能误判为已触发。 */
  CHECK(AxisHome_IsTriggered(AXIS_HOME_COUNT) == 0U,
        "invalid axis id never reports triggered");
  CHECK(AxisHome_TriggeredDebounced(AXIS_HOME_COUNT, 15U) == 0U,
        "invalid axis id debounce returns 0");
}

/* USART3 台架控制台：电脑串口发单字符 = 虚拟按键。
 * 仿真只验证命令解析、短按/长按的保持时间、释放和输出文本；
 * 真机上还要确认 PB11（USART3_RX）接到 USB-TTL 的 TX、115200 8N1，
 * 以及按键动作仍受“安全 + 无任务”门控。 */
static void test_bench_console(void)
{
  uint8_t pressed;

  printf("test 12: bench console (USART3) virtual keys and commands\n");

  host_reset();
  BenchConsole_Init();
  CHECK(host_uart_rx_arm_calls == 1U,
        "Init arms the USART3 single-byte RX interrupt once");
  CHECK(strstr(host_uart_text(), "BENCH CONSOLE READY") != NULL,
        "Init prints the console banner");

  /* 短按 KEY2：100 ms 内保持按下，之后自动释放，其他键不受影响。 */
  host_tick_ms = 1000U;
  BenchConsole_FeedByte((uint8_t)'2');
  BenchConsole_Process();
  CHECK(strstr(host_uart_text(), "TAP K2") != NULL, "'2' echoes TAP K2");
  CHECK(BenchConsole_GetVirtualKey(2U, &pressed) == 1U, "'2' presses KEY2");
  CHECK(pressed == 1U, "virtual KEY2 level is pressed");
  CHECK(BenchConsole_GetVirtualKey(0U, &pressed) == 0U,
        "KEY0 is untouched by '2'");
  host_tick_ms = 1099U;
  CHECK(BenchConsole_GetVirtualKey(2U, &pressed) == 1U,
        "KEY2 still pressed at 99 ms");
  host_tick_ms = 1100U;
  CHECK(BenchConsole_GetVirtualKey(2U, &pressed) == 0U,
        "KEY2 auto-releases after 100 ms");

  /* 长按 KEY0：1500 ms，覆盖“按住满 1 s 存投放口”的判据。 */
  host_tick_ms = 2000U;
  BenchConsole_FeedByte((uint8_t)'A');
  BenchConsole_Process();
  CHECK(strstr(host_uart_text(), "HOLD K0") != NULL, "'A' echoes HOLD K0");
  host_tick_ms = 3000U;
  CHECK((BenchConsole_GetVirtualKey(0U, &pressed) == 1U) && (pressed == 1U),
        "KEY0 is still held after 1000 ms (drop-save window)");
  host_tick_ms = 3500U;
  CHECK(BenchConsole_GetVirtualKey(0U, &pressed) == 0U,
        "KEY0 auto-releases after 1500 ms");

  /* x 立即释放；越界/空指针安全返回。 */
  host_tick_ms = 4000U;
  BenchConsole_FeedByte((uint8_t)'B');
  BenchConsole_Process();
  CHECK(BenchConsole_GetVirtualKey(1U, &pressed) == 1U, "'B' presses KEY1");
  BenchConsole_FeedByte((uint8_t)'x');
  BenchConsole_Process();
  CHECK(strstr(host_uart_text(), "RELEASE ALL") != NULL,
        "'x' reports release");
  CHECK(BenchConsole_GetVirtualKey(1U, &pressed) == 0U,
        "'x' releases the virtual key immediately");
  CHECK(BenchConsole_GetVirtualKey(9U, &pressed) == 0U,
        "out-of-range key index is safe");
  CHECK(BenchConsole_GetVirtualKey(0U, NULL) == 0U,
        "NULL output pointer is safe");

  /* 帮助、未知命令、串口助手附带的 CR/LF。 */
  host_reset();
  BenchConsole_FeedByte((uint8_t)'h');
  BenchConsole_Process();
  CHECK(strstr(host_uart_text(), "0..3") != NULL, "'h' prints the help text");

  host_reset();
  BenchConsole_FeedByte((uint8_t)'z');
  BenchConsole_Process();
  CHECK(strstr(host_uart_text(), "UNKNOWN CMD") != NULL,
        "unknown command is reported");

  host_reset();
  BenchConsole_FeedByte((uint8_t)'\r');
  BenchConsole_FeedByte((uint8_t)'\n');
  BenchConsole_Process();
  CHECK(host_uart_tx_count == 0U,
        "CR/LF from a serial terminal produces no output");
}

/* USART3 台架运动指令：X/T/Z/G/R。
 * 仿真只验证“命令解析 + 目标值 + 序列时序”，真实行程/方向仍需实机确认。 */
static void feed_command(const char *text)
{
  while (*text != '\0')
  {
    (void)BenchMotion_HandleByte((uint8_t)*text);
    ++text;
  }
  /* 回车终结命令：数字命令是在“遇到非数字”时才执行的。 */
  (void)BenchMotion_HandleByte((uint8_t)'\n');
}

static void test_bench_motion(void)
{
  printf("test 13: bench motion commands (USART3 X/T/Z/G/R)\n");

  host_reset();
  StepperMotor_Init();
  ZServo_Init();
  ServoGripper_Init();
  SafetyManager_Init();
  BenchMotion_Init();

  /* 与 SortingTask_ApplyMotionConfig() 相同的速度和软限位。 */
  StepperMotor_SetSpeed(STEPPER_X, SORT_XY_MAX_SPEED_STEPS_S);
  StepperMotor_SetAcceleration(STEPPER_X, SORT_XY_ACCEL_STEPS_S2);
  StepperMotor_SetSoftLimit(STEPPER_X, SORT_X_MIN_STEP, SORT_X_MAX_STEP);
  StepperMotor_SetSpeed(STEPPER_T, SORT_T_MAX_SPEED_STEPS_S);
  StepperMotor_SetAcceleration(STEPPER_T, SORT_T_ACCEL_STEPS_S2);
  StepperMotor_SetSoftLimit(STEPPER_T, SORT_T_MIN_STEP, SORT_T_MAX_STEP);
  StepperMotor_SetPosition(STEPPER_X, 0);
  StepperMotor_SetPosition(STEPPER_T, 0);
  host_mission_active = 0U;

  /* X<步数>：只设目标，不阻塞 */
  feed_command("X1200");
  CHECK(StepperMotor_IsBusy(STEPPER_X) == 1U, "X1200 starts an X move");
  CHECK(StepperMotor_GetRemaining(STEPPER_X) == 1200,
        "X1200 targets 1200 absolute steps");
  CHECK(BenchMotion_GetLastX() == 1200, "last X is cached for R");

  /* 单字符按键仍然归按键控制台，不能被运动解析吞掉 */
  CHECK(BenchMotion_HandleByte((uint8_t)'2') == 0U,
        "digit '2' is not consumed by the motion parser");

  /* 软限位/数值范围 */
  StepperMotor_Stop(STEPPER_X);
  feed_command("X30000");
  CHECK(StepperMotor_IsBusy(STEPPER_X) == 0U,
        "X30000 is rejected by the soft limit");

  /* Z：ZT/ZB/Z<us>，越界不改脉宽 */
  feed_command("ZT");
  CHECK(host_tim2_regs.CCR2 == 1200U, "ZT moves Z to the top (1200 us)");
  feed_command("ZB");
  CHECK(host_tim2_regs.CCR2 == 2500U, "ZB moves Z to the bottom (2500 us)");
  feed_command("Z2200");
  CHECK(host_tim2_regs.CCR2 == 2200U, "Z2200 sets an explicit pulse");
  feed_command("Z9999");
  CHECK(host_tim2_regs.CCR2 == 2200U, "Z9999 is rejected, pulse unchanged");

  /* 夹爪：GO/GC 直接到位，G<us> 越界被拒 */
  feed_command("GO");
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_OPEN_PULSE_US,
        "GO opens the gripper");
  feed_command("GC");
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_CLOSE_PULSE_US,
        "GC closes the gripper");
  feed_command("G100");
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_CLOSE_PULSE_US,
        "G100 is rejected by the pulse window");

  /* 转盘：T<1..6>，按仓表单向补角 */
  host_reset();
  feed_command("T9");
  CHECK(strstr(host_uart_text(), "REJECT BIN") != NULL,
        "T9 reports an invalid bin");
  CHECK(StepperMotor_IsBusy(STEPPER_T) == 0U, "T9 does not move the turret");
  feed_command("T3");
  CHECK(StepperMotor_IsBusy(STEPPER_T) == 1U, "T3 starts the turret");
  CHECK(StepperMotor_GetRemaining(STEPPER_T) == SORT_T_BIN_STEP(2),
        "T3 targets bin 3 (5547 steps from zero)");
  CHECK(BenchMotion_GetLastBin() == 3U, "last bin is cached for R");

  /* 快进到仓 3，让转盘空闲 */
  StepperMotor_SetPosition(STEPPER_T, SORT_T_BIN_STEP(2));
  BenchMotion_Process();

  /* 全流程：X 与 T 同时起步 */
  feed_command("R1200,1");
  CHECK(BenchMotion_IsBusy() == 1U, "R1200,1 starts the sequence");
  CHECK(StepperMotor_IsBusy(STEPPER_X) == 1U, "sequence moves X");
  CHECK(StepperMotor_GetRemaining(STEPPER_X) == 1200,
        "sequence X target is 1200");
  CHECK(StepperMotor_IsBusy(STEPPER_T) == 1U,
        "sequence starts the turret at the same time");

  /* 序列运行期间，直接运动指令被拒 */
  feed_command("X100");
  CHECK(StepperMotor_GetRemaining(STEPPER_X) == 1200,
        "X100 is rejected while the sequence runs");

  /* 快进 X 到位 -> 开爪 -> Z 下降 -> 闭爪 -> Z 上升 -> 等转盘 */
  StepperMotor_SetPosition(STEPPER_X, 1200);
  BenchMotion_Process();
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_OPEN_PULSE_US,
        "sequence opens the gripper when X arrives");

  host_tick_ms += BENCH_MOTION_GRIP_SETTLE_MS;
  BenchMotion_Process();
  CHECK(host_tim2_regs.CCR2 == 2500U, "sequence lowers Z to the bottom");

  host_tick_ms += BENCH_MOTION_Z_SETTLE_MS;
  BenchMotion_Process();
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_CLOSE_PULSE_US,
        "sequence closes the gripper at the bottom");

  host_tick_ms += BENCH_MOTION_GRIP_SETTLE_MS;
  BenchMotion_Process();
  CHECK(host_tim2_regs.CCR2 == 1220U,
        "sequence lifts Z to the travel height (1220 us)");

  host_tick_ms += BENCH_MOTION_Z_SETTLE_MS;
  BenchMotion_Process();
  CHECK(BenchMotion_IsBusy() == 1U, "sequence waits for the turret");

  StepperMotor_SetPosition(STEPPER_T, 0);
  BenchMotion_Process();
  CHECK(BenchMotion_IsBusy() == 0U, "sequence finishes when the turret is idle");

  /* 自动任务抢占：序列必须停轴退出 */
  feed_command("R1200,2");
  CHECK(BenchMotion_IsBusy() == 1U, "R1200,2 starts another sequence");
  host_mission_active = 1U;
  BenchMotion_Process();
  CHECK(BenchMotion_IsBusy() == 0U, "a mission preempts the sequence");
  CHECK(StepperMotor_IsBusy(STEPPER_T) == 0U,
        "preemption stops the turret pulses");
  host_mission_active = 0U;

  /* RA 中止 */
  feed_command("R1200,3");
  CHECK(BenchMotion_IsBusy() == 1U, "sequence restarts");
  feed_command("RA");
  CHECK(BenchMotion_IsBusy() == 0U, "RA aborts the sequence");

  /* 中止后单轴指令恢复可用 */
  feed_command("X500");
  CHECK(StepperMotor_IsBusy(STEPPER_X) == 1U, "X500 works after the abort");
  CHECK(StepperMotor_GetRemaining(STEPPER_X) == 700,
        "X500 moves from 1200 back to 500");

  /* 行解析细节：空格、单独查询、坏参数、超时丢弃 */
  StepperMotor_SetPosition(STEPPER_X, 500);
  host_reset();
  feed_command("X 800");
  CHECK(StepperMotor_GetRemaining(STEPPER_X) == 300,
        "X 800 (with a space) moves 300 steps from 500");
  StepperMotor_Stop(STEPPER_X);

  host_reset();
  feed_command("X");
  CHECK(strstr(host_uart_text(), "X POS=") != NULL,
        "bare X prints the current position");

  host_reset();
  feed_command("X12x");
  CHECK(strstr(host_uart_text(), "BAD ARGUMENT") != NULL,
        "trailing junk is reported");
  CHECK(StepperMotor_IsBusy(STEPPER_X) == 0U,
        "a bad command does not move the axis");

  /* 半条命令超时丢弃后，0~3 仍然能当按键用 */
  host_reset();
  (void)BenchMotion_HandleByte((uint8_t)'X');
  (void)BenchMotion_HandleByte((uint8_t)'1');
  host_tick_ms += BENCH_MOTION_LINE_TIMEOUT_MS + 1U;
  BenchMotion_Process();
  CHECK(strstr(host_uart_text(), "DROPPED") != NULL,
        "a partial command is dropped after the timeout");
  CHECK(BenchMotion_HandleByte((uint8_t)'2') == 0U,
        "keys work again after a dropped command");

  /* R 允许空格和逗号 */
  host_reset();
  feed_command("R 1000 , 2");
  CHECK(BenchMotion_IsBusy() == 1U, "R 1000 , 2 accepts spaces and comma");
  feed_command("RA");
  CHECK(BenchMotion_IsBusy() == 0U, "RA aborts the spaced R command");
}

int main(void)
{
  printf("== host simulation for lcd_st7789 / servo_z / servo_gripper / hall_sensor / axis_home / bench_console / bench_motion ==\n");

  test_fill_color();
  test_draw_image();
  test_bad_arguments();
  test_write_command_and_data();
  test_address_window();
  test_init_sequence();
  test_z_servo();
  test_gripper_servo();
  test_hall_sensor();
  test_turret_bin_math();
  test_axis_home();
  test_bench_console();
  test_bench_motion();

  printf("== checks=%d failures=%d ==\n", checks, failures);
  if (failures == 0)
  {
    printf("RESULT: PASS\n");
    return 0;
  }
  printf("RESULT: FAIL\n");
  return 1;
}
