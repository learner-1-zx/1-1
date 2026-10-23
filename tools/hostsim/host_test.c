/* 主机仿真测试：不接硬件，复核 ST7789V2 驱动和 Z 轴舵机驱动的行为。
 * 用真实 HAL 头文件编译，链接 host_stub.c 提供的 HAL 桩函数。 */
#include "host_stub.h"
#include "tim.h"
#include "lcd_st7789.h"
#include "servo_z.h"
#include "servo_gripper.h"
#include "hall_sensor.h"
#include "sorting_config.h"
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
        "below range is clamped to 1000 us");
  CHECK(ZServo_GetPulse() == ZSERVO_PULSE_MIN_US, "reported pulse follows clamp");

  ZServo_SetPulse(2500U);
  CHECK(host_tim2_regs.CCR2 == ZSERVO_PULSE_MAX_US,
        "above range is clamped to 2000 us");

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

  CHECK((ZSERVO_PULSE_TRAVEL_US < ZSERVO_PULSE_APPROACH_US) &&
        (ZSERVO_PULSE_APPROACH_US < ZSERVO_PULSE_GRASP_US),
        "height order is TRAVEL < APPROACH < GRASP");
}

static void test_gripper_servo(void)
{
  /* BDS300 2026-10-20 实测：闭合临界 1390 us、张开临界 1800 us，
   * 正式值各留 20 us 余量（1410/1780）；换舵机后同步修改这里。
   * 上电 ServoGripper_Init() 不启动 PWM、不写 CCR1：夹爪位置由状态机
   * 在 SORT_SYS_INIT 里调用 Open() 决定，避免先摆到 1500 us 再张开。
   * 当前夹爪用 PA8 / TIM1_CH1 硬件 PWM，不再需要更新/比较中断。 */
  printf("test 8: gripper servo TIM1_CH1 (PA8) hardware PWM\n");
  host_reset();
  htim1.Instance = &host_tim1_regs;

  CHECK(SERVO_PWM_Pin == GPIO_PIN_8, "gripper PWM pin must be PA8");
  CHECK(SERVO_PWM_GPIO_Port == GPIOA, "gripper PWM port must be GPIOA");

  ServoGripper_Init();
  CHECK(host_pwm_grip_start == 0U,
        "ServoGripper_Init no longer starts TIM1 channel 1");
  CHECK(host_pwm_z_start == 0U, "gripper init does not touch TIM2");
  CHECK(host_pwm_start_it == 0U,
        "gripper uses hardware PWM, not update/compare interrupts");
  CHECK(host_delay_ms == 0U, "ServoGripper_Init no longer blocks");
  CHECK(host_tim1_regs.CCR1 == 0U,
        "power-on leaves CCR1 untouched (no 1500 us intermediate position)");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_OPEN_PULSE_US,
        "driver state starts from the open pulse");
  CHECK(ServoGripper_IsClosed() == 0U, "gripper reports open after init");

  /* 上电后第一件事就是按 KEY1 的场景：JumpTo 必须自己补启动 PWM，
   * 并且第一帧就是目标脉宽（不能先输出 1500 或旧值）。 */
  ServoGripper_JumpTo(SERVO_GRIPPER_CLOSE_PULSE_US, 1U);
  CHECK(host_pwm_grip_start == 1U,
        "first JumpTo starts TIM1 channel 1 when PWM is not running");
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_CLOSE_PULSE_US,
        "first JumpTo writes the close pulse to CCR1");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_CLOSE_PULSE_US,
        "reported pulse follows the first JumpTo");
  CHECK(ServoGripper_IsClosed() == 1U,
        "first JumpTo updates the closed flag (closed)");

  ServoGripper_Open();
  CHECK(host_pwm_grip_start == 1U,
        "Open reuses the already running PWM (no double start)");
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_OPEN_PULSE_US,
        "Open writes the open pulse to CCR1");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_OPEN_PULSE_US,
        "reported pulse follows Open");
  CHECK(ServoGripper_IsClosed() == 0U, "gripper reports open");

  ServoGripper_Close();
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_CLOSE_PULSE_US,
        "Close writes the close pulse to CCR1");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_CLOSE_PULSE_US,
        "reported pulse follows Close");
  CHECK(ServoGripper_IsClosed() == 1U, "gripper reports closed");

  /* 台架按键一次到位：JumpTo 直接写 CCR1，不经过 5 us/20 ms 缓动。 */
  ServoGripper_JumpTo(SERVO_GRIPPER_MAX_PULSE_US, 0U);
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_MAX_PULSE_US,
        "JumpTo writes the requested pulse to CCR1 immediately");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_MAX_PULSE_US,
        "reported pulse follows JumpTo");
  CHECK(ServoGripper_IsClosed() == 0U,
        "JumpTo updates the closed flag (open)");

  ServoGripper_JumpTo(1000U, 1U);
  CHECK(host_tim1_regs.CCR1 == SERVO_GRIPPER_MIN_PULSE_US,
        "JumpTo clamps below the measured 1390 us mechanical window");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_MIN_PULSE_US,
        "reported pulse follows the JumpTo clamp");
  CHECK(ServoGripper_IsClosed() == 1U,
        "JumpTo updates the closed flag (closed)");

  CHECK((SERVO_GRIPPER_OPEN_PULSE_US - SERVO_GRIPPER_CLOSE_PULSE_US) == 370U,
        "open-close travel is 370 us (410 us measured, 20 us margin each side)");
  CHECK((SERVO_GRIPPER_CLOSE_PULSE_US >= SERVO_GRIPPER_MIN_PULSE_US) &&
        (SERVO_GRIPPER_OPEN_PULSE_US <= SERVO_GRIPPER_MAX_PULSE_US),
        "both positions stay inside the measured 1390~1800 us window");
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

int main(void)
{
  printf("== host simulation for lcd_st7789 / servo_z / servo_gripper / hall_sensor ==\n");

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

  printf("== checks=%d failures=%d ==\n", checks, failures);
  if (failures == 0)
  {
    printf("RESULT: PASS\n");
    return 0;
  }
  printf("RESULT: FAIL\n");
  return 1;
}
