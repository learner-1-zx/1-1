/* 主机仿真测试：不接硬件，复核 ST7789V2 驱动和 Z 轴舵机驱动的行为。
 * 用真实 HAL 头文件编译，链接 host_stub.c 提供的 HAL 桩函数。 */
#include "host_stub.h"
#include "tim.h"
#include "lcd_st7789.h"
#include "servo_z.h"
#include "servo_gripper.h"
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
  printf("test 7: Z servo TIM1_CH1 pulse clamping in microseconds\n");
  host_reset();
  htim1.Instance = &host_tim1_regs;

  ZServo_Init();
  CHECK(host_pwm_start_ch1 == 1U, "ZServo_Init starts TIM1 channel 1");
  CHECK(host_tim1_regs.CCR1 == ZSERVO_PULSE_SAFE_US,
        "power on pulse is 1500 us");
  CHECK(ZServo_GetPulse() == ZSERVO_PULSE_SAFE_US, "reported pulse is 1500 us");

  ZServo_SetPulse(500U);
  CHECK(host_tim1_regs.CCR1 == ZSERVO_PULSE_MIN_US,
        "below range is clamped to 1000 us");
  CHECK(ZServo_GetPulse() == ZSERVO_PULSE_MIN_US, "reported pulse follows clamp");

  ZServo_SetPulse(2500U);
  CHECK(host_tim1_regs.CCR1 == ZSERVO_PULSE_MAX_US,
        "above range is clamped to 2000 us");

  ZServo_SetPulse(1750U);
  CHECK(host_tim1_regs.CCR1 == 1750U, "in range pulse is written unchanged");

  ZServo_Up();
  CHECK(host_tim1_regs.CCR1 == ZSERVO_PULSE_UP_US, "Up uses the UP macro");
  ZServo_Down();
  CHECK(host_tim1_regs.CCR1 == ZSERVO_PULSE_DOWN_US, "Down uses the DOWN macro");
  ZServo_SetSafePosition();
  CHECK(host_tim1_regs.CCR1 == ZSERVO_PULSE_SAFE_US, "safe position is 1500 us");
}

static void test_gripper_servo(void)
{
  /* 注意：1170/1870 us 是 SG90 试装的临时值；夹爪最终选型为 BDS300，
   * 标定后需要同步修改这里的断言。
   * PD8 没有定时器通道，这里模拟 TIM1 更新 / CH2 比较两次中断，
   * 验证软件 PWM 的高电平、CCR2 和低电平顺序。 */
  printf("test 8: gripper software PWM on PD8 via TIM1_CH2 compare (production init)\n");
  host_reset();
  CHECK(SERVO_PWM_Pin == GPIO_PIN_8, "gripper PWM pin must be PD8");
  CHECK(SERVO_PWM_GPIO_Port == GPIOD, "gripper PWM port must be GPIOD");

  ServoGripper_Init();
  CHECK(host_pwm_start_ch2 == 1U,
        "ServoGripper_Init starts TIM1 channel 2 compare interrupt");
  CHECK(host_pwm_start_ch1 == 0U, "gripper no longer drives TIM1 channel 1");
  CHECK((host_tim1_regs.DIER & TIM_IT_CC2) != 0U,
        "ServoGripper_Init enables the TIM1 CH2 compare interrupt");
  CHECK((host_tim1_regs.DIER & TIM_IT_UPDATE) != 0U,
        "ServoGripper_Init also enables the TIM1 update interrupt");
  CHECK(host_delay_ms == 0U, "ServoGripper_Init no longer blocks for a self test");
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_OPEN_PULSE_US,
        "power-on default is the open pulse");
  CHECK(ServoGripper_IsClosed() == 0U, "gripper reports open after init");

  /* 模拟 TIM1 更新中断：写入 CCR2 并把 PD8 拉高。 */
  ServoGripper_TimerUpdateIsr();
  CHECK(host_tim1_regs.CCR2 == SERVO_GRIPPER_OPEN_PULSE_US,
        "update ISR writes the open pulse to CCR2");
  CHECK(host_servo_level == 1U, "update ISR drives PD8 high");

  /* 模拟 TIM1_CH2 比较匹配：PD8 拉低。 */
  htim1.Channel = HAL_TIM_ACTIVE_CHANNEL_2;
  HAL_TIM_PWM_PulseFinishedCallback(&htim1);
  CHECK(host_servo_level == 0U, "CC2 callback drives PD8 low");

  ServoGripper_Close();
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_CLOSE_PULSE_US,
        "Close selects the close pulse");
  CHECK(ServoGripper_IsClosed() == 1U, "gripper reports closed");
  ServoGripper_TimerUpdateIsr();
  CHECK(host_tim1_regs.CCR2 == SERVO_GRIPPER_CLOSE_PULSE_US,
        "next update ISR applies the close pulse");
  CHECK(host_servo_level == 1U, "PD8 is high during the close pulse");

  htim1.Channel = HAL_TIM_ACTIVE_CHANNEL_2;
  HAL_TIM_PWM_PulseFinishedCallback(&htim1);
  CHECK(host_servo_level == 0U, "CC2 callback ends the close pulse");

  ServoGripper_Open();
  CHECK(ServoGripper_GetPulse() == SERVO_GRIPPER_OPEN_PULSE_US,
        "Open selects the open pulse");
  CHECK(ServoGripper_IsClosed() == 0U, "gripper reports open");

  CHECK((SERVO_GRIPPER_OPEN_PULSE_US - SERVO_GRIPPER_CLOSE_PULSE_US) == 700U,
        "open-close travel is 700 us (63 degrees with SG90 provisional values)");
  CHECK((SERVO_GRIPPER_CLOSE_PULSE_US >= SERVO_GRIPPER_MIN_PULSE_US) &&
        (SERVO_GRIPPER_OPEN_PULSE_US <= SERVO_GRIPPER_MAX_PULSE_US),
        "both positions stay inside the 500~2500 us window");
}

int main(void)
{
  printf("== host simulation for lcd_st7789 / servo_z / servo_gripper ==\n");

  test_fill_color();
  test_draw_image();
  test_bad_arguments();
  test_write_command_and_data();
  test_address_window();
  test_init_sequence();
  test_z_servo();
  test_gripper_servo();

  printf("== checks=%d failures=%d ==\n", checks, failures);
  if (failures == 0)
  {
    printf("RESULT: PASS\n");
    return 0;
  }
  printf("RESULT: FAIL\n");
  return 1;
}
