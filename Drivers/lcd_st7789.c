#include "lcd_st7789.h"
#include "spi.h"

/* 分块发送的字节数：一次 HAL_SPI_Transmit 最多发送这么多字节，
 * 避免整屏填充时长时间阻塞主循环。 */
#define LCD_CHUNK_BYTES  512U
/* 单次 SPI 传输超时时间。 */
#define LCD_SPI_TIMEOUT  200U

/* ST7789V2 常用命令 */
#define LCD_CMD_SWRESET      0x01U
#define LCD_CMD_SLPOUT       0x11U
#define LCD_CMD_NORON        0x13U
#define LCD_CMD_INVON        0x21U
#define LCD_CMD_DISPON       0x29U
#define LCD_CMD_CASET        0x2AU
#define LCD_CMD_RASET        0x2BU
#define LCD_CMD_RAMWR        0x2CU
#define LCD_CMD_MADCTL       0x36U
#define LCD_CMD_COLMOD       0x3AU
#define LCD_CMD_PORCTRL      0xB2U
#define LCD_CMD_GCTRL        0xB7U
#define LCD_CMD_VCOMS        0xBBU
#define LCD_CMD_LCMCTRL      0xC0U
#define LCD_CMD_VDVVRHEN     0xC2U
#define LCD_CMD_VRHS         0xC3U
#define LCD_CMD_VDVS         0xC4U
#define LCD_CMD_FRCTRL2      0xC6U
#define LCD_CMD_PWCTRL1      0xD0U
#define LCD_CMD_PVGAMCTRL    0xE0U
#define LCD_CMD_NVGAMCTRL    0xE1U

/* 一次分块发送使用的缓冲区，按 16 bit 像素计算最多 256 个像素。 */
static uint8_t lcd_chunk[LCD_CHUNK_BYTES];

/* ST7789V2 的伽马校正参数，采用常见开源驱动的通用值。 */
static const uint8_t lcd_pvgam[] =
{
  0xD0U, 0x04U, 0x0DU, 0x11U, 0x13U, 0x2BU, 0x3FU,
  0x54U, 0x4CU, 0x18U, 0x0DU, 0x0BU, 0x1FU, 0x23U
};

static const uint8_t lcd_nvgam[] =
{
  0xD0U, 0x04U, 0x0CU, 0x11U, 0x13U, 0x2CU, 0x3FU,
  0x44U, 0x51U, 0x2FU, 0x1FU, 0x1FU, 0x20U, 0x23U
};

static void LCD_ChipSelect(uint8_t selected);
static void LCD_SendCommand(uint8_t cmd);
static void LCD_SendData(const uint8_t *data, uint16_t size);
static void LCD_WriteCommandData(uint8_t cmd, const uint8_t *data,
                                 uint16_t size);
static void LCD_SetAddressWindowInternal(uint16_t x0, uint16_t y0,
                                         uint16_t x1, uint16_t y1);
static void LCD_WritePixelBlock(const uint16_t *pixels, uint32_t count);
static void LCD_FillChunk(uint16_t color, uint16_t pixel_count);

/* 片选由软件控制：低有效。 */
static void LCD_ChipSelect(uint8_t selected)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin,
                    (selected != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* 不带片选控制的底层发送，供已经持有片选的调用者使用。 */
static void LCD_SendCommand(uint8_t cmd)
{
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  (void)HAL_SPI_Transmit(&hspi2, &cmd, 1U, LCD_SPI_TIMEOUT);
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
}

static void LCD_SendData(const uint8_t *data, uint16_t size)
{
  if ((data == NULL) || (size == 0U))
  {
    return;
  }
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  (void)HAL_SPI_Transmit(&hspi2, data, size, LCD_SPI_TIMEOUT);
}

void LCD_BacklightOn(void)
{
  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, LCD_BLK_ACTIVE_STATE);
}

void LCD_BacklightOff(void)
{
  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin,
                    (LCD_BLK_ACTIVE_STATE == GPIO_PIN_SET)
                      ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void LCD_WriteCommand(uint8_t cmd)
{
  /* 单独发送命令：CS 低 → DC 低 → 发送 → CS 高。 */
  LCD_ChipSelect(1U);
  LCD_SendCommand(cmd);
  LCD_ChipSelect(0U);
}

void LCD_WriteData(uint8_t *data, uint16_t size)
{
  if ((data == NULL) || (size == 0U))
  {
    return;
  }

  /* 单独发送数据：CS 低 → DC 高 → 发送 → CS 高。
     空指针和 0 长度直接忽略，避免空传输卡死。 */
  LCD_ChipSelect(1U);
  LCD_SendData(data, size);
  LCD_ChipSelect(0U);
}

/* 命令 + 参数必须在同一次片选内完成：
 * ST7789 要求一条命令和它的参数之间不能中断片选。 */
static void LCD_WriteCommandData(uint8_t cmd, const uint8_t *data,
                                 uint16_t size)
{
  LCD_ChipSelect(1U);
  LCD_SendCommand(cmd);
  if (size > 0U)
  {
    LCD_SendData(data, size);
  }
  LCD_ChipSelect(0U);
}

void LCD_Reset(void)
{
  /* 复位时序：CS 拉高、背光关闭、RST 低约 20 ms，再拉高等约 120 ms。 */
  LCD_ChipSelect(0U);
  LCD_BacklightOff();
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(20U);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(120U);
}

void LCD_SetAddressWindow(uint16_t x0, uint16_t y0,
                          uint16_t x1, uint16_t y1)
{
  LCD_ChipSelect(1U);
  LCD_SetAddressWindowInternal(x0, y0, x1, y1);
  LCD_ChipSelect(0U);
}

static void LCD_SetAddressWindowInternal(uint16_t x0, uint16_t y0,
                                         uint16_t x1, uint16_t y1)
{
  uint8_t data[4];
  uint16_t start_x = (uint16_t)(x0 + LCD_X_OFFSET);
  uint16_t start_y = (uint16_t)(y0 + LCD_Y_OFFSET);
  uint16_t end_x = (uint16_t)(x1 + LCD_X_OFFSET);
  uint16_t end_y = (uint16_t)(y1 + LCD_Y_OFFSET);

  data[0] = (uint8_t)(start_x >> 8);
  data[1] = (uint8_t)(start_x & 0xFFU);
  data[2] = (uint8_t)(end_x >> 8);
  data[3] = (uint8_t)(end_x & 0xFFU);
  LCD_SendCommand(LCD_CMD_CASET);
  LCD_SendData(data, 4U);

  data[0] = (uint8_t)(start_y >> 8);
  data[1] = (uint8_t)(start_y & 0xFFU);
  data[2] = (uint8_t)(end_y >> 8);
  data[3] = (uint8_t)(end_y & 0xFFU);
  LCD_SendCommand(LCD_CMD_RASET);
  LCD_SendData(data, 4U);
}

/* 把同一个颜色重复 pixel_count 个像素后发送出去（调用者已持有片选）。 */
static void LCD_FillChunk(uint16_t color, uint16_t pixel_count)
{
  uint16_t index;

  for (index = 0U; index < pixel_count; index++)
  {
    lcd_chunk[index * 2U] = (uint8_t)(color >> 8);
    lcd_chunk[index * 2U + 1U] = (uint8_t)(color & 0xFFU);
  }
  LCD_SendData(lcd_chunk, (uint16_t)(pixel_count * 2U));
}

/* 按 16 bit 大端顺序发送一段像素数据（调用者已持有片选）。 */
static void LCD_WritePixelBlock(const uint16_t *pixels, uint32_t count)
{
  uint32_t index = 0U;
  uint16_t filled;
  uint16_t color;

  while (index < count)
  {
    filled = 0U;
    while ((filled < (LCD_CHUNK_BYTES / 2U)) && (index < count))
    {
      color = pixels[index];
      lcd_chunk[filled * 2U] = (uint8_t)(color >> 8);
      lcd_chunk[filled * 2U + 1U] = (uint8_t)(color & 0xFFU);
      filled++;
      index++;
    }
    LCD_SendData(lcd_chunk, (uint16_t)(filled * 2U));
  }
}

void LCD_FillColor(uint16_t color)
{
  uint32_t remaining = (uint32_t)LCD_WIDTH * (uint32_t)LCD_HEIGHT;

  /* 窗口设置、写显存命令和像素数据放在同一次片选内，避免中断传输。 */
  LCD_ChipSelect(1U);
  LCD_SetAddressWindowInternal(0U, 0U, LCD_WIDTH - 1U, LCD_HEIGHT - 1U);
  LCD_SendCommand(LCD_CMD_RAMWR);

  while (remaining > 0U)
  {
    if (remaining > (LCD_CHUNK_BYTES / 2U))
    {
      LCD_FillChunk(color, (uint16_t)(LCD_CHUNK_BYTES / 2U));
      remaining -= (LCD_CHUNK_BYTES / 2U);
    }
    else
    {
      LCD_FillChunk(color, (uint16_t)remaining);
      remaining = 0U;
    }
  }
  LCD_ChipSelect(0U);
}

void LCD_DrawImage(uint16_t x, uint16_t y,
                   uint16_t width, uint16_t height,
                   const uint16_t *data)
{
  if ((data == NULL) || (width == 0U) || (height == 0U))
  {
    return;
  }
  if (((uint32_t)x + width > LCD_WIDTH) ||
      ((uint32_t)y + height > LCD_HEIGHT))
  {
    /* 超出屏幕范围直接丢弃，避免坐标越界写到窗口之外。 */
    return;
  }

  /* 窗口设置、写显存命令和像素数据放在同一次片选内。 */
  LCD_ChipSelect(1U);
  LCD_SetAddressWindowInternal(x, y,
                               (uint16_t)(x + width - 1U),
                               (uint16_t)(y + height - 1U));
  LCD_SendCommand(LCD_CMD_RAMWR);
  LCD_WritePixelBlock(data, (uint32_t)width * (uint32_t)height);
  LCD_ChipSelect(0U);
}

void LCD_Init(void)
{
  uint8_t data[5];

  LCD_Reset();

  LCD_WriteCommand(LCD_CMD_SWRESET);
  HAL_Delay(120U);
  LCD_WriteCommand(LCD_CMD_SLPOUT);
  HAL_Delay(120U);

  /* 16 bit 像素格式（RGB565） */
  data[0] = 0x55U;
  LCD_WriteCommandData(LCD_CMD_COLMOD, data, 1U);

  /* 方向：竖屏、从左到右、从上到下 */
  data[0] = (uint8_t)LCD_MADCTL_VALUE;
  LCD_WriteCommandData(LCD_CMD_MADCTL, data, 1U);

  /* 前后廊控制 */
  data[0] = 0x0CU;
  data[1] = 0x0CU;
  data[2] = 0x00U;
  data[3] = 0x33U;
  data[4] = 0x33U;
  LCD_WriteCommandData(LCD_CMD_PORCTRL, data, 5U);

  /* 栅极控制、VCOM、电源相关参数 */
  data[0] = 0x35U;
  LCD_WriteCommandData(LCD_CMD_GCTRL, data, 1U);

  data[0] = 0x19U;
  LCD_WriteCommandData(LCD_CMD_VCOMS, data, 1U);

  data[0] = 0x2CU;
  LCD_WriteCommandData(LCD_CMD_LCMCTRL, data, 1U);

  data[0] = 0x01U;
  LCD_WriteCommandData(LCD_CMD_VDVVRHEN, data, 1U);

  data[0] = 0x12U;
  LCD_WriteCommandData(LCD_CMD_VRHS, data, 1U);

  data[0] = 0x20U;
  LCD_WriteCommandData(LCD_CMD_VDVS, data, 1U);

  /* 刷新率 60 Hz */
  data[0] = 0x0FU;
  LCD_WriteCommandData(LCD_CMD_FRCTRL2, data, 1U);

  data[0] = 0xA4U;
  data[1] = 0xA1U;
  LCD_WriteCommandData(LCD_CMD_PWCTRL1, data, 2U);

  LCD_WriteCommandData(LCD_CMD_PVGAMCTRL, lcd_pvgam,
                       (uint16_t)sizeof(lcd_pvgam));
  LCD_WriteCommandData(LCD_CMD_NVGAMCTRL, lcd_nvgam,
                       (uint16_t)sizeof(lcd_nvgam));

  /* IPS 模块通常需要反显；若实测颜色相反，去掉本命令。 */
  LCD_WriteCommand(LCD_CMD_INVON);

  LCD_WriteCommand(LCD_CMD_NORON);
  LCD_WriteCommand(LCD_CMD_DISPON);
  HAL_Delay(50U);

  LCD_BacklightOn();
}
