#ifndef __LCD_ST7789_H
#define __LCD_ST7789_H

#include "main.h"
#include <stdint.h>

/* 2.0 英寸 IPS 模块：ST7789V2 驱动、SPI 接口、240 x 320。
 *
 * 接线（即 diankong.ioc 中的网络名）：
 *   LCD_SCK  PB13 / SPI2_SCK
 *   LCD_MOSI PB15 / SPI2_MOSI
 *   LCD_CS   PB12（软件片选，低有效）
 *   LCD_DC   PE7 （低 = 命令，高 = 数据）
 *   LCD_RST  PE8 （低 = 复位，高 = 正常工作）
 *   LCD_BLK  PE9 （背光，默认高电平点亮，实测如需反相改下面宏）
 *   LCD_MISO PB14（预留，当前只写不读） */

#define LCD_WIDTH   240U
#define LCD_HEIGHT  320U

/* 240 x 320 的 ST7789 模块行列偏移均为 0；若换成 240 x 240 方屏，
 * 需要把 X/Y 偏移改为 0 / 80。 */
#define LCD_X_OFFSET 0U
#define LCD_Y_OFFSET 0U

/* 背光有效电平：模块实测若为低电平点亮，把宏改成 GPIO_PIN_RESET。 */
#define LCD_BLK_ACTIVE_STATE GPIO_PIN_SET

/* MADCTL(0x36) 方向设置：
 *   0x00 竖屏（默认）
 *   0x60 横屏
 *   0xC0 竖屏反转
 *   0xA0 横屏反转
 * 若红蓝颜色互换，在取值上加 0x08 切换 RGB/BGR 顺序。 */
#define LCD_MADCTL_VALUE 0x00U

/* RGB565 常用颜色 */
#define LCD_COLOR_BLACK   0x0000U
#define LCD_COLOR_WHITE   0xFFFFU
#define LCD_COLOR_RED     0xF800U
#define LCD_COLOR_GREEN   0x07E0U
#define LCD_COLOR_BLUE    0x001FU
#define LCD_COLOR_YELLOW  0xFFE0U
#define LCD_COLOR_CYAN    0x07FFU
#define LCD_COLOR_MAGENTA 0xF81FU

/* 初始化 SPI 显示屏：复位时序 + ST7789V2 初始化命令 + 打开背光。 */
void LCD_Init(void);
/* 硬件复位时序：RST 拉低约 20 ms，再拉高并等待约 120 ms。 */
void LCD_Reset(void);

void LCD_WriteCommand(uint8_t cmd);
void LCD_WriteData(uint8_t *data, uint16_t size);
void LCD_SetAddressWindow(uint16_t x0, uint16_t y0,
                          uint16_t x1, uint16_t y1);
/* 用指定颜色填充整个屏幕。 */
void LCD_FillColor(uint16_t color);
/* 在 (x, y) 处绘制 width x height 的 RGB565 图片数据（行优先）。 */
void LCD_DrawImage(uint16_t x, uint16_t y,
                   uint16_t width, uint16_t height,
                   const uint16_t *data);

void LCD_BacklightOn(void);
void LCD_BacklightOff(void);

#endif
