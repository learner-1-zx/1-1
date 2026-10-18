# 主机仿真复核（tools/hostsim）

不接硬件的纯软件复核工具，用于验证显示驱动、Z 轴舵机和夹爪舵机驱动的行为。
**该目录不参与 Keil 工程构建**（`diankong.uvprojx` 只列出 Core/Drivers/App/Service 下的文件），
只在本机用 MinGW gcc 编译运行。

## 复核内容

- `LCD_FillColor` 整屏填充：CASET/RASET 窗口值、RAMWR 命令、像素字节数
  （240x320x2 = 153600）、RGB565 大端字节序、512 字节分块；
- `LCD_DrawImage`：窗口换算、行优先大端像素流；
- 越界坐标、宽高为 0、空指针调用不得产生任何 SPI 流量；
- 一次逻辑事务（命令 + 参数 / 窗口设置 + 像素流）期间 CS 只拉低一次，
  避免在命令和参数之间中断片选；
- `LCD_Init` 复位时序、初始化命令顺序、背光点亮；
- `ZServo_*`：PA8 / TIM1_CH1，依赖 TIM1 的 1 us 分辨率，比较值即脉宽微秒数，
  并验证 1000～2000 us 限幅和 1500 us 安全位置；
- `ServoGripper_*`：PD8 软件 PWM（TIM1_CH2 内部比较，无引脚输出）。
  模拟 TIM1 更新中断/CH2 比较中断，验证更新中断已使能、先写入 CCR2
  再拉高 PD8、比较匹配后拉低；上电初始化不阻塞、不执行往返自检，
  默认输出张开脉宽。当前仍按 SG90 临时脉宽验证 1170/1870 us
  和 500～2500 us 限幅窗口。夹爪最终选型为 BDS300，标定后需同步更新
  这些断言。同时断言夹爪不再占用 TIM1_CH1、也不再使用 PA9。

## 原理

- 使用工程里真实的 HAL 头文件（`Core/Inc`、`Drivers/STM32F4xx_HAL_Driver/Inc`、
  CMSIS）编译，保证宏和寄存器结构体与固件完全一致；
- `host_stub.c` 提供 `HAL_GPIO_WritePin`、`HAL_SPI_Transmit`、`HAL_Delay`、
  `HAL_TIM_PWM_Start`、`HAL_TIM_PWM_Start_IT` 的桩实现，
  把每次调用的电平、字节流记录下来；
- `host_test.c` 按记录结果做断言，最后打印 `RESULT: PASS` / `RESULT: FAIL`
  和 exit code（0 / 1）。

## 运行方法（Windows PowerShell，在工程根目录执行）

```powershell
gcc -std=c99 -Wall -Wextra -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast `
  -o tools/hostsim/hostsim.exe `
  tools/hostsim/host_test.c tools/hostsim/host_stub.c `
  Drivers/lcd_st7789.c Drivers/servo_z.c Drivers/servo_gripper.c `
  -I Core/Inc -I Drivers -I Drivers/STM32F4xx_HAL_Driver/Inc `
  -I Drivers/CMSIS/Device/ST/STM32F4xx/Include -I Drivers/CMSIS/Include `
  -DUSE_HAL_DRIVER -DSTM32F407xx
.\tools\hostsim\hostsim.exe
```

注意：

- 主机是 64 位环境，CMSIS 头文件里少量指针/整数互转会产生告警，
  所以加 `-Wno-int-to-pointer-cast -Wno-pointer-to-int-cast`；
  这两个告警只与主机位宽有关，不影响固件。
- 仿真只复核软件逻辑（时序、字节流、限幅），不能代替示波器测量
  和实机空载测试。
