# 主机仿真复核（tools/hostsim）

不接硬件的纯软件复核工具，用于验证显示驱动、Z 轴舵机、夹爪舵机和霍尔驱动的行为。
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
- `ZServo_*`：PA1 / TIM2_CH2，依赖 TIM2 的 1 us 分辨率，比较值即脉宽微秒数，
  验证 TRAVEL/APPROACH/GRASP 三档高度、1000～2000 us 限幅，
  以及上电和 `SetSafePosition()` 回到运输高度；
- `ServoGripper_*`：PA8 / TIM1_CH1 硬件 PWM。
  上电 `ServoGripper_Init()` 不启动 PWM、不写 CCR1（PA8 无输出、舵机不上力）；
  第一次 `Open()`/`Close()`/`JumpTo()` 才先写 CCR1 再启动 PWM，第一帧即目标脉宽。
  `Open()`/`Close()` 校验实测值 1410/1780 us、1390～1800 us 限幅窗口和
  `IsClosed()` 状态。test 8 同时校验台架按键用的 `JumpTo()`：直接写 CCR1、
  限幅、同步 `IsClosed()`，以及“PWM 未启动时由第一次 JumpTo 补启动”。
  当前不再使用 TIM1 更新/比较中断；同时断言夹爪不占用 PD8，也不使用 PA9。
- `HallSensor_*`：实际接 PD12 / `T_HOME`，内部上拉、低有效（有磁铁 = 低）。
  test 9 校验引脚映射、无磁铁高电平、磁铁靠近低电平，以及
  `HALL_SENSOR_ACTIVE_HIGH = 0` 的极性换算；
  真实 0 V / 3.3 V 必须上电后用万用表在 PD12 实测，仿真不能代替实机测量。
  PD11 已改名 `SPARE_PD11`，上拉备用，不再作为霍尔输入。
- `sorting_config.h` 的 6 仓步数：test 10 校验 16640 步/圈、标称 2773 步/仓，
  以及六个四舍五入仓距之和严格等于一圈（否则每圈会累积 2 步误差）。

## 原理

- 使用工程里真实的 HAL 头文件（`Core/Inc`、`Drivers/STM32F4xx_HAL_Driver/Inc`、
  CMSIS）编译，保证宏和寄存器结构体与固件完全一致；
- `host_stub.c` 提供 `HAL_GPIO_WritePin`、`HAL_GPIO_ReadPin`、`HAL_SPI_Transmit`、
  `HAL_Delay`、`HAL_TIM_PWM_Start`、`HAL_TIM_PWM_Start_IT` 的桩实现，
  把每次调用的电平、字节流记录下来；
- `host_test.c` 按记录结果做断言，最后打印 `RESULT: PASS` / `RESULT: FAIL`
  和 exit code（0 / 1）。

## 运行方法（Windows PowerShell，在工程根目录执行）

```powershell
gcc -std=c99 -Wall -Wextra -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast `
  -o tools/hostsim/hostsim.exe `
  tools/hostsim/host_test.c tools/hostsim/host_stub.c `
  Drivers/lcd_st7789.c Drivers/servo_z.c Drivers/servo_gripper.c Drivers/hall_sensor.c `
  -I Core/Inc -I App -I Drivers -I Drivers/STM32F4xx_HAL_Driver/Inc `
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
