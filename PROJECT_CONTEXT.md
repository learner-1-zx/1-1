# 智能分拣工程上下文

## 工程概况

- MCU：STM32F407VET6
- HAL 配置工具：STM32CubeMX
- 编译工具：Keil MDK-ARM
- Keil 工程：`MDK-ARM/diankong.uvprojx`
- CubeMX 配置：`diankong.ioc`
- 主机仿真复核：`tools/hostsim`（gcc + HAL 桩函数，不参与 Keil 构建）

## 当前外设

- ADC1：电池电压采样，通道为 ADC1_IN4（PA4）
- TIM1：50 Hz、1 us 分辨率舵机时基
  - CH2：仅作内部比较（无引脚输出），配合 PB8 软件 PWM 驱动夹爪
  （比较值即脉宽 us）；CH1 不再使用
- TIM2：50 Hz、1 us 分辨率，PA1 / TIM2_CH2 驱动 Z 轴舵机硬件 PWM
- TIM6：基础定时器，步进电机节拍
- SPI2：ST7789V2 显示屏（240 x 320，SPI Mode 0）
- USART2：OpenART mini 视觉模块
- USART3：调试 / 备用串口（原串口屏方案已废弃）

## 显示屏接口（2026-10-14 起）

- `LCD_SCK`：PB13 / SPI2_SCK
- `LCD_MOSI`：PB15 / SPI2_MOSI
- `LCD_CS`：PB12（软件片选，普通 GPIO）
- `LCD_DC`：PE7（命令 / 数据选择）
- `LCD_RST`：PE8（复位）
- `LCD_BLK`：PE9（背光）
- `LCD_MISO`：PB14（预留，当前只写不读）
- 驱动：`Drivers/lcd_st7789.c`、`Drivers/lcd_st7789.h`

## 电机控制信号

- X 轴：`X_STEP`、`X_DIR`、`X_EN`
- Y 轴：`Y_STEP`、`Y_DIR`、`Y_EN`
- T 轴：`T_STEP`、`T_DIR`、`T_EN`

## 输入与扩展资源

- `T_HOME`：PD12，上拉输入
- `EMERGENCY_STOP`：PD13，上拉输入，低电平有效
- `GRIP_SENSOR`：PD14，上拉输入
- `PB8`：夹爪舵机软件 PWM 信号（`SERVO_PWM`），由上拉输入改为推挽输出
- `PA8`、`PD8`：原 Z 轴 / 夹爪舵机信号，当前已释放，固件未配置
- `PD9`～`PD11`：新增模块的**优先扩展口**，2026-10-18 起不再作为限位专用保留；
  当前固件未配置，限位方案后续单独考虑
- 板载测试按键：PD0/KEY0、PD2/KEY2、PD3/KEY3、PD4/KEY4；
  当前正式固件未配置，PA0/KEY1（按下高电平）空闲

## 当前程序状态

[main.c](Core/Src/main.c) 已完成 HAL、GPIO、TIM1、TIM2、TIM6、SPI2、USART2、USART3、
ADC1 的初始化，并启动 Z 轴舵机、夹爪舵机和 ST7789V2 显示屏驱动。
主循环只运行正式流程：`SafetyManager_Process()`、`Protocol_Process()`、
`SortingTask_Process()`。T 轴/X 轴台架测试和夹爪上电自检已删除。

## 舵机接口

- Z 轴舵机：**PA1 / TIM2_CH2**，驱动在 `Drivers/servo_z.c`
  （TIM2：Prescaler 83、Period 19999，APB1 定时器时钟 84 MHz，
  50 Hz、1 us 分辨率，比较值即脉宽微秒数；初始 1500 us，
  限幅 1000～2000 us。PA1 的板载 LED 已拆除，换板时必须重新确认）
- 夹爪舵机：**PB8（普通 GPIO，软件 PWM）**，驱动在 `Drivers/servo_gripper.c`
  （PB8 虽然有 TIM4_CH3 等复用，但当前仍借用 TIM1_CH2 的内部比较：
  TIM1 更新中断先写 CCR2、再把 PB8 拉高，CH2 比较中断把 PB8 拉低；
  注意 `HAL_TIM_PWM_Start_IT()` 只开 CH2 比较中断，更新中断需用
  `__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE)` 单独使能；
  每 20 ms 仅两次中断，不影响 Z 轴 PA1 / TIM2 的硬件 PWM；
  最终选型恢复为 **BDS300**：7.4 V、10 kg·cm、0.13 s/60°；
  当前代码仍保留 SG90 试装的临时脉宽 1170/1870 us 和 500～2500 us 限幅，
  仅作占位，尚未按 BDS300 标定；上电不再执行往返自检，但 PWM 仍会输出
  张开脉宽，**完成 BDS300 空载标定前不要直接接 BDS300 上电**；
  齿条行程按 20 齿、模数 0.8 齿轮计算：3.5 个齿距 ≈ 8.8 mm = 63°；
  若实测 BDS300 需要 500 Hz，可单独调整 TIM1，或改用 PB8 / TIM4_CH3
  硬件 PWM，不会影响 Z 轴的 TIM2）
- 引脚变更原因：
  1. PA8 / TIM1_CH1 方案停用后，Z 轴改到 PA1 / TIM2_CH2；PA1 原经 510R
     接板载 LED D2，必须先拆除 LED，否则 PWM 只会驱动 LED；
  2. PA9 虽然在芯片上是 TIM1_CH2，但 **PCB 没有引出**，不能作为夹爪信号；
  3. 2026-10-18 起夹爪由 PD8 改到 PB8。PB8 原为上拉输入（可能预留给
     Z_HOME），改成推挽输出前必须确认底板上没有外部电路；PD8 释放为空闲候选，
     PD9～PD11 继续作为新增模块优先扩展口；限位功能后续再单独处理。

## 架构约束

1. 不把全部业务逻辑堆在 `main.c` 中。
2. 步进电机控制、串口协议、安全保护和 ADC 检测独立模块化。
3. 急停具有最高安全优先级；限位方案后续单独确定，接入后同样按最高优先级处理。
4. CubeMX 生成代码尽量只在 `USER CODE` 区域修改。
5. 每次修改后都要能够重新编译 Keil 工程。
6. 台架测试不直接改正式主循环；测试代码另建分支或临时工程。

## 待确认事项

1. USART2（OpenART）和 USART3 的实际连接设备；显示屏已改为 SPI2，USART3 目前空闲。
2. X、Y、T 三个轴的机械含义。
3. 步进驱动器的使能电平和方向电平。
4. 急停和抓取传感器的有效电平（限位方案后续确定时再一并确认）。
5. 自动分拣动作流程。
6. Z 轴舵机（PA1 / TIM2_CH2）的脉宽方向（1000 us 为上还是为下）和实际可用行程。
7. ST7789V2 模块背光的有效电平（默认高电平点亮）与是否需要反显（INVON）。
8. 夹爪最终恢复 BDS300（PB8 软件 PWM，7.4 V、10 kg·cm）：
   SG90 因塑料齿轮/输出轴易损已放弃；BDS300 的 PWM 频率、脉宽范围、
   3.3 V 信号兼容性和 63° 行程对应脉宽均待空载实测。
   当前固件仍是 SG90 临时脉宽 1170/1870 us；上电自检已删除，但 PWM
   仍会输出张开脉宽，测试前不要直接接 BDS300。
   方向相反时对调 `SERVO_GRIPPER_OPEN_PULSE_US` 与
   `SERVO_GRIPPER_CLOSE_PULSE_US`。
9. 夹爪信号目前用 PB8 软件 PWM。PB8 本身可复用为 TIM4_CH3，若后续需要硬件 PWM，
   可改 TIM4_CH3；只需同步修改 `main.h` 的 `SERVO_PWM_*`、`gpio.c` 输出配置、
   `tim.c` 的 MSP 和 `.ioc`。注意 PA9 不可用（PCB 未引出）。PD7/PD8 已释放为空闲候选。
10. 板载测试按键 PD0/PD2/PD3/PD4（KEY0/KEY2/KEY3/KEY4）的实际按下电平，
    写测试代码前用万用表或示波器实测；PA0/KEY1 为按下高电平，不能照搬。

## 测试与历史记录

- `IO.md`：当前引脚分配。
- `LESSONS.md`：台架调试教训和未闭环事项。
- `test.md`、`test_context.md`：旧舵机/步进台架测试记录，已删除；
  可复用结论已合并到 `LESSONS.md`。
- `HISTORY_CONTEXT.md`：历史设计文档，仅作归档；其中的旧测试模式和
  旧引脚分配不代表当前固件。
