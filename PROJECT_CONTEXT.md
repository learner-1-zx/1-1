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
  - CH1：PA8 硬件 PWM，驱动夹爪舵机（比较值即脉宽 us）
  - CH2：当前未使用
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

- `T_HOME`：PD12，上拉输入；接转盘零位霍尔，**低电平有效**（有磁铁 = 低），
  读取接口在 `Drivers/hall_sensor.c`（`HallSensor_IsTriggered()`）
- `X_HOME` / `Y_HOME`：**PD6 / PD5**，上拉输入；接 X/Y 零点机械限位开关
  （NO→IO、C→GND），**低电平有效**；读取和 15 ms 消抖在 `Drivers/axis_home.c`。
  上电按 X → Y → T 慢速回零，回零后软限位为 `0 ~ SORT_X/Y_TRAVEL_STEPS`；
  当前 `SORT_ENABLE_Y_AXIS=0`，Y 驱动器不使能、不回零也不参与运动
- `EMERGENCY_STOP`：PD13，上拉输入，低电平有效
- `GRIP_SENSOR`：PD14，上拉输入
- `SPARE_PD11`：PD11，上拉输入备用；不再接霍尔（2026-10-23 改名）
- `PA8`：夹爪舵机 `SERVO_PWM`，TIM1_CH1 硬件 PWM
- `PB8`、`PD8`：当前释放为空闲候选，固件未配置
- `PD9`、`PD10`：新增模块的**优先扩展口**，当前固件未配置
  （X/Y 限位已用 PD5/PD6，不再占用 PD8～PD10）
- 板载测试按键：PD0/KEY0、PD1/KEY1、PD2/KEY2、PD3/KEY3；
  当前固件在“安全且无任务”时允许 KEY0/KEY1 按下一次跳到张开/闭合位
  （投放口标定模式下 KEY0 还支持“按住 1 s 存当前 X 为投放口”）；
  KEY2/KEY3 已恢复正常功能（`ZSERVO_CALIB_MODE=0`）：安全且无任务、X 空闲时
  点动 X 轴（正/反，一次一个电机整圈；`SORT_XY_CALIB_MODE=1` 时 100 微步慢速）；
  改回 0 后恢复“安全且无任务、X 空闲时点动 X 轴（正/反，一次一个电机整圈）”；
  PA0/WK_UP（按下高电平）空闲

## 当前程序状态

[main.c](Core/Src/main.c) 已完成 HAL、GPIO、TIM1、TIM2、TIM6、SPI2、USART2、USART3、
ADC1 的初始化，并启动 Z 轴舵机、夹爪舵机和 ST7789V2 显示屏驱动。
主循环运行正式流程：`SafetyManager_Process()`、`ServoGripper_Process()`、
`Protocol_Process()`、`SortingTask_Process()`；台架按键在“安全且无任务”时
（含上电回零期间、回零失败后的 ERROR）生效，KEY2/KEY3 当前用于 Z 轴
一键到最下/最上，`ZSERVO_CALIB_MODE=0` 时恢复在 X 空闲时点动 X 轴。
**2026-10-26 起 X/Y 有真实零点**：X=PD6、Y=PD5（普通限位开关，上拉低有效），
上电按 **X → Y（`SORT_ENABLE_Y_AXIS=1` 时）→ T** 的顺序慢速回零
（搜索 800 / 退让 400 / 慢速接近 120 步/秒），二次触发边沿记为 0 点，
回零后软限位 `0 ~ SORT_X/Y_TRAVEL_STEPS`。当前 `SORT_ENABLE_Y_AXIS=0`，
Y 驱动器不使能、不回零也不参与运动，先只调 X。
**投放口由 STM32 自己保存**（上位机只发取料坐标）：默认值取
`SORT_DROP_X/Y_STEP`；`SORT_XY_CALIB_MODE=1` 时 KEY2/KEY3 慢速 100 微步点动，
KEY0 **按下即张开夹爪，按住满 `SORT_DROP_KEY_LONG_PRESS_MS`（1000 ms）
才把当前 X 存成投放口**，并从 USART3（PB10，115200）打印 `DROP_X=`；
把数值写回宏即可掉电保持（`SORT_DEBUG_UART` 控制调试输出）。
**2026-10-25 起夹爪恢复正式模式**（`SERVO_GRIPPER_CALIB_MODE=0`、
`SORT_BENCH_KEY_JUMP=1`）：上电不启动 PWM、不上力，进入主循环后由
`SORT_SYS_INIT` 调 `ServoGripper_Open()` 张开一次，不会先摆到 1500 us 中间位；
KEY0/KEY1 一次到位到 1730/1420 us。
需要重新标定夹爪时改成 `CALIB_MODE=1` + `JUMP=0`：上电即输出 `START_PULSE`
（1500 us）零点并保持，`Open()/Close()` 只改标志不动脉宽，限幅放宽到
800~2200 us，KEY0/KEY1 每次 ±50 us 点动（步骤见 SORTING_FLOW.md 8.12）；
标定完必须把 `SERVO_GRIPPER_CALIB_MODE` 改回 0，否则自动流程不会动夹爪。
**2026-10-27 Z 轴齿条标定完成**：实测机械行程 **最上面 1200 us、
最下面 2500 us**（脉宽小 = 上，行程 1300 us）。已切回正常模式
（`ZSERVO_CALIB_MODE=0`）：上电到运输位 1220 us，限幅 1200~2500 us，
`Travel()/Approach()/Grasp()` 正常控制，工作高度占位 1220/1850/2480 us
（两端各留 20 us，GRASP/APPROACH 仍需按物块微调）。
KEY2/KEY3 恢复 X 轴点动。重新标定时把 `ZSERVO_CALIB_MODE` 改 1：
上电停在 1850 us（中点），`Travel/Approach/Grasp` 不改脉宽，
KEY2/KEY3 逐档 +50/-50 us、按住连发并打印 `Z_PULSE=`；
标定完必须改回 0（步骤见 SORTING_FLOW.md 8.14）。
T 轴/X 轴台架测试和夹爪上电自检已删除。

自动分拣状态机已按 UM2 式 Cartesian 结构实现：
XY 取料 -> Z 接近 -> 开爪 -> Z 下降 -> 闭爪 -> Z 抬升并并行转 T
-> XY 投放 -> 等 T 到位 -> 开爪 -> 回安全位。等待时间统一使用
`HAL_GetTick()`（ms），TIM6 只负责 20 kHz 步进脉冲。
完整流程、协议和标定清单见 [SORTING_FLOW.md](SORTING_FLOW.md)。

## 舵机接口

- Z 轴舵机：**PA1 / TIM2_CH2**，驱动在 `Drivers/servo_z.c`
  （TIM2：Prescaler 83、Period 19999，APB1 定时器时钟 84 MHz，
  50 Hz、1 us 分辨率，比较值即脉宽微秒数；2026-10-27 实测机械行程
  **最上面 1200 us、最下面 2500 us**（行程 1300 us），限幅 1200～2500 us；
  工作高度占位 运输/接近/抓取 = 1220/1850/2480 us；
  `ZSERVO_CALIB_MODE=0`（当前正常模式）上电到运输位 1220 us，
  `Travel()/Approach()/Grasp()` 正常控制；重新标定把该宏改 1，
  上电 1850 us、KEY2/KEY3 逐档 ±50 us（`ZServo_StepPulse()`），
  `ZSERVO_BENCH_KEY2/3_JUMP=1` 时一键到 2500/1200（`ZServo_JumpTo()`）。
  PA1 的板载 LED 已拆除，换板时必须重新确认）
  PA1 的板载 LED 已拆除，换板时必须重新确认）
  PA1 的板载 LED 已拆除，换板时必须重新确认）
  PA1 的板载 LED 已拆除，换板时必须重新确认）
- 夹爪舵机：**PA8 / TIM1_CH1 硬件 PWM**，驱动在 `Drivers/servo_gripper.c`
  （TIM1 为 50 Hz、1 us 分辨率，脉宽直接写 CCR1；上电不启动 PWM、舵机不上力，
  软件脉宽复位到安全中位 1500 us，第一次 `Open()`/`Close()` 先以 1500 us 启动
  PWM，再由 `ServoGripper_Process()` 按 50 us / 20 ms 缓动到 1730/1420 us
  （约 140 ms），避免一帧跳完 310 us 撞进端点导致堵转；
  上电待机期间 PA8 被拉成推挽低电平（定时器 `MOE=0` 时 AF 引脚是高阻，
  浮空会被噪声触发抖动，且毛刺不受软件限幅），第一次输出脉宽前才切回
  `GPIO_AF1_TIM1`；最终选型 **BDS300**：7.4 V、10 kg·cm、
  0.13 s/60°；2026-10-25 重装舵机臂后重新实测闭合临界 1400 us、张开临界 1750 us，
  正式值 1420/1730 us，限幅 1400～1750 us；
  齿条行程按 20 齿、模数 0.8 齿轮计算：3.5 个齿距 ≈ 8.8 mm = 63°；
  实测两端死点间距 350 us 只代表“舵机臂装角 + 机械死点”的位置差，
  舵机在死点处顶住后多给的脉宽并不产生位移，所以不要再按理论角度换算脉宽）
- 引脚变更原因：
  1. Z 轴最终使用 PA1 / TIM2_CH2；PA1 原经 510R 接板载 LED D2，
     必须先拆除 LED，否则 PWM 只会驱动 LED；
  2. PA9 虽然在芯片上是 TIM1_CH2，但 **PCB 没有引出**，不能作为夹爪信号；
  3. 夹爪最终使用 PA8 / TIM1_CH1 硬件 PWM；PB8、PD8 释放为空闲候选，
     PD9、PD10 继续作为新增模块优先扩展口，PD11 改名 `SPARE_PD11` 上拉备用；
     限位功能后续再单独处理；
  4. 2026-10-18 曾把夹爪改到 PB8 软件 PWM，后续已恢复 PA8 硬件 PWM，
     以本文件当前代码为准。

## T 轴（转盘）与零位霍尔

- 电机：42-40 两相步进（1.8°，200 整步/圈）+ 行星减速箱 5.2:1；
  驱动器按 1/16 细分计算，转盘 **16640 步/圈**、标称 2773 步/仓。
  若 TB6600 拨码不是 1/16，只改 `SORT_T_DRIVER_MICROSTEP`，仓距和软限位自动跟着变。
- TB6600S 电流档：1/16 拨码为 S1=OFF、S2=OFF、S3=ON。设备属性里的电机图
  没有标 A/相；若铭牌是 1.7 A，推荐 **1.5 A 工作 / 1.7 A 峰值**
  （S4=ON、S5=ON、S6=OFF），无铭牌时先从 **1.0 A / 1.2 A** 验证。
  工作电流和峰值是同一档配对，不能单独调；24 V 供电，详见 IO.md 2.7。
- 6 仓目标用 `SORT_T_BIN_STEP(index)` 四舍五入分配；六段之和严格等于一圈。
  不要写 `bin * 2773`：16640 不能被 6 整除，每次 6 仓循环会累积 2 步误差。
- 零位：霍尔接 **PD12/T_HOME**，上拉、低有效；磁铁为 11.5×4.5 mm 带孔强磁，
  两片叠装、固定在转盘内侧，气隙 7～8 mm。只装这一处零位标记即可，
  不需要 6 个磁铁：霍尔只给出“有/无磁铁”，6 个相同磁铁在上电时无法区分仓位。
- 回零流程：搜索触发 → 反向 Jog 直到霍尔释放 → 慢速同向接近 → 二次触发清零。
  退让不依赖磁铁有效宽度，所以 W 不需要实测；换磁铁/气隙也不用改参数。
- 2026-10-23 实机：霍尔触发点相对“1 号仓正对投放口”还差一个逆时针小角度，
  暂用 `SORT_T_HOME_OFFSET_STEP = 0`；机械位置固定后按注释公式补偿。
- 台架：安全且无任务、X 空闲时，KEY2/KEY3 点动 X 轴（正/反，一次一个电机整圈），
  用于测试 X 轴方向与行程；X 无回零开关，点动后位置照常累加，无需重新回零。

## 架构约束

1. 不把全部业务逻辑堆在 `main.c` 中。
2. 步进电机控制、串口协议、安全保护和 ADC 检测独立模块化。
3. 急停具有最高安全优先级；限位方案后续单独确定，接入后同样按最高优先级处理。
4. CubeMX 生成代码尽量只在 `USER CODE` 区域修改。
5. 每次修改后都要能够重新编译 Keil 工程。
6. 台架测试不直接改正式主循环；测试代码另建分支或临时工程。

## 待确认事项

1. USART2（OpenART）和 USART3 的实际连接设备；显示屏已改为 SPI2，USART3 目前空闲。
2. X/Y 已确认为 UM2 式 Cartesian 独立轴；T 为 42-40 + 5.2 行星减速转盘，
   按 1/16 细分取 16640 步/圈；实际每圈步数应在装好磁铁后用回零重复性复核。
3. 步进驱动器的使能电平和方向电平。
4. 急停和抓取传感器的有效电平（限位方案后续确定时再一并确认）。
5. 自动分拣流程已实现，状态机、协议和标定清单见
   [SORTING_FLOW.md](SORTING_FLOW.md)；动作时间和坐标仍待实机标定。
6. Z 轴舵机（PA1 / TIM2_CH2）的脉宽方向（1000 us 为上还是为下）和实际可用行程。
7. ST7789V2 模块背光的有效电平（默认高电平点亮）与是否需要反显（INVON）。
8. 夹爪最终选型 BDS300（**PA8 / TIM1_CH1 硬件 PWM**，7.4 V、10 kg·cm）：
   SG90 因塑料齿轮/输出轴易损已放弃；BDS300 已于 2026-10-25 重装舵机臂后
   重新空载标定：3.3 V 信号可直驱，闭合临界 1400 us、张开临界 1750 us，
   正式值 1420/1730 us，限幅 1400～1750 us。
   换舵机或重新装舵机臂后必须重新标定。
   方向相反时对调 `SERVO_GRIPPER_OPEN_PULSE_US` 与
   `SERVO_GRIPPER_CLOSE_PULSE_US`。
9. 夹爪已恢复 PA8 / TIM1_CH1 硬件 PWM；PB8、PD7/PD8 已释放为空闲候选。
   若后续改用其他定时器，只需同步修改 `main.h` 的 `SERVO_PWM_*`、`gpio.c`
   输出配置、`tim.c` 的 MSP 和 `.ioc`。注意 PA9 不可用（PCB 未引出）。
10. 板载测试按键 PD0/PD1/PD2/PD3（KEY0/KEY1/KEY2/KEY3）的实际按下电平，
    写测试代码前用万用表或示波器实测；PA0/WK_UP 为按下高电平，不能照搬。
11. 霍尔开关已确认：4.5～24 V 供电、开集电极、低有效，实际接 PD12/T_HOME；
    PD12 内部上拉，`hall_sensor.h` 的 `HALL_SENSOR_ACTIVE_HIGH = 0`。
    无磁铁约 3.3 V，磁铁靠近约 0 V；磁铁装反不会触发，需要翻面。
    2026-10-23 起 PD11 改名 `SPARE_PD11`，不再作为霍尔输入。

## 测试与历史记录

- `IO.md`：当前引脚分配。
- `LESSONS.md`：台架调试教训和未闭环事项。
- `test.md`、`test_context.md`：旧舵机/步进台架测试记录，已删除；
  可复用结论已合并到 `LESSONS.md`。
- `HISTORY_CONTEXT.md`：历史设计文档，仅作归档；其中的旧测试模式和
  旧引脚分配不代表当前固件。
