# AI_CONTEXT — 当前有效上下文（2026-11-18）

> **AI 默认只读这三个文件：**
> `AI_CONTEXT.md`、`AI_RULES.md`、`CHANGELOG.md`。
>
> 其他 md 是历史/专项资料，里面的数值可能已经过时；任务确实需要时再定向查，
> **不要全文读取**。当前参数以本文件为准。

## 1. 项目概况

- 主控：STM32F407VET6，8 MHz HSE，168 MHz。
- 结构：UM2 式 Cartesian 龙门，X/Y 两轴 + T 转盘 + Z 舵机 + 夹爪舵机。
- 通信：
  - **USART2（PA2/PA3，115200 8N1）**：OpenART 视觉二进制协议。
  - **USART3（PB10/PB11，115200 8N1）**：台架文字控制台/调试输出。
- 构建：Keil MDK（`MDK-ARM/diankong.uvprojx`）。
- 主机仿真：`tools/hostsim`（MinGW gcc，当前 847 checks / 0 failures）。

## 2. 当前权威配置

### 2.1 模式

```c
SORT_XY_BENCH_MODE        = 1   /* X/Y 台架找限位模式 */
SORT_ENABLE_Y_AXIS        = 1
SORT_XY_BENCH_KEEP_TURRET = 1   /* X/Y 与 T 同时回零 */
ZSERVO_CALIB_MODE         = 0   /* Z 正式模式，自动流程控制 */
SORT_XY_CALIB_MODE        = 0   /* 关闭“当前位置存投放口”标记 */
SORT_T_ALLOW_DROP_STEP_FALLBACK = 0
```

### 2.2 投放口

- 投放口 = **X/Y 限位零点 0/0**。
- 零点由回零时 `SORT_XY_HOMING_APPROACH` 的**限位开关二次触发边沿**建立。
- 不使用 x0/y0 软件标记；不再用 KEY0 长按保存投放坐标。
- `SORT_DROP_X_STEP = 0`、`SORT_DROP_Y_STEP = 0`。

### 2.3 视觉坐标 → 机械微步

```c
SORT_VISION_COORD_MAP           = 1
SORT_VISION_ZERO_X_STEP         = 2200
SORT_VISION_ZERO_Y_STEP         = 3000
SORT_VISION_STEPS_PER_COORD_NUM = 45   /* 900 微步 = 2 cm = 20 mm */
SORT_VISION_STEPS_PER_COORD_DEN = 1
SORT_VISION_X_SIGN              = (+1)
SORT_VISION_Y_SIGN              = (+1)
```

公式：

```c
machine_step = zero + sign * vision_coord * NUM / DEN
```

- 默认视觉坐标单位 = **mm**；视觉 (0,0) → 机械 X2200 / Y3000。
- 视觉发 cm：`NUM=450, DEN=1`。
- 视觉发 0.1 mm：`NUM=45, DEN=10`。
- 视觉发像素：先在视觉端换算成 mm，或改 NUM/DEN。
- 方向相反时把对应 `SIGN` 改成 `-1`。
- 转换函数：`SortingTask_MapVisionMission()`，在 `Protocol_HandleFrame()` 里
  `SortingTask_Submit()` 之前调用。
- 转换后仍走 X/Y 软限位；越界回 `0x83 error=1`。
- USART3 会打印 `VISION_X/Y` 和 `MAP_X_STEP/MAP_Y_STEP`，用于核对映射。

### 2.4 Z 轴

```c
ZSERVO_CALIB_MODE        = 0
ZSERVO_PULSE_TOP_US      = 1200   /* 最上面，上电 ztop */
ZSERVO_PULSE_TRAVEL_US   = 1200
ZSERVO_PULSE_APPROACH_US = 1200   /* 不再用 1850 中间位 */
ZSERVO_PULSE_GRASP_US    = 2150   /* 最低工作位 */
ZSERVO_PULSE_BOTTOM_US   = 2150
ZSERVO_PULSE_MIN_US      = 1200
ZSERVO_PULSE_MAX_US      = 2150
```

- 上电直接 **ztop=1200 us**。
- 下降到最低 **2150 us**。
- `zmid` 已删除；`ZM` 提示 `ZM REMOVED`。
- `zup`/`ztop` → 1200；`zdown`/`zbottom` → 2150。

### 2.5 T 轴（当前调试状态）

- 电机：42-40，5.2:1 减速箱；TB6600；DIP 1/16 + 1.5A。
- STM32 → **ULN2003ADR** → TB6600 PUL-/DIR-/ENA-（三路都经过 ULN2003）。
- 代码边界：**X/Y 驱动/线序补偿不要动**；T 的极性单独宏：

```c
T_STEP_ACTIVE_LOW = 0
T_DIR_INVERT      = 0
T_EN_ACTIVE_LOW   = 1
```

- T 速度（按旧测试 1200 上限）：

```c
SORT_T_MAX_SPEED_STEPS_S          1200
SORT_T_ACCEL_STEPS_S2             1250
SORT_T_HOME_SPEED_STEPS_S         1000
SORT_T_HOME_APPROACH_SPEED_STEPS_S 200
SORT_T_HOME_BACKOFF_SPEED_STEPS_S 400
```

- 已知问题：T 轴仍存在噪声大/来回打转的现象，尚未最终解决；
  历史对照见 `TURNTABLE_TEST_2026-10-12.md`（Git 提交 `59207c1`）。
- 手动测试命令：`tjog` / `tjog-`（T 相对 ±2000 步）。

## 3. USART3 常用命令

```text
ok        回显 ok
s         状态（含 T_POS_OK / T_BUSY / T_HOME）
zup       1200；zdown 2150；ztop 1200；zbottom 2150
xmax/xmin X 软限位两端
zero      重新回零（限位开关触发）
zk/gb     夹爪开/合
tjog/tjog-  T 相对点动
run/abort R 序列
```

版本标记（上电/复位）：

```text
FW=2026-11-08 T-PARALLEL ULN2003
```

## 4. 代码地图

| 文件 | 作用 |
|---|---|
| `App/sorting_config.h` | 所有可调参数（视觉映射、Z、T、按键、超时） |
| `App/sorting_task.c/.h` | 分拣状态机、回零、视觉映射、投放点 |
| `Service/protocol.c/.h` | USART2 二进制协议 |
| `Service/bench_motion.c/.h` | USART3 运动指令、tjog、R 序列 |
| `Service/bench_console.c/.h` | USART3 虚拟按键、ok/版本标记 |
| `Drivers/stepper_motor.c/.h` | X/Y/T 步进驱动（T 参数单独宏） |
| `Drivers/servo_z.h` | Z 舵机脉宽/工作高度 |
| `Drivers/servo_gripper.h` | 夹爪舵机 |
| `tools/hostsim` | 主机仿真回归 |
| `tools/pc` | USART2/USART3 电脑串口工具 |
| `tools/vision` | OpenART 视觉发送脚本 |

## 5. 构建与验证

- Keil：`MDK-ARM/diankong.uvprojx`，Rebuild + Download。
- 主机仿真（在工程根目录）：

```powershell
gcc -std=c99 -Wall -Wextra -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast `
  -o tools/hostsim/hostsim.exe `
  tools/hostsim/host_test.c tools/hostsim/host_stub.c `
  Service/bench_console.c Service/bench_motion.c `
  Service/debug_uart.c Service/safety_manager.c `
  Drivers/lcd_st7789.c Drivers/servo_z.c Drivers/servo_gripper.c `
  Drivers/hall_sensor.c Drivers/axis_home.c `
  -I Core/Inc -I App -I Service -I Drivers `
  -I Drivers/STM32F4xx_HAL_Driver/Inc `
  -I Drivers/CMSIS/Device/ST/STM32F4xx/Include -I Drivers/CMSIS/Include `
  -DUSE_HAL_DRIVER -DSTM32F407xx `
  -DSERVO_GRIPPER_CALIB_MODE=0 -DZSERVO_CALIB_MODE=0 `
  -DBENCH_CONSOLE_NO_STATUS=1
.\tools\hostsim\hostsim.exe
```

## 6. 文档阅读策略（减少无用信息）

- **默认读**：`AI_CONTEXT.md`、`AI_RULES.md`、`CHANGELOG.md`。
- **按需读**：`VISION_PROTOCOL.md`、`tools/*/README.md`。
- **历史资料，不要全文读**：
  `IO.md`、`LESSONS.md`、`PROJECT_CONTEXT.md`、`SORTING_FLOW.md`。
  这些文件含旧的 2500/1850/1220/2480、旧视觉绝对微步等内容，当前值以本文件为准。
- 已删除 `XY_INTEGRATION_GUIDE.md`：它是一次性“合并队友 XY 分支”的旧说明，
  里面的行程/投放口/协议（0x05/0x87）都已过时，避免误导。
- 旧的 `ls.md` 调试记录不再维护；修改记录统一写 `CHANGELOG.md`。

## 7. 当前不要动的地方

- X/Y 驱动代码、`dir_invert`、线序补偿，除非用户明确要求。
- TB6600/ULN2003 极性宏只在用户确认硬件后改。
- 不要为了“整理”而删除/移动大文档；只在顶部加历史提示。
