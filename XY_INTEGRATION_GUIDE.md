# XY 部分合并说明

## 1. 对比基线

本说明以当前 GitHub `main` 和本地队友分支为基线：

- 原始代码：`origin/main`，提交 `60ba64a`
- 队友 XY 分支：`feature/20261031-MINGYU-xy-softlimit-dropcheck`，提交 `6870d75`
- 队友分支包含的提交：`fe368a8`、`4423338`、`6870d75`

`60ba64a` 比队友分支的共同祖先更新，新增了 USART3 台架控制台、视觉协议、PC 工具和 hostsim 测试。因此不能把队友分支整个目录覆盖到 GitHub 最新 `main` 上。当前工作区另外还有未提交的 2026-11-01 台架测试修改，本文最后单独说明，不能把它们和已提交功能混为一组。

## 2. 队友代码实际增加的功能

| 功能 | 原始代码 | 队友代码 |
|---|---|---|
| X/Y 零点 | 只有旧的单轴/串行处理 | X、Y 都支持搜索、退让、低速接近三个阶段 |
| X/Y 回零 | 共享一个轴参数 | 两轴联动回零，各轴独立消抖和阶段状态；任一轴失败都会报回零错误 |
| 软限位 | 主要是通用行程占位值 | 使用实测行程和安全余量，运动前拒绝越界 |
| 方向 | X DIR 未反相 | X 轴通过 `dir_invert=1` 修正，不能用修改坐标方向代替 |
| 投放口 | 只有固定流程坐标 | 可保存当前 X/Y 投放口坐标，并支持回访确认 |
| 禁入区 | 无 | 当前工作区未提交版本增加 X/Y 矩形禁入区和直线穿越检查 |
| 调试按键 | 原有夹爪/Z 轴按键 | 可切换为 X 点动、投放口标定 |
| 上位机协议 | 无投放口回访命令 | 新增命令 `0x05`，确认事件 `0x87` |
| 工程文件 | 原来的 Keil 配置 | 还混入了 Keil Pack/RTE 配置变化，和 XY 逻辑无关 |

GitHub 最新 `main` 另外已经包含以下内容，合并 XY 时必须保留：

- `Service/bench_console.*`、`Service/bench_motion.*`；
- `VISION_PROTOCOL.md`；
- `tools/pc/*`、`tools/vision/*`；
- hostsim 的台架控制和视觉协议测试；
- `main.c`、`protocol.c` 中与台架控制台/视觉协议有关的调用。

## 3. XY 相关文件清单

### 必须合并（在最新 main 上逐项合并）

1. [`App/sorting_config.h`](./App/sorting_config.h)  
   增加或更新 X/Y 回零、软限位、投放口和台架调试宏。队友已提交版本的主要参数是：

   ```c
   SORT_ENABLE_Y_AXIS              1
   SORT_X_HOME_DIR                 (-1)
   SORT_Y_HOME_DIR                 (-1)
   SORT_X_MECH_TRAVEL_STEPS       12720
   SORT_Y_MECH_TRAVEL_STEPS       16720
   SORT_XY_SOFT_LIMIT_MARGIN_STEPS 200
   SORT_DROP_X_STEP                11600
   SORT_DROP_Y_STEP                3280
   ```

   当前工作区未提交版本又根据最新实测改成了 Y `+1`、X/Y 机械行程 `6100/7550`、余量 `100`、投放口 `300/-500`，并启用了禁入区。这些值不能误写成队友已提交的版本参数，合并前要根据实际机械重新标定。

   队友已提交版本由此得到的软限位是：

   - X：`0 ~ 12520`
   - Y：`0 ~ 16520`
   - 回零保护约为 X `14520`、Y `18520`

   这些数值只代表队友提交时的机械标定，移植到另一台机械时必须重新标定。

2. [`App/sorting_task.c`](./App/sorting_task.c)  
   合并核心状态机逻辑：

   - X/Y 联动回零：搜索 → 退让 → 低速接近；
   - 每个轴独立维护回零阶段、方向、超时和消抖；
   - 所有 XY 相对移动统一经过软限位和禁入区检查；
   - 投放口坐标保存、回访确认；
   - 失败时进入 `SORT_ERR_HOMING`、`SORT_ERR_XY_TIMEOUT` 或 `SORT_ERR_COORD`。

   不要用整文件覆盖最新 `main`。应以最新代码中现有的 Z/T/夹爪、台架控制台和视觉流程为主，将 XY 回零和安全检查合并进去。

3. [`App/sorting_task.h`](./App/sorting_task.h)  
   增加状态：

   - `SORT_XY_HOMING_SEARCH`
   - `SORT_XY_HOMING_BACKOFF`
   - `SORT_XY_HOMING_APPROACH`
   - `SORT_DROP_CHECK`

   状态追加在枚举末尾是有意设计，不能重新排序已有状态，否则会改变上位机收到的 `0x84` 状态编号。

4. [`Drivers/axis_home.c`](./Drivers/axis_home.c) 和 [`Drivers/axis_home.h`](./Drivers/axis_home.h)  
   增加 X/Y 限位读取、低有效判断、触发消抖、释放消抖，以及只重置单个轴消抖状态的接口。

   当前硬件约定：

   - X 零点：`PD6`
   - Y 零点：`PD5`
   - 普通 NO 限位开关接 IO 与 GND
   - 输入使用上拉，触发为低电平

5. [`Drivers/stepper_motor.c`](./Drivers/stepper_motor.c)  
   X 轴 `dir_invert` 改为 `1U`。这是电机实际方向修正；不要为了修正电机方向而修改 `SORT_X_HOME_DIR`，后者会同时改变坐标系和软限位范围。

6. [`Core/Src/main.c`](./Core/Src/main.c)  
   合并台架按键门控、X 点动、投放口标定和 XY 安全检查调用。注意保留最新 `main` 中 USART3 台架控制台和 Z 轴按键逻辑，使用配置宏决定按键归属。

7. [`Service/protocol.c`](./Service/protocol.c) 和 [`Service/protocol.h`](./Service/protocol.h)  
   增加或保留：

   - `PROTOCOL_CMD_DROP_CHECK = 0x05`
   - `PROTOCOL_EVENT_DROP_CHECK_ACK = 0x87`

   `0x05` 只请求 STM32 移动到内部保存的投放口坐标；投放口坐标不是由上位机任务包传入。若最新 `main` 已经扩展了协议命令表，不能用队友的旧 `protocol.c/h` 整文件覆盖。

### 建议合并

- [`IO.md`](./IO.md)
- [`SORTING_FLOW.md`](./SORTING_FLOW.md)
- [`PROJECT_CONTEXT.md`](./PROJECT_CONTEXT.md)
- [`LESSONS.md`](./LESSONS.md)

这些文件记录了引脚、坐标系、标定流程和已知风险。代码合并后应同步更新，避免后续按错误坐标调试。

### 不建议为 XY 功能合并

- [`MDK-ARM/diankong.uvprojx`](./MDK-ARM/diankong.uvprojx)
- [`MDK-ARM/RTE/_diankong/RTE_Components.h`](./MDK-ARM/RTE/_diankong/RTE_Components.h)
- `.gitignore` 中的 Keil 构建日志规则
- `Core/Src/adc.c` 的未提交变化

这些改动不是 XY 逻辑；除非你的 Keil 环境确实需要，否则不要用队友版本覆盖原工程配置。

## 4. 推荐合并方式

### 方案 A：原代码与队友分支没有其他并行修改

在原代码仓库中执行（先确认工作区没有未提交修改）：

```bash
git fetch origin
git checkout main
git pull --ff-only
git cherry-pick fe368a8 4423338 6870d75
```

如果只需要 XY 代码，`cherry-pick` 过程中重点解决 `main.c`、`protocol.c`、`sorting_task.c` 和 `sorting_config.h` 冲突，必须保留最新 main 的台架控制台/视觉协议；不要用队友分支最终目录删除 `Service/bench_*`、`VISION_PROTOCOL.md` 和 `tools/*`。

### 方案 B：原代码已经有自己的 Z/T/协议修改（推荐）

不要整文件复制。按以下顺序手工合并：

1. 先合并 `axis_home.*` 和 `stepper_motor.c` 的硬件层变化。
2. 把 `sorting_config.h` 中的 XY 宏合并到原配置，保留原代码已有的 Z/T 参数。
3. 在 `sorting_task.h` 枚举末尾追加 XY 状态。
4. 将 `sorting_task.c` 的 XY 回零、软限位、禁入区和投放口函数合并到原状态机。
5. 在 `main.c` 接入台架按键和安全门控。
6. 最后合并协议 `0x05/0x87`。
7. 确认 `Service/bench_*`、`VISION_PROTOCOL.md`、`tools/pc/*`、`tools/vision/*` 和 hostsim 测试仍存在。
8. 用 Keil 编译，解决冲突后再进行实机回零测试。

冲突解决原则：保留原代码已有的 T 轴、Z 轴和夹爪流程；保留队友新增的 XY 安全检查；已有状态编号不重排。

## 5. 当前工作区的未提交实验改动

当前工作区相对队友最后一个提交还修改了：

- [`App/sorting_config.h`](./App/sorting_config.h)
- [`App/sorting_task.c`](./App/sorting_task.c)
- [`App/sorting_task.h`](./App/sorting_task.h)
- [`Core/Src/main.c`](./Core/Src/main.c)
- [`IO.md`](./IO.md)
- [`LESSONS.md`](./LESSONS.md)
- [`PROJECT_CONTEXT.md`](./PROJECT_CONTEXT.md)
- [`SORTING_FLOW.md`](./SORTING_FLOW.md)

主要内容是：

- `SORT_BENCH_KEY_XY4=1`：KEY0/1 控 X 正负，KEY2/3 控 Y 正负；
- `SORT_XY_LIMIT_TEST`：上电后自动执行软限位巡检；
- 实测机械行程改为 X `6100`、Y `7550`，安全余量 `100`；
- Y 零点换到另一侧，坐标改为负方向；
- 增加 X/Y 禁入矩形；
- 投放口当前暂定为 `300 / -500`。

这组改动适合台架标定，不建议直接作为比赛固件。合入原代码时，先确认机械和按键接线，再决定是否保留；正式运行前至少确认：

```c
#define SORT_XY_LIMIT_TEST 0
#define SORT_BENCH_KEY_XY4 0
#define SORT_ENABLE_BENCH_KEYS 0
```

并把实测后的投放口坐标写回 `SORT_DROP_X_STEP`、`SORT_DROP_Y_STEP`。

## 6. 合并后的验证清单

1. 检查 X_HOME/Y_HOME 的引脚、上拉和有效电平。
2. 单独点动 X、Y，确认正方向与坐标正负一致。
3. 上电执行回零：两轴同时搜索，触发后分别退让并低速接近。
4. 拔掉一个限位开关，确认最终进入回零错误而不是持续撞击。
5. 在软限位边界附近点动，确认越界动作被拒绝。
6. 测试禁入区：进入区域的直线移动应被拒绝，离开区域应允许。
7. 记录 `X_POS`、`Y_POS`，标定并写回投放口坐标。
8. 发送协议 `0x05`，确认收到 `0x87`，并确认轴到达投放口。
9. 关闭全部台架测试宏，再编译一次正式配置。
10. 检查任务原有的取料、Z 轴、转盘、投放和回停车位流程没有被改坏。
