# 2026-10-12 转盘步进电机测试修改说明

## 测试目的

本次修改仅用于验证 TB6600 驱动的 42 步进电机和转盘传动是否能够正常：

- 使能；
- 转动；
- 持续运行；
- 通过加速斜坡提升到高速；
- 通过按键立即停止并保持位置。

当前代码不是项目最终运行逻辑，后续正式联调时需要恢复完整的分拣任务、通信、安全和回零流程。

## 本次测试使用的引脚

T 轴（转盘）使用原工程分配：

| 信号 | STM32 引脚 |
| --- | --- |
| T_STEP / PUL | PB6 |
| T_DIR / DIR | PC2 |
| T_EN / ENA | PC5 |
| 测试停止按键 | PA0 |

PA0 在正式工程中原本保留给板载 KEY1/WK_UP。本次仅为转盘单独测试临时配置为低电平有效的停止按键。

## 本次代码修改

### 1. 启用转盘测试模式

在 `Core/Src/main.c` 中启用：

```c
#define TURNTABLE_TEST_MODE 1
```

测试模式下：

- 不初始化和处理完整的串口分拣协议；
- 不运行 `SortingTask_Process()` 分拣任务状态机；
- 上电后自动启动 T 轴转盘测试；
- PA0 按下后经过约 20 ms 去抖立即停止，并保持使能防止滑移。

后期恢复项目完整运行逻辑时，应将其改回：

```c
#define TURNTABLE_TEST_MODE 0
```

并确认协议、分拣任务、回零和完整安全流程均已恢复验证。

### 2. T 轴测试速度与加速

TIM6 基础节拍约为 20 kHz（每 tick 50 us），目标速率与分频值的关系为
`STEP_INTERVAL = 20000 / SPEED_HZ`。

```c
#define TURNTABLE_TEST_TICK_HZ 20000U
#define TURNTABLE_TEST_SPEED_START_HZ 200U
#define TURNTABLE_TEST_SPEED_MAX_HZ 1200U
#define TURNTABLE_TEST_RAMP_INTERVAL_MS 20U
#define TURNTABLE_TEST_RAMP_STEP_HZ 25U
```

速度调整过程：

- 2026-10-12 初始测试为固定 250 steps/s（分频 80）；
- 2026-10-13 第一次提速为固定 400 steps/s（分频 50）；
- 2026-10-13 竞速测试改造：改为加速斜坡，从 200 steps/s 起步，
  每 20 ms 提升 25 steps/s，约 800 ms 到达最高 1200 steps/s。

加斜坡的原因是静止直接跳到高速几乎必然丢步，而丢步会污染"停机位置是否准确"的判断。

寻找最佳速度的方法：

1. 在 Keil 观察窗口监视 `turntable_test_speed_hz`；
2. 逐步提高 `TURNTABLE_TEST_SPEED_MAX_HZ`（例如 1200 -> 1600 -> 2000）；
3. 在某一速率出现丢步、异响或振动时，把 `TURNTABLE_TEST_SPEED_MAX_HZ` 降到该速率以下。

该速度仅用于当前空载或低风险传动观察，正式项目必须根据电机、电流、细分、减速机构和负载重新设置。

### 3. TB6600 共阳接线适配

当前测试按以下共阳方式连接：

```text
PUL+ -> 5V
DIR+ -> 5V
ENA+ -> 5V
PUL- -> PB6
DIR- -> PC2
ENA- -> PC5
```

T 轴 STEP 按低有效脉冲处理：

- 空闲时 PB6 保持高电平；
- 输出有效步进时 PB6 拉低；
- 有效脉冲保持一个 TIM6 周期，约 50 us；
- 脉冲结束后恢复高电平。

这样用于满足 TB6600 光耦输入对脉冲宽度的要求。

### 4. T 轴步进节拍接口

在 `Drivers/stepper_motor.c` 和 `Drivers/stepper_motor.h` 中增加了每个轴独立的步进节拍分频。

同时增加了脉冲状态保持，避免 STEP 在同一个中断中立即拉高又拉低，导致 TB6600 无法可靠识别脉冲。

2026-10-13 新增 `StepperMotor_HoldStop()`：
立即停止指定轴的脉冲输出，但保留驱动器使能，用保持力矩防止停机后滑移；
与 `StepperMotor_StopAll()` 的区别是后者会关闭使能，用于急停等安全停机。

### 5. 测试模式安全逻辑

测试模式新增 `SafetyManager_ProcessTurntableTest()`：

- 仍然监控并保留 `PD13/EMERGENCY_STOP` 急停保护；
- 暂时不检查 X/Y 限位；
- 避免未参与本次测试的 X/Y 限位输入影响 T 轴转盘测试。

正式运行逻辑恢复后，必须重新使用完整的 `SafetyManager_Process()`，不能长期跳过 X/Y 限位保护。

### 6. 停止逻辑

PA0 配置为上拉输入，低电平表示按下：

1. 按键低电平持续约 20 ms 后确认按下；
2. 立即停止输出步进脉冲（调用 `StepperMotor_HoldStop()`），不再额外前进；
3. 保留驱动器使能，用保持力矩防止停机后滑移；
4. 记录停机时的软件步数 `turntable_test_stop_position` 和速率 `turntable_test_stop_speed_hz`；
5. 测试结束，不再自动重新启动。

注意：20 ms 去抖在最高速率 1200 steps/s 下对应约 24 步，
即从按下按键到真正停机之间仍会多走约 24 步。
这段延迟由去抖时间和当前速率共同决定，是核对"停机位置是否一致"时的固定参考，
不是丢步；若连续多次在标记处按下按键，停机步数应当基本稳定。

### 7. 连续运行方式

原实现每走完 1000 步就重新下发一次 `StepperMotor_MoveTo()`，
段与段之间会出现"走到目标即关闭使能 -> 主循环立刻重新使能"的瞬间，
高速下容易造成抖动和丢步。

2026-10-13 改为一次性下发 `TURNTABLE_TEST_RUN_STEPS`（2000000 步）的大目标，
使能全程保持，由 PA0 随时打断，运动更平稳，也更适合竞速测试。

## 后续恢复项目完整逻辑时的检查项

恢复正式工程前，必须完成以下工作：

1. 将 `TURNTABLE_TEST_MODE` 改为 `0`；
2. 恢复 `Protocol_Init()` 和 `Protocol_Process()`；
3. 恢复 `SortingTask_Init()` 和 `SortingTask_Process()`；
4. 恢复完整的 X/Y/T 回零流程；
5. 恢复 X/Y 限位监控；
6. 确认 PA0 是否继续作为板载 KEY1，不能默认作为正式停止输入；
7. 根据 TB6600 的实际接线确认 ENA、DIR、PUL 的有效电平；
8. 增加正式的步进电机加减速；
9. 对急停、限位、回零失败、运动超时进行完整测试；
10. 确认测试代码不会被正式发布固件保留。

## 当前状态

本文件和相关代码修改仅作为 2026-10-12 转盘电机测试记录。按照当前要求，本次修改暂不提交到 GitHub，后续确认测试完成并恢复正式工程逻辑后再统一整理提交。
