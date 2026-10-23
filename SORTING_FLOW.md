# 自动分拣流程说明（2026-10-20）

## 1. 机械拓扑

- X/Y：UM2 式 Cartesian 龙门，两根独立步进轴，可直接分别定位。
  当前协议仍使用**绝对步数** `pick_x_step` / `pick_y_step`，不强制换算成 mm。
- Z：舵机 + 齿条，三个工作高度：
  - `TRAVEL`：运输/初始高度，XY 平移前必须在此高度；
  - `APPROACH`：物块上方接近高度；
  - `GRASP`：物块抓取高度。
- 夹爪：PA8 / TIM1_CH1 硬件 PWM 舵机。
- T：42-40 两相步进 + 5.2 行星减速，带动 6 仓转盘；
  PD12 为 `T_HOME` 零位霍尔（上拉、低有效，有磁铁 = 低）。
- 投放方式：夹爪把物块送到固定投放口，转盘把目标仓转到投放口下方。

### 1.1 转盘零位与 6 仓

- 零位磁铁：11.5 mm × 4.5 mm 带孔强磁，两片叠装、固定在转盘内侧，
  与霍尔气隙 7～8 mm；霍尔 datasheet 为 4.5～24 V 供电、开集电极、低有效。
- **只装一个零位磁铁即可**。霍尔只输出“有/无磁铁”，6 个相同磁铁在上电时
  无法区分是哪个仓；步进 + 5.2 行星在轻载下不掉步，每次上电/故障复位回零，
  再用步数定位 6 个仓就够了。6 个磁铁只有在做“逐仓闭环校验”时才有意义，
  且需要能区分磁铁特征（宽度/极性），当前不采用。
- 42-40 电机 200 整步/圈 × 1/16 细分 × 5.2 = **16640 步/圈**，
  标称 2773 步/仓。16640 不能被 6 整除，目标仓必须用
  `SORT_T_BIN_STEP(index)` 四舍五入；六段之和严格等于 16640，
  否则每次 6 仓循环会累积 2 步误差。
- 2026-10-23 实机：霍尔触发点相对“1 号仓正对投放口”还差一个逆时针小角度，
  暂用 `SORT_T_HOME_OFFSET_STEP = 0`；机械位置固定后按
  `N = 角度/360 × 16640` 补偿（符号取决于 `+步数` 的物理方向）。

## 2. 状态机

```
INIT
  -> HOMING (T 轴找 T_HOME)
  -> T_HOMING_SEARCH -> BACKOFF -> APPROACH -> READY

READY --收到任务-->
  MOVE_TO_PICK_XY       X/Y 同时定位到取料点
  -> Z_APPROACH         Z 到接近高度
  -> GRIP_OPEN          确认夹爪张开
  -> Z_DESCEND          Z 到抓取高度
  -> GRIP_CLOSE         夹紧
  -> [GRIP_VERIFY]      可选：IR 对射确认夹住
  -> LIFT_AND_TURRET    Z 抬升，同时 T 轴开始转到目标仓
  -> MOVE_TO_DROP_XY    Z 到运输高度后，X/Y 去投放口
  -> TURRET_WAIT        等 T 到位 + 稳定时间
  -> DROP_OPEN          开爪投放
  -> RETURN_SAFE        回停车位/安全位
  -> TASK_DONE -> READY
```

关键互锁：

1. 只有 X/Y 都到位后，Z 才从接近高度下降到抓取高度。
2. 只有夹爪闭合稳定后，Z 才抬升，T 才启动。
3. 只有 Z 回到运输高度后，X/Y 才横向去投放口。
4. 只有 T 到位并稳定后，夹爪才打开投放。
5. `STOP` 保持夹爪当前脉宽，不会在半空主动开爪。

## 3. 时间与超时

所有状态等待使用 `HAL_GetTick()`（1 ms），不再使用 TIM6 的 20 kHz 计数。

| 参数 | 默认值 | 位置 |
| --- | --- | --- |
| Z 接近 | 600 ms | `SORT_Z_APPROACH_MS` |
| 夹爪张开稳定 | 400 ms | `SORT_GRIP_OPEN_MS` |
| Z 下降 | 600 ms | `SORT_Z_DESCEND_MS` |
| 夹爪闭合稳定 | 400 ms | `SORT_GRIP_CLOSE_MS` |
| Z 抬升 | 600 ms | `SORT_Z_LIFT_MS` |
| T 到位稳定 | 200 ms | `SORT_T_SETTLE_MS` |
| 投放开爪 | 400 ms | `SORT_DROP_OPEN_MS` |
| XY 单段超时 | 10000 ms | `SORT_XY_TIMEOUT_MS` |
| T 到位超时 | 10000 ms | `SORT_T_TIMEOUT_MS` |
| 整任务超时 | 45000 ms | `SORT_MISSION_TIMEOUT_MS` |

这些时间都是占位值，必须按实机舵机速度、Z 行程和转盘惯量调整。

T 轴稳定计时从“观察到 T 轴空闲”开始，而不是从进入 `TURRET_WAIT` 开始；
XY 与 T 并行运动时，T 的超时在 XY 到位前也会检查，避免 XY 较慢掩盖 T 卡死。

## 4. 运动层

`Drivers/stepper_motor.c`：

- TIM6 20 kHz，`step_interval` 以 50 us 节拍为单位；
- 每轴独立梯形/线性加减速：`SetSpeed()` + `SetAcceleration()`；
- `MoveTo()` 设置绝对目标，`MoveRelative()` 用于回零退让；
- `JogStart()` 用于找 T_HOME，必须由状态机限时并调用 `Stop()`；
  回零退让阶段反向 Jog 到霍尔释放，再同向慢速接近，不依赖固定退让步数；
- 到位后保持驱动器使能（有保持力矩）；`StopAll()` 才关闭使能；
- 软限位在 `MoveTo()` 中检查；
- 位置可信标志 `position_valid`：T 回零后才有效；X/Y 当前按上电原点假设。

## 5. 协议（USART2，版本 1）

帧格式不变：

```
AA 55 | version(1) | command(1) | length(1) | payload | CRC16(低字节在前)
```

分拣任务载荷固定 17 个有效字节：

| 偏移 | 类型 | 字段 |
| --- | --- | --- |
| 0 | u16 | task_id |
| 2 | u8 | class_id |
| 3 | u8 | box_id |
| 4 | i32 | pick_x_step |
| 8 | i32 | pick_y_step |
| 12 | i32 | drop_step |
| 16 | u8 | state |

> 结构体在 ARM 上 `sizeof` 可能为 20（尾部 padding）。接收端只要求
> `length >= 17`，按固定偏移解析，多余字节忽略；发送 17 或 20 字节都可。

事件：

| 事件 | 含义 |
| --- | --- |
| `0x80` | 任务已接收（ACK） |
| `0x81` | 忙，任务未接收（BUSY） |
| `0x82` | 任务完成（DONE） |
| `0x83` | 错误，载荷 task_id + error_code |
| `0x84` | 状态，载荷 state + error_code |
| `0x85` | 复位故障已受理 |
| `0x86` | 回零请求已受理 |

错误码见 `App/sorting_task.h` 的 `SortError_t`。

`box_id` 默认按 **1..6** 编号（`SORT_BOX_ID_ONE_BASED = 1`）。
若视觉侧使用 0..5，把该宏改为 0。

## 6. 必须实机标定的参数

在 `App/sorting_config.h`：

1. `SORT_DROP_X_STEP` / `SORT_DROP_Y_STEP`：投放口 XY 绝对步数；
2. `SORT_PARK_X_STEP` / `SORT_PARK_Y_STEP`：停车位；
3. `SORT_X_MIN_STEP` / `MAX`、`SORT_Y_MIN_STEP` / `MAX`：软限位；
4. `SORT_XY_MAX_SPEED_STEPS_S`、`SORT_XY_ACCEL_STEPS_S2`：X/Y 速度/加速度；
5. T 轴机械：42-40（200 整步/圈）+ 5.2 行星减速，1/16 细分 →
   `SORT_T_STEPS_PER_REV = 16640`；`SORT_T_STEPS_PER_BIN` 只是标称 2773，
   正式目标用 `SORT_T_BIN_STEP(index)` 四舍五入，六段之和严格等于一圈；
   `SORT_T_MIN_STEP` / `SORT_T_MAX_STEP`：T 轴软限位，默认按 3×整圈步数自动计算
   （位置每任务归一化到一圈内）；
6. `SORT_T_HOME_DIR`、`SORT_T_HOME_OFFSET_STEP`：回零方向和 1 号仓偏移。
   退让不再使用固定步数：搜索触发后反向 Jog 直到霍尔释放，再同向慢速接近，
   所以磁铁有效宽度 W 不需要测量；`SORT_T_HOME_BACKOFF_SPEED_STEPS_S`
   只影响退让时间。霍尔卡死或 20 s 内退不出来会报 `SORT_ERR_HOMING`；
7. `SORT_T_MAX_SPEED_STEPS_S`、`SORT_T_ACCEL_STEPS_S2`、回零速度和接近速度；
8. `SORT_T_USE_SHORTEST_PATH`：有滑环可设 1 走最短路径，否则保持 0 单向；
   `SORT_T_ROTATE_SIGN`：单向转的优选方向（+1/-1），建议与 `SORT_T_HOME_DIR` 同号；
9. `SORT_BENCH_T_JOG_STEPS`：READY 时 KEY2/KEY3 单仓点动步数，默认一个仓距 2773；
   点动后 T 位置作废，必须重新回零；
10. 所有 `SORT_*_MS` 动作等待时间。

在 `Drivers/servo_z.h`：

- `ZSERVO_PULSE_TRAVEL_US`、`APPROACH`、`GRASP`；
- 若方向相反，直接对调三个脉宽值。

在 `Drivers/servo_gripper.h`：

- `SERVO_GRIPPER_OPEN_PULSE_US`、`CLOSE`；
- BDS300 标定前不要直接接舵机上电。

## 6.1 首次上台联调顺序（T 轴 + 霍尔）

1. 断电接线：霍尔 VCC → 5 V、GND → GND、OUT → PD12；磁铁两片叠装，
   与霍尔气隙 7～8 mm。
2. 只上电、不使能电机：万用表量 PD12，无磁铁约 3.3 V、磁铁靠近约 0 V；
   如果相反，先查供电和是否低有效开集电极，不要先改代码。
3. 确认 TB6600S 细分和电流拨码：1/16 = S1=OFF、S2=OFF、S3=ON；
   电流按电机铭牌（1.7 A → 1.5 A/1.7 A，S4=ON、S5=ON、S6=OFF；
   没有铭牌先从 1.0 A/1.2 A 验证，详见 IO.md 2.7）。细分不是 1/16
   就改 `SORT_T_DRIVER_MICROSTEP`，再重新编译；否则 6 仓角度会整体差一个比例。
4. 上电回零：观察“搜索 → 反向退出 → 慢速接近 → READY”是否一次完成，
   听有无丢步/异响，摸电机是否过热。
5. 重复回零 10 次（发 HOME 或复位），每次在 1 号仓位置做标记；
   偏差应在 1～2 mm 内。偏差大时先查磁铁/气隙和退让速度。
6. READY 时按 KEY2 六次走一圈，检查 1～6 仓是否依次正对投放口；
   若六次后明显不是一圈（差很多），先查 TB6600 细分和减速比，
   不要用 `SORT_T_HOME_OFFSET_STEP` 硬凑；只有固定的小角度偏差才用偏移补偿。
7. 断电重启 3 次，确认每次都能回零；再连续跑 20～50 次换仓，
   观察是否累积偏仓（正常情况下不应累积）。
8. 比赛固件把 `SORT_ENABLE_BENCH_KEYS` 置 0，重新编译并复核。

## 7. 已知限制

1. **X/Y 没有回零开关**：当前 `SORT_ASSUME_XY_ORIGIN = 1`，
   上电时把 X/Y 软件位置清零。操作者必须先把龙门推到机械原点再上电/复位。
   装限位后应改为 0，并补充 X/Y 回零状态。
2. **T 轴必须接 T_HOME**：找不到开关会在 20 s 后进入 `ERROR`；
   霍尔卡死/短路导致退让阶段 20 s 内不释放，也会报 `SORT_ERR_HOMING`。
3. **Z 轴齿条不自锁**：断电或舵机掉电时可能下坠，需要恒力弹簧/配重/
   自锁机构；软件只能保证上电期间保持位置。
4. **夹爪 IR 未默认启用**：`SORT_GRIP_VERIFY_ENABLE = 0`；标定极性和
   安装位置后改为 1。
5. **协议仍是步数坐标**：视觉侧给 mm 时，需要在发送端或新增一层
   `mm -> steps` 换算；本次没有单方面改协议版本。
6. **回零为 T 轴单轴**：X/Y 的回零开关方案尚未接入。
7. **复位会重新回零并在 HOMING 入口开爪**：如果故障/软停时夹爪还夹着物块，
   应先人工取出物块再发 `RESET_FAULT`，避免开爪时掉落。
8. **投放高度固定**：当前在 Z 运输高度直接开爪，假设投放口有漏斗/导向。
   若需要下降到仓口再开爪，应在 `TURRET_WAIT` 后增加 Z 下降/抬升状态和
   对应的 `ZSERVO_PULSE_*` 标定值。

## 8. 夹爪调试记录与标定结果（2026-10-20）

> 目的：BDS300 的脉宽范围未标定，用手动小步进找到机械开合临界值。
> 2026-10-20 已完成标定，并按 8.7 恢复自动开合、上电 PWM 和 READY 门控；
> 台架按键随后按 8.8 改为“一次到位”，再按 8.9 修正上电动作和按键响应。
> 下面保留调试过程和实测结果，换舵机/重新装舵机臂时按 8.3 重标。

### 8.1 调试期改动清单（已恢复）

| 文件 | 位置 | 原逻辑 | 调试期逻辑 |
| --- | --- | --- | --- |
| `Drivers/servo_gripper.c` | `ServoGripper_Init()` | 写 `CCR1 = 1500 us` 并 `HAL_TIM_PWM_Start()` | 已注释：上电不输出 PWM，舵机不上力、不回归任何位置 |
| `Drivers/servo_gripper.c` | `ServoGripper_SetPulse()` | 直接写 `CCR1` | 新增 `ServoGripper_EnsurePwm()`：第一次真正输出脉宽时才启动 TIM1_CH1 |
| `Core/Src/main.c` | 主循环台架按键 | 仅 `SORT_SYS_READY` 时调用 `BenchKey_Process()` | 已注释：改为 `SafetyManager_IsSafe()` 即允许按键，T 轴没回零进入 `ERROR` 也能标定 |
| `App/sorting_task.c` | 9 处自动开合 | `Submit`、`INIT`、`HOMING`、`T_HOMING_APPROACH`、`Z_APPROACH`、`Z_DESCEND`、`TURRET_WAIT`、`RETURN_SAFE`、`TASK_DONE` 调 `ServoGripper_Open/Close` | 已全部注释，避免自动流程覆盖 KEY0/KEY1 设定的脉宽 |

> 上表记录的是调试期状态；2026-10-20 已按 8.7 全部恢复，
> 当前代码不再有这些临时注释；台架按键本身按 8.8 改为一次到位，
> 门控和上电动作又按 8.9 修正（以 8.9 为准）。

调试期未改动、现在仍生效的部分：

- `ServoGripper_Stop()` 在 `Fail()`、`Stop()`、`ResetFault()`、急停分支中仍会
  把目标脉宽冻结为当前输出，不会主动开爪；
- PD13 急停仍然有效，急停按下时台架按键被 `SafetyManager_IsSafe()` 挡住；
- 限幅 `SERVO_GRIPPER_MIN_PULSE_US` / `MAX_PULSE_US` 已按 8.6 实测收紧为
  1390/1800 us，手动按键最多到机械临界、不会越过。

### 8.2 调试期实际行为（已恢复）

> 以下为调试期行为，2026-10-20 已恢复；当前固件按 8.7 的正式逻辑运行，
> 台架按键按 8.8 改为一次到位，上电动作与按键门控按 8.9 修正。

1. **上电**：`ServoGripper_Init()` 不写 CCR1、不启动 TIM1_CH1，PA8 没有 PWM，
   舵机不上力、不回归中位。
2. **按键**：KEY0（PD0）松开 → 目标脉宽 +10 us；KEY1（PD1）松开 → −10 us；
   30 ms 消抖，只在**释放沿**触发一次，按住不会连发。
3. **第一次按键**：`ServoGripper_Process()` 调 `ServoGripper_SetPulse()`，
   此时才启动 PWM，调试期起点是 1500 us（宏 `SERVO_GRIPPER_START_PULSE_US`
   已在 8.9 删除），按 **5 us / 20 ms** 的斜率缓动到目标。
   → 调试期需要先手动把齿条放到中间行程，否则舵机会先跳到 1500 us 附近。
4. **状态机**：仍会跑，但夹爪不再自动开合；如果 T_HOME 没接，20 s 后会进入
   `SORT_ERROR`，这不影响按键（8.1 已取消 READY 门控）。
5. **观察脉宽**：没有串口回显，用 Keil Watch 看 `gripper_pulse_us`，
   或调用 `ServoGripper_GetPulse()`；`ServoGripper_IsClosed()` 只反映最后一次
   按键方向，不代表机械真的到位。

### 8.3 标定步骤

1. **空载**（舵机不装齿条）：先小范围上下步进，确认舵机响应、方向、
   是否有异响；记录能稳定响应的最小/最大脉宽。
2. **装齿条**：从中间位置开始，按 10 us 逐步向“合”方向走，找到夹爪刚好
   闭合且不顶死的脉宽，记为 `CLOSE`；再向“开”方向走到刚好完全张开，
   记为 `OPEN`。两端都要留 20～50 us 余量，不要贴着机械死点。
3. **复核机械行程**：齿轮 20 齿、模数 0.8，闭合到张开约 3.5 个齿距
   ≈ 8.8 mm，对应 63°。若实测脉宽差对应角度明显偏离 63°，先检查齿条
   啮合和舵机安装角，再改 `OPEN/CLOSE`。
4. **连续开合 20 次**：确认不丢步、不啃齿、舵机不烫、电源不被拉垮。
5. 把 `OPEN/CLOSE` 写回 `Drivers/servo_gripper.h`；方向相反时两个宏对调。
   本次结果见 8.6，已写入 `1410/1780`。

### 8.4 BDS300 手册与参数获取

BDS300 在本项目里是**机械爪开合舵机**：舵机轴带动 20 齿齿轮，齿轮推动齿条，
齿条带动夹爪开合（与 PA1/TIM2 的 Z 轴舵机是两套独立机构）。它不控制 Z 高度。

手册拿不到时按下面顺序处理：

1. **找卖家/厂家要 datasheet**：订单页客服、淘宝/京东/立创商城商品页，
   明确索要“PWM 频率、脉宽范围（min/center/max）、死区、堵转扭矩、
   空载速度、工作电压、堵转电流、线序”。
2. **看舵机本体和包装标签**：型号、批次、厂家；不同批次参数可能不同，
   以实物为准。
3. **厂家官网/公众号/技术群**：搜索“BDS300 舵机 规格书/参数”。
4. **拿不到手册就直接实测**（本项目推荐）：
   - 用舵机测试仪或本固件的 KEY0/KEY1 从中间位置小步进/一次到位向两端找临界；
   - 示波器确认 PA8 周期 20 ms、脉宽随按键变化；
   - 记录舵机开始响应/停止响应的脉宽、中位、方向；
   - 用限流电源观察堵转电流，确认 7.4 V 供电和 3.3 V 信号是否可靠；
   - 若舵机对 3.3 V 信号无响应，需要 5 V 电平转换或换 5 V 供电的控制器，
     不能直接把信号接到 7.4 V。

> 注意：`Drivers/servo_gripper.h` 里的 7.4 V、10 kg·cm、0.13 s/60°
> 来自选型资料；脉宽临界已按 8.3 实测（见 8.6），PWM 频率仍按 50 Hz 输出。
> 换舵机或重新装舵机臂后必须重新标定，不能直接沿用 1410/1780 us。

### 8.5 调试期注意事项

1. 舵机电源独立 7.4 V，就近并 100～2200 uF 电解电容，必须与 MCU 共地；
   不要用 STM32 的 3.3 V 或 ST-Link 的 5 V 给舵机供电。
2. 限幅已收紧为 1390～1800 us（8.6 实测机械临界），按键到端不会越过；
   但顶在临界仍会堵转，听到嗡嗡声立即反向退 2～3 次。
3. 上电不再固定输出 1500 us（8.9）：`ServoGripper_Init()` 不启动 PWM，
   进入主循环后由状态机张开到 1780 us；换舵机重标时用 KEY0/KEY1 逐步走。
4. 急停 PD13 低有效：按下后按键被挡住；松开后无需复位即可继续按键标定。
5. `tools/hostsim` 的 test 8 校验“上电不启动 PWM、第一次 Open/JumpTo 才启动”
   以及 Open/Close 后的 1410/1780 us；当前 700 checks / 0 failures。

### 8.6 实测结果（2026-10-20）

测试条件：BDS300 装齿条、空载，软件起点 1500 us，每次按键 10 us，
每按一次等舵机走完再按。

| 方向 | 从 1500 起的按键 | 实测临界脉宽 | 说明 |
| --- | --- | --- | --- |
| 闭合（KEY1） | 11 次 | 1390 us | 夹爪刚好完全闭合 |
| 张开（KEY0） | 30 次 | 1800 us | 夹爪刚好完全张开 |

- 总行程 410 us。按 BDS300 300°/2000 us 估算约 61.5°，与机构 63°
  （3.5 个齿距 / 20 齿）基本吻合，说明齿条啮合和舵机安装角正常。
- 写入值各留 20 us 余量：`CLOSE = 1410 us`、`OPEN = 1780 us`。
  这样自动流程不会长时间顶在机械死点。若空载时发现差一点没合拢/张开，
  可把余量减到 10 us（1400/1790）；不要直接用 1390/1800 长时间顶住。
- 限幅已收紧为 `SERVO_GRIPPER_MIN/MAX_PULSE_US = 1390/1800`，
  手动按键最多到机械临界，不会越过。
- 以上数值只对当前这台 BDS300 和当前舵机臂安装有效；换舵机或重新装
  舵机臂后必须重新标定（回到 8.3）。

### 8.7 恢复清单（2026-10-20 已完成）

1. `Drivers/servo_gripper.h`：已写入 `1410/1780`，限幅 `1390/1800`；
   换舵机后重新按 8.3 标定并更新。
2. `Drivers/servo_gripper.c` `ServoGripper_Init()`：先恢复过“上电写
   `CCR1 = 1500 us` 并启动 TIM1_CH1”；后按 8.9 改为上电不写 CCR1、
   不启动 PWM，第一次 Open/Close/JumpTo 才输出。
3. `Core/Src/main.c`：已恢复 READY 门控，删除 `SafetyManager_IsSafe()`
   版本的 `BenchKey_Process()` 调用。
4. `App/sorting_task.c`：已恢复 9 处 `ServoGripper_Open/Close`。
5. `SORT_GRIP_OPEN_MS` / `SORT_GRIP_CLOSE_MS = 400 ms`，足够 410 us 行程
   （BDS300 0.13 s/60°）；`tools/hostsim` 已通过（689 checks / 0 failures），
   完整空载分拣流程待实机复核。

> 2026-10-20 后续又按台架需要新增了“按键一次到位”，见 8.8；
> 自动流程和 `Open/Close` 未改动；上电写 1500 us 已被 8.9 取代。

### 8.8 台架按键一次到位（2026-10-20，当前行为）

视觉尚未接入、无法跑完整流程时，需要手动模拟夹爪开/合。按键从“每次 10 us”
改为“一次到位”，配置集中在 `App/sorting_config.h`：

| 宏 | 默认 | 作用 |
| --- | --- | --- |
| `SORT_ENABLE_BENCH_KEYS` | 1 | 台架按键总开关；正式比赛改 0 |
| `SORT_BENCH_KEY_JUMP` | 1 | 1 = 一次到位；0 = 每次按下 10 us 的标定模式 |
| `SORT_BENCH_KEY_AT_LIMIT` | 0 | 0 = 跳到 1780/1410 us（与自动流程一致，留 20 us 余量）；1 = 跳到实测机械临界 1800/1390 us |
| `SORT_BENCH_KEY_DEBOUNCE_MS` | 20 | 按下电平连续保持多久算一次有效按键 |

实际行为：

1. 在**按下沿**触发一次，20 ms 消抖，按住不放不会连发；
2. KEY0（PD0）→ `ServoGripper_JumpTo(SERVO_GRIPPER_OPEN_PULSE_US, 0)`；
   KEY1（PD1）→ `ServoGripper_JumpTo(SERVO_GRIPPER_CLOSE_PULSE_US, 1)`。
   `JumpTo()` 直接写 CCR1 并同步目标值，没有 5 us / 20 ms 缓动；
3. 门控：`SafetyManager_IsSafe()` 为真，且 `SortingTask_HasMission() == 0`。
   即 READY、上电回零期间、回零失败后的 ERROR 都能按键；任务执行期间不响应；
4. 上电由 `SORT_SYS_INIT` / `SORT_T_HOMING_APPROACH` 调 `ServoGripper_Open()`，
   进入 READY 时夹爪已在 1780 us，所以第一次按 KEY0 基本不动、按 KEY1
   才会闭合，这是正常现象；回零完成那一次 `Open()` 会覆盖回零期间的手动闭合；
5. 急停 PD13 按下时按键一律不响应。

> 比赛固件只需要把 `SORT_ENABLE_BENCH_KEYS` 改成 0。`tools/hostsim` 已增加
> `JumpTo()` 的直接写 CCR1 / 限幅 / `IsClosed()` 校验；8.9 修正后为
> 700 checks / 0 failures。

### 8.9 上电动作与按键响应修正（2026-10-20，当前行为）

台架反馈两个现象：上电后夹爪先往“半张开”位置收一下、随后才张开；KEY0/KEY1
大部分时候没反应、偶尔才有效。原因如下：

1. **上电先合一下**：`ServoGripper_Init()` 原先写 `CCR1 = 1500 us` 并立即
   `HAL_TIM_PWM_Start()`；而 `main()` 紧接着执行 `LCD_Init()`，其中有
   20 + 120 + 120 + 120 + 50 ≈ 430 ms 的 `HAL_Delay`。这 430 ms 里 PA8
   一直输出 1500 us，舵机就往半开位走；等主循环进入 `SORT_SYS_INIT`
   调 `ServoGripper_Open()`，才又跳到 1780 us，看起来就是“先合一下再张开”。
2. **按键大多无效**：原门控只在 `READY` 或 `(SORT_ERROR && SORT_ERR_HOMING)`
   放行。没接 T_HOME 时要等 `SORT_T_HOME_TIMEOUT_MS`（20 s）超时才进 ERROR，
   这 20 s 内按键全部被忽略；另外按键绑在**释放沿**、消抖 30 ms，
   轻点或按住不放时都像“没反应”。

修正（当前代码）：

- `ServoGripper_Init()` 只复位驱动状态，**不写 CCR1、不启动 TIM1_CH1**：
  上电 PA8 无脉冲、舵机不上力。第一次 `Open()`/`Close()`/`JumpTo()` 由
  `EnsurePwm()` **先写 CCR1 再启动 PWM**，第一帧就是目标脉宽。
  于是上电动作只剩一次：`SORT_SYS_INIT` → `ServoGripper_Open()` → 1780 us。
- 按键改为**按下沿**触发，消抖改为 `SORT_BENCH_KEY_DEBOUNCE_MS = 20 ms`。
- 门控改为 `SafetyManager_IsSafe() && (SortingTask_HasMission() == 0)`，
  上电回零期间即可用；为此在 `sorting_task.c/h` 新增只读接口
  `SortingTask_HasMission()`。
- 删除 `SORT_BENCH_KEY_ALLOW_HOMING_ERROR`（门控不再依赖错误码）；
  删除 `SERVO_GRIPPER_START_PULSE_US`（上电不再输出中间脉宽）。
- `tools/hostsim` test 8 改为校验：Init 不启动 PWM、不写 CCR1；第一次
  JumpTo/Open 才启动 PWM 且第一帧即目标值。当前 700 checks / 0 failures。

> 自动流程没有削弱：`SORT_SYS_INIT`、`SORT_SYS_HOMING`、
> `SORT_T_HOMING_APPROACH`、投放和 `TASK_DONE` 仍然按原逻辑开合夹爪，
> 只是上电不再先输出一个 1500 us 的中间位。
