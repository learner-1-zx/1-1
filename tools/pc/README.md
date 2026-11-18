> 项目当前状态见根目录 [`AI_CONTEXT.md`](../../AI_CONTEXT.md)；本文件只说明串口工具。


# 电脑串口测试（tools/pc）

按键不够、接线时腾不出手按板载 KEY0~KEY3 时，可以用电脑串口代替手工按键。
两条通道互相独立，也可以同时接：

| 通道 | 引脚 | 用途 | 固件开关 |
|---|---|---|---|
| USART3 文字控制台 | PB10=TX、PB11=RX，115200 8N1 | **虚拟按键**（短按/长按）、状态打印 | `SORT_DEBUG_CONSOLE`（`App/sorting_config.h`） |
| USART2 二进制协议 | PA2=TX、PA3=RX，115200 8N1 | 回零、下发分拣任务、停止、复位、查状态 | 常开（视觉/上位机口） |

> **USART2 是留给 OpenART 摄像头的，不要和摄像头同时接。**
> 摄像头在线时只用 USART3 控制台；要单独测 `status/home/stop/reset/mission`，
> 先把摄像头从 PA2/PA3 断开，再接 USB-TTL，否则两个发送端会顶在同一根线上。
>
> ！！**`ok` / `zup` / `zero` / `zk` / `gb` / `tjog` 这些文字命令只走 USART3
> （PB10/PB11）**。USART2（PA2/PA3）是 `AA 55 ... CRC16` 二进制视觉协议，
> 用 ASCII 串口助手看只会是乱码，发 `ok` 也不会有文字回复。

> 电平是 **3.3 V TTL**，必须用 USB-TTL（CH340/CP2102 等）并共地；
> 不能把 RS232（±12 V）直接接到 MCU 引脚上。
> USART3 至少要接三根：PB10→TTL 的 RX（看打印）、PB11→TTL 的 TX（发命令）、GND→GND。
> 只接 PB10 时只能看输出，串口发的命令进不来。

## 1. USART3 虚拟按键（推荐）

固件端命令定义见 `Service/bench_console.h`：

| 发送 | 动作 |
|---|---|
| `0` `1` `2` `3` | 短按 KEY0~KEY3（保持 100 ms，超过 20 ms 消抖） |
| `A` `B` `C` `D` | 长按 KEY0~KEY3（保持 1500 ms，KEY0 用它测“按住 1 s 存投放口 X”） |
| `s` | 打印状态、轴位置、Z/夹爪脉宽、四个按键原始电平（约 100 ms 后输出） |
| `x` | 释放虚拟按键（约 100 ms 后执行） |
| `h` / `?` | 帮助（大写 `H`/`?` 立即输出） |

用任意串口助手（XCOM、SSCOM、PuTTY、VSCode Serial Monitor）发单字符即可，
不用加回车（固件会忽略 CR/LF）。也可以用本目录脚本交互：

```powershell
py -3 tools/pc/serial_tool.py -p COM6 console
> 2       # 发 KEY2：Z 轴一键到最下（当前 SORT_BENCH_KEY_Z_SERVO=1）
> 3       # 发 KEY3：Z 轴一键到最上
> a       # 长按 KEY0：开爪 + 按住满 1 s 存投放口 X，串口打印 DROP_X=
> s       # 看状态和位置
> q
```

要点：

- 虚拟按键**只提供原始电平**，消抖、连发、“安全 + 无任务”门控、KEY0 长按判据
  全部复用 `Core/Src/main.c` 的 `BenchKey_Process()`，和真实按键走同一条路径；
- 有任务正在执行（`MISSION=1`）时串口按键同样不响应，这时按 `s` 看 `STATE=` 定位卡点；
- 长按 1500 ms 期间，可连发的键（夹爪标定 KEY1、Z 逐档 KEY2/KEY3）会按
  `SORT_BENCH_KEY_REPEAT_MS`（150 ms）连发，和按住实体键一致；
- 虚拟按键到期自动失效，不会把某个键“卡在按下”。

### 1.0 当前台架模式：XY 找零点 / 找软限位（2026-11-05）

`App/sorting_config.h` 现在是 `SORT_XY_BENCH_MODE = 1`，按键语义整体换成 XY 点动：

| 发送 | 动作（`SORT_XY_BENCH_KEY_GOTO_END=1`，当前） |
|---|---|
| `0` / `1` | X 到零点 `0` / X 到行程上限 `5400`（一键到底，约 7 s） |
| `2` / `3` | Y 到零点 `0` / Y 到行程上限 `6800`（一键到底，约 9 s） |
| `s` | 看 `STATE=` / `X_POS=` / `Y_POS=` / `POS_OK=` / `X_BUSY=` |

（`SORT_XY_BENCH_KEY_GOTO_END=0` 时改回点动：一次 200 微步，`A`~`D` 长按按
`SORT_XY_BENCH_JOG_REPEAT` 决定是否连发。）

- 上电 X、Y **同时**慢速回零（不改 T），串口按
  `X/Y STAGE=1(搜索) → 2(退让) → 3(慢速接近) → X_HOME OK DIR=-1` 打印，
  期间还会打 `X_SW_PRESS` / `X_SW_RELEASE`（开关动作边沿）；
  **压到开关后会先退一点再慢慢压回去才停**，正常；
- 回零方向不对时电机往行程外走：**按任意方向键立刻停**（串口发 `0`~`3` 也行），
  打印 `HOMING ABORT (key pressed)`；再用 `1`/`3` 确认哪边朝零点开关，
  优先改 `Drivers/stepper_motor.c` 里该轴的 `dir_invert`（保持 `SORT_X/Y_HOME_DIR=-1`）；
- 回零完成后用 `0`/`1`/`2`/`3`（或实体键）走到另一端，每次打印 `X_POS=` / `Y_POS=`
  （本次目标位置）和 `POS_OK=1`（位置可信）；
- **2026-11-05 行程已实测**（一按一步 200 微步）：X 点 28 次 = 5600、Y 点 36 次 = 7200，
  取 `SORT_X_TRAVEL_STEPS=5400`（留 200 步）、`SORT_Y_TRAVEL_STEPS=6800`
  （留 400 步；一键到底到 7000 时 Y 余量偏小，又多让 200 步），
  `SORT_XY_BENCH_WIDE_LIMIT=0`：越界会打印
  `X JOG REJECT LIMIT` / `Y JOG REJECT LIMIT`，不再撞机械端；
- 两轴独立，可以同时动：串口先后发 `1`、`3` 让 X+ 和 Y+ 一起走（虚拟键同时有效），
  也可直接用运动指令 `X3000` / `Y5000` 快速定位（受软限位保护）。

## 1.1 USART3 运动指令（不开视觉也能跑全流程）

同一个 USART3、同一套串口设置，**大写字母开头**的命令直接驱动电机/舵机。
运动指令是“一行一条”，**必须以回车/换行结束**（串口助手勾选“发送新行”，
本目录脚本会自动补 `\n`）；单字符按键 `0~3`/`A~D`/`s`/`h`/`x` 仍然即时生效、不用回车。
全部走正式驱动，不经过 `sorting_task.c` 的状态机，所以队友改自动流程时这里不受影响。

| 命令 | 作用 |
|---|---|
| `X1200` | X 轴移动到绝对微步 1200（范围 `0..SORT_X_TRAVEL_STEPS`） |
| `X` | 只打印当前 X 位置 |
| `Y500` | Y 轴移动（`SORT_ENABLE_Y_AXIS=0` 时回 `DISABLED`；当前已启用 Y） |
| `T3` | 转盘转到 3 号仓（1..6，按仓表单向补角） |
| `T` | 只打印当前 T 位置 |
| `ZT` / `ZB` | Z 到最上 1200 / 最低工作位 2150 us（不再有 ZM） |
| `Z2000` | Z 到指定脉宽（1200..2150） |
| `Z` | 只打印当前 Z 脉宽 |
| `GO` / `GC` | 夹爪张开 / 闭合（一次到位，标定模式也有效） |
| `G1650` | 夹爪到指定脉宽（1400..1750，标定模式 800..2200） |
| `G` | 只打印当前夹爪脉宽 |
| `R` | 用上次的 X 和仓号跑一遍全流程 |
| `R1200` | 设定 X=1200，用上次仓号跑 |
| `R1200,3` | 设定 X=1200、仓 3，然后跑全流程 |
| `RA` | 中止正在跑的全流程 |

`R` 的顺序（每步都会从 USART3 打印）：

```
R1200,3
SEQ START X=1200
SEQ START BIN=3
SEQ MOVE XY + T        ← X 和转盘同时起步
SEQ GRIP OPEN          ← X 到位后开爪
SEQ Z DOWN US=2150     ← 350 ms 后 Z 到最低工作位
SEQ GRIP CLOSE
SEQ Z UP US=1200       ← 闭爪后 Z 回最上面
SEQ WAIT TURRET
SEQ DONE (drop move belongs to the auto flow)
```

典型会话：

```text
py -3 tools/pc/serial_tool.py -p COM6 console
> T3          # 先让转盘到 3 号仓
> X1200       # 手动把 X 对到物块
> GO          # 开爪
> ZB          # 下降到最下
> GC          # 闭爪
> ZT          # 回升到最上
> R1200,3     # 或者一条命令串起整个流程
> RA          # 中途终止
> s           # 看 STATE / X_POS / Z_US / TEST_X / TEST_BIN / SEQ_BUSY
```

注意事项：

- **需要先回零**：X/T 位置不可信时命令会回 `REJECT NOT HOMED`，等上电回零完成
  （或先发视觉协议的 `home`）再用。
- **互锁**：任何指令都要求“安全 + 无自动任务”；序列运行期间视觉任务会被回 `BUSY`，
  直接运动指令会回 `REJECT SEQ BUSY`，用 `RA` 退出。
- 全流程**不包含“移动到投放口 + 投放开爪”**，那属于正式自动流程（队友负责）。
- `Z` 用的是 `ZSERVO_PULSE_BOTTOM_US`（2150，最低工作位）和
  `ZSERVO_PULSE_TRAVEL_US`（1200，最上面）；不再使用 1850 的中间位。

## 1.2 USART3 小写单词指令（全流程手动测试，2026-11-05）

大写指令要记缩写、还要“发送新行”；全流程联调时更推荐直接发小写单词。
同一个 USART3、115200 8N1，**不勾“发送新行”也能用**：最后一个字符后
100 ms 没有新字符就自动执行；勾了“发送新行”则收到 CR/LF 立即执行。
上电后固件先发一次 `ok`，之后每次发 `ok` 都会回一次 `ok`。

| 发送 | 动作 |
|---|---|
| `ok` | 回显 `ok`（上电自动发一次；任何状态都回，可当心跳） |
| `zup` | Z 轴到最上面 `ZSERVO_PULSE_TRAVEL_US`（1200 us） |
| `zdown` | Z 轴下降到最低工作位 `ZSERVO_PULSE_GRASP_US`（2150 us） |
| `ztop` / `zbottom` | Z 到 1200 / 2150 us（最上 / 最低工作位；无 zmid） |
| `xmax` / `xmin` | X 到软限位上限 `SORT_X_MAX_STEP`（5400）/ 零点 `0` |
| `ymax` / `ymin` | Y 到软限位上限 `SORT_Y_MAX_STEP`（6800）/ 零点 `0` |
| `tjog` / `tjog-` | 转盘相对 +2000 / -2000 微步（约 ±43°；不要求先回零，查 T 驱动器/接线） |
| `zero` | 恢复零点：READY 时重新回零；ERROR/急停松开后先清故障再回零 |
| `zk` | 机械爪张开（同 `GO`） |
| `gb` | 机械爪闭合（同 `GC`） |
| `stop` | 软停止（进 ERROR，需要 `reset` 或 `zero`） |
| `reset` | 清急停锁存并重新回零（急停仍按住时回 REJECT） |
| `status` | 打印状态/位置/脉宽（同 `s`） |
| `help` | 打印帮助（同 `h`） |
| `run` | 用上次的 X/仓号跑一遍 `R` 序列；`abort` 中止 |
| `x1200` / `y500` / `t3` / `z2200` / `g1650` | 小写数字指令，等价于 `X1200` / `Y500` / `T3` / `Z2200` / `G1650` |

单字符 `0~3`、`A~D`、`s`、`h`、`x` 仍然可用（`s/h/x` 会延迟约 100 ms 执行）。
所有动作和视觉协议共用“安全 + 无任务”门控；急停或自动任务执行期间只回
REJECT，`ok` 不受影响。

一次完整的手动取放流程示例：

```text
ok            -> ok
zero          -> ZERO HOMING START        （等回零完成，s 看 STATE=5）
xmax          -> X -> 5400                （X 到取料点附近，或直接 x1200）
zk            -> G OPEN US=1730
zdown         -> Z DOWN(GRASP) US=2150
gb            -> G CLOSE US=1420
zup           -> Z UP(TRAVEL) US=1200
xmin          -> X -> 0
```

> **当前台架配置**：`SORT_XY_BENCH_MODE=1`、`SORT_XY_BENCH_KEEP_TURRET=1`，
> 上电时 **X/Y 和 T 同时开始回零**，三轴全部完成后才进 READY。串口应依次看到
> `XY_HOME OK ...`、`T_HOME START`、`T_HOME_PIN=`、`XY+T HOME OK -> READY`。
>
> - 如果没接 PD12 的 T_HOME 霍尔/磁铁，T 会等满 20 s 后进 ERROR；
>   这时把 `SORT_XY_BENCH_KEEP_TURRET` 改回 **0**（只回 XY）。
> - 想恢复正式流程的 X→Y→T 顺序和夹爪/Z 四键语义，把
>   `SORT_XY_BENCH_MODE` 改回 **0**。
> - T 位置无效时，`T3` / `R` 会回 `REJECT NOT HOMED`，视觉任务会被
>   `SORT_ERR_NOT_READY` 拒绝；可以先用 `tjog` / `tjog-` 单独确认 T 驱动器。
> - 自动流程还需要把 `Drivers/servo_z.h` 的 `ZSERVO_CALIB_MODE` 改成 0
>   （否则状态机不控制 Z 高度）。

## 2. USART2 协议帧

帧格式：`AA 55 | version(1) | command | length | payload | CRC16(低字节在前)`，
CRC16 为 Modbus 反射算法（0xA001），覆盖 `version/command/length/payload`，
实现见 `Service/protocol.c`。

```powershell
py -3 tools/pc/serial_tool.py -p COM5 status
py -3 tools/pc/serial_tool.py -p COM5 home
py -3 tools/pc/serial_tool.py -p COM5 stop
py -3 tools/pc/serial_tool.py -p COM5 reset
py -3 tools/pc/serial_tool.py -p COM5 mission --id 1 --box 3 --x 0 --y 0
py -3 tools/pc/serial_tool.py -p COM5 monitor
```

- 2026-11-08 起 `SORT_VISION_COORD_MAP=1`：`--x/--y` 是**视觉坐标（默认 mm，相对视觉零点）**，
  不是绝对微步；视觉零点对应机械 X2200/Y3000，900 微步 = 2 cm；
- 投放口固定为 X/Y 限位零点，不再由 STM32 保存任意投放坐标；
- 回零/任务/停止/复位分别应答 `0x86/0x80/0x82/0x85`，出错是 `0x83` 并带错误码；
- 不接硬件也能先跑帧自检：

```powershell
py -3 tools/pc/serial_tool.py --selftest
```

依赖 `pip install pyserial`（只有真正开串口时才需要，`--selftest` 不需要）。

> 视觉模块（OpenART mini）要发同样的帧，组帧代码、像素→微步换算和重试策略见
> 根目录的 `VISION_PROTOCOL.md`。

## 3. 零代码备选：杜邦线当按键

板载按键是 **上拉输入、低电平按下**，所以用一根杜邦线把 PD0/PD1/PD2/PD3
任意一脚短接到 GND，就等价于按下对应按键（松手即释放）：

- 短接一下 = 短按；按住 1 s 以上 = 长按（KEY0 会存投放口 X）；
- 千万不要碰 **PD13（急停）**，那一路短接会让系统进入急停并锁存故障；
- 这个方法只能一次按一个键，也做不了“精确按住 1.5 s”，复杂场景还是用串口控制台。

## 4. 关闭方式（正式比赛）

`App/sorting_config.h`：

```c
#define SORT_ENABLE_BENCH_KEYS   0   /* 关闭所有手动按键入口 */
#define SORT_DEBUG_CONSOLE       0   /* 关闭 USART3 虚拟按键/状态打印 */
#define SORT_DEBUG_UART          0   /* 关闭 USART3 调试打印 */
```

`SORT_DEBUG_CONSOLE=0` 时整个控制台不参与编译，USART3 回到“只发送调试信息”或完全空闲。
控制台回显复用 `Service/debug_uart.c`，所以 `SORT_DEBUG_UART=0` 也能用；
但 `SORT_ENABLE_BENCH_KEYS=0` 时没有按键扫描，虚拟按键不生效，只剩 `s` 能看状态。
