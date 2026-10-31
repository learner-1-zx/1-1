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
| `s` | 打印状态、轴位置、Z/夹爪脉宽、四个按键原始电平 |
| `x` | 立即释放虚拟按键 |
| `h` / `?` | 帮助 |

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

## 1.1 USART3 运动指令（不开视觉也能跑全流程）

同一个 USART3、同一套串口设置，**大写字母开头**的命令直接驱动电机/舵机。
运动指令是“一行一条”，**必须以回车/换行结束**（串口助手勾选“发送新行”，
本目录脚本会自动补 `\n`）；单字符按键 `0~3`/`A~D`/`s`/`h`/`x` 仍然即时生效、不用回车。
全部走正式驱动，不经过 `sorting_task.c` 的状态机，所以队友改自动流程时这里不受影响。

| 命令 | 作用 |
|---|---|
| `X1200` | X 轴移动到绝对微步 1200（范围 `0..SORT_X_TRAVEL_STEPS`） |
| `X` | 只打印当前 X 位置 |
| `Y500` | Y 轴移动（`SORT_ENABLE_Y_AXIS=0` 时回 `DISABLED`） |
| `T3` | 转盘转到 3 号仓（1..6，按仓表单向补角） |
| `T` | 只打印当前 T 位置 |
| `ZT` / `ZB` / `ZM` | Z 到最上 1200 / 最下 2500 / 中间 1850 us |
| `Z2200` | Z 到指定脉宽（1200..2500） |
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
SEQ Z DOWN US=2500     ← 350 ms 后 Z 到最下
SEQ GRIP CLOSE
SEQ Z UP US=1220       ← 闭爪后 Z 回运输高度
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
- `Z` 用的是 `ZSERVO_PULSE_BOTTOM_US`（2500，机械端点）和
  `ZSERVO_PULSE_TRAVEL_US`（1220）；长时间停在 2500 会顶住齿条，测试短停即可。

## 2. USART2 协议帧

帧格式：`AA 55 | version(1) | command | length | payload | CRC16(低字节在前)`，
CRC16 为 Modbus 反射算法（0xA001），覆盖 `version/command/length/payload`，
实现见 `Service/protocol.c`。

```powershell
py -3 tools/pc/serial_tool.py -p COM5 status
py -3 tools/pc/serial_tool.py -p COM5 home
py -3 tools/pc/serial_tool.py -p COM5 stop
py -3 tools/pc/serial_tool.py -p COM5 reset
py -3 tools/pc/serial_tool.py -p COM5 mission --id 1 --box 3 --x 1200 --y 0
py -3 tools/pc/serial_tool.py -p COM5 monitor
```

- `mission` 的取料坐标是绝对步数（X 回零后零点为 0），投放口由 STM32 自己保存；
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
