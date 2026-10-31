# 视觉端脚本（OpenART mini）

`openart_send.py` 是给 **逐飞 OpenART mini（MicroPython）** 用的发送脚本：
把视觉算出来的取料坐标按协议发给 STM32（USART2），并等 `ACK / DONE`。

完整协议说明（帧格式、坐标换算、重试策略、联调顺序）见根目录
[`VISION_PROTOCOL.md`](../../VISION_PROTOCOL.md)。

## 接线

| OpenART mini UART2 排针 | 接到 STM32 |
| --- | --- |
| `TX` | `PA3`（USART2_RX） |
| `RX` | `PA2`（USART2_TX） |
| `GND` | `GND`（必须共地） |

115200 8N1、3.3 V TTL。`5V` 建议不接（OpenART 单独 USB 供电），
不要和 USB 同时供电。

## 使用

1. 把 `openart_send.py` 拷到 OpenART 的 U 盘（会以 `main.py` 或手动运行）；
2. 先只跑 `demo()`：每 2 s 发一条固定坐标（x=1200、box=3）的任务，
   看到串口打印 `-> DONE` 就说明“接线 + 协议 + 机械”整条链路通了；
3. 再按 `VISION_PROTOCOL.md` 第 5 节标定 `A_COEF/B_COEF/C_COEF` 和
   `CAMERA_TO_GRIPPER_STEPS`，填好 `CLASS_TO_BOX`，把 `run(find_blocks)`
   接到你的模型输出上。

## 注意

- `UART(2)` 对应板上丝印 `UART2`；如果你的固件里编号不同（有的版本是 `UART(3)`），
  按逐飞例程改。
- 坐标是 **绝对微步**（相对 X 回零零点），范围 `0..20000`；越界固件会回
  `0x83 error=1`，脚本会先做 `SKIP_RANGE` 保护。
- 未收到 `DONE` 不要发下一条：任务执行期间固件会回 `BUSY`。
- 电脑上想先验证协议，用 `tools/pc/serial_tool.py`（此时要先拔掉摄像头）。
