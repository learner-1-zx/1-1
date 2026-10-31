# 视觉坐标上传协议（OpenART mini → STM32F407）

> 设备：逐飞 **OpenART mini**（NXP 主控，MicroPython），实物见
> `设备属性or数据图片/摄像头模块1 (1).jpg`，板上 UART2 排针丝印为 `RX TX GND 5V`。
>
> **一句话结论**：视觉模块用 UART2 排针接到 STM32 的 **USART2（PA2/PA3）**，
> 按版本 1 的二进制帧发一条 **17 字节任务载荷**（里面放 `pick_x_step` /
> `pick_y_step` / `box_id`），STM32 回 `ACK`，动作完成后回 `DONE`。
> 坐标必须是 **相对 X 回零零点的绝对微步**，不是像素、也不是 mm
> （mm 换算在视觉端做，见第 5 节）。

---

## 1. 硬件接线

| OpenART mini（UART2 排针） | 接到 STM32 | 说明 |
| --- | --- | --- |
| `TX` | **PA3 / USART2_RX** | 视觉发 → 单片机收 |
| `RX` | **PA2 / USART2_TX** | 单片机发 → 视觉收（ACK/DONE） |
| `GND` | **GND** | **必须共地**，否则握手随机失败 |
| `5V` | 建议不接 | 见下方供电说明 |

- 电平均为 **3.3 V TTL**，可以直接对接；**不能**接 RS232（±12 V）。
- 波特率 **115200、8 数据位、无校验、1 停止位、无流控**，两端必须一致
  （`huart2.Init.BaudRate = 115200`，见 `Core/Src/usart.c`）。
- 供电二选一：OpenART 单独用 USB 供电，或从板上 5V 供电（**不要再插 USB**），
  两种方式都要和 STM32 共地。舵机/电机大电流不要走这根地线。
- **USART2 就是给摄像头的**，调试期不要同时把 USB-TTL 接在 PA2/PA3 上
  （两个 TX 顶线）。要用电脑代替视觉，先把摄像头拔掉。

> 方向记法：**TX 接对方的 RX**。接反的现象是“单片机收不到任何字节”，
> 用 `s` 控制台或示波器看 PA3 有没有波形能立刻定位。

---

## 2. 数据流：谁负责什么

```
OpenART mini（视觉）                          STM32F407（电控）
┌──────────────────────────┐               ┌────────────────────────────────┐
│ 已训练模型 → 物块中心像素  │               │ Protocol_Process() 收帧        │
│   (u, v)                 │               │   → CRC16 校验                 │
│ 像素 → 机械微步（标定矩阵）│  17 字节任务帧 │   → SortingTask_Submit()      │
│ 物块类别 → box_id(1..6)   │ ────────────→ │      READY/安全/位置可信/      │
│ 组帧 AA55+CRC16           │               │      软限位/box_id 校验        │
│                          │ ←──────────── │   → 回 0x80 ACK / 0x81 BUSY /  │
│ 等 ACK → 等 DONE          │  ACK / BUSY   │      0x83 ERROR                │
│                          │ ←──────────── │   → 状态机执行 XY→Z→夹爪→T→投放│
└──────────────────────────┘   DONE 0x82   └────────────────────────────────┘
```

分工要点：

- **视觉只发“取料点”**（物块在哪、去哪个仓）。**投放口坐标由 STM32 自己保存**
  （`SORT_DROP_X_STEP`，可用 KEY0 长按标定），不要从视觉发 drop 坐标。
- 视觉**不需要**知道 X 是否回零、软限位多少：越界会被拒（ERROR 1），
  未 READY 会被拒（BUSY），按第 7 节重试即可。
- 整个流程是“一问一答”：**一条任务发出去，必须等到 DONE 才能发下一条**
  （任务执行期间 `SortingTask_Submit()` 直接返回 0）。

---

## 3. 帧格式（协议版本 1，USART2）

```
AA 55 | version=0x01 | command | length | payload(length 字节) | CRC16(低字节在前)
```

- `CRC16` 覆盖 **version + command + length + payload**（**不含** `AA 55`），
  算法是 **CRC-16/MODBUS**：初值 `0xFFFF`，反射多项式 `0xA001`，
  即 `crc = (crc >> 1) ^ 0xA001`（当 `crc & 1`）。
  与固件 `Service/protocol.c` 的 `Protocol_Crc16()` 完全一致。
- 单帧最大 payload 32 字节（固件 `PROTOCOL_MAX_PAYLOAD`）。

### 3.1 任务命令 `0x10`：payload 固定 17 个有效字节

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | u16 | `task_id` | 任务号，视觉自增，原样回传，便于匹配 |
| 2 | u8 | `class_id` | 物块类别（视觉模型输出，STM32 只透传） |
| 3 | u8 | `box_id` | 目标仓 **1..6**（`SORT_BOX_ID_ONE_BASED=1`） |
| 4 | i32 | `pick_x_step` | 取料点 X **绝对微步**（相对 X 回零零点） |
| 8 | i32 | `pick_y_step` | 取料点 Y 绝对微步（`SORT_ENABLE_Y_AXIS=0` 时被忽略） |
| 12 | i32 | `drop_step` | 备用投放步数；`box_id` 合法时不用，填 0 |
| 16 | u8 | `state` | 任务状态标记，填 1 |

**全部小端**。`length` 填 17 或 20 都可以（结构体在 ARM 上有尾部 padding），
固件只要求 `length >= 17`，多余字节忽略。

### 3.2 其它命令（调试/上位机用）

| command | 名称 | payload | 用途 |
| --- | --- | --- | --- |
| `0x01` | STATUS | 无 | 查询状态，回 `0x84` |
| `0x02` | HOME | 无 | 重新回零，回 `0x86` |
| `0x13` | STOP | 无 | 软停止，进 ERROR，需 `0x30` 复位 |
| `0x30` | RESET_FAULT | 无 | 故障复位（会重新回零） |

### 3.3 事件（STM32 → 视觉）

| 事件 | 载荷 | 含义 |
| --- | --- | --- |
| `0x80` ACK | `task_id`(2) | 任务已接收，开始执行 |
| `0x81` BUSY | `task_id`(2) | 忙/未 READY，任务**未**接收 |
| `0x82` DONE | `task_id`(2) | 任务完成 |
| `0x83` ERROR | `task_id`(2) + `error`(1) | 出错，错误码见 `App/sorting_task.h` |
| `0x84` STATUS | `state` + `error` | 状态查询应答 |
| `0x85` / `0x86` | 0 / 0 | 复位、回零请求已受理 |

### 3.4 一条真实存在的示例帧（可直接对着核对）

发：`task_id=1, class_id=2, box_id=3, pick_x_step=1200, pick_y=0, drop=0, state=1`

```
AA 55 01 10 11 | 01 00 02 03 | B0 04 00 00 | 00 00 00 00 | 00 00 00 00 | 01 | D6 9C
↑帧头  ↑版本/命令/长度  task_id  class box   pick_x=0x000004B0=1200   pick_y=0     drop=0    state  CRC16
```

`D6 9C` 是 CRC 低字节在前（实际值 `0x9CD6`）。
另外两条常用帧：

```
状态查询 : AA 55 01 01 00 21 90
请求回零 : AA 55 01 02 00 21 60
```

> 这三条帧由 `tools/pc/serial_tool.py` 用同一算法生成，
> 可先用 `py -3 tools/pc/serial_tool.py --selftest` 验证电脑侧实现，
> 再拿视觉模块发同样的字节，就能把“协议问题”和“机械问题”分开。

---

## 4. 各字段怎么填

### 4.1 `task_id`

视觉每发一个新任务自增一次（`0..65535` 循环），STM32 原样回传。
用途：区分“这条 DONE 是哪一条任务”，防止超时重发后把旧 DONE 当成新任务的。

### 4.2 `box_id`：物块类别 → 仓号（**1..6**）

固件默认 `SORT_BOX_ID_ONE_BASED = 1`，即 1..6；如果视觉习惯用 0..5，
必须把 `App/sorting_config.h` 的这个宏改成 0，**两端必须一致**。

视觉端维护一张映射表即可（类别号来自你的模型）：

```python
CLASS_TO_BOX = {1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6}   # 按比赛规则填
```

`box_id` 非法（不在 1..6）时，固件只有在 `SORT_T_ALLOW_DROP_STEP_FALLBACK=1`
且 `drop_step` 在 T 轴软限位内才接受；否则直接回 `0x83 error=1`（坐标非法）。
**建议永远发合法 box_id，drop_step 填 0。**

### 4.3 `pick_x_step` / `pick_y_step`：**绝对微步**，不是像素、不是 mm

- 零点 = X 轴回零开关（PD6）触发后的位置，回零后 `X_POS=0`；
- 软限位：`SORT_X_MIN_STEP=0`、`SORT_X_MAX_STEP=SORT_X_TRAVEL_STEPS=20000`
  （`SORT_X_HOME_DIR=-1` 时是 `0 ~ +行程`），越界 → `0x83 error=1`；
  20000 只是保守占位（≈250 mm），实测行程后要收紧，宁小勿大；
- `SORT_ENABLE_Y_AXIS=0`（当前）时 Y 驱动器不使能，`pick_y_step`
  只解析不校验不使用——**照发 0 即可**，等 Y 接好再改 1。

### 4.4 `state`

任务状态标记，固定填 `1`。固件当前只透传，不参与判断，保留给以后
“重发/取消”等语义。

---

## 5. 像素 → 微步：换算放在视觉端做

STM32 只认微步，所以视觉必须先把“模型框中心 `(u, v)`”换算成机械坐标。
X 轴传动链已固定：42BYGH47（200 整步/圈）× A4988 1/16 细分 = 3200 微步/圈，
同步轮 20T、齿距 2 mm → 每圈 40 mm：

```
80 微步/mm   (3200 / 40)
```

### 5.1 推荐做法：3 点仿射标定（一条命令就能算）

STM32 不参与标定，只要你在视觉端拟合出下面这组系数：

```
X_step = a * u + b * v + c
Y_step = d * u + e * v + f      # Y 轴启用后才需要
```

标定步骤：

1. 用 `py -3 tools/pc/serial_tool.py -p COM5 status` 或 USART3 控制台的 `s`
   确认 `X_POS=0` 且 `STATE=5(READY)`；必要时发 `home` 回零；
2. 把标定物摆在工位上，记录 **3 个不同位置**：
   - 视觉读到的像素 `(u, v)`；
   - 该点的机械 X 微步（用 KEY2/KEY3 点动到“夹爪正对物块”后按 `s` 读 `X_POS`，
     或直接按 `微步 = mm × 80` 算）；
3. 解这个 3×3 线性方程组（`numpy.linalg.solve` 或 `micropython-ulab`）：

   ```
   | u1 v1 1 |   | a |   | X1 |
   | u2 v2 1 | * | b | = | X2 |
   | u3 v3 1 |   | c |   | X3 |
   ```

4. 把 `a, b, c` 写成常量烧进视觉脚本。表面对准得很正时（相机轴与 X 轴平行），
   可以简化成 `X_step = (u - u0) * 80 / mm_per_px`，只需要一个基准点。

### 5.2 别忘了“相机与夹爪不同轴”的固定偏移

相机装在夹爪侧面时，看到的位置不等于夹爪要对准的位置：

```
pick_x_step = a*u + b*v + c + CAMERA_TO_GRIPPER_STEPS
```

`CAMERA_TO_GRIPPER_STEPS` 用一次实测确定：让视觉报同一个物块，
手动把夹爪对到物块中心，读 `X_POS`，和视觉算出的值相减即得。
标定完这几条经验值，整场都不用再改。

---

## 6. OpenART mini 端代码（MicroPython，可直接用）

**现成文件**：`tools/vision/openart_send.py`（拷到 OpenART 的 U 盘即可，
`if __name__ == "__main__"` 会跑 `demo()`，每 2 s 发一条固定坐标任务做链路自检）。
下面把同一份代码贴出来，方便阅读。只依赖 `machine.UART` / `struct` / `time`。

```python
# OpenART mini: 视觉 -> STM32 分拣任务
import time
import struct
from machine import UART

uart = UART(2, baudrate=115200)          # 板上 UART2 排针 TX/RX/GND

PROTO_VER   = 1
CMD_MISSION = 0x10
EV_ACK, EV_BUSY, EV_DONE, EV_ERROR = 0x80, 0x81, 0x82, 0x83

_rx = b""                                 # 接收缓存：可能只读到半帧

def crc16(data):
    """CRC-16/MODBUS，与固件 Protocol_Crc16() 完全一致"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

def build_frame(cmd, payload=b""):
    body = bytes([PROTO_VER, cmd, len(payload)]) + payload
    return b"\xAA\x55" + body + struct.pack("<H", crc16(body))

def poll_event(timeout_ms):
    """读一个校验通过的事件帧；返回 (cmd, payload)，超时返回 None。"""
    global _rx
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        chunk = uart.read()               # 无数据返回 None
        if chunk:
            _rx += chunk
        while True:
            i = _rx.find(b"\xAA\x55")
            if i < 0:
                _rx = b""                 # 没有帧头，整段丢弃
                break
            if len(_rx) < i + 5:
                _rx = _rx[i:]             # 帧头不完整，等后续字节
                break
            n = _rx[i + 4]                # length
            total = i + 7 + n
            if len(_rx) < total:
                _rx = _rx[i:]             # 帧体不完整，等后续字节
                break
            body = _rx[i + 2:i + 5 + n]
            crc_rx = struct.unpack("<H", _rx[i + 5 + n:total])[0]
            if crc_rx == crc16(body):
                cmd = _rx[i + 3]
                payload = _rx[i + 5:i + 5 + n]
                _rx = _rx[total:]
                return cmd, payload
            _rx = _rx[i + 1:]             # CRC 错：跳过这个帧头继续找
        time.sleep_ms(2)
    return None
```

```python
def send_mission(task_id, class_id, box_id, x_step, y_step=0, drop_step=0, state=1):
    """发一条任务，返回 'ACK' / 'BUSY' / 'ERRn' / 'NO_ACK'。"""
    payload = struct.pack("<HBBiiiB",
                          task_id & 0xFFFF, class_id & 0xFF, box_id & 0xFF,
                          int(x_step), int(y_step), int(drop_step), state & 0xFF)
    uart.write(build_frame(CMD_MISSION, payload))
    ev = poll_event(800)                  # 等 ACK；800 ms 足够跑完一次调度
    if ev is None:
        return "NO_ACK"
    cmd, data = ev
    if cmd == EV_ACK:
        return "ACK"
    if cmd == EV_BUSY:
        return "BUSY"
    if cmd == EV_ERROR:
        return "ERR%d" % data[2]          # data = task_id(2) + error(1)
    return "EVENT_%02X" % cmd

def wait_done(task_id, timeout_ms=30000):
    """等这一条任务的 DONE / ERROR；返回 'DONE' / 'ERRn' / 'TIMEOUT'。"""
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        ev = poll_event(200)
        if ev is None:
            continue
        cmd, data = ev
        if cmd == EV_DONE and struct.unpack("<H", data)[0] == task_id:
            return "DONE"
        if cmd == EV_ERROR:
            return "ERR%d" % data[2]
    return "TIMEOUT"
```

### 6.1 主循环怎么用（配合你的模型）

```python
# 3 点仿射标定得到的常量（下面是示例，必须自己标定后替换）
A_COEF, B_COEF, C_COEF = 0.0, -1.28, 3200.0
CAMERA_TO_GRIPPER_STEPS = 0
CLASS_TO_BOX = {1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6}   # 按比赛规则填

def pixel_to_step_x(u, v):
    return int(A_COEF * u + B_COEF * v + C_COEF) + CAMERA_TO_GRIPPER_STEPS

task_id = 0
while True:
    blocks = find_blocks()                # 你的模型：至少要有 cx, cy, class_id
    if not blocks:
        time.sleep_ms(200)
        continue

    b = blocks[0]
    x_step = pixel_to_step_x(b.cx(), b.cy())
    if not (0 <= x_step <= 20000):        # 和固件软限位一致，越界直接丢弃
        time.sleep_ms(100)
        continue

    task_id = (task_id + 1) & 0xFFFF
    box_id = CLASS_TO_BOX.get(b.code(), 1)
    result = send_mission(task_id, b.code(), box_id, x_step, 0)
    if result == "ACK":
        result = wait_done(task_id)
    print("task", task_id, "x", x_step, "box", box_id, "->", result)
    time.sleep_ms(300)
```

> **注意**：`UART(2)` 是板上丝印 `UART2` 的那个；如果 OpenART 固件里编号不同，
> 按逐飞例程里的 UART 编号改（有的版本是 `UART(3)`）。
> 不确定时先在 OpenART 串口终端里 `uart.write(b"hi")`，用 USB-TTL 接它的 TX 确认。

---

## 7. 应答与重试策略（很重要，决定比赛稳不稳）

| 收到 | 含义 | 视觉该做什么 |
| --- | --- | --- |
| `0x80` ACK | 已接收，开始动 | 调 `wait_done(task_id)`，**不要再发新任务** |
| `0x82` DONE | 完成 | 可以发下一条（换下一块） |
| `0x81` BUSY | 未 READY 或上一条没完 | 等 300~500 ms 后**用同一个 task_id 重发**，最多 3 次；仍忙就先发 `STATUS` 查状态 |
| `0x83` error=1 | 坐标非法（软限位/box_id） | **不要原样重发**，检查 `x_step` 是否 0..20000、box 是否 1..6 |
| `0x83` error=5 | 回零失败 | 检查零点开关/磁铁，需人工处理 |
| `0x83` error=6 | 急停被按下 | 检查急停按钮，复位后重新回零 |
| `0x83` error=9 | 位置不可信（未回零） | 发 `HOME`，等 `0x86`，再等 READY |
| 无任何应答 | 链路问题 | 查 TX/RX 是否交叉、波特率、共地；100 ms 后重发，最多 3 次 |

**不要“超时就当失败、直接换下一块”**：STM32 可能还在动，换块会导致重复抓取或撞块。
正确顺序永远是：`ACK → DONE/ERROR` 或 `STATUS` 查明状态。

---

## 8. 联调顺序（按这个顺序能少走很多弯路）

1. **视觉单独跑通**：不接 STM32，在 OpenART 串口终端打印
   `x_step / box_id / 类别`，确认数值落在 `0..20000`、box 在 `1..6`。
2. **拔掉摄像头**，用电脑 `tools/pc/serial_tool.py`（走 PA2/PA3）发
   `status` → 应回 `STATUS state=5(READY)`；再发
   `mission --id 1 --box 3 --x 1200` → 应回 `ACK task_id=1`，动作后回 `DONE`。
   这一步证明“协议 + 机械”都通，后面出问题就只在视觉侧。
3. **接上摄像头**（TX→PA3、RX→PA2、GND），先发一条任务，看是否 `ACK`。
   同时可以用另一个 USB-TTL 的 **RX 并接 PA2** 监听 STM32 应答
   （只接 RX 和 GND，不要接 TX）。
4. **连续跑 20 条**：不同 box、不同 x_step，确认不累积偏仓、不丢步。
5. 再切 `ZSERVO_CALIB_MODE=0` 跑完整自动流程（见第 9 节）。

---

## 9. 常见故障对照表

| 现象 | 最可能原因 | 处理 |
| --- | --- | --- |
| 发完没有任何应答，USART3 也没打印 | TX/RX 接反、没共地、相机 TX 没接到 PA3、波特率不是 115200 | 逐一量线；“TX 接对方 RX” |
| 偶尔回、偶尔丢 | 没共地 / 杜邦线过长 / 电机干扰 | 短线、共地、远离电机线 |
| 有 ACK 但机构不动 | `ZSERVO_CALIB_MODE=1`（Z 完全不动）、`SORT_ENABLE_Y_AXIS=0`（Y 不动是正常的）、X 已在该位置 | 跑自动流程前必须把 Z 标定模式改 0 |
| 回 BUSY | 上电还在 X→Y→T 回零，或上一条任务没做完 | 先 `status` 看到 `state=5` 再发；重试间隔 300~500 ms |
| 回 ERROR 1 | `pick_x_step` 越界（负数或 >20000）、`box_id` 越界 | 检查像素→微步换算和零点方向；软限位见 `sorting_config.h` |
| 回 ERROR 5 | 回零失败（限位开关/磁铁/方向） | 查 PD6 开关和 `SORT_X_HOME_DIR` |
| 回 ERROR 6 | 急停按下 | 松开 PD13 后发 `0x30` 复位 |
| 回 ERROR 9 | 位置不可信（未回零） | 发 `0x02` HOME，等 READY |
| 动作一半 ERROR 2/3 | XY / T 超时（速度过快、卡滞、丢步） | 降速、查机械 |
| 帧被静默丢弃（无任何应答） | CRC 错、`length` 与实际 payload 不符、payload <17 | 用 `tools/pc/serial_tool.py` 生成的标准帧对照 hex；CRC **不含** `AA 55` |

> 固件对“帧头不对 / CRC 错 / 版本不是 1”都是**静默丢弃**，不会回错误帧——
> 所以“完全没反应”优先怀疑链路和 CRC，不要先怀疑状态机。

---

## 10. 比赛前检查清单

固件（`App/sorting_config.h` / `Drivers/servo_z.h`）：

- [ ] `ZSERVO_CALIB_MODE = 0` —— **不改的话 Z 轴完全不动**（当前还是 1）
- [ ] `SORT_ENABLE_BENCH_KEYS = 0`、`SORT_DEBUG_CONSOLE = 0`、`SORT_DEBUG_UART = 0`
- [ ] `SORT_X_TRAVEL_STEPS` 按实测行程收紧（越界会真顶机械端）
- [ ] `SORT_DROP_X_STEP` 写回用 KEY0 长按标定出来的 `DROP_X`（RAM 值掉电丢失）
- [ ] `SORT_BOX_ID_ONE_BASED` 与视觉发送端一致
- [ ] `SORT_ENABLE_Y_AXIS` 与 Y 轴实际接线一致
- [ ] 夹爪 / Z 脉宽是正式值（不是标定临界值）

视觉端：

- [ ] `x_step` 发送前做 `0..20000` 范围保护
- [ ] `task_id` 每发一条自增
- [ ] 调参用 `ACK → DONE` 一问一答，未收到 DONE 不换下一块
- [ ] 供电稳定（单独 USB 或 5V，别和舵机抢电）

---

## 11. 相关文件

- 固件收帧与分发：`Service/protocol.c`（`Protocol_Process()` / `Protocol_HandleFrame()`）
- 任务校验与执行：`App/sorting_task.c`（`SortingTask_Submit()`）、`App/sorting_task.h`（错误码）
- 参数与软限位：`App/sorting_config.h`
- 电脑侧对照工具：`tools/pc/serial_tool.py`、`tools/pc/README.md`
- 串口/引脚表：`IO.md` 2.4
