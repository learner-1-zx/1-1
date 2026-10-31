# -*- coding: utf-8 -*-
"""OpenART mini（MicroPython）：把视觉坐标作为分拣任务发给 STM32F407。

硬件：板上 UART2 排针 TX/RX/GND，TX→PA3(USART2_RX)、RX→PA2(USART2_TX)、共地。
协议：AA 55 | ver=1 | cmd | len | payload | CRC16(低字节在前)，
      CRC 覆盖 ver+cmd+len+payload，算法 CRC-16/MODBUS（同固件 Protocol_Crc16）。
任务：cmd=0x10，17 字节 payload（小端）：
      u16 task_id | u8 class_id | u8 box_id | i32 pick_x_step |
      i32 pick_y_step | i32 drop_step | u8 state
应答：0x80 ACK / 0x81 BUSY / 0x82 DONE / 0x83 ERROR(task_id+error)。

说明文档：VISION_PROTOCOL.md（像素→微步标定、重试策略、联调顺序）。
把本文件拷到 OpenART 的 U 盘，按需改 CLASS_TO_BOX / 标定常量即可。
"""

import time
import struct
from machine import UART

uart = UART(2, baudrate=115200)          # 板上丝印 UART2 的排针

PROTO_VER   = 1
CMD_MISSION = 0x10
EV_ACK, EV_BUSY, EV_DONE, EV_ERROR = 0x80, 0x81, 0x82, 0x83

PICK_X_MIN, PICK_X_MAX = 0, 20000        # 与 SORT_X_MIN/MAX_STEP 保持一致

_rx = b""                                 # 接收缓存：可能只读到半帧


def crc16(data):
    """CRC-16/MODBUS，与固件 Protocol_Crc16() 完全一致。"""
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
        chunk = uart.read()               # 无数据时返回 None
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


def send_mission(task_id, class_id, box_id, x_step, y_step=0, drop_step=0, state=1):
    """发一条任务；返回 'ACK' / 'BUSY' / 'ERRn' / 'NO_ACK'。"""
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
        return "ERR%d" % data[2]
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


def send_and_wait(task_id, class_id, box_id, x_step, y_step=0):
    """完整跑一条任务：发送 → 等 ACK → 等 DONE。返回最终结果字符串。"""
    if not (PICK_X_MIN <= int(x_step) <= PICK_X_MAX):
        return "SKIP_RANGE"               # 越界不发，避免回 ERROR 1
    result = send_mission(task_id, class_id, box_id, x_step, y_step)
    if result == "ACK":
        result = wait_done(task_id)
    return result


# ---------------- 以下按项目实际情况修改 ----------------

# 3 点仿射标定得到的常量（示例值，必须自己标定后替换）
A_COEF, B_COEF, C_COEF = 0.0, -1.28, 3200.0
CAMERA_TO_GRIPPER_STEPS = 0
CLASS_TO_BOX = {1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6}   # 按比赛规则填


def pixel_to_step_x(u, v):
    return int(A_COEF * u + B_COEF * v + C_COEF) + CAMERA_TO_GRIPPER_STEPS


def demo():
    """不接模型时的一键自检：每 2 s 发一条固定坐标的任务。

    预期：串口收到 ACK → 机械抓取并投放 → 收到 DONE。
    如果一直 NO_ACK，先查接线（TX 接 PA3）、波特率和共地。
    """
    task_id = 0
    while True:
        task_id = (task_id + 1) & 0xFFFF
        result = send_and_wait(task_id, 2, 3, 1200, 0)
        print("demo task", task_id, "->", result)
        time.sleep_ms(2000)


def run(find_blocks):
    """接你的模型：find_blocks() 返回带 cx()/cy()/code() 的物块列表。"""
    task_id = 0
    while True:
        blocks = find_blocks()
        if not blocks:
            time.sleep_ms(200)
            continue

        b = blocks[0]
        x_step = pixel_to_step_x(b.cx(), b.cy())
        box_id = CLASS_TO_BOX.get(b.code(), 1)
        task_id = (task_id + 1) & 0xFFFF

        result = send_and_wait(task_id, b.code(), box_id, x_step, 0)
        print("task", task_id, "x", x_step, "box", box_id, "->", result)
        time.sleep_ms(300)


if __name__ == "__main__":
    demo()
