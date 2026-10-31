#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电脑侧串口测试工具（智能分拣电控）。

用途：按键不够时用电脑串口代替手工按键 / 上位机：

1) USART3 文字控制台（台架虚拟按键，固件 SORT_DEBUG_CONSOLE=1）
   python tools/pc/serial_tool.py -p COM6 console
   固件端把 0~3 当成 KEY0~KEY3 短按、A~D 当成 1500 ms 长按，
   s 打印状态，x 释放，h 帮助。命令见 Service/bench_console.h。

2) USART2 二进制协议（视觉/上位机口，AA 55 + CRC16）
   python tools/pc/serial_tool.py -p COM5 status
   python tools/pc/serial_tool.py -p COM5 home
   python tools/pc/serial_tool.py -p COM5 stop
   python tools/pc/serial_tool.py -p COM5 reset
   python tools/pc/serial_tool.py -p COM5 mission --id 1 --class 2 --box 3 --x 1200 --y 0
   python tools/pc/serial_tool.py -p COM5 monitor

依赖：pip install pyserial（只有真正收发串口时才需要；--selftest 不依赖）。
"""

import argparse
import struct
import sys
import time

PROTOCOL_VERSION = 1
FRAME_HEAD = b"\xAA\x55"

CMD_STATUS = 0x01
CMD_HOME = 0x02
CMD_SORT_MISSION = 0x10
CMD_STOP = 0x13
CMD_RESET_FAULT = 0x30

EVENT_NAMES = {
    0x80: "ACK",
    0x81: "BUSY",
    0x82: "DONE",
    0x83: "ERROR",
    0x84: "STATUS",
    0x85: "RESET_ACK",
    0x86: "HOME_ACK",
}

STATE_NAMES = {
    0: "INIT",
    1: "HOMING",
    2: "T_HOMING_SEARCH",
    3: "T_HOMING_BACKOFF",
    4: "T_HOMING_APPROACH",
    5: "READY",
    6: "MOVE_TO_PICK_XY",
    7: "Z_APPROACH",
    8: "GRIP_OPEN",
    9: "Z_DESCEND",
    10: "GRIP_CLOSE",
    11: "GRIP_VERIFY",
    12: "LIFT_AND_TURRET",
    13: "MOVE_TO_DROP_XY",
    14: "TURRET_WAIT",
    15: "DROP_OPEN",
    16: "RETURN_SAFE",
    17: "TASK_DONE",
    18: "ERROR",
    19: "EMERGENCY_STOP",
    20: "XY_HOMING_SEARCH",
    21: "XY_HOMING_BACKOFF",
    22: "XY_HOMING_APPROACH",
}

ERROR_NAMES = {
    0: "NONE",
    1: "COORD",
    2: "XY_TIMEOUT",
    3: "T_TIMEOUT",
    4: "GRIP",
    5: "HOMING",
    6: "EMERGENCY",
    7: "PROTOCOL",
    8: "STOPPED",
    9: "NOT_READY",
    10: "MISSION_TIMEOUT",
}


def crc16_modbus(data):
    """与 Service/protocol.c 的 Protocol_Crc16 完全一致（0xA001 反射多项式）。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def build_frame(command, payload=b""):
    """AA 55 | version | command | length | payload | CRC16(低字节在前)。

    CRC 只覆盖 version/command/length/payload，和固件发送端一致。
    """
    body = bytes([PROTOCOL_VERSION, command, len(payload)]) + payload
    crc = crc16_modbus(body)
    return FRAME_HEAD + body + struct.pack("<H", crc)


def mission_payload(task_id, class_id, box_id, pick_x, pick_y, drop_step, state):
    """SortMission_t 的 17 字节线缆格式，全部小端。"""
    return struct.pack(
        "<HBBiiiB",
        task_id & 0xFFFF,
        class_id & 0xFF,
        box_id & 0xFF,
        int(pick_x),
        int(pick_y),
        int(drop_step),
        state & 0xFF,
    )


def parse_frames(buffer):
    """从字节流中切出所有校验通过的帧，返回 (frames, rest)。"""
    frames = []
    index = 0
    while True:
        start = buffer.find(FRAME_HEAD, index)
        if start < 0:
            return frames, b""
        if len(buffer) < start + 5:
            return frames, buffer[start:]
        length = buffer[start + 4]
        total = start + 7 + length
        if len(buffer) < total:
            return frames, buffer[start:]
        version, command, payload = (
            buffer[start + 2],
            buffer[start + 3],
            buffer[start + 5 : start + 5 + length],
        )
        expected = struct.unpack_from("<H", buffer, start + 5 + length)[0]
        actual = crc16_modbus(buffer[start + 2 : start + 5 + length])
        if (version == PROTOCOL_VERSION) and (expected == actual):
            frames.append((command, payload, buffer[start:total]))
            index = total
        else:
            # 校验失败：只跳过当前帧头，继续找下一个 0xAA 0x55。
            index = start + 1


def describe(command, payload):
    """把事件帧翻译成一行可读文本。"""
    name = EVENT_NAMES.get(command, "EVENT_%02X" % command)
    if command in (0x80, 0x81, 0x82, 0x85, 0x86) and len(payload) >= 2:
        task_id = struct.unpack_from("<H", payload)[0]
        return "%s task_id=%d" % (name, task_id)
    if command == 0x83 and len(payload) >= 3:
        task_id = struct.unpack_from("<H", payload)[0]
        error = payload[2]
        return "%s task_id=%d error=%d(%s)" % (
            name,
            task_id,
            error,
            ERROR_NAMES.get(error, "?"),
        )
    if command == 0x84 and len(payload) >= 2:
        return "%s state=%d(%s) error=%d(%s)" % (
            name,
            payload[0],
            STATE_NAMES.get(payload[0], "?"),
            payload[1],
            ERROR_NAMES.get(payload[1], "?"),
        )
    return "%s payload=%s" % (name, payload.hex(" "))


def open_port(name, baud):
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("缺少 pyserial：先执行 pip install pyserial")
    return serial.Serial(name, baud, timeout=0.05)


def read_events(port, duration_s):
    deadline = time.time() + duration_s
    buffer = b""
    while time.time() < deadline:
        chunk = port.read(64)
        if chunk:
            buffer += chunk
            frames, buffer = parse_frames(buffer)
            for command, payload, _raw in frames:
                print("RX " + describe(command, payload))
        else:
            time.sleep(0.01)


def cmd_simple(port, command, wait_s=0.5):
    frame = build_frame(command)
    print("TX " + frame.hex(" "))
    port.write(frame)
    port.flush()
    read_events(port, wait_s)


def cmd_mission(port, args, wait_s=1.0):
    payload = mission_payload(
        args.id, args.cls, args.box, args.x, args.y, args.drop, args.state
    )
    frame = build_frame(CMD_SORT_MISSION, payload)
    print("TX " + frame.hex(" "))
    port.write(frame)
    port.flush()
    read_events(port, wait_s)


def cmd_monitor(port, args):
    print("# 监听 USART2 事件帧，Ctrl+C 退出")
    try:
        while True:
            read_events(port, args.seconds if args.seconds > 0 else 1.0)
            if args.seconds > 0:
                return
    except KeyboardInterrupt:
        print("\n# stopped")


def cmd_console(port, args):
    """USART3 台架控制台：一行输入会被逐字符发出去（固件忽略 CR/LF）。"""
    print(
        "# USART3 控制台：0~3 短按 KEY0~KEY3，A~D 长按，"
        "s 状态，x 释放，h 帮助；q 退出"
    )
    buffer = b""
    while True:
        try:
            line = input("> ")
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if line.strip().lower() == "q":
            return
        if not line:
            continue
        port.write(line.encode("ascii", "ignore") + b"\r\n")
        port.flush()
        # 打印固件回显（含 DebugUart 的状态行）。
        time.sleep(args.echo_wait)
        while port.in_waiting:
            chunk = port.read(port.in_waiting)
            buffer += chunk
            while b"\n" in buffer:
                line_out, buffer = buffer.split(b"\n", 1)
                print("RX " + line_out.rstrip(b"\r").decode("utf-8", "replace"))


def run_selftest():
    """不接硬件的自检：CRC 已知向量 + 帧组装/解析往返。"""
    assert crc16_modbus(b"123456789") == 0x4B37, "Modbus CRC16 known vector"

    frame = build_frame(CMD_STATUS)
    assert frame[:5] == b"\xAA\x55\x01\x01\x00", frame.hex(" ")
    assert crc16_modbus(frame[2:5]) == struct.unpack_from("<H", frame, 5)[0]

    payload = mission_payload(7, 2, 3, -1200, 0, 0, 1)
    assert len(payload) == 17, "wire size must be 17 bytes"
    assert payload[:4] == bytes([0x07, 0x00, 0x02, 0x03])
    assert struct.unpack_from("<i", payload, 4)[0] == -1200

    stream = build_frame(CMD_SORT_MISSION, payload) + build_frame(
        0x84, bytes([5, 0])
    )
    frames, rest = parse_frames(b"junk" + stream)
    assert rest == b""
    assert [f[0] for f in frames] == [CMD_SORT_MISSION, 0x84]
    assert frames[0][1] == payload
    assert describe(0x84, frames[1][1]).startswith("STATUS state=5(READY)")

    # 校验错误的帧必须被丢弃，不能误当成有效命令。
    bad = bytearray(build_frame(CMD_STOP))
    bad[-1] ^= 0xFF
    frames, _rest = parse_frames(bytes(bad))
    assert frames == [], "corrupted CRC must be rejected"

    print("selftest: PASS (CRC16 vector, 17-byte mission, frame round-trip)")


def main():
    parser = argparse.ArgumentParser(description="智能分拣电控电脑侧串口测试工具")
    parser.add_argument("-p", "--port", help="串口号，例如 COM5")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("--selftest", action="store_true", help="只跑不接硬件的自检")
    sub = parser.add_subparsers(dest="action")

    sub.add_parser("status", help="查询状态（USART2 0x01）")
    sub.add_parser("home", help="请求回零（USART2 0x02）")
    sub.add_parser("stop", help="软停止（USART2 0x13）")
    sub.add_parser("reset", help="故障复位（USART2 0x30）")

    mission = sub.add_parser("mission", help="下发分拣任务（USART2 0x10）")
    mission.add_argument("--id", type=int, default=1, help="task_id (0~65535)")
    mission.add_argument("--class", dest="cls", type=int, default=1, help="class_id")
    mission.add_argument("--box", type=int, default=1, help="box_id (1~6)")
    mission.add_argument("--x", type=int, default=0, help="取料 pick_x_step")
    mission.add_argument("--y", type=int, default=0, help="取料 pick_y_step")
    mission.add_argument("--drop", type=int, default=0, help="drop_step 回退值")
    mission.add_argument("--state", type=int, default=1, help="mission.state")

    monitor = sub.add_parser("monitor", help="监听 USART2 事件帧")
    monitor.add_argument("--seconds", type=float, default=0.0, help="0 = 一直监听")

    console = sub.add_parser("console", help="USART3 台架虚拟按键控制台")
    console.add_argument(
        "--echo-wait", type=float, default=0.15, help="每次发送后等待回显的秒数"
    )

    args = parser.parse_args()

    if args.selftest:
        run_selftest()
        return

    if not args.port:
        parser.error("需要 -p COMx（或使用 --selftest）")
    if not args.action:
        parser.error("需要一个子命令：status/home/stop/reset/mission/monitor/console")

    port = open_port(args.port, args.baud)
    try:
        if args.action == "status":
            cmd_simple(port, CMD_STATUS)
        elif args.action == "home":
            cmd_simple(port, CMD_HOME)
        elif args.action == "stop":
            cmd_simple(port, CMD_STOP)
        elif args.action == "reset":
            cmd_simple(port, CMD_RESET_FAULT)
        elif args.action == "mission":
            cmd_mission(port, args)
        elif args.action == "monitor":
            cmd_monitor(port, args)
        elif args.action == "console":
            cmd_console(port, args)
    finally:
        port.close()


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except AttributeError:
        pass
    main()
