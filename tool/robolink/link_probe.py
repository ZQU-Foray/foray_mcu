#!/usr/bin/env python3
"""RoboLink 上板探测：在 PC 上扮演上位机，与 link_dm_mc02 固件联调。

- 打开板子的 USB 虚拟串口（raw 模式），按 200 ms 重发 HELLO 直到收到 HELLO_ACK；
- 之后每 20 ms 发一个保活帧，可选按 1 kHz 发控制快照（--control）；
- 解析收到的每一帧（魔数定界、帧头、CRC、槽位），统计状态与姿态的频率、帧序号跳变、样本在 MCU 里的停留时间
  （帧头时刻 − 采样时刻）、IMU 状态的变化；
- 可选测拔线重连（--reconnect）：关串口 1 s 再打开，量重新握手的时间；
- 控制快照里的云台目标可以给固定值（--gimbal-yaw、--gimbal-pitch），也可以给正弦扫描（--sweep 振幅,频率），
  扫描时同时发对应的角速度前馈。这两个选项会自动打开 --control。

组帧用 make_golden.py 里只照协议文本写的那套实现；解析也只照协议写，不用本仓的 C++。

用法：
    link_probe.py                                  # 自动找板子的串口，跑 10 s
    link_probe.py --duration 60 --control          # 跑 60 s，同时按 1 kHz 发控制快照
    link_probe.py --port /dev/ttyACM0 --reconnect  # 指定串口，最后测一次重连
    link_probe.py --follow                         # 实时打印 yaw/pitch/roll，Ctrl-C 停止
    link_probe.py --follow --interval 0.2          # 实时打印，每 0.2 s 一行
    link_probe.py --gimbal-yaw 0.3 --gimbal-pitch 0.1   # 发固定的云台目标（自瞄接管的测试）
    link_probe.py --sweep 0.3,0.5                  # yaw 以 0.3 rad、0.5 Hz 正弦扫描
没有 pyserial 也能用（termios）。串口没有读写权限时，先让负责人加 udev 规则（见对账记录）。
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import select
import struct
import sys
import termios
import time
import tty
import zlib
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_golden as rl

DEFAULT_PORT_GLOB = "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_*"
IMU_STATES = {0: "SETTLING", 1: "READY", 2: "DEGRADED"}
LINK_STATES = {0: "DISCONNECTED", 1: "USB_CONNECTED", 2: "READY", 3: "ACTIVE"}


def open_port(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)  # cooked tty 会把字节回显给板子
    attributes = termios.tcgetattr(fd)
    attributes[6][termios.VMIN] = 0
    attributes[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attributes)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


@dataclass
class Stats:
    frames: int = 0
    crc_errors: int = 0
    header_errors: int = 0
    slot_errors: int = 0
    sequence_gaps: int = 0
    statuses: int = 0
    attitudes: int = 0
    keepalives: int = 0
    handshake_acks: int = 0
    first_status_s: float | None = None
    first_attitude_s: float | None = None
    last_status: dict = field(default_factory=dict)
    last_diagnostics: dict = field(default_factory=dict)
    last_attitude: dict = field(default_factory=dict)
    mcu_hello: dict = field(default_factory=dict)
    sample_age_us: list = field(default_factory=list)  # 帧头时刻 − 采样时刻
    sample_interval_us: list = field(default_factory=list)  # 相邻姿态的采样间隔
    offset_us: list = field(default_factory=list)  # 本机接收时刻 − 帧头时刻（各自时钟）
    imu_transitions: list = field(default_factory=list)  # (本机时刻, 状态)
    attitude_invalid: int = 0
    first_euler: tuple | None = (
        None  # 第一份有效、READY 的姿态（ZYX：yaw、pitch、roll，度）
    )
    last_euler: tuple | None = None
    first_euler_s: float = 0.0
    last_euler_s: float = 0.0
    quaternion_norm_error: float = 0.0


class Parser:
    """§1.4 字节流定界：找魔数；帧头不合法从候选头第二字节重扫；CRC 错丢整帧。"""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes, stats: Stats) -> list[bytes]:
        self.buffer += data
        frames = []
        while True:
            start = self.buffer.find(rl.MAGIC)
            if start < 0:
                keep = self.buffer[-1:] == rl.MAGIC[:1]
                self.buffer = self.buffer[-1:] if keep else bytearray()
                return frames
            del self.buffer[:start]
            if len(self.buffer) < 14:
                return frames
            version, flags, length, _, _, slots, reserved = struct.unpack_from(
                "<BBHHIBB", self.buffer, 2
            )
            keepalive = bool(flags & rl.KEEPALIVE)
            if (
                version != rl.VERSION
                or flags & ~0x03
                or not 18 <= length <= 512
                or reserved != 0
                or keepalive != (slots == 0)
            ):
                stats.header_errors += 1
                del self.buffer[:1]
                continue
            if len(self.buffer) < length:
                return frames
            frame = bytes(self.buffer[:length])
            del self.buffer[:length]
            if (
                zlib.crc32(frame[:-4]) & 0xFFFFFFFF
                != struct.unpack_from("<I", frame, length - 4)[0]
            ):
                stats.crc_errors += 1
                continue
            frames.append(frame)


def slots_of(frame: bytes) -> list[tuple[int, int, int, bytes]] | None:
    count = frame[12]
    offset = 14
    end = len(frame) - 4
    slots = []
    for _ in range(count):
        if end - offset < 6:
            return None
        message_type, flags, sequence, length = struct.unpack_from(
            "<BBHH", frame, offset
        )
        offset += 6
        if length > end - offset or rl.CLASS_FLAGS.get(message_type) != flags:
            return None
        slots.append((message_type, flags, sequence, frame[offset : offset + length]))
        offset += length
    return slots if offset == end else None


def euler_zyx(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw, pitch, roll


class Probe:
    def __init__(
        self,
        path: str,
        send_control: bool,
        follow_interval: float | None = None,
        gimbal_yaw: float = 0.125,
        gimbal_pitch: float = -0.0625,
        sweep: tuple[float, float] | None = None,
    ) -> None:
        self.path = path
        self.control_enabled = send_control
        self.gimbal_yaw = gimbal_yaw
        self.gimbal_pitch = gimbal_pitch
        self.sweep = sweep
        self.control_start_s = self.now()
        self.follow_interval = follow_interval
        self.fd = open_port(path)
        self.parser = Parser()
        self.stats = Stats()
        self.t0 = time.monotonic()
        self.frame_seq = 0
        self.hello_seq = 0
        self.control_seq = 0
        self.control_msg_seq = 0
        self.ready = False
        self.hello_sent_s = 0.0
        self.handshake_s: float | None = None
        self.ready_at_s: float | None = None
        self.last_mcu_seq: int | None = None
        self.last_sample_time: int | None = None
        self.last_imu_state: int | None = None
        self.last_attitude_received_s: float | None = None

    def now(self) -> float:
        return time.monotonic() - self.t0

    def write(self, slots: list[bytes]) -> None:
        stamp = int(self.now() * 1e6) & 0xFFFFFFFF
        data = rl.frame(self.frame_seq & 0xFFFF, stamp, slots)
        self.frame_seq += 1
        try:
            os.write(self.fd, data)
        except BlockingIOError:
            pass

    def send_hello(self) -> None:
        payload = rl.hello(2, 0, 1, [0, 0, 0], 0, rl.ALL_TYPES, 20, 512, 0)
        self.write([rl.slot(rl.HELLO, self.hello_seq, payload)])
        self.hello_sent_s = self.now()

    def gimbal_setpoint(self) -> tuple[float, float, float, float]:
        """云台目标与前馈：固定值，或者围绕 yaw 目标的正弦扫描（同时给出角速度前馈）。"""
        if self.sweep is None:
            return self.gimbal_yaw, self.gimbal_pitch, 0.0, 0.0
        amplitude, frequency = self.sweep
        phase = 2.0 * math.pi * frequency * (self.now() - self.control_start_s)
        yaw = self.gimbal_yaw + amplitude * math.sin(phase)
        yaw_rate = amplitude * 2.0 * math.pi * frequency * math.cos(phase)
        return yaw, self.gimbal_pitch, yaw_rate, 0.0

    def send_control(self) -> None:
        yaw, pitch, yaw_rate, pitch_rate = self.gimbal_setpoint()
        payload = rl.control(
            self.control_seq & 0xFFFFFFFF,
            2,
            2,
            0x0002,
            [0.0, 0.0, 0.0, yaw, pitch, yaw_rate, pitch_rate, 0.0, 0.0],
            0,
        )
        self.write(
            [rl.slot(rl.CONTROL_SNAPSHOT, self.control_msg_seq & 0xFFFF, payload)]
        )
        self.control_seq += 1
        self.control_msg_seq += 1

    def handle(self, frame: bytes, received_s: float) -> None:
        stats = self.stats
        slots = slots_of(frame)
        if slots is None:
            stats.slot_errors += 1
            return
        stats.frames += 1
        sequence, stamp = struct.unpack_from("<HI", frame, 6)
        if (
            self.last_mcu_seq is not None
            and sequence != (self.last_mcu_seq + 1) & 0xFFFF
            and sequence != 0
        ):
            stats.sequence_gaps += 1
        self.last_mcu_seq = sequence
        stats.offset_us.append(received_s * 1e6 - stamp)
        if not slots:
            stats.keepalives += 1
        for message_type, _, _, payload in slots:
            if message_type == rl.HELLO_ACK:
                acked, _, status, _, _ = struct.unpack_from("<HBBHH", payload)
                stats.handshake_acks += 1
                if status == 0 and acked == self.hello_seq and not self.ready:
                    self.ready = True
                    self.ready_at_s = received_s
                    self.handshake_s = received_s - self.hello_sent_s
                    self.frame_seq = 0
                    self.last_mcu_seq = None
            elif message_type == rl.HELLO:
                fields = struct.unpack_from("<BBB3sIQHHI", payload)
                stats.mcu_hello = {
                    "proto": f"{fields[0]}.{fields[1]}",
                    "device_type": fields[2],
                    "fw_version": ".".join(str(b) for b in fields[3]),
                    "git_short": f"0x{fields[4]:08x}",
                    "capabilities": f"0x{fields[5]:016x}",
                    "heartbeat_ms": fields[6],
                    "max_frame": fields[7],
                }
            elif message_type == rl.SYSTEM_STATUS:
                stats.statuses += 1
                if stats.first_status_s is None:
                    stats.first_status_s = received_s
                core = struct.unpack_from("<IBBBBIIIHBBI", payload)
                stats.last_status = {
                    "uptime_ms": core[0],
                    "mode": core[1],
                    "control_source": core[2],
                    "safety_state": core[3],
                    "last_rx_control_seq": core[5],
                    "last_accepted_control_seq": core[6],
                    "last_applied_control_seq": core[7],
                    "control_msg_age_ms": core[8],
                    "link_state": LINK_STATES.get(core[9], core[9]),
                    "fault_flags": f"0x{core[11]:08x}",
                }
                if len(payload) >= 52:
                    diagnostics = struct.unpack_from("<IIIHHHBBI", payload, 28)
                    stats.last_diagnostics = {
                        "rx_frames": diagnostics[0],
                        "tx_frames": diagnostics[1],
                        "crc_errors": diagnostics[2],
                        "rx_overflow": diagnostics[3],
                        "tx_busy_drop": diagnostics[4],
                        "queue_drop": diagnostics[5],
                        "event_queue_depth": diagnostics[6],
                    }
            elif message_type == rl.GIMBAL_ATTITUDE:
                stats.attitudes += 1
                if stats.first_attitude_s is None:
                    stats.first_attitude_s = received_s
                values = struct.unpack_from("<I4f3fBBH", payload)
                sample_time = values[0]
                quaternion = values[1:5]
                gyro = values[5:8]
                imu_state, flags = values[8], values[9]
                valid = bool(flags & 0x01)
                sample_age = ((stamp - sample_time + 2**31) % 2**32) - 2**31
                stats.sample_age_us.append(sample_age)
                if self.last_sample_time is not None:
                    stats.sample_interval_us.append(
                        (sample_time - self.last_sample_time) % 2**32
                    )
                self.last_sample_time = sample_time
                if imu_state != self.last_imu_state:
                    stats.imu_transitions.append(
                        (received_s, IMU_STATES.get(imu_state, imu_state))
                    )
                    self.last_imu_state = imu_state
                if not valid:
                    stats.attitude_invalid += 1
                else:
                    if imu_state == 1:
                        euler = tuple(math.degrees(a) for a in euler_zyx(quaternion))
                        if stats.first_euler is None:
                            stats.first_euler, stats.first_euler_s = euler, received_s
                        stats.last_euler, stats.last_euler_s = euler, received_s
                    norm = math.sqrt(sum(c * c for c in quaternion))
                    stats.quaternion_norm_error = max(
                        stats.quaternion_norm_error, abs(norm - 1.0)
                    )
                self.last_attitude_received_s = received_s
                stats.last_attitude = {
                    "sample_time_us": sample_time,
                    "quaternion_wxyz": [round(c, 5) for c in quaternion],
                    "euler_zyx_deg": [
                        round(math.degrees(a), 3) for a in euler_zyx(quaternion)
                    ],
                    "gyro_rad_per_sec": [round(g, 5) for g in gyro],
                    "imu_state": IMU_STATES.get(imu_state, imu_state),
                    "valid": valid,
                }

    def run(self, duration: float | None) -> None:
        last_hello = -1.0
        last_keepalive = 0.0
        next_control = 0.0
        next_follow = 0.0
        end = None if duration is None else self.now() + duration
        while end is None or self.now() < end:
            now = self.now()
            if not self.ready and now - last_hello >= 0.2:
                self.send_hello()
                last_hello = now
            if self.ready and self.control_enabled and now >= next_control:
                self.send_control()
                next_control = now + 0.001
                last_keepalive = now
            elif self.ready and now - last_keepalive >= 0.02:
                self.write([])
                last_keepalive = now
            readable, _, _ = select.select([self.fd], [], [], 0.0005)
            if readable:
                try:
                    data = os.read(self.fd, 4096)
                except BlockingIOError:
                    data = b""
                received = self.now()
                for frame in self.parser.feed(data, self.stats):
                    self.handle(frame, received)
            if self.follow_interval is not None:
                current = self.now()
                if current >= next_follow:
                    self.print_follow(current)
                    next_follow = current + self.follow_interval

    def print_follow(self, now: float) -> None:
        """实时打印一行：状态、yaw/pitch/roll（度）与这份姿态收到的时间。"""
        if not self.ready:
            print(f"[{now:7.2f} s] 等待握手（HELLO → HELLO_ACK）…", flush=True)
            return
        attitude = self.stats.last_attitude
        if not attitude:
            print(f"[{now:7.2f} s] 已握手，等待姿态…", flush=True)
            return
        yaw, pitch, roll = attitude["euler_zyx_deg"]
        valid = "有效" if attitude["valid"] else "无效"
        received_s = self.last_attitude_received_s or 0.0
        age_ms = (now - received_s) * 1000.0
        print(
            f"[{now:7.2f} s] {attitude['imu_state']:>8} {valid}  "
            f"yaw {yaw:8.2f}°  pitch {pitch:7.2f}°  roll {roll:7.2f}°  "
            f"收 {age_ms:4.0f} ms 前",
            flush=True,
        )

    def reconnect(self) -> float | None:
        os.close(self.fd)
        time.sleep(1.0)
        self.fd = open_port(self.path)
        self.parser = Parser()
        self.ready = False
        self.handshake_s = None
        self.ready_at_s = None
        start = self.now()
        self.run(3.0)
        return None if self.ready_at_s is None else self.ready_at_s - start


def summarize(values: list[float]) -> str:
    if not values:
        return "—"
    ordered = sorted(values)
    return (
        f"最小 {ordered[0]:.0f} · 中位 {ordered[len(ordered) // 2]:.0f} · "
        f"p99 {ordered[int(len(ordered) * 0.99)]:.0f} · 最大 {ordered[-1]:.0f}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="串口路径（默认自动找 DM-MC02 的虚拟串口）")
    parser.add_argument("--duration", type=float, default=10.0, help="联调时长（秒）")
    parser.add_argument(
        "--control", action="store_true", help="握手后按 1 kHz 发控制快照"
    )
    parser.add_argument(
        "--reconnect", action="store_true", help="最后关串口 1 s 再打开，量重新握手"
    )
    parser.add_argument(
        "--gimbal-yaw", type=float, help="控制快照里的云台 yaw 目标（rad，默认 0.125）"
    )
    parser.add_argument(
        "--gimbal-pitch",
        type=float,
        help="云台 pitch 目标（rad，低头为正，默认 −0.0625）",
    )
    parser.add_argument(
        "--sweep",
        help="正弦目标：振幅(rad),频率(Hz)，如 0.3,0.5；围绕 yaw 目标扫描，同时发角速度前馈",
    )
    parser.add_argument(
        "--follow",
        action="store_true",
        help="实时打印姿态的 yaw/pitch/roll，直到 Ctrl-C（忽略 --duration）",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="--follow 时每隔多少秒打印一行（默认 0.5）",
    )
    args = parser.parse_args()

    sweep = None
    if args.sweep is not None:
        try:
            amplitude_text, frequency_text = args.sweep.split(",")
            sweep = (float(amplitude_text), float(frequency_text))
        except ValueError:
            print("--sweep 的写法是 振幅,频率，例如 0.3,0.5")
            return 2
        if sweep[0] < 0.0 or sweep[1] <= 0.0:
            print("--sweep 的振幅不能为负、频率必须大于 0")
            return 2
    gimbal_yaw = 0.125 if args.gimbal_yaw is None else args.gimbal_yaw
    gimbal_pitch = -0.0625 if args.gimbal_pitch is None else args.gimbal_pitch
    send_control = (
        args.control
        or args.gimbal_yaw is not None
        or args.gimbal_pitch is not None
        or sweep is not None
    )

    path = args.port
    if path is None:
        candidates = sorted(glob.glob(DEFAULT_PORT_GLOB))
        if not candidates:
            print(
                "找不到 DM-MC02 的虚拟串口（/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_*）"
            )
            return 2
        path = candidates[0]
    if args.follow:
        if args.interval <= 0:
            print("--interval 必须大于 0")
            return 2
        if args.reconnect:
            print("--follow 与 --reconnect 不能同时用")
            return 2
        probe = Probe(
            path, send_control, args.interval, gimbal_yaw, gimbal_pitch, sweep
        )
        print(f"串口 {path}，实时打印姿态（Ctrl-C 停止；每 {args.interval:g} s 一行）")
        try:
            probe.run(None)
        except KeyboardInterrupt:
            print("\n已停止")
        finally:
            os.close(probe.fd)
        return 0
    probe = Probe(path, send_control, None, gimbal_yaw, gimbal_pitch, sweep)
    probe.run(args.duration)
    s = probe.stats
    elapsed = args.duration
    print(f"串口 {path}，联调 {elapsed:.0f} s")
    handshake = (
        "—" if probe.handshake_s is None else f"{probe.handshake_s * 1000:.1f} ms"
    )
    print(
        f"握手：{'成功' if probe.ready else '失败'}（HELLO → HELLO_ACK {handshake}）；MCU 的 HELLO：{s.mcu_hello}"
    )
    print(
        f"收到 {s.frames} 帧：状态 {s.statuses}、姿态 {s.attitudes}、保活 {s.keepalives}；"
        f"CRC 错 {s.crc_errors}、帧头错 {s.header_errors}、槽位错 {s.slot_errors}、帧序号跳变 {s.sequence_gaps}"
    )
    attitude_span = (
        None if s.first_attitude_s is None else probe.now() - s.first_attitude_s
    )
    if attitude_span:
        print(
            f"姿态频率 {s.attitudes / attitude_span:.1f} Hz；无效 {s.attitude_invalid} 份；四元数模长误差最大 {s.quaternion_norm_error:.2e}"
        )
    print(f"样本在 MCU 里的停留（帧头 − 采样，µs）：{summarize(s.sample_age_us)}")
    print(f"相邻姿态的采样间隔（µs）：{summarize(s.sample_interval_us)}")
    if s.offset_us:
        print(
            f"本机接收 − 帧头时刻的最小值（时钟偏移估计）：{min(s.offset_us) / 1000:.3f} ms"
        )
    if s.first_euler is not None and s.last_euler_s > s.first_euler_s:
        span = s.last_euler_s - s.first_euler_s
        drift = [last - first for first, last in zip(s.first_euler, s.last_euler)]
        print(
            f"READY 时 {span:.1f} s 内姿态变化（度，yaw/pitch/roll）：{[round(d, 3) for d in drift]}，"
            f"yaw 约 {drift[0] / span * 60:.3f} °/min（板子静止时即零偏漂移）"
        )
    print(
        f"IMU 状态变化（本机时刻 s）：{[(round(t, 2), state) for t, state in s.imu_transitions]}"
    )
    print(f"最后的状态：{s.last_status}")
    print(f"最后的诊断尾部：{s.last_diagnostics}")
    print(f"最后的姿态：{s.last_attitude}")
    if args.control:
        print(
            f"发出控制快照 {probe.control_seq} 份；MCU 回报最近收到 {s.last_status.get('last_rx_control_seq')}"
        )

    checks = [
        ("握手成功", probe.ready),
        (
            "没有 CRC、帧头、槽位错误",
            s.crc_errors == 0 and s.header_errors == 0 and s.slot_errors == 0,
        ),
        ("帧序号连续", s.sequence_gaps == 0),
        (
            "状态约 200 Hz",
            s.first_status_s is not None
            and s.statuses >= 180 * (probe.now() - s.first_status_s) - 5,
        ),
        (
            "姿态 ≥ 900 Hz",
            attitude_span is not None and s.attitudes >= 900 * attitude_span,
        ),
        (
            "样本停留 < 2 ms（§4.8）",
            bool(s.sample_age_us) and max(s.sample_age_us) < 2000,
        ),
        ("MCU 状态 ACTIVE", s.last_status.get("link_state") == "ACTIVE"),
    ]
    if args.control:
        checks.append(
            (
                "MCU 收到控制快照（last_rx 接近发出的序号）",
                s.last_status.get("last_rx_control_seq", 0) + 20
                >= probe.control_seq - 1,
            )
        )
    if args.reconnect:
        took = probe.reconnect()
        print(f"重连：{'—' if took is None else f'{took * 1000:.1f} ms'} 后重新握手")
        checks.append(
            ("关串口再打开后 2 s 内重新握手", took is not None and took < 2.0)
        )
    failed = 0
    for name, ok in checks:
        print(f"{'通过' if ok else '失败'} {name}")
        failed += 0 if ok else 1
    os.close(probe.fd)
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
