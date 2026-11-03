#!/usr/bin/env python3
"""RoboLink v0.2 的标准帧（黄金向量）生成器（Decision-20261103-01）。

只照 foray_interfaces 的 protocol/robolink.md（v0.2，提交 5973436）编写，不读本仓的 C++ 实现：
CRC 用 zlib.crc32（CRC-32/ISO-HDLC），整数与浮点用 struct 的小端编码。MCU 端的宿主测试
（test/host/unit/robolink）逐字节对照本脚本的输出，两份独立的实现对同一份协议文本互相校验。

输出是文本文件，每行一个向量：`<名称> <期望> <十六进制字节>`，`#` 开头的行是注释。
期望为 ok（合法帧），或帧被拒的原因：header、crc、layout、unknown_type、slot_flags、class、short、
ack_flag；payload 表示帧合法、但消息内容按协议应拒收。

用法：
    tool/robolink/make_golden.py            # 重新生成 test/data/robolink/golden_v0_2.txt
    tool/robolink/make_golden.py --check    # 只核对文件与脚本输出一致
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import zlib
from pathlib import Path

OUTPUT = (
    Path(__file__).resolve().parents[2]
    / "test"
    / "data"
    / "robolink"
    / "golden_v0_2.txt"
)

# §1.1、§1.2 帧层
MAGIC = b"\xd3\x91"
VERSION = 0x02
ACK_PRESENT = 0x01
KEEPALIVE = 0x02

# §2.1 槽位标志
ACK_REQ = 0x01
LATEST_ONLY = 0x02
EVENT = 0x04
RESPONSE = 0x08

# §3.1 消息类型
HELLO = 0x01
HELLO_ACK = 0x02
HEARTBEAT = 0x03
ACK = 0x04
ERROR = 0x05
SYSTEM_STATUS = 0x10
CONTROL_SNAPSHOT = 0x20
GIMBAL_ATTITUDE = 0x30

# §3.1 的"类"决定槽位标志（§2.1 与注册表类属性一致）：EVENT → EVENT 位，LATEST → LATEST_ONLY 位，
# STATUS → 不置位，"+ RESPONSE" → RESPONSE 位。
CLASS_FLAGS = {
    HELLO: EVENT,
    HELLO_ACK: EVENT | RESPONSE,
    HEARTBEAT: 0,
    ACK: RESPONSE,
    ERROR: EVENT,
    SYSTEM_STATUS: 0,
    CONTROL_SNAPSHOT: LATEST_ONLY,
    GIMBAL_ATTITUDE: 0,
}


def f32(value: float) -> bytes:
    """单精度小端。向量只用 float32 能精确表示的值，C++ 测试里写同样的字面量不会有舍入差异。"""
    packed = struct.pack("<f", value)
    if not math.isnan(value) and struct.unpack("<f", packed)[0] != value:
        raise ValueError(f"{value} 不能用 float32 精确表示")
    return packed


def slot(
    message_type: int,
    sequence: int,
    payload: bytes,
    flags: int | None = None,
    length: int | None = None,
) -> bytes:
    """§2.1 槽位：类型、标志、消息序号、载荷长度（6 B）+ 载荷。flags、length 可以改写，用来造坏帧。"""
    flags = CLASS_FLAGS[message_type] if flags is None else flags
    length = len(payload) if length is None else length
    return struct.pack("<BBHH", message_type, flags, sequence, length) + payload


def frame(
    sequence: int,
    timestamp_us: int,
    slots: list[bytes],
    *,
    flags: int | None = None,
    version: int = VERSION,
    slot_count: int | None = None,
    reserved: int = 0,
    length: int | None = None,
    corrupt_crc: bool = False,
) -> bytes:
    """§1.1 整帧：14 B 帧头 + 槽位 + CRC-32（覆盖魔数首字节到 CRC 前一字节，小端）。

    默认值按协议填写：有 ACK 槽位置 ACK_PRESENT，零槽置 KEEPALIVE。其余参数用来造坏帧。
    """
    body = b"".join(slots)
    if flags is None:
        has_ack = any(s[0] == ACK for s in slots)
        flags = (ACK_PRESENT if has_ack else 0) | (KEEPALIVE if not slots else 0)
    slot_count = len(slots) if slot_count is None else slot_count
    length = 14 + len(body) + 4 if length is None else length
    header = MAGIC + struct.pack(
        "<BBHHIBB", version, flags, length, sequence, timestamp_us, slot_count, reserved
    )
    crc = zlib.crc32(header + body) & 0xFFFFFFFF
    if corrupt_crc:
        crc ^= 0x00000001
    return header + body + struct.pack("<I", crc)


# §4 载荷（显式小端，偏移见各表）


def hello(
    major,
    minor,
    device_type,
    fw_version,
    git_short,
    capabilities,
    heartbeat_ms,
    max_frame,
    digest,
) -> bytes:
    """§4.1 HELLO（26 B）。"""
    return struct.pack(
        "<BBB3sIQHHI",
        major,
        minor,
        device_type,
        bytes(fw_version),
        git_short,
        capabilities,
        heartbeat_ms,
        max_frame,
        digest,
    )


def hello_ack(acked_seq, negotiated_major, status, heartbeat_ms, reserved=0) -> bytes:
    """§4.2 HELLO_ACK（8 B）。"""
    return struct.pack(
        "<HBBHH", acked_seq, negotiated_major, status, heartbeat_ms, reserved
    )


def ack(acked_type, acked_seq, state, error_code, reserved=0) -> bytes:
    """§4.4 ACK（7 B）。"""
    return struct.pack("<BHBHB", acked_type, acked_seq, state, error_code, reserved)


def error(code, detail: bytes) -> bytes:
    """§4.5 ERROR（2 B + N）。"""
    return struct.pack("<H", code) + detail


def status_core(
    uptime_ms,
    mode,
    source,
    safety,
    rx_seq,
    accepted_seq,
    applied_seq,
    age_ms,
    link_state,
    faults,
):
    """§4.6 SYSTEM_STATUS 核心（28 B），两个保留字节为 0。"""
    return struct.pack(
        "<IBBBBIIIHBBI",
        uptime_ms,
        mode,
        source,
        safety,
        0,
        rx_seq,
        accepted_seq,
        applied_seq,
        age_ms,
        link_state,
        0,
        faults,
    )


def status_diagnostics(
    rx_frames, tx_frames, crc_errors, rx_overflow, tx_busy_drop, queue_drop, depth
) -> bytes:
    """§4.6 SYSTEM_STATUS 诊断尾部（24 B），保留字节为 0。"""
    return struct.pack(
        "<IIIHHHBBI",
        rx_frames,
        tx_frames,
        crc_errors,
        rx_overflow,
        tx_busy_drop,
        queue_drop,
        depth,
        0,
        0,
    )


def control(seq, source, mode, flags, floats, validity_ms, reserved=0) -> bytes:
    """§4.7 CONTROL_SNAPSHOT（48 B）：9 个 float 依次为 chassis_vx、vy、wz、gimbal_yaw、pitch、yaw_rate、
    pitch_rate、friction_rads、feeder_rads。"""
    assert len(floats) == 9
    return (
        struct.pack("<IBBH", seq, source, mode, flags)
        + b"".join(f32(v) for v in floats)
        + struct.pack("<HH", validity_ms, reserved)
    )


def attitude(sample_time_us, quaternion, gyro, imu_state, flags, reserved=0) -> bytes:
    """§4.8 GIMBAL_ATTITUDE（36 B）。"""
    return (
        struct.pack("<I", sample_time_us)
        + b"".join(f32(v) for v in quaternion)
        + b"".join(f32(v) for v in gyro)
        + struct.pack("<BBH", imu_state, flags, reserved)
    )


def capabilities(*types: int) -> int:
    """§4.1 capabilities：bit n ⇔ Type n。"""
    return sum(1 << t for t in types)


# 两端都支持的第一版消息集（§3.1 的 8 条）
ALL_TYPES = capabilities(
    HELLO,
    HELLO_ACK,
    HEARTBEAT,
    ACK,
    ERROR,
    SYSTEM_STATUS,
    CONTROL_SNAPSHOT,
    GIMBAL_ATTITUDE,
)

# 合法的控制快照：C++ 测试按同样的字段值解码比对
CONTROL_FLOATS = [0.5, -0.25, 1.5, -3.0, 0.125, 0.5, -0.0625, 600.0, 12.5]
CONTROL_PAYLOAD = control(2, 2, 2, 0x0002, CONTROL_FLOATS, 30)


def vectors() -> list[tuple[str, str, bytes]]:
    v: list[tuple[str, str, bytes]] = []

    # 合法帧：MCU → PC（C++ 测试按同样的字段值编码，逐字节比对）
    v.append(("keepalive", "ok", frame(0x1234, 0x89ABCDEF, [])))
    v.append(
        (
            "mcu_handshake",
            "ok",
            frame(
                0,
                1000,
                [
                    slot(HELLO_ACK, 0, hello_ack(7, 2, 0, 20)),
                    slot(
                        HELLO,
                        0,
                        hello(2, 0, 0, [0, 1, 0], 0x1234ABCD, ALL_TYPES, 20, 512, 0),
                    ),
                ],
            ),
        )
    )
    v.append(
        (
            "mcu_status_attitude",
            "ok",
            frame(
                0x0102,
                0xFFFFFF00,
                [
                    slot(
                        SYSTEM_STATUS,
                        0xFFFF,
                        status_core(123456, 7, 0, 0, 41, 0, 0, 65535, 3, 0x80000001),
                    ),
                    slot(
                        GIMBAL_ATTITUDE,
                        5,
                        attitude(
                            0xFFFFFE00,
                            [0.5, -0.5, 0.5, 0.5],
                            [0.125, -2.5, 3.0],
                            1,
                            0x01,
                        ),
                    ),
                ],
            ),
        )
    )
    v.append(
        (
            "mcu_status_full",
            "ok",
            frame(
                1,
                2000,
                [
                    slot(
                        SYSTEM_STATUS,
                        0,
                        status_core(2, 7, 0, 0, 0, 0, 0, 65535, 2, 0)
                        + status_diagnostics(1000, 2000, 3, 4, 5, 6, 7),
                    )
                ],
            ),
        )
    )
    v.append(
        (
            "mcu_attitude_invalid",
            "ok",
            frame(
                9,
                70000,
                [slot(GIMBAL_ATTITUDE, 1, attitude(69000, [0.0] * 4, [0.0] * 3, 0, 0))],
            ),
        )
    )

    # 合法帧：PC → MCU（C++ 测试解码后逐字段比对）
    pc_hello = hello(2, 0, 1, [0, 0, 0], 0, ALL_TYPES, 20, 512, 0)
    v.append(("pc_hello", "ok", frame(0, 5000, [slot(HELLO, 0, pc_hello)])))
    # 旧 PC（没声明 0x30）与主版本不同的 PC：帧合法，握手结果由会话决定
    v.append(
        (
            "pc_hello_without_attitude",
            "ok",
            frame(
                0,
                5000,
                [
                    slot(
                        HELLO,
                        3,
                        hello(
                            2,
                            0,
                            1,
                            [1, 2, 3],
                            0xDEADBEEF,
                            ALL_TYPES & ~(1 << 48),
                            50,
                            96,
                            9,
                        ),
                    )
                ],
            ),
        )
    )
    v.append(
        (
            "pc_hello_major_1",
            "ok",
            frame(
                0,
                5000,
                [slot(HELLO, 0, hello(1, 0, 1, [0, 0, 0], 0, ALL_TYPES, 0, 512, 0))],
            ),
        )
    )
    v.append(
        (
            "hello_bad_device_type",
            "payload",
            frame(
                0,
                5000,
                [slot(HELLO, 0, hello(2, 0, 2, [0, 0, 0], 0, ALL_TYPES, 20, 512, 0))],
            ),
        )
    )
    v.append(
        (
            "pc_control",
            "ok",
            frame(3, 123456789, [slot(CONTROL_SNAPSHOT, 2, CONTROL_PAYLOAD)]),
        )
    )
    # §7.1：载荷末尾追加的字段（如角加速度前馈）旧接收方按长度截断读取
    v.append(
        (
            "pc_control_extended",
            "ok",
            frame(
                4,
                123457789,
                [slot(CONTROL_SNAPSHOT, 3, CONTROL_PAYLOAD + f32(1.0) + f32(-1.0))],
            ),
        )
    )
    v.append(
        ("pc_heartbeat_slot", "ok", frame(5, 123458789, [slot(HEARTBEAT, 0, b"")]))
    )
    v.append(
        (
            "pc_ack",
            "ok",
            frame(6, 123459789, [slot(ACK, 1, ack(SYSTEM_STATUS, 0x0203, 1, 0))]),
        )
    )
    v.append(
        ("pc_error", "ok", frame(7, 123460789, [slot(ERROR, 0, error(2, b"len"))]))
    )
    v.append(
        (
            "pc_hello_ack",
            "ok",
            frame(8, 123461789, [slot(HELLO_ACK, 0, hello_ack(0, 2, 0, 20))]),
        )
    )

    # 消息内容按协议应拒收（帧本身合法）
    bad = control(2, 2, 2, 0x0010, CONTROL_FLOATS, 30)  # §4.7 flags 保留位非 0
    v.append(
        (
            "control_reserved_flags",
            "payload",
            frame(3, 0, [slot(CONTROL_SNAPSHOT, 2, bad)]),
        )
    )
    bad = control(2, 2, 2, 0x0002, CONTROL_FLOATS, 30, reserved=1)  # §4.7 reserved 非 0
    v.append(
        (
            "control_reserved_tail",
            "payload",
            frame(3, 0, [slot(CONTROL_SNAPSHOT, 2, bad)]),
        )
    )
    nan_floats = list(CONTROL_FLOATS)
    nan_floats[3] = math.nan  # gimbal_yaw 要求有限值
    bad = control(2, 2, 2, 0x0002, nan_floats, 30)
    v.append(("control_nan", "payload", frame(3, 0, [slot(CONTROL_SNAPSHOT, 2, bad)])))
    bad = hello_ack(0, 2, 0, 20, reserved=1)  # §4.2 reserved 非 0
    v.append(("hello_ack_reserved", "payload", frame(8, 0, [slot(HELLO_ACK, 0, bad)])))

    # 帧被拒
    good = [slot(CONTROL_SNAPSHOT, 2, CONTROL_PAYLOAD)]
    v.append(("bad_crc", "crc", frame(3, 0, good, corrupt_crc=True)))
    v.append(("bad_version", "header", frame(3, 0, good, version=0x03)))
    v.append(("unknown_frame_flag", "header", frame(3, 0, good, flags=0x04)))
    v.append(("keepalive_with_slots", "header", frame(3, 0, good, flags=KEEPALIVE)))
    v.append(("slots_missing_keepalive", "header", frame(3, 0, [], flags=0)))
    v.append(("header_reserved", "header", frame(3, 0, good, reserved=1)))
    v.append(("length_below_minimum", "header", frame(3, 0, [], length=17)))
    v.append(("length_above_maximum", "header", frame(3, 0, good, length=513)))
    v.append(
        (
            "unknown_type",
            "unknown_type",
            frame(3, 0, [struct.pack("<BBHH", 0x06, 0, 0, 0)]),
        )
    )
    v.append(
        (
            "unknown_slot_flag",
            "slot_flags",
            frame(3, 0, [slot(CONTROL_SNAPSHOT, 2, CONTROL_PAYLOAD, flags=0x12)]),
        )
    )
    v.append(
        (
            "class_mismatch",
            "class",
            frame(3, 0, [slot(CONTROL_SNAPSHOT, 2, CONTROL_PAYLOAD, flags=0x00)]),
        )
    )
    v.append(
        (
            "control_too_short",
            "short",
            frame(3, 0, [slot(CONTROL_SNAPSHOT, 2, CONTROL_PAYLOAD[:47])]),
        )
    )
    v.append(
        (
            "slot_overrun",
            "layout",
            frame(3, 0, [slot(CONTROL_SNAPSHOT, 2, CONTROL_PAYLOAD, length=49)]),
        )
    )
    v.append(("slot_count_short", "layout", frame(3, 0, good + good, slot_count=1)))
    v.append(("slot_count_long", "layout", frame(3, 0, good, slot_count=2)))
    v.append(
        (
            "payload_above_maximum",
            "layout",
            frame(3, 0, [slot(ERROR, 0, b"\x00" * 487)]),
        )
    )
    v.append(
        (
            "ack_flag_missing",
            "ack_flag",
            frame(6, 0, [slot(ACK, 1, ack(0x10, 1, 1, 0))], flags=0),
        )
    )
    v.append(("ack_flag_spurious", "ack_flag", frame(3, 0, good, flags=ACK_PRESENT)))
    return v


def render() -> str:
    lines = [
        "# RoboLink v0.2 标准帧：由 tool/robolink/make_golden.py 生成，不要手改（Decision-20261103-01）。",
        "# 来源：foray_interfaces protocol/robolink.md v0.2（提交 5973436）；CRC 为 zlib.crc32。",
        "# 每行：<名称> <期望> <十六进制字节>",
    ]
    names = set()
    for name, expect, data in vectors():
        if name in names:
            raise ValueError(f"向量名重复：{name}")
        names.add(name)
        lines.append(f"{name} {expect} {data.hex()}")
    return "\n".join(lines) + "\n"


def self_check() -> None:
    """先核对 CRC 参数：CRC-32/ISO-HDLC 对 "123456789" 的校验值是 0xCBF43926（§1.3）。"""
    if zlib.crc32(b"123456789") & 0xFFFFFFFF != 0xCBF43926:
        raise AssertionError("zlib.crc32 不是 CRC-32/ISO-HDLC")
    sizes = {
        "HELLO": (len(hello(2, 0, 0, [0, 0, 0], 0, 0, 20, 512, 0)), 26),
        "HELLO_ACK": (len(hello_ack(0, 2, 0, 20)), 8),
        "ACK": (len(ack(0, 0, 0, 0)), 7),
        "SYSTEM_STATUS": (len(status_core(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)), 28),
        "diagnostics": (len(status_diagnostics(0, 0, 0, 0, 0, 0, 0)), 24),
        "CONTROL_SNAPSHOT": (len(CONTROL_PAYLOAD), 48),
        "GIMBAL_ATTITUDE": (len(attitude(0, [0.0] * 4, [0.0] * 3, 0, 0)), 36),
    }
    for name, (actual, expected) in sizes.items():
        if actual != expected:
            raise AssertionError(f"{name} 载荷 {actual} B，协议规定 {expected} B")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="只核对文件与脚本输出一致")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()

    self_check()
    text = render()
    if args.check:
        if not args.output.exists() or args.output.read_text(encoding="utf-8") != text:
            print(
                f"{args.output} 与生成器输出不一致，运行 tool/robolink/make_golden.py 重新生成",
                file=sys.stderr,
            )
            return 1
        print(f"{args.output}：一致")
        return 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding="utf-8", newline="\n")
    print(f"已写入 {args.output}（{text.count(chr(10)) - 3} 个向量）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
