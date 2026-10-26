#!/usr/bin/env python3
"""解码 ImuDiagnostics 的陀螺采集存档（Capture，版本 2）。

结构定义见 legacy/Application/Imu/ImuDiagnostics.cpp（仅 Debug 构建，
IMU_ENABLE_DIAGNOSTICS）：温度在恒温点 ±0.5 ℃ 内稳定 3 s 后，统计 20000 个
陀螺样本，并保存前 1024 个原始样本，然后冻结（state=2）。

输入可以是整块 RAM 快照（配合 --base 与 --address），也可以是单独提取的
16720 字节结构体。不指定 --address 时在输入中搜索候选。

用法示例：
    imu_capture_decode.py axi_0x24000000.bin --base 0x24000000 --find
    imu_capture_decode.py axi_0x24000000.bin --base 0x24000000 \\
        --address 0x24000788 --extract capture.bin --csv samples.csv \\
        --json summary.json
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import sys
from pathlib import Path

CAPTURE_SIZE = 16720
CAPTURE_VERSION = 2
STORED_SAMPLES = 1024
RECORD_OFFSET = 144
RECORD_SIZE = 16
CHANNEL_OFFSET = 16528
CHANNEL_SIZE = 96
STATE_NAMES = {0: "等待温度", 1: "记录中", 2: "已冻结"}


def parse_int(text: str) -> int:
    return int(text, 0)


def looks_like_capture(blob: bytes, offset: int) -> bool:
    if offset + CAPTURE_SIZE > len(blob):
        return False
    version, state, count, stored = struct.unpack_from("<4I", blob, offset)
    if version != CAPTURE_VERSION or state > 2 or stored > STORED_SAMPLES:
        return False
    if count < stored:
        return False
    minimum = struct.unpack_from("<3h", blob, offset + 120)
    maximum = struct.unpack_from("<3h", blob, offset + 126)
    untouched = minimum == (32767,) * 3 and maximum == (-32768,) * 3
    filled = count > 0 and all(a <= b for a, b in zip(minimum, maximum))
    return untouched or filled


def decode(blob: bytes) -> dict:
    head = struct.unpack_from("<16I", blob, 0)
    (
        version,
        state,
        count,
        stored,
        tick_hz,
        first_tick,
        last_tick,
        sequence_gaps,
        min_interval,
        max_interval,
        drdy_start,
        drdy_end,
        coalesced_start,
        coalesced_end,
        errors_start,
        errors_end,
    ) = head
    temperature_start, temperature_end = struct.unpack_from("<2f", blob, 64)
    sums = struct.unpack_from("<3q", blob, 72)
    squares = struct.unpack_from("<3Q", blob, 96)
    minimum = struct.unpack_from("<3h", blob, 120)
    maximum = struct.unpack_from("<3h", blob, 126)
    (last_sequence,) = struct.unpack_from("<I", blob, 132)
    (elapsed_ticks,) = struct.unpack_from("<Q", blob, 136)

    axes = {}
    for index, name in enumerate("xyz"):
        if count:
            mean = sums[index] / count
            variance = max(squares[index] / count - mean * mean, 0.0)
            std = math.sqrt(variance)
        else:
            mean = std = float("nan")
        axes[name] = {
            "mean_lsb": mean,
            "std_lsb": std,
            "min_lsb": minimum[index],
            "max_lsb": maximum[index],
        }

    channels = {}
    for index, name in enumerate(("accelerometer", "gyroscope")):
        values = struct.unpack_from(
            "<17I3Q", blob, CHANNEL_OFFSET + index * CHANNEL_SIZE
        )
        keys = (
            "count",
            "read_gaps",
            "drdy_gaps",
            "last_read",
            "last_drdy",
            "last_tick",
            "coalesced_start",
            "coalesced_end",
            "errors_start",
            "errors_end",
            "overflow_start",
            "overflow_end",
            "high_water",
            "late_starts",
            "ambiguous",
            "max_start_ticks",
            "max_complete_ticks",
            "max_harvest_ticks",
            "elapsed_ticks",
            "sum_start_ticks",
            "sum_complete_ticks",
        )
        channels[name] = dict(zip(keys, values))

    elapsed_s = elapsed_ticks / tick_hz if tick_hz else float("nan")
    rate_hz = (count - 1) / elapsed_s if count > 1 and elapsed_s > 0 else float("nan")
    return {
        "version": version,
        "state": state,
        "state_name": STATE_NAMES.get(state, "未知"),
        "count": count,
        "stored": stored,
        "tick_hz": tick_hz,
        "first_tick": first_tick,
        "last_tick": last_tick,
        "elapsed_ticks": elapsed_ticks,
        "elapsed_s": elapsed_s,
        "rate_hz": rate_hz,
        "sequence_gaps": sequence_gaps,
        "min_interval_ticks": min_interval,
        "max_interval_ticks": max_interval,
        "last_sequence": last_sequence,
        "drdy": [drdy_start, drdy_end],
        "coalesced": [coalesced_start, coalesced_end],
        "errors": [errors_start, errors_end],
        "temperature_c": [temperature_start, temperature_end],
        "axes": axes,
        "channels": channels,
    }


def records(blob: bytes, stored: int) -> list[tuple[int, int, int, int, int, int]]:
    result = []
    for index in range(stored):
        offset = RECORD_OFFSET + index * RECORD_SIZE
        result.append(struct.unpack_from("<IIhhhH", blob, offset))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input", type=Path, help="RAM 快照或提取的结构体")
    parser.add_argument("--base", type=parse_int, default=0, help="输入文件首字节地址")
    parser.add_argument("--address", type=parse_int, help="Capture 结构体地址")
    parser.add_argument("--find", action="store_true", help="只搜索候选地址")
    parser.add_argument("--extract", type=Path, help="保存原始结构体")
    parser.add_argument("--csv", type=Path, help="保存原始样本 CSV")
    parser.add_argument("--json", type=Path, help="保存统计摘要 JSON")
    args = parser.parse_args()

    blob = args.input.read_bytes()
    if args.find or args.address is None:
        found = [
            args.base + offset
            for offset in range(0, len(blob) - CAPTURE_SIZE + 1, 4)
            if looks_like_capture(blob, offset)
        ]
        for address in found:
            capture = decode(blob[address - args.base :])
            print(
                f"0x{address:08x} state={capture['state']} count={capture['count']}"
                f" stored={capture['stored']} tick_hz={capture['tick_hz']}"
            )
        if args.find or not found:
            return 0 if found else 1
        args.address = found[0]

    offset = args.address - args.base
    if offset < 0 or offset + CAPTURE_SIZE > len(blob):
        print("地址超出输入范围", file=sys.stderr)
        return 1
    capture_blob = blob[offset : offset + CAPTURE_SIZE]
    if not looks_like_capture(capture_blob, 0):
        print("该地址处不是版本 2 的 Capture 结构体", file=sys.stderr)
        return 1

    summary = decode(capture_blob)
    summary["address"] = f"0x{args.address:08x}"
    if args.extract:
        args.extract.write_bytes(capture_blob)
    if args.csv:
        with args.csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(["tick", "sequence", "gx_lsb", "gy_lsb", "gz_lsb", "flags"])
            writer.writerows(records(capture_blob, summary["stored"]))
    if args.json:
        args.json.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
