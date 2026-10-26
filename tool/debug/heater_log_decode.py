#!/usr/bin/env python3
"""解析 swd_poll.py 记录的 BMI088 加热器日志，输出逐行 CSV 与升温指标。

日志中应包含三个区域（顺序可用参数指定）：
- DWT_CYCCNT（0xE0001004，1 个字）：时间基准；
- 加热器对象（Bmi088Heater，40 个字 = 160 字节）：尾部成员布局与
  legacy/Libraries/Device/bmi088/Bmi088Heater.hpp 一致（偏移 128 起：
  状态与标志、目标温度、当前温度、占空比、温度读取次数、两个时间戳）；
  偏移 0–127 在 a2cfd16 中是 PID 对象，在原固件中布局不同，原样输出为
  w00–w31（按 float 解释）；
- 可选：陀螺采集结构头部（ImuDiagnostics Capture，至少 3 个字）。

时间以目标的周期计数器为准（主机时间可能跳变）；"自 IMU 启动"的时间由
温度读取次数对时间做线性拟合外推得到，是估计值。

用法示例：
    heater_log_decode.py heater.log --csv heater.csv --json metrics.json
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import struct
import sys
from pathlib import Path

STATE_NAMES = {0: "关闭", 1: "等待温度", 2: "预热", 3: "恒温调节", 4: "故障"}


def as_float(word: int) -> float:
    return struct.unpack("<f", struct.pack("<I", word & 0xFFFFFFFF))[0]


def load(path: Path, cycle_index: int, heater_index: int, capture_index: int):
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        fields = line.split("|")
        regions = [[int(value, 0) for value in field.split()] for field in fields[1:]]
        cycle = regions[cycle_index][0]
        heater = regions[heater_index]
        capture = regions[capture_index] if 0 <= capture_index < len(regions) else []
        rows.append((cycle, heater, capture))
    return rows


def unwrap_cycles(cycles: list[int], cycle_hz: float) -> list[float]:
    times = [0.0]
    for previous, current in itertools.pairwise(cycles):
        delta = (current - previous) & 0xFFFFFFFF
        times.append(times[-1] + delta / cycle_hz)
    return times


def linear_fit(xs: list[float], ys: list[float]) -> tuple[float, float]:
    count = len(xs)
    mean_x = sum(xs) / count
    mean_y = sum(ys) / count
    sxx = sum((x - mean_x) ** 2 for x in xs)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    slope = sxy / sxx
    return slope, mean_y - slope * mean_x


def first_time(times, values, predicate):
    for time_s, value in zip(times, values):
        if predicate(value):
            return time_s
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log", type=Path)
    parser.add_argument("--cycle-region", type=int, default=0)
    parser.add_argument("--heater-region", type=int, default=1)
    parser.add_argument("--capture-region", type=int, default=2)
    parser.add_argument("--cycle-hz", type=float, default=480e6)
    parser.add_argument("--band", type=float, default=0.25, help="稳定带宽 ±℃")
    parser.add_argument("--steady-window", type=float, default=300.0, help="稳态统计 s")
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()

    rows = load(args.log, args.cycle_region, args.heater_region, args.capture_region)
    if len(rows) < 2:
        print("日志数据不足", file=sys.stderr)
        return 1
    times = unwrap_cycles([row[0] for row in rows], args.cycle_hz)
    reads = [row[1][36] for row in rows]
    rate, intercept = linear_fit(times, reads)
    offset = intercept / rate  # 第一行相对 IMU 启动（读取次数为 0）的时间
    since_start = [time_s + offset for time_s in times]

    records = []
    for time_s, start_s, (_, heater, capture) in zip(times, since_start, rows):
        flags = heater[32]
        record = {
            "t_s": round(time_s, 4),
            "since_imu_start_s": round(start_s, 3),
            "state": flags & 0xFF,
            "initialized": (flags >> 8) & 0xFF,
            "enabled": (flags >> 16) & 0xFF,
            "sample_seen": (flags >> 24) & 0xFF,
            "target_c": as_float(heater[33]),
            "temperature_c": as_float(heater[34]),
            "duty_raw": heater[35] & 0xFFFF,
            "read_count": heater[36],
            "capture_state": capture[1] if len(capture) > 2 else "",
            "capture_count": capture[2] if len(capture) > 2 else "",
        }
        for index in range(32):
            record[f"w{index:02d}"] = round(as_float(heater[index]), 4)
        records.append(record)

    target = records[-1]["target_c"]
    temps = [record["temperature_c"] for record in records]
    states = [record["state"] for record in records]
    peak = max(temps)
    peak_time = since_start[temps.index(peak)]
    outside = [
        start_s
        for start_s, temp in zip(since_start, temps)
        if abs(temp - target) > args.band
    ]
    settle = None
    if not outside:
        settle = since_start[0]
    elif outside[-1] < since_start[-1]:
        later = [start_s for start_s in since_start if start_s > outside[-1]]
        settle = later[0] if later else None
    window_start = since_start[-1] - args.steady_window
    steady = [rec for rec in records if rec["since_imu_start_s"] >= window_start]
    steady_temps = [rec["temperature_c"] for rec in steady]
    steady_duty = [rec["duty_raw"] for rec in steady]
    capture_start = first_time(
        since_start,
        [rec["capture_state"] for rec in records],
        lambda value: value == 1,
    )

    def rounded(value):
        return None if value is None else round(value, 2)

    metrics = {
        "rows": len(records),
        "duration_s": round(times[-1], 2),
        "read_rate_hz": round(rate, 3),
        "first_row_since_imu_start_s": round(offset, 2),
        "target_c": target,
        "start_temperature_c": temps[0],
        "start_state": STATE_NAMES.get(states[0], states[0]),
        "time_to_target_minus_0_5_s": rounded(
            first_time(since_start, temps, lambda t: t >= target - 0.5)
        ),
        "time_to_target_s": rounded(
            first_time(since_start, temps, lambda t: t >= target)
        ),
        "time_to_regulating_s": rounded(
            first_time(since_start, states, lambda s: s == 3)
        ),
        "peak_temperature_c": peak,
        "overshoot_c": round(peak - target, 3),
        "peak_time_s": round(peak_time, 2),
        "band_c": args.band,
        "settle_time_s": rounded(settle),
        "steady_window_s": args.steady_window,
        "steady_mean_c": round(sum(steady_temps) / len(steady_temps), 3),
        "steady_min_c": min(steady_temps),
        "steady_max_c": max(steady_temps),
        "steady_duty_mean": round(sum(steady_duty) / len(steady_duty), 2),
        "steady_duty_min": min(steady_duty),
        "steady_duty_max": max(steady_duty),
        "faults": sum(1 for state in states if state == 4),
        "capture_start_s": rounded(capture_start),
    }

    if args.csv:
        with args.csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle, fieldnames=list(records[0]), lineterminator="\n"
            )
            writer.writeheader()
            writer.writerows(records)
    if args.json:
        args.json.write_text(
            json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
