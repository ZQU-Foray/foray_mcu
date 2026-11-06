#!/usr/bin/env python3
"""单轴的模型辨识：录到的 CSV（EmberProbe 按切片导出）→ J、b₀、库仑摩擦、重力与延迟。

回归式（τ 是下发的总力矩，含重力前馈；每个轴单独做）：
    τ[k−d] = J·ω̇[k] + B·ω[k] + Fc·tanh(ω/ωs) + G·cos θ[k]     ← pitch
    τ[k−d] = J·ω̇[k] + B·ω[k] + Fc·tanh(ω/ωs) + c               ← yaw（c：线缆之类的常值力矩）
ω̇ 用 Savitzky–Golay 求导（窗口 21 点、3 次），ω 用同一窗口平滑，ωs = 0.01 rad/s；
延迟 d 在 0–15 拍（--max-delay）里取 R² 最大的，并在峰值附近抛物线插值、报成小数拍（中心差分的
时间对齐会多算约半拍，实物按需减 0.5）。只用出力模式（HOLD / TRACK / MANUAL）的
样本，进入出力后的前 0.2 s 不要。pitch 不加常数项（cos θ 在小范围里几乎是常数，
和常数项分不开）。B 估不准（粘滞摩擦与库仑摩擦分不开），只作参考。

用法：
    tool/control/identify_axis.py build/capture/pitch_sweep_2hz --axis pitch
    tool/control/identify_axis.py build/capture/pitch_sweep_* --axis pitch   # 多段合起来拟合
    tool/control/identify_axis.py --simulate --axis pitch

输入：每个参数是一个目录（里面是 EmberProbe 按切片导出的 CSV）或单个 CSV 文件。
脚本按 `index` 列把切片拼起来，检查没有缺口；列名按成员名匹配（允许前缀，如
`g_gimbal_capture[0].pitch_rad`）。第一次导出后要核对实际的列名。
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter

# 云台 Mode 的数值：DISABLED 0、ENABLING 1、HOLD 2、TRACK 3、MANUAL 4
OUTPUT_MODES = (2, 3, 4)
SETTLE_SECONDS = 0.2
SAVGOL_WINDOW = 21
SAVGOL_POLY = 3
FRICTION_SPEED = 0.01
# 步兵 yaw（同步带）约 9 拍、pitch 约 5 拍；峰值落在上限上时 report() 会提示加大 --max-delay
MAX_DELAY_SAMPLES = 15
# 中心差分求导的时间对齐比实际多算半拍（见脚本开头），自检时先减掉
HALF_SAMPLE = 0.5
# 真值（仿真自检用）：DM-J4310 云台量级
TRUE_J = 0.005
TRUE_B = 0.02
TRUE_FC = 0.2
TRUE_G = 0.5
TRUE_C = 0.05
TRUE_DELAY = 2


@dataclass
class Segment:
    """一段录制（或仿真出来的）数据，时间已经换到 s 并从 0 开始。"""

    time_s: np.ndarray
    angle: np.ndarray
    rate: np.ndarray
    torque: np.ndarray
    feedback: np.ndarray
    mode: np.ndarray
    dt: float = 0.0
    usable: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))


def find_column(header: list[str], suffix: str) -> str:
    """按成员名找列：允许带前缀（EmberProbe 可能导出 `g_gimbal_capture[3].pitch_rad` 这种名字）。"""
    matches = [name for name in header if name.strip().endswith(suffix)]
    if not matches:
        raise SystemExit(
            f"CSV 里找不到列 {suffix}；现有的列：{', '.join(header)}（第一次导出后核对实际列名）"
        )
    return matches[0]


def read_csv_file(
    path: Path, axis: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """读一个切片，返回 index、time_us、angle、rate、torque、feedback、mode。"""
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise SystemExit(f"{path} 没有表头")
        header = list(reader.fieldnames)
        columns = {
            "index": find_column(header, "index"),
            "time_us": find_column(header, "time_us"),
            "angle": find_column(header, f"{axis}_rad"),
            "rate": find_column(header, f"{axis}_rate"),
            "torque": find_column(header, f"{axis}_torque_newton_meter"),
            "feedback": find_column(header, f"{axis}_feedback_torque_newton_meter"),
            "mode": find_column(header, "mode"),
        }
        values: dict[str, list[float]] = {key: [] for key in columns}
        for row in reader:
            if row[columns["index"]] is None or row[columns["index"]].strip() == "":
                continue
            for key, column in columns.items():
                values[key].append(float(row[column]))
    return tuple(
        np.array(values[key])
        for key in ("index", "time_us", "angle", "rate", "torque", "feedback", "mode")
    )


def load_segment(path: Path, axis: str) -> Segment:
    files = sorted(path.glob("*.csv")) if path.is_dir() else [path]
    if not files:
        raise SystemExit(f"{path} 里没有 CSV")
    blocks = [read_csv_file(item, axis) for item in files]
    index = np.concatenate([block[0] for block in blocks])
    time_us = np.concatenate([block[1] for block in blocks])
    angle = np.concatenate([block[2] for block in blocks])
    rate = np.concatenate([block[3] for block in blocks])
    torque = np.concatenate([block[4] for block in blocks])
    feedback = np.concatenate([block[5] for block in blocks])
    mode = np.concatenate([block[6] for block in blocks])

    order = np.argsort(index)
    index = index[order]
    time_us = time_us[order]
    angle = angle[order]
    rate = rate[order]
    torque = torque[order]
    feedback = feedback[order]
    mode = mode[order]

    if np.any(np.diff(index) != 1):
        missing = int(np.sum(np.diff(index) != 1))
        raise SystemExit(
            f"{path} 的切片按 index 拼起来有 {missing} 处缺口（少导出了切片？）"
        )

    # time_us 是 32 位、按 us 计；过一圈要接着算
    raw = time_us.astype(np.int64)
    steps = np.diff(raw)
    steps[steps < 0] += 1 << 32
    absolute_us = np.concatenate([[0], np.cumsum(steps)]) + int(raw[0])
    time_s = (absolute_us - absolute_us[0]) * 1e-6
    return Segment(
        time_s=time_s,
        angle=angle,
        rate=rate,
        torque=torque,
        feedback=feedback,
        mode=np.rint(mode).astype(int),
    )


def prepare(segment: Segment, name: str = "") -> Segment:
    """换成均匀采样（dt 取中位数）、算出可用样本。仿真数据已经是均匀的。"""
    if len(segment.time_s) < SAVGOL_WINDOW + MAX_DELAY_SAMPLES:
        raise SystemExit(f"{name or '这一段'} 的样本太少（{len(segment.time_s)} 条）")
    steps = np.diff(segment.time_s)
    if not np.all(steps > 0.0):
        raise SystemExit(f"{name or '这一段'} 的 time_us 没有严格递增")
    dt = float(np.median(steps))
    if not 0.0 < dt <= 0.05:
        raise SystemExit(f"{name or '这一段'} 的采样周期不正常（{dt * 1e3:.3f} ms）")
    if segment.dt != 0.0:
        grid = segment.time_s  # 仿真数据已经均匀
    else:
        grid = np.arange(segment.time_s[0], segment.time_s[-1], dt)
        if len(grid) > 4 * len(segment.time_s):
            raise SystemExit(
                f"{name or '这一段'} 的 time_us 有跳变（同一段录制里时间不能重置）："
                f"{len(segment.time_s)} 条样本对应 {len(grid)} 个均匀采样点"
            )
    angle = np.interp(grid, segment.time_s, segment.angle)
    rate = np.interp(grid, segment.time_s, segment.rate)
    torque = np.interp(grid, segment.time_s, segment.torque)
    feedback = np.interp(grid, segment.time_s, segment.feedback)
    mode = np.rint(np.interp(grid, segment.time_s, segment.mode)).astype(int)
    return Segment(
        time_s=grid,
        angle=angle,
        rate=rate,
        torque=torque,
        feedback=feedback,
        mode=mode,
        dt=dt,
        usable=usable_mask(grid, mode),
    )


def usable_mask(time_s: np.ndarray, mode: np.ndarray) -> np.ndarray:
    """出力模式下、并且进入出力已经超过 0.2 s 的样本。"""
    usable = np.zeros(len(time_s), dtype=bool)
    entered_s: float | None = None
    for index in range(len(time_s)):
        if mode[index] in OUTPUT_MODES:
            if entered_s is None:
                entered_s = float(time_s[index])
            if time_s[index] - entered_s >= SETTLE_SECONDS:
                usable[index] = True
        else:
            entered_s = None
    return usable


def design_and_target(
    segment: Segment, axis: str, delay: int
) -> tuple[np.ndarray, np.ndarray]:
    """一个延迟下的回归矩阵与目标。导数在均匀网格上算。"""
    rate_smooth = savgol_filter(segment.rate, SAVGOL_WINDOW, SAVGOL_POLY, deriv=0)
    rate_dot = savgol_filter(
        segment.rate, SAVGOL_WINDOW, SAVGOL_POLY, deriv=1, delta=segment.dt
    )
    count = len(segment.rate)
    half = SAVGOL_WINDOW // 2
    valid = np.zeros(count, dtype=bool)
    valid[half : count - half] = True
    valid &= segment.usable
    delayed_usable = np.zeros(count, dtype=bool)
    delayed_usable[delay:] = (
        segment.usable[: count - delay] if delay > 0 else segment.usable
    )
    mask = valid & delayed_usable
    index = np.nonzero(mask)[0]
    columns = [
        rate_dot[index],
        rate_smooth[index],
        np.tanh(rate_smooth[index] / FRICTION_SPEED),
    ]
    if axis == "pitch":
        columns.append(np.cos(segment.angle[index]))
    else:
        columns.append(np.ones(len(index)))
    return np.column_stack(columns), segment.torque[index - delay]


def fit(segments: list[Segment], axis: str, delay: int) -> dict:
    matrices = []
    targets = []
    for segment in segments:
        matrix, target = design_and_target(segment, axis, delay)
        if len(target) == 0:
            continue
        matrices.append(matrix)
        targets.append(target)
    if not matrices:
        return {"valid": False, "r2": -math.inf, "residual_rms": math.inf, "count": 0}
    matrix = np.vstack(matrices)
    target = np.concatenate(targets)
    coefficients, _, _, _ = np.linalg.lstsq(matrix, target, rcond=None)
    predicted = matrix @ coefficients
    residual = target - predicted
    total = float(np.sum((target - target.mean()) ** 2))
    r2 = 1.0 - float(np.sum(residual**2)) / total if total > 0.0 else -math.inf
    return {
        "valid": True,
        "coefficients": coefficients,
        "r2": r2,
        "residual_rms": float(np.sqrt(np.mean(residual**2))),
        "count": len(target),
    }


def fit_feedback(segments: list[Segment], max_delay: int = MAX_DELAY_SAMPLES) -> dict:
    """电机反馈力矩对下发力矩的比例与延迟（看电机的力矩跟得怎么样）。"""
    best: dict = {"valid": False, "r2": -math.inf}
    for delay in range(max_delay + 1):
        ratios = []
        targets = []
        predictions = []
        usable = []
        for segment in segments:
            count = len(segment.torque)
            mask = np.zeros(count, dtype=bool)
            mask[delay:] = (
                segment.usable[: count - delay] if delay > 0 else segment.usable
            )
            mask &= segment.usable
            index = np.nonzero(mask)[0]
            if len(index) == 0:
                continue
            command = segment.torque[index - delay]
            feedback = segment.feedback[index]
            denominator = float(np.dot(command, command))
            if denominator <= 0.0:
                continue
            ratio = float(np.dot(feedback, command) / denominator)
            ratios.append(ratio)
            targets.append(feedback)
            predictions.append(ratio * command)
            usable.append(len(index))
        if not targets:
            continue
        target = np.concatenate(targets)
        predicted = np.concatenate(predictions)
        total = float(np.sum((target - target.mean()) ** 2))
        r2 = (
            1.0 - float(np.sum((target - predicted) ** 2)) / total
            if total > 0.0
            else -math.inf
        )
        if r2 > best["r2"]:
            best = {
                "valid": True,
                "ratio": float(np.mean(ratios)),
                "delay": delay,
                "r2": r2,
                "count": int(sum(usable)),
            }
    return best


def interpolate_delay(scans: dict[int, dict], delay: int, max_delay: int) -> float:
    """在 R² 峰值附近做抛物线插值，延迟报成小数拍（峰值在两端时不插值）。"""
    if delay <= 0 or delay >= max_delay:
        return float(delay)
    previous = scans[delay - 1]["r2"]
    peak = scans[delay]["r2"]
    following = scans[delay + 1]["r2"]
    denominator = previous - 2.0 * peak + following
    if denominator == 0.0 or not math.isfinite(denominator):
        return float(delay)
    offset = 0.5 * (previous - following) / denominator
    return float(delay + max(-0.5, min(0.5, offset)))


def identify(
    segments: list[Segment], axis: str, max_delay: int = MAX_DELAY_SAMPLES
) -> dict:
    scans = {delay: fit(segments, axis, delay) for delay in range(max_delay + 1)}
    if not any(scan["valid"] for scan in scans.values()):
        modes = sorted({int(mode) for segment in segments for mode in segment.mode})
        raise SystemExit(
            "没有可用样本：只用出力模式（HOLD 2 / TRACK 3 / MANUAL 4）、进入出力 "
            f"{SETTLE_SECONDS:g} s 以后的样本，这些录制里出现的模式是 {modes}。"
            "检查录制时云台是不是在出力。"
        )
    delay = max(scans, key=lambda value: scans[value]["r2"])
    delay_samples = interpolate_delay(scans, delay, max_delay)
    result = dict(scans[delay])
    result["delay"] = delay
    result["max_delay"] = max_delay
    result["delay_samples"] = delay_samples
    result["scans"] = scans
    coefficients = result["coefficients"]
    result["j"] = float(coefficients[0])
    result["b"] = float(coefficients[1])
    result["fc"] = float(coefficients[2])
    if axis == "pitch":
        result["g"] = float(coefficients[3])
    else:
        result["c"] = float(coefficients[3])
    result["axis"] = axis
    dt = float(np.median([segment.dt for segment in segments]))
    result["dt"] = dt
    result["b0"] = 1.0 / result["j"] if result["j"] != 0.0 else math.inf
    dely_s = delay_samples * dt
    result["delay_s"] = dely_s
    result["bandwidth_limit"] = 0.5 / (dely_s + 0.0005)
    result["feedback"] = fit_feedback(segments, max_delay)
    return result


def report(result: dict, gravity_feedforward: float | None) -> None:
    scans = result["scans"]
    print(
        "延迟扫描（R²）："
        + "、".join(f"d={delay} {scans[delay]['r2']:.4f}" for delay in sorted(scans))
    )
    print(
        f"选定延迟：{result['delay']} 拍（峰值附近插值 {result['delay_samples']:.2f} 拍，"
        f"{result['delay_s'] * 1e3:.2f} ms；中心差分求导多算约半拍），"
        f"R² = {result['r2']:.5f}，残差 RMS = {result['residual_rms']:.4f} N·m（{result['count']} 个样本）"
    )
    if result["delay"] >= result["max_delay"]:
        print(
            f"注意：R² 的峰值落在扫描上限 {result['max_delay']} 拍上，真实延迟可能更大，"
            "加大 --max-delay 再算"
        )
    print(f"J = {result['j']:.6g} kg·m² → b₀ = 1/J = {result['b0']:.6g} rad/s² 每 N·m")
    print(
        f"B = {result['b']:.6g} N·m·s/rad（不可靠：粘滞摩擦与库仑摩擦分不开，不用它）"
    )
    print(f"Fc = {result['fc']:.6g} N·m")
    if result["axis"] == "pitch":
        print(f"G = {result['g']:.6g} N·m（cos θ 的系数）")
        if gravity_feedforward is not None and abs(result["g"]) > 0.0:
            difference = abs(result["g"] - gravity_feedforward) / abs(result["g"])
            if difference > 0.1:
                print(
                    f"注意：G 与 params.hpp 的 pitch_gravity_newton_meter"
                    f"（{gravity_feedforward:.6g}）相差 {difference:.1%}，先改那个参数（PID 也受益）"
                )
    else:
        print(f"c = {result['c']:.6g} N·m（线缆之类的常值力矩）")
    feedback = result["feedback"]
    if feedback["valid"]:
        print(
            f"电机反馈力矩 / 下发力矩：比例 {feedback['ratio']:.3f}，"
            f"延迟 {feedback['delay']} 拍，R² {feedback['r2']:.4f}"
        )
    print(
        f"建议值：b₀ = {result['b0']:.6g}；ω₀ 上限 = 0.5/(延迟 + 0.5 ms) = "
        f"{result['bandwidth_limit']:.0f} rad/s"
    )


def simulate_segment(
    axis: str, frequency: float, seed: int, seconds: float = 4.0
) -> Segment:
    """闭环仿真：PI 角速度环 + P 角度环跟踪正弦目标，力矩延迟 2 拍、带噪声。"""
    rng = np.random.default_rng(seed)
    dt = 1e-3
    count = int(seconds / dt)
    time_s = np.arange(count) * dt
    target = 0.15 * np.sin(2.0 * math.pi * frequency * time_s)
    inertia, damping, friction, friction_speed = TRUE_J, TRUE_B, TRUE_FC, FRICTION_SPEED
    gravity = TRUE_G if axis == "pitch" else 0.0
    constant = TRUE_C if axis == "yaw" else 0.0
    angle = 0.0
    rate = 0.0
    integral = 0.0
    pending = [0.0] * TRUE_DELAY
    angles = np.empty(count)
    rates = np.empty(count)
    torques = np.empty(count)
    applied_history = np.empty(count)
    for cycle in range(count):
        angle_measured = angle + rng.normal(0.0, 1.0e-4)
        rate_measured = rate + rng.normal(0.0, 0.01)
        rate_target = np.clip(30.0 * (target[cycle] - angle_measured), -6.0, 6.0)
        integral += (rate_target - rate_measured) * dt
        torque_value = np.clip(
            0.6 * (rate_target - rate_measured) + 3.0 * integral, -1.0, 1.0
        )
        applied = pending[0]
        pending = pending[1:] + [float(torque_value)]
        # c 是负载（要由 τ 补掉），与回归式里的 +c 同号
        acceleration = (
            applied
            - constant
            - damping * rate
            - friction * np.tanh(rate / friction_speed)
            - gravity * math.cos(angle)
        ) / inertia
        rate += acceleration * dt
        angle += rate * dt
        angles[cycle] = angle_measured
        rates[cycle] = rate_measured
        torques[cycle] = torque_value
        applied_history[cycle] = applied
    feedback = np.empty(count)
    feedback[0] = 0.97 * 0.0
    feedback[1:] = 0.97 * torques[:-1]  # 反馈比下发晚一拍
    return Segment(
        time_s=time_s,
        angle=angles,
        rate=rates,
        torque=torques,
        feedback=feedback,
        mode=np.full(count, 2),
    )


def selftest(axis: str, seed: int) -> int:
    segments = [
        prepare(simulate_segment(axis, frequency, seed), f"{frequency:g} Hz")
        for frequency in (1.0, 2.0, 4.0)
    ]
    result = identify(segments, axis)
    print(
        f"仿真（{axis}，三段的真值 J = {TRUE_J}、B = {TRUE_B}、Fc = {TRUE_FC}、延迟 {TRUE_DELAY} 拍）"
    )
    report(result, None)
    failures = 0

    def check(name: str, ok: bool) -> None:
        nonlocal failures
        print(f"{'通过' if ok else '失败'} {name}")
        failures += 0 if ok else 1

    check(
        f"J 在 ±5% 内（{result['j']:.6g}）", abs(result["j"] - TRUE_J) / TRUE_J <= 0.05
    )
    check(
        f"Fc 在 ±15% 内（{result['fc']:.6g}）",
        abs(result["fc"] - TRUE_FC) / TRUE_FC <= 0.15,
    )
    if axis == "pitch":
        check(
            f"G 在 ±5% 内（{result['g']:.6g}）",
            abs(result["g"] - TRUE_G) / TRUE_G <= 0.05,
        )
    else:
        check(
            f"c 在 ±15% 内（{result['c']:.6g}）",
            abs(result["c"] - TRUE_C) / TRUE_C <= 0.15,
        )
    # 求导用中心差分，时间对齐会多算约半拍：先减掉再比，否则 ±1 拍的条件
    # 对 2 拍的真值几乎不起作用（2、3 拍都算过）。
    physical_delay = result["delay_samples"] - HALF_SAMPLE
    check(
        f"延迟（插值 {result['delay_samples']:.2f} 拍，减半拍后 {physical_delay:.2f}）"
        f"在真值 ±0.5 拍内",
        abs(physical_delay - TRUE_DELAY) <= 0.5,
    )
    idle = simulate_segment(axis, 1.0, seed)
    idle.mode[:] = 0  # 全程 DISABLED：没有可用样本
    try:
        identify([prepare(idle, "不出力")], axis)
        check("不出力的数据给出清楚的提示（没有提示）", False)
    except SystemExit as error:
        check("不出力的数据给出清楚的提示", "没有可用样本" in str(error))
    return 0 if failures == 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "segments",
        nargs="*",
        type=Path,
        help="每段一个目录（里面是导出的 CSV 切片）或单个 CSV",
    )
    parser.add_argument(
        "--axis", choices=("pitch", "yaw"), required=True, help="辨识哪个轴"
    )
    parser.add_argument(
        "--simulate", action="store_true", help="不读文件，自己生成仿真数据并检查精度"
    )
    parser.add_argument(
        "--seed", type=int, default=7, help="--simulate 的随机种子（默认 7）"
    )
    parser.add_argument(
        "--max-delay",
        type=int,
        default=MAX_DELAY_SAMPLES,
        help=f"延迟扫描的上限（拍，默认 {MAX_DELAY_SAMPLES}）",
    )
    parser.add_argument(
        "--gravity-feedforward",
        type=float,
        help="params.hpp 的 pitch_gravity_newton_meter，用来提示 G 偏了多少",
    )
    args = parser.parse_args()
    if args.max_delay < 2:
        print("--max-delay 至少是 2（峰值两边都要有点才能插值）")
        return 2

    if args.simulate:
        return selftest(args.axis, args.seed)
    if not args.segments:
        print("要给至少一段录制（目录或 CSV），或者用 --simulate")
        return 2
    segments = [
        prepare(load_segment(path, args.axis), str(path)) for path in args.segments
    ]
    print(
        f"{args.axis}：{len(segments)} 段，共 {sum(len(segment.time_s) for segment in segments)} 个样本"
    )
    report(identify(segments, args.axis, args.max_delay), args.gravity_feedforward)
    return 0


if __name__ == "__main__":
    sys.exit(main())
