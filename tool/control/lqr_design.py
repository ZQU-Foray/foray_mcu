#!/usr/bin/env python3
"""线性二次型（LQR）状态反馈的设计工具：模型 + Q、R → K。

只在本机跑（需要 numpy、scipy），板上不解 Riccati 方程：K 算好以后填进
params.hpp。输入是 models.py 里的模型名与连续时间的 Q、R（对角元）；工具做
零阶保持离散化、解离散代数 Riccati 方程（DARE），打印 K、闭环极点，以及
一段能直接粘贴的 C++。云台用 double-integrator（加了 ESO 补偿以后的单轴）。

用法：
    tool/control/lqr_design.py double-integrator --dt 0.001 --q 2.56e6,0 --r 1
    tool/control/lqr_design.py double-integrator --dt 0.001 --bandwidth 40 --damping 0.707
    tool/control/lqr_design.py --selftest

--bandwidth/--damping 只对 double-integrator 有效：按连续时间的闭式解反算 Q
（q_θ = ω_n⁴·r，q_ω = (4ζ² − 2)·ω_n²·r），r 固定 1。
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.linalg import expm, solve_discrete_are

sys.path.insert(0, str(Path(__file__).resolve().parent))
from models import MODELS, Model

SQRT_HALF = 0.7071067811865476


def parse_diagonal(
    text: str, count: int, name: str, require_positive: bool = False
) -> np.ndarray:
    """把 "2.56e6,0" 这样的逗号分隔值读成对角向量；个数必须与状态（或输入）数一致。

    Q 的对角元可以有个别为 0（只罚某些状态），R 的每个对角元都必须大于 0
    （输入不允许"免费"）。
    """
    try:
        values = [float(item) for item in text.replace(" ", "").split(",")]
    except ValueError:
        raise SystemExit(f"{name} 的写法是逗号分隔的数，例如 2.56e6,0") from None
    if len(values) != count:
        raise SystemExit(f"{name} 要有 {count} 个对角元，现在给了 {len(values)} 个")
    if require_positive and any(value <= 0.0 for value in values):
        raise SystemExit(f"{name} 的对角元必须都大于 0（输入不能没有代价）")
    if not require_positive and any(value < 0.0 for value in values):
        raise SystemExit(f"{name} 的对角元不能为负")
    if all(value == 0.0 for value in values):
        raise SystemExit(f"{name} 不能全为 0")
    return np.array(values)


def discretize(model: Model, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """零阶保持离散化：e^([[A, B], [0, 0]]·dt) 的左上块是 Ad、右上块是 Bd。"""
    states = model.a.shape[0]
    inputs = model.b.shape[1]
    augment = np.zeros((states + inputs, states + inputs))
    augment[:states, :states] = model.a
    augment[:states, states:] = model.b
    exponential = expm(augment * dt)
    return exponential[:states, :states], exponential[:states, states:]


def solve_discrete(
    ad: np.ndarray, bd: np.ndarray, q: np.ndarray, r: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """解离散 DARE，返回 P 与 K = (R + Bdᵀ·P·Bd)⁻¹·Bdᵀ·P·Ad。"""
    p = solve_discrete_are(ad, bd, np.diag(q), np.diag(r))
    k = np.linalg.solve(np.diag(r) + bd.T @ p @ bd, bd.T @ p @ ad)
    return p, k


def dare_relative_residual(
    ad: np.ndarray, bd: np.ndarray, q: np.ndarray, r: np.ndarray, p: np.ndarray
) -> float:
    """把 P 代回 DARE 的残差，按 ‖P‖ 归一化。"""
    r_matrix = np.diag(r)
    residual = (
        ad.T @ p @ ad
        - p
        - ad.T @ p @ bd @ np.linalg.solve(r_matrix + bd.T @ p @ bd, bd.T @ p @ ad)
        + np.diag(q)
    )
    return float(np.linalg.norm(residual) / np.linalg.norm(p))


def continuous_poles(
    ad: np.ndarray, bd: np.ndarray, k: np.ndarray, dt: float
) -> tuple[np.ndarray, np.ndarray]:
    """闭环极点：z 平面的，以及换到 s 平面的 s = ln(z)/dt（负实部表示稳定）。"""
    discrete = np.linalg.eigvals(ad - bd @ k)
    return discrete, np.log(discrete.astype(complex)) / dt


def dominant_bandwidth_damping(poles: np.ndarray) -> tuple[float, float]:
    """主导的一对极点（|s| 最小的两个）的 ω_n 与 ζ。

    复极点对：ω_n = |s|、ζ = −Re/|s|。两个实极点（过阻尼）：ω_n = √(p₁·p₂)、
    ζ = −(p₁+p₂)/(2·ω_n)——这样 ζ > 1 时报出的仍是设计时给的阻尼，而不是恒等于 1；
    ω_n 也不再只是那个慢极点。

    注意：这里把 |s| 最小的两个极点当成一对。云台只有两个状态，正好就是那一对；
    以后轮腿的状态多了，这两个可能不是同一个模态（比如一个实极点加一个复极点对里的
    一个），那时要按模态配对，或者逐个列出来、不做合并。
    """
    if len(poles) == 0:
        return 0.0, 1.0
    ordered = poles[np.argsort(np.abs(poles))]
    first = ordered[0]
    if len(ordered) >= 2:
        second = ordered[1]
        if abs(first.imag) < 1.0e-9 and abs(second.imag) < 1.0e-9:
            product = float(first.real * second.real)
            if product > 0.0:
                speed = float(np.sqrt(product))
                return speed, float(-(first.real + second.real) / (2.0 * speed))
    speed = float(abs(first))
    if speed <= 0.0:
        return 0.0, 1.0
    return speed, float(-first.real / speed)


def format_value(value: float) -> str:
    """6 位有效数字。"""
    return f"{value:.6g}"


def format_pole(pole: complex) -> str:
    return f"{pole.real:.6g}{pole.imag:+.6g}j"


def command_line(
    model_name: str,
    dt: float,
    q: np.ndarray | None,
    r: np.ndarray | None,
    bandwidth: float | None,
    damping: float | None,
) -> str:
    """生成能原样重跑这条结果的命令行（写进 C++ 注释）。"""
    parts = ["tool/control/lqr_design.py", model_name, f"--dt {dt:g}"]
    if bandwidth is not None:
        parts.append(f"--bandwidth {bandwidth:g}")
        parts.append(f"--damping {damping:g}")
    else:
        parts.append("--q " + ",".join(f"{value:g}" for value in q))
        parts.append("--r " + ",".join(f"{value:g}" for value in r))
    return " ".join(parts)


def gain_name(model: Model, input_index: int, state_index: int) -> str:
    name = f"{model.states[state_index]}_gain"
    return name if len(model.inputs) == 1 else f"{model.inputs[input_index]}_{name}"


def bandwidth_to_q(bandwidth: float, damping: float, r: float = 1.0) -> np.ndarray:
    """双积分器的闭式解反算 Q：q_θ = ω_n⁴·r，q_ω = (4ζ² − 2)·ω_n²·r（ζ ≥ 0.707）。"""
    if bandwidth <= 0.0:
        raise SystemExit("--bandwidth 必须大于 0")
    if damping < SQRT_HALF - 1.0e-3:
        raise SystemExit("--damping 要大于等于 0.707（再小 q_ω 会变成负数）")
    q_angle = bandwidth**4 * r
    q_rate = max(0.0, (4.0 * damping**2 - 2.0) * bandwidth**2 * r)
    return np.array([q_angle, q_rate])


@dataclass(frozen=True)
class Design:
    model: Model
    dt: float
    q: np.ndarray
    r: np.ndarray
    k: np.ndarray
    poles_z: np.ndarray
    poles_s: np.ndarray
    residual: float

    @property
    def bandwidth(self) -> float:
        return dominant_bandwidth_damping(self.poles_s)[0]

    @property
    def damping(self) -> float:
        return dominant_bandwidth_damping(self.poles_s)[1]


def design(model_name: str, dt: float, q: np.ndarray, r: np.ndarray) -> Design:
    model = MODELS[model_name]()
    ad, bd = discretize(model, dt)
    p, k = solve_discrete(ad, bd, q, r)
    poles_z, poles_s = continuous_poles(ad, bd, k, dt)
    return Design(
        model=model,
        dt=dt,
        q=q,
        r=r,
        k=k,
        poles_z=poles_z,
        poles_s=poles_s,
        residual=dare_relative_residual(ad, bd, q, r, p),
    )


def print_design(result: Design, command: str) -> None:
    model = result.model
    states = len(model.states)
    inputs = len(model.inputs)
    print(
        f"模型：{states} 个状态（{', '.join(model.states)}）、"
        f"{inputs} 个输入（{', '.join(model.inputs)}），dt = {result.dt:g} s"
    )
    print(
        "Q = diag("
        + ", ".join(f"{value:g}" for value in result.q)
        + ")，R = diag("
        + ", ".join(f"{value:g}" for value in result.r)
        + ")"
    )
    print()
    print("K（行是输入、列是状态）：")
    for input_index in range(inputs):
        for state_index in range(states):
            value = format_value(float(result.k[input_index, state_index]))
            print(f"  K[{input_index}, {model.states[state_index]}] = {value}")
    print()
    print(
        "闭环极点（z）："
        + "、".join(format_pole(complex(value)) for value in result.poles_z)
    )
    print(
        "闭环极点（s = ln(z)/dt）："
        + "、".join(format_pole(complex(value)) for value in result.poles_s)
    )
    print(f"闭环 ω_n：{result.bandwidth:.4g} rad/s，ζ：{result.damping:.4g}")
    print()
    print("C++（填进 params.hpp）：")
    print(f"// {command}")
    print(f"// 闭环 ω_n {result.bandwidth:.4g} rad/s、ζ {result.damping:.4g}")
    if inputs == 1:
        for state_index in range(states):
            value = format_value(float(result.k[0, state_index]))
            print(f"{value}F, // {gain_name(model, 0, state_index)}")
    else:
        for input_index in range(inputs):
            row = ", ".join(
                f"{format_value(float(result.k[input_index, state_index]))}F"
                for state_index in range(states)
            )
            print(f"{{{row}}}, // {model.inputs[input_index]}")


def selftest() -> int:
    """自检：0.1 ms 下与连续公式对账、DARE 残差、复现设计表的四行、两条命令给同样的 K。"""
    failures = 0

    def check(name: str, ok: bool) -> None:
        nonlocal failures
        print(f"{'通过' if ok else '失败'} {name}")
        failures += 0 if ok else 1

    # 1) dt = 0.1 ms：离散 K 与连续闭式解相差 < 3%（离散化误差）
    q = np.array([2.56e6, 0.0])
    r = np.array([1.0])
    result = design("double-integrator", 0.0001, q, r)
    k_angle = float(np.sqrt(q[0] / r[0]))
    k_rate = float(np.sqrt(q[1] / r[0] + 2.0 * k_angle))
    angle_error = abs(float(result.k[0, 0]) - k_angle) / k_angle
    rate_error = abs(float(result.k[0, 1]) - k_rate) / k_rate
    check(
        f"0.1 ms 的离散 K 与连续公式相差 < 3%（{angle_error:.2%}、{rate_error:.2%}）",
        angle_error < 0.03 and rate_error < 0.03,
    )

    # 2) DARE 残差
    check(f"DARE 残差 / ‖P‖ < 1e-9（{result.residual:.2e}）", result.residual < 1.0e-9)

    # 3) 复现设计表的四行
    table = [
        (30.0, 8.1e5, 0.0, 881.1, 41.98),
        (40.0, 2.56e6, 0.0, 1555.4, 55.77),
        (40.0, 2.56e6, 3200.0, 1537.3, 77.64),
        (60.0, 1.296e7, 0.0, 3450.5, 83.07),
    ]
    for width, q_angle, q_rate, expected_angle, expected_rate in table:
        outcome = design(
            "double-integrator", 0.001, np.array([q_angle, q_rate]), np.array([1.0])
        )
        actual = outcome.k[0]
        elementwise = max(
            abs(actual[0] - expected_angle) / expected_angle,
            abs(actual[1] - expected_rate) / expected_rate,
        )
        label = f"ω_n {width:g}"
        if q_rate > 0.0:
            label += "、ζ 1"
        check(
            f"{label}：K = [{actual[0]:.2f}, {actual[1]:.2f}]，与设计表相差 < 0.1%"
            f"（{elementwise:.2e}）",
            elementwise < 1.0e-3,
        )
        check(
            f"{label}：离散闭环换回 s 平面的 ω_n = {outcome.bandwidth:.2f} rad/s",
            abs(outcome.bandwidth - width) < 0.01 * width,
        )

    # 4) 过阻尼（ζ = 1.5）：按两个实极点报 ω_n 与 ζ，不是 ζ 恒等于 1
    overdamped = design(
        "double-integrator", 0.001, bandwidth_to_q(40.0, 1.5), np.array([1.0])
    )
    check(
        f"ζ 1.5：ω_n = {overdamped.bandwidth:.2f} rad/s、ζ = {overdamped.damping:.3f}",
        abs(overdamped.bandwidth - 40.0) < 0.4 and abs(overdamped.damping - 1.5) < 0.03,
    )

    # 5) R 的每个对角元都要大于 0
    try:
        parse_diagonal("1,0", 2, "--r", require_positive=True)
    except SystemExit:
        rejected = True
    else:
        rejected = False
    check("--r 里有 0 时拒绝", rejected)

    # 6) --bandwidth 40 --damping 0.707 与 --q 2.56e6,0 给出同样的 K
    q_from_bandwidth = bandwidth_to_q(40.0, 0.707)
    from_bandwidth = design(
        "double-integrator", 0.001, q_from_bandwidth, np.array([1.0])
    )
    from_q = design(
        "double-integrator", 0.001, np.array([2.56e6, 0.0]), np.array([1.0])
    )
    difference = float(
        np.max(np.abs(from_bandwidth.k - from_q.k)) / np.max(np.abs(from_q.k))
    )
    check(
        f"--bandwidth 40 --damping 0.707 与 --q 2.56e6,0 的 K 相同（{difference:.2e}）",
        difference < 1.0e-9,
    )
    return 0 if failures == 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "model",
        nargs="?",
        default="double-integrator",
        help=f"模型名（models.py 里）：{', '.join(sorted(MODELS))}",
    )
    parser.add_argument(
        "--dt", type=float, help="离散化的周期（s），与板上控制周期相同"
    )
    parser.add_argument("--q", help="Q 的对角元（逗号分隔），如 2.56e6,0")
    parser.add_argument("--r", default="1", help="R 的对角元（逗号分隔），默认 1")
    parser.add_argument(
        "--bandwidth",
        type=float,
        help="双积分器的闭环自然频率 ω_n（rad/s），用来反算 Q",
    )
    parser.add_argument(
        "--damping",
        type=float,
        help="双积分器的闭环阻尼 ζ（≥ 0.707），与 --bandwidth 一起用",
    )
    parser.add_argument("--selftest", action="store_true", help="只跑自检")
    args = parser.parse_args()

    if args.selftest:
        return selftest()

    model_name = args.model.replace("_", "-")
    if model_name not in MODELS:
        print(f"没有模型 {args.model}；可选：{', '.join(sorted(MODELS))}")
        return 2
    if args.dt is None or not args.dt > 0.0:
        print("--dt 必须给出且大于 0（秒）")
        return 2

    if args.bandwidth is not None:
        if args.q is not None:
            print("--bandwidth 与 --q 只能用其中一个")
            return 2
        if args.damping is None:
            print("--bandwidth 要配 --damping")
            return 2
        if model_name != "double-integrator":
            print("--bandwidth/--damping 只对 double-integrator 有效")
            return 2
        q = bandwidth_to_q(args.bandwidth, args.damping)
        r = np.array([1.0])
        command = command_line(
            model_name, args.dt, None, None, args.bandwidth, args.damping
        )
    else:
        if args.q is None:
            print("要给 --q（Q 的对角元），或者用 --bandwidth/--damping")
            return 2
        model = MODELS[model_name]()
        q = parse_diagonal(args.q, len(model.states), "--q")
        r = parse_diagonal(args.r, len(model.inputs), "--r", require_positive=True)
        command = command_line(model_name, args.dt, q, r, None, None)

    print_design(design(model_name, args.dt, q, r), command)
    return 0


if __name__ == "__main__":
    sys.exit(main())
