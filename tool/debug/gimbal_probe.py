#!/usr/bin/env python3
"""云台固件（infantry_dm_mc02）的上板调参助手：不停核读出云台、安全门、遥控与任务统计，隔 1 s 再读一次。

- 只读：不停核、不复位、不写目标内存（OpenOCD 连接时只写调试寄存器）。
- 地址与结构布局由 gdb 从同一个 ELF 的调试信息离线算出（gdb 不连目标），所以 ELF 必须是板上正在跑的那一个。
- 与 bringup_probe.py 共用读内存与解析布局的代码，只是要读的对象不同。

调参时看这几行：
- `gimbal.mode`：DISABLED / ENABLING / HOLD / TRACK（自瞄）/ MANUAL（手动）；
- `gimbal.command`：这一轮收到的云台指令模式（0 HOLD、1 RATE、2 ABSOLUTE，超时按 HOLD）；
- `gimbal.last_disable_reason`：每拍判定的原因（0 无、1 姿态无效、2 标定中、3 姿态过期、
  4 关节不可用、5 关节故障、6 出力中掉了、7 使能超时、8 指令被拒绝）；
- `gimbal.output_stop_reason`：从出力状态掉出来那一刻的原因，保留到下次使能——想知道"刚才为什么掉了"
  就看这一条（`last_disable_reason` 会被后续的条件覆盖，比如驱动重新核对后变成"关节不可用"）；
- `gimbal.pitch_rad` / `pitch_target_rad` / `pitch_torque_newton_meter` 与
  `gimbal.pitch_diagnostics.compensation_newton_meter`（水平静止时它的稳态值就是重力前馈 τ_g）；
- `gimbal.yaw_*` 同理；`safety.state`（0 锁住、1 放开）；`remote.running`；
- `runner[1].*`：gimbal 任务的轮数、单轮最长耗时、错过周期。

用法（DM-MC02 + CMSIS-DAP，先烧录同一个 ELF）：
    gimbal_probe.py --elf build/arm-debug-infantry/firmware_infantry_dm_mc02.elf
    gimbal_probe.py --elf ... --interval 0.5 --log gimbal.log
需要 arm-none-eabi-gdb（--gdb）与 openocd（--openocd）在 PATH 上或显式给出。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bringup_probe as probe  # 同目录的工具，复用 gdb 取地址与内存解码

ANON = "'(anonymous namespace)::{}'"
ITEMS = [
    ("gimbal", ANON.format("g_gimbal") + ".report_"),
    ("command", ANON.format("g_command") + ".report_"),
    ("safety", ANON.format("g_safety_gate") + ".report_"),
    ("remote", ANON.format("g_remote") + ".report_"),
    ("dm", ANON.format("g_dm_can_1") + ".statistics_"),
    ("runner", ANON.format("g_runner") + ".status_._M_elems[]"),
    ("topic.attitude", "'topics::imu::attitude'.publish_count_"),
    ("topic.control", "'topics::pc_link::control'.publish_count_"),
]
TASKS = ["imu", "gimbal", "pc_link", "remote"]  # 与组合根的任务表同序
MODES = ["DISABLED", "ENABLING", "HOLD", "TRACK", "MANUAL"]
REASONS = [
    "无",
    "姿态无效",
    "标定中",
    "姿态过期",
    "关节不可用",
    "关节故障",
    "出力中掉了",
    "使能超时",
    "指令被拒绝",
]
SAFETY_STATES = ["锁住", "放开"]


def decode_values(regions: list, sample: dict[str, list[int]]) -> dict[str, object]:
    values: dict[str, object] = {}
    for name, _, fields, size in regions:
        data = probe.region_bytes(sample, name)[:size]
        for field, value in probe.decode(fields, data).items():
            values[f"{name}.{field}"] = value
    return values


def judge(decoded: list[dict[str, object]]) -> list[tuple[bool, str]]:
    first, second = decoded

    def delta(key: str) -> int:
        return int(second[key]) - int(first[key])

    checks: list[tuple[bool, str]] = []
    mode = (
        MODES[second["gimbal.mode"]]
        if second["gimbal.mode"] < len(MODES)
        else second["gimbal.mode"]
    )
    reason = (
        REASONS[second["gimbal.last_disable_reason"]]
        if second["gimbal.last_disable_reason"] < len(REASONS)
        else second["gimbal.last_disable_reason"]
    )
    stop_reason = (
        REASONS[second["gimbal.output_stop_reason"]]
        if second["gimbal.output_stop_reason"] < len(REASONS)
        else second["gimbal.output_stop_reason"]
    )
    safety = (
        SAFETY_STATES[second["safety.state"]]
        if second["safety.state"] < len(SAFETY_STATES)
        else second["safety.state"]
    )
    checks.append(
        (
            True,
            f"云台模式 {mode}，最近一次回 DISABLED 的原因：{reason}；掉出出力状态的原因：{stop_reason}",
        )
    )
    checks.append(
        (
            bool(second["safety.has_message"]),
            (
                f"安全门 {safety}：收到 safety.stop {delta('safety.release_count')} 次放开，"
                f"超时 {delta('safety.timeout_count')} 次"
            ),
        )
    )
    checks.append(
        (
            bool(second["remote.connected"]),
            (
                f"遥控连着：{bool(second['remote.connected'])}；"
                f"仲裁：连着 {bool(second['command.input_connected'])}，"
                f"运行 {bool(second['command.run'])}，自瞄 {bool(second['command.auto_aim'])}，"
                f"开火 {bool(second['command.fire'])}，"
                f"PC 目标有效 {bool(second['command.pc_target_valid'])}，"
                f"接管 {bool(second['command.engaged'])}，"
                f"接管误差 {second['command.engage_error_rad']:.3f} rad"
            ),
        )
    )
    runs = delta("runner[1].run_count")
    checks.append(
        (
            800 <= runs <= 1200,
            f"gimbal 任务每秒 {runs} 轮（按 imu.attitude 触发，应约 1000）",
        )
    )
    checks.append(
        (
            delta("runner[1].overrun_count") == 0,
            f"gimbal 没有错过周期，单轮最长 {second['runner[1].max_run_us']} µs",
        )
    )
    checks.append(
        (
            int(second["dm.enable_failure_count"]) == 0
            and int(second["dm.request_timeout_count"]) == 0,
            (
                f"达妙：使能失败 {second['dm.enable_failure_count']} 次，"
                f"请求超时 {second['dm.request_timeout_count']} 次，"
                f"反馈 {delta('dm.feedback_count')} 帧/次读数"
            ),
        )
    )
    if mode in ("HOLD", "TRACK", "MANUAL"):
        # 4310 的峰值力矩 12.5 N·m：超过它说明限幅或符号出了问题
        checks.append(
            (
                abs(float(second["gimbal.yaw_torque_newton_meter"])) <= 12.5
                and abs(float(second["gimbal.pitch_torque_newton_meter"])) <= 12.5,
                (
                    f"力矩：yaw {second['gimbal.yaw_torque_newton_meter']:.3f} N·m，"
                    f"pitch {second['gimbal.pitch_torque_newton_meter']:.3f} N·m（重力前馈 "
                    f"{second['gimbal.pitch_gravity_newton_meter']:.3f} N·m）"
                ),
            )
        )
        checks.append(
            (
                True,
                (
                    f"pitch：测量 {second['gimbal.pitch_rad']:.4f} rad，目标 "
                    f"{second['gimbal.pitch_target_rad']:.4f} rad，补偿力矩 "
                    f"{second['gimbal.pitch_diagnostics.compensation_newton_meter']:.3f} N·m；"
                    f"yaw：测量 {second['gimbal.yaw_rad']:.4f} rad，目标 "
                    f"{second['gimbal.yaw_target_rad']:.4f} rad，补偿力矩 "
                    f"{second['gimbal.yaw_diagnostics.compensation_newton_meter']:.3f} N·m"
                ),
            )
        )
    checks.append(
        (
            bool(second["gimbal.attitude_valid"]),
            (
                f"姿态有效：{bool(second['gimbal.attitude_valid'])}，imu.attitude 每秒 "
                f"{delta('topic.attitude.value')} 次，"
                f"pc_link.control 每秒 {delta('topic.control.value')} 次"
            ),
        )
    )
    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--elf", type=Path, required=True, help="板上正在运行的固件 ELF"
    )
    parser.add_argument("--gdb", default="arm-none-eabi-gdb")
    parser.add_argument("--openocd", default="openocd")
    parser.add_argument(
        "--interval", type=float, default=1.0, help="两次读数的间隔（s）"
    )
    parser.add_argument("--log", type=Path, help="把 OpenOCD 的原始输出另存到这个文件")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只解析 ELF，打印地址、字段与脚本，不连目标",
    )
    parser.add_argument(
        "--cwd", type=Path, help="OpenOCD 的工作目录（WSL 里调用 Windows 版时用）"
    )
    args = parser.parse_args()

    regions, _ = probe.resolve(args.gdb, args.elf, ITEMS)
    if args.dry_run:
        for name, address, fields, size in regions:
            print(f"{name} @ {address:#010x}（{size} 字节）")
            for field in fields:
                print(f"    +{field.offset:<3} {field.name}（{field.kind}）")
        print(probe.build_tcl(regions, int(args.interval * 1000)))
        return 0
    with tempfile.NamedTemporaryFile(
        "w", suffix=".tcl", delete=False, encoding="utf-8"
    ) as script:
        script.write(probe.build_tcl(regions, int(args.interval * 1000)))
    result = subprocess.run(
        [args.openocd, *probe.DEFAULT_SETUP, "-f", script.name],
        check=False,
        cwd=args.cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    Path(script.name).unlink(missing_ok=True)
    if args.log:
        args.log.write_text(result.stdout, encoding="utf-8")
    samples, _ = probe.parse_samples(result.stdout)
    if len(samples) != 2:
        print(result.stdout)
        print("没有读到两组数据：检查调试器连接与目标供电", file=sys.stderr)
        return 2

    decoded = [decode_values(regions, sample) for sample in samples]
    print(f"{'项':<52}{'第一次':>16}{'第二次':>16}{'增量':>12}")
    for key in decoded[0]:
        first, second = decoded[0][key], decoded[1][key]
        delta = (
            second - first if isinstance(first, int) and isinstance(second, int) else ""
        )
        print(f"{key:<52}{first!s:>16}{second!s:>16}{delta!s:>12}")

    checks = judge(decoded)
    print()
    failed = 0
    for passed, text in checks:
        print(("通过 " if passed else "未过 ") + text)
        failed += 0 if passed else 1
    print(f"\n{len(checks) - failed}/{len(checks)} 项通过")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
