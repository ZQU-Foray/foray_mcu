#!/usr/bin/env python3
"""bringup 固件的上板检查：不停核读出各能力的统计，隔 1 s 再读一次，给出增量与判定（Decision-20261101-03）。

- 只读：不停核、不复位、不写目标内存（OpenOCD 连接时只写调试寄存器）。
- 地址与结构布局由 gdb 从同一个 ELF 的调试信息离线算出（gdb 不连目标），所以 ELF 必须是板上正在跑的那一个。
- 输出每一项的两次读数与增量，最后按检查清单（docs/ai/decisions/2026-11-01-F2-外设上板检查清单.md）给出判定。

用法（DM-MC02 + CMSIS-DAP，先烧录同一个 ELF）：
    bringup_probe.py --elf build/arm-debug/firmware_bringup_dm_mc02.elf
    bringup_probe.py --elf ... --interval 2 --log probe.log     # 隔 2 s，原始输出另存
需要 arm-none-eabi-gdb（--gdb）与 openocd（--openocd）在 PATH 上或显式给出。
"""

from __future__ import annotations

import argparse
import re
import struct
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

ANON = "'(anonymous namespace)::{}'"
# (名称, gdb 表达式)。结构按字段展开；以 [] 结尾的是数组，按元素展开。
ITEMS = [
    ("runner", ANON.format("g_runner") + ".status_._M_elems[]"),
    ("monitor", ANON.format("g_monitor") + ".report_"),
    ("imu", ANON.format("g_imu_check") + ".report_"),
    ("dm", ANON.format("g_dm_check") + ".report_"),
    ("can_2", ANON.format("g_can_2") + ".report_"),
    ("can_3", ANON.format("g_can_3") + ".report_"),
    ("sbus", ANON.format("g_sbus") + ".report_"),
    ("usb", ANON.format("g_usb_report") + ".report_"),
    ("topic.heartbeat", "'topics::bringup::heartbeat'.publish_count_"),
    ("topic.imu_event", "'topics::bringup::imu_event'.publish_count_"),
    ("topic.imu", "'topics::bringup::imu'.publish_count_"),
    ("topic.sbus", "'topics::bringup::sbus'.publish_count_"),
]
TASKS = ["imu", "monitor", "heartbeat", "io", "dm"]  # 与组合根的任务表同序
# bringup::DmCheck::Stage 的取值顺序
DM_STAGES = [
    "核对",
    "失能保活",
    "使能",
    "零力矩",
    "转动",
    "刹车",
    "通讯超时",
    "恢复",
    "再使能",
    "失能",
    "基准：使能",
    "基准",
    "基准：失能",
    "结束",
    "失败",
]
DM_COMMUNICATION_LOST = 0xD
TIM2_CNT = 0x40000024
DWT_PCSR = 0xE000101C
PC_SAMPLES = 16

DEFAULT_SETUP = [
    "-f",
    "interface/cmsis-dap.cfg",
    "-c",
    "transport select swd",
    "-f",
    "target/stm32h7x.cfg",
]

LEAF = re.compile(r"^/\*\s+(\d+)\s+\|\s+(\d+)\s+\*/\s+(.+?)\s+(\w+);$")
OPEN = re.compile(r"^/\*\s+(\d+)\s+\|\s+(\d+)\s+\*/\s+(?:struct|class)\s.*\{$")
CLOSE = re.compile(r"^\}\s*(\w+);$")


@dataclass
class Field:
    name: str
    offset: int
    size: int
    kind: str  # struct 格式字符


def field_kind(type_name: str, size: int) -> str:
    if type_name == "float":
        return "f"
    if type_name == "bool":
        return "?"
    signed = not (
        "unsigned" in type_name or type_name.startswith(("uint", "std::uint"))
    )
    if "Microseconds" in type_name:
        signed = False
    return (
        {1: "b", 2: "h", 4: "i", 8: "q"}[size]
        if signed
        else {1: "B", 2: "H", 4: "I", 8: "Q"}[size]
    )


def parse_layout(text: str) -> list[Field]:
    """解析 gdb 的 ptype/o 输出：叶子字段；没有叶子的 4 字节结构（std::atomic）当作 uint32。"""
    fields: list[Field] = []
    stack: list[tuple[int, int, int]] = []  # (偏移, 大小, 开始时的字段数)
    for raw in text.splitlines():
        line = raw.strip()
        if (
            line.startswith("/* offset")
            or "type = " in line
            and line.endswith("{")
            and not stack
        ):
            stack.append((0, 0, 0))
            continue
        match = OPEN.match(line)
        if match:
            stack.append((int(match.group(1)), int(match.group(2)), len(fields)))
            continue
        match = CLOSE.match(line)
        if match and len(stack) > 1:
            offset, size, count = stack.pop()
            name = match.group(1)
            if len(fields) == count and size == 4:  # std::atomic<uint32_t> 等
                fields.append(Field(name, offset, 4, "I"))
            else:
                for field in fields[count:]:
                    field.name = f"{name}.{field.name}"
            continue
        match = LEAF.match(line)
        if match and stack:
            offset, size, type_name, name = match.groups()
            if "static" in type_name:
                continue
            fields.append(
                Field(name, int(offset), int(size), field_kind(type_name, int(size)))
            )
    return fields


def gdb_query(gdb: str, elf: Path, expressions: list[str]) -> list[str]:
    """逐条执行 gdb 命令，按分隔标记切开输出。"""
    args = [gdb, "-batch", "-nx", str(elf)]
    for expression in expressions:
        args += ["-ex", expression, "-ex", "echo @@END@@\\n"]
    result = subprocess.run(
        args, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    parts = result.stdout.split("@@END@@\n")
    return parts[: len(expressions)]


Region = tuple[str, int, list[Field], int]


def resolve(gdb: str, elf: Path) -> tuple[list[Region], dict[str, int]]:
    """每一项：(名称, 地址, 字段, 字节数)。数组展开成 名称[i]。"""
    commands = []
    for _, expression in ITEMS:
        base = expression.removesuffix("[]")
        element = base + "[0]" if expression.endswith("[]") else base
        commands += [
            f"print/x &({base})",
            f"ptype/o {element}",
            f"print sizeof({base})",
            f"print sizeof({element})",
        ]
    commands += ["info address Error_Handler", "info address HardFault_Handler"]
    outputs = gdb_query(gdb, elf, commands)
    resolved = []
    for index, (name, expression) in enumerate(ITEMS):
        address_text, layout_text, total_text, element_text = outputs[
            4 * index : 4 * index + 4
        ]
        address = int(re.search(r"0x[0-9a-fA-F]+", address_text).group(0), 16)
        total = int(re.search(r"= (\d+)", total_text).group(1))
        element = int(re.search(r"= (\d+)", element_text).group(1))
        fields = parse_layout(layout_text) or [
            Field("value", 0, element, field_kind("uint32_t", element))
        ]
        if expression.endswith("[]"):
            for item in range(total // element):
                resolved.append(
                    (f"{name}[{item}]", address + item * element, fields, element)
                )
        else:
            resolved.append((name, address, fields, total))
    handlers = {}
    for label, text in zip(("Error_Handler", "HardFault_Handler"), outputs[-2:]):
        match = re.search(r"at address (0x[0-9a-fA-F]+)", text)
        if match:
            handlers[label] = int(match.group(1), 16) & ~1
    return resolved, handlers


def build_tcl(regions: list[Region], interval_ms: int) -> str:
    lines = ["init"]
    for sample in (0, 1):
        lines.append(f'echo "@@SAMPLE {sample}"')
        # 主机毫秒数（OpenOCD 的 ms）紧跟 TIM2 读出：两次之差是 TIM2 的独立参照
        lines.append(f'echo "TIM2 W [read_memory {TIM2_CNT:#x} 32 1] [ms]"')
        for name, address, _, size in regions:
            # Tcl 双引号里的 [ ] 是命令替换，名字里的数组下标要转义
            label = name.replace("[", "\\[").replace("]", "\\]")
            if address % 4 == 0:
                # 按 32 位字读：比逐字节快 4 倍，每个字段一次读出，不会读到任务改了一半的值
                words = (size + 3) // 4
                lines.append(f'echo "{label} W [read_memory {address:#x} 32 {words}]"')
            else:
                lines.append(f'echo "{label} B [read_memory {address:#x} 8 {size}]"')
        if sample == 0:
            lines.append(f"sleep {interval_ms}")
    for _ in range(PC_SAMPLES):
        lines.append(f'echo "PC [read_memory {DWT_PCSR:#x} 32 1]"')
        lines.append("sleep 30")
    lines.append("shutdown")
    return "\n".join(lines) + "\n"


def decode(fields: list[Field], data: bytes) -> dict[str, object]:
    values = {}
    for field in fields:
        chunk = data[field.offset : field.offset + field.size]
        values[field.name] = (
            struct.unpack("<" + field.kind, chunk)[0]
            if len(chunk) == field.size
            else None
        )
    return values


def parse_samples(text: str) -> tuple[list[dict[str, list[int]]], list[int]]:
    """每组读数：名称 → 数值列表（W 为 32 位字，B 为字节）。"""
    samples: list[dict[str, list[int]]] = []
    pcs: list[int] = []
    for line in text.splitlines():
        if line.startswith("@@SAMPLE"):
            samples.append({})
        elif line.startswith("PC "):
            pcs.append(int(line.split()[1], 0))
        elif samples and " " in line:
            name, _, rest = line.partition(" ")
            width, _, values = rest.partition(" ")
            if width not in ("W", "B"):
                continue
            try:
                samples[-1][name] = [int(token, 0) for token in values.split()]
            except ValueError:
                continue
            samples[-1][name + "@width"] = [4 if width == "W" else 1]
    return samples, pcs


def region_bytes(sample: dict[str, list[int]], name: str) -> bytes:
    values = sample.get(name, [])
    if sample.get(name + "@width", [1])[0] == 4:
        return b"".join(struct.pack("<I", value & 0xFFFFFFFF) for value in values)
    return bytes(values)


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
        help="只解析 ELF，打印地址、字段与 OpenOCD 脚本，不连目标",
    )
    args = parser.parse_args()

    regions, handlers = resolve(args.gdb, args.elf)
    if args.dry_run:
        for name, address, fields, size in regions:
            print(f"{name} @ {address:#010x}（{size} 字节）")
            for field in fields:
                print(f"    +{field.offset:<3} {field.name}（{field.kind}）")
        print({name: hex(address) for name, address in handlers.items()})
        print(build_tcl(regions, int(args.interval * 1000)))
        return 0
    with tempfile.NamedTemporaryFile(
        "w", suffix=".tcl", delete=False, encoding="utf-8"
    ) as script:
        script.write(build_tcl(regions, int(args.interval * 1000)))
    result = subprocess.run(
        [args.openocd, *DEFAULT_SETUP, "-f", script.name],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    Path(script.name).unlink(missing_ok=True)
    if args.log:
        args.log.write_text(result.stdout, encoding="utf-8")
    samples, pcs = parse_samples(result.stdout)
    if len(samples) != 2:
        print(result.stdout)
        print("没有读到两组数据：检查调试器连接与目标供电", file=sys.stderr)
        return 2

    decoded = []
    for sample in samples:
        tim2, host_ms = (sample.get("TIM2", []) + [0, 0])[:2]
        values = {"TIM2": tim2, "host_ms": host_ms}
        for name, _, fields, size in regions:
            data = region_bytes(sample, name)[:size]
            for field, value in decode(fields, data).items():
                values[f"{name}.{field}"] = value
        decoded.append(values)

    print(f"{'项':<44}{'第一次':>16}{'第二次':>16}{'增量':>12}")
    for key in decoded[0]:
        first, second = decoded[0][key], decoded[1][key]
        delta = (
            second - first if isinstance(first, int) and isinstance(second, int) else ""
        )
        print(f"{key:<44}{first!s:>16}{second!s:>16}{delta!s:>12}")

    checks = judge(decoded, pcs, handlers)
    print()
    failed = 0
    for passed, text in checks:
        failed += 0 if passed else 1
        print(("通过 " if passed else "未过 ") + text)
    print(f"\n{len(checks) - failed}/{len(checks)} 项通过")
    return 0 if failed == 0 else 1


def judge(decoded, pcs, handlers) -> list[tuple[bool, str]]:
    first, second = decoded

    def delta(key: str) -> int:
        return (second[key] - first[key]) & 0xFFFFFFFF

    # 两组读数之间除了 --interval 还有读取本身的时间：速率按目标自己的时基（TIM2 走过的时间）算，
    # TIM2 再用主机时钟核对
    host_ms = delta("host_ms")
    seconds = delta("TIM2") / 1e6

    def per_second(key: str) -> float:
        return delta(key) / seconds

    checks = []
    tim2 = delta("TIM2") / (host_ms / 1000.0) if host_ms else 0.0
    checks.append(
        (
            abs(tim2 - 1e6) < 2e4,
            f"时基：TIM2 走了 {delta('TIM2')}，主机计时 {host_ms} ms，合每秒 {tim2:.0f}（应约 1,000,000）",
        )
    )
    heartbeat_runs = per_second("runner[2].run_count")
    checks.append(
        (
            900 <= heartbeat_runs <= 1100,
            f"heartbeat 任务每秒 {heartbeat_runs:.0f} 轮（应约 1000）",
        )
    )
    checks.append((delta("runner[2].overrun_count") == 0, "heartbeat 没有错过周期"))
    # 心跳序号从 1 起逐次加 1：收到次数等于最近序号，就是每一次发布都收到了。两者在同一个结构里，
    # 但不是同一时刻读出，monitor 在两次读之间收到一次时序号会多 1
    gaps = [
        sample["monitor.last_sequence"] - sample["monitor.received"]
        for sample in decoded
    ]
    checks.append(
        (
            delta("monitor.received") > 0
            and all(0 <= gap <= 1 for gap in gaps)
            and second["monitor.lost"] == 0
            and second["monitor.out_of_order"] == 0,
            (
                f"monitor 收到全部心跳：收到 {delta('monitor.received')}"
                f"（同期发布 {delta('topic.heartbeat.value')}，读出时刻不同，可差一两次），"
                f"最近序号减收到次数 {gaps}，丢失 {second['monitor.lost']}，"
                f"乱序 {second['monitor.out_of_order']}"
            ),
        )
    )
    checks.append(
        (
            bool(second["imu.ready"]) and not second["imu.error"],
            "BMI088 已配置好，没有出错",
        )
    )
    gyro = per_second("imu.gyro_samples")
    accel = per_second("imu.accel_samples")
    checks.append((1900 <= gyro <= 2100, f"陀螺样本每秒 {gyro:.0f}（应约 2000）"))
    checks.append((1500 <= accel <= 1700, f"加速度计样本每秒 {accel:.0f}（应约 1600）"))
    events = per_second("topic.imu_event.value")
    checks.append(
        (
            events >= gyro + accel,
            f"中断里发布 bringup.imu_event 每秒 {events:.0f} 次（不少于数据就绪次数）",
        )
    )
    checks.append(
        (
            delta("imu.events") > 0
            and delta("imu.late") == 0
            and delta("runner[0].timeout_count") == 0,
            (
                f"IMU 任务被中断发布的 topic 叫醒：每秒 {per_second('imu.events'):.0f} 轮，"
                f"等待超时 {delta('runner[0].timeout_count')} 次，"
                f"超过 200 µs 的 {delta('imu.late')} 次，最大延迟 {second['imu.max_latency_us']} µs"
            ),
        )
    )
    temperature = second["imu.temperature_celsius"]
    checks.append(
        (
            20.0 <= temperature <= 60.0,
            f"BMI088 温度 {temperature:.1f} ℃（加热目标 50 ℃，冷启动约 25 s 到）",
        )
    )
    for task_index, task in enumerate(TASKS):
        free = second[f"runner[{task_index}].stack_free_words"]
        checks.append((free > 32, f"{task} 任务剩余栈 {free} 字"))
    checks.extend(judge_dm(second, delta, per_second))
    for bus in ("can_2", "can_3"):
        checks.append(
            (
                bool(second[f"{bus}.bus_ready"]),
                (
                    f"{bus} 已启动：收到 {delta(f'{bus}.frames')} 帧，"
                    f"Bus-Off {second[f'{bus}.statistics.bus_off_count']} 次（没接设备时收到 0 帧属正常）"
                ),
            )
        )
    checks.append(
        (
            bool(second["sbus.port_ready"]),
            f"SBUS 端口在接收：每秒 {per_second('sbus.frames'):.0f} 帧",
        )
    )
    if second["usb.connected"]:
        lines = per_second("usb.lines")
        # connected 指 USB 配置完成；主机没打开串口时第一行一直没被取走，之后的行都被跳过
        hint = "" if lines >= 8 else "；只跳过不发送时是主机没打开串口，打开后重跑"
        checks.append(
            (
                lines >= 8,
                (
                    f"USB 状态行每秒 {lines:.0f} 行（应约 10），"
                    f"跳过每秒 {per_second('usb.skipped'):.0f} 次{hint}"
                ),
            )
        )
    else:
        checks.append(
            (True, "USB 没有连上主机：插上 USB 线并打开串口后重跑，可检查状态行")
        )
    bad = [
        pc
        for pc in pcs
        if any(abs(pc - address) < 0x40 for address in handlers.values())
    ]
    checks.append(
        (
            not bad and pcs,
            f"PC 采样 {len(pcs)} 次都不在 Error_Handler / HardFault_Handler",
        )
    )
    return checks


def judge_dm(dm, delta, per_second) -> list[tuple[bool, str]]:
    """达妙电机检查（Decision-20261102-02）：上电后只跑一遍，读的是结果与各阶段测得的值。"""

    def stage(value: int) -> str:
        return DM_STAGES[value] if 0 <= value < len(DM_STAGES) else str(value)

    where = stage(dm["dm.stage"])
    if dm["dm.stage"] == DM_STAGES.index("失败"):
        where += f"（失败在{stage(dm['dm.failed_stage'])}）"
    checks = [
        (
            bool(dm["dm.passed"]),
            (
                f"达妙电机检查：{'通过' if dm['dm.passed'] else '未通过，停在' + where}；"
                f"核对用时 {dm['dm.verified_after_us'] / 1000:.0f} ms，"
                f"使能确认 {dm['dm.enable_latency_us'] / 1000:.1f} ms，"
                f"清错后再使能 {dm['dm.reenable_latency_us'] / 1000:.1f} ms"
            ),
        ),
        (
            bool(dm["dm.motion_ok"]),
            (
                f"达妙电机转动：最大速度 {dm['dm.spin_max_velocity_rad_per_second']:.2f} rad/s，"
                f"2 s 转过 {dm['dm.spin_position_delta_rad']:.2f} rad，"
                f"结束时力矩反馈 {dm['dm.spin_torque_feedback_newton_meter']:.2f} N·m"
                f"（指令 {dm['dm.spin_command_newton_meter']:g} N·m、Kd {dm['dm.spin_damping']:g}，"
                f"不计摩擦时稳态 {dm['dm.spin_command_newton_meter'] / dm['dm.spin_damping']:g} rad/s）；"
                f"刹车后 {dm['dm.brake_end_velocity_rad_per_second']:.2f} rad/s；"
                f"零力矩时位置漂移 {dm['dm.zero_torque_drift_rad']:.3f} rad"
            ),
        ),
        (
            dm["dm.silence_fault_state"] == DM_COMMUNICATION_LOST,
            (
                f"通讯超时保护：停发 300 ms 后电机报状态 0x{dm['dm.silence_fault_state']:X}"
                f"（应为 0xD 通讯丢失），恢复用时 {dm['dm.recover_after_us'] / 1000:.0f} ms；"
                f"驱动{'因离线重新核对了' if dm['dm.reverified'] else '没有重新核对（只撤销了出力）'}"
            ),
        ),
    ]
    feedback = per_second("dm.feedback_count")
    checks.append(
        (
            dm["dm.phase"] == 2
            and bool(dm["dm.online"])
            and dm["dm.state"] == 0
            and 900 <= feedback <= 1100,
            (
                f"达妙电机失能保活：每秒反馈 {feedback:.0f} 帧（应约 1000），驱动阶段 {dm['dm.phase']}（2 = READY），"
                f"状态 0x{dm['dm.state']:X}；读到的设置：模式 {dm['dm.control_mode']}，"
                f"量程 {dm['dm.position_max_rad']:g} / {dm['dm.velocity_max_rad_per_second']:g} / "
                f"{dm['dm.torque_max_newton_meter']:g}，超时 {dm['dm.timeout_ticks']} 个计数；"
                f"MOS {dm['dm.mos_temperature_celsius']} ℃、线圈 {dm['dm.rotor_temperature_celsius']} ℃"
            ),
        )
    )
    checks.append(judge_dm_benchmark(dm))
    queued = per_second("dm.tx_queued")
    checks.append(
        (
            900 <= queued <= 1100
            and delta("dm.tx_rejected") == 0
            and delta("dm.bus_off") == 0,
            (
                f"CAN1 发送：每秒放进发送队列 {queued:.0f} 帧（应约 1000），本次间隔内未放进 {delta('dm.tx_rejected')} 帧；"
                f"累计未放进 {dm['dm.tx_rejected']}（电机上电前没有节点应答，队列会满）、Bus-Off {dm['dm.bus_off']}、"
                f"重复应答 {dm['dm.stale_replies']}、请求超时 {dm['dm.request_timeouts']}、"
                f"不认识的帧 {dm['dm.unknown_frames']}"
            ),
        )
    )
    return checks


def judge_dm_benchmark(dm) -> tuple[bool, str]:
    """1 kHz 控制环基准（Decision-20261102-03 的 M0、M7）：跑满轮数、dm 任务没有错过周期就算通过。
    抖动与延迟只报告：抖动主要来自更高优先级的任务，Debug 构建下尤其明显（2026-11-02 实测）。"""
    if dm["dm.bench_duration_us"] == 0:
        return True, "1 kHz 控制环基准：参数里关掉了"
    expected = dm["dm.bench_duration_us"] // 1000
    cycles = dm["dm.bench_cycles"]
    samples = max(dm["dm.bench_latency_samples"], 1)
    finished = dm["dm.stage"] == DM_STAGES.index("结束")
    return (
        finished
        and cycles >= expected * 0.99
        and dm[f"runner[{TASKS.index('dm')}].overrun_count"] == 0,
        (
            f"1 kHz 控制环基准：{cycles} 轮（应约 {expected}），"
            f"单轮耗时平均 {dm['dm.bench_exec_sum_us'] / max(cycles, 1):.1f} µs、最长 {dm['dm.bench_exec_max_us']} µs；"
            f"周期抖动最大 {dm['dm.bench_jitter_max_us']} µs，超过 1.1 ms 的 {dm['dm.bench_late_cycles']} 轮；"
            f"反馈接收 → 指令进发送队列 平均 {dm['dm.bench_latency_sum_us'] / samples:.0f} µs、"
            f"最长 {dm['dm.bench_latency_max_us']} µs；用到的反馈最旧 {dm['dm.bench_feedback_age_max_us']} µs；"
            f"位置保持误差最大 {dm['dm.bench_hold_error_max_rad']:.4f} rad"
        ),
    )


if __name__ == "__main__":
    sys.exit(main())
