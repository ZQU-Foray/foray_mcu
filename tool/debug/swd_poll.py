#!/usr/bin/env python3
"""通过 OpenOCD 不停核地周期读取目标内存，记录成文本日志。

- 只读：不停核、不复位、不写入目标（OpenOCD 连接时只写调试寄存器）。
- --wait-power-cycle：先等目标断电，再等重新上电，上电后立即开始记录，
  用于记录冷启动过程（例如加热器从室温升到恒温点）。
- 记录中途连接断开会自动重连，直到达到 --duration。
- --dump 在记录结束后把指定内存整块保存为二进制文件。

日志格式：以 # 开头的是说明行；数据行为
    <主机 UNIX 时间 s> | <区域 1 的 32 位字...> | <区域 2 的 32 位字...>

用法示例（DM-MC02 + CMSIS-DAP；Windows 上 --openocd 指向 openocd.exe）：
    swd_poll.py --region 0xE0001004:1 --region 0x2400490c:40 \\
        --rate 5 --duration 600 --wait-power-cycle --out heater.log \\
        --dump 0x24000000:0x50000:axi_after.bin
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_SETUP = [
    "-f",
    "interface/cmsis-dap.cfg",
    "-c",
    "transport select swd",
    "-f",
    "target/stm32h7x.cfg",
]
# STM32H7 DBGMCU_IDC，能读出即说明目标已上电并可访问。
PROBE_ADDRESS = 0x5C001000
DUMP_CHUNK_WORDS = 4096


def parse_int(text: str) -> int:
    return int(text, 0)


def parse_region(text: str) -> tuple[int, int]:
    address, words = text.split(":")
    return parse_int(address), parse_int(words)


def parse_dump(text: str) -> tuple[int, int, Path]:
    address, size, path = text.split(":", 2)
    return parse_int(address), parse_int(size), Path(path)


def log(message: str) -> None:
    stamp = time.strftime("%H:%M:%S")
    print(f"[{stamp}] {message}", flush=True)


class OpenOcd:
    def __init__(self, args: argparse.Namespace) -> None:
        self.executable = args.openocd
        self.setup = DEFAULT_SETUP
        self.cwd = args.cwd

    def command(self, commands: list[str]) -> list[str]:
        result = [self.executable, *self.setup, "-c", "init"]
        for command in commands:
            result += ["-c", command]
        return result + ["-c", "shutdown"]

    def run(self, commands: list[str], timeout: float) -> list[str]:
        try:
            completed = subprocess.run(
                self.command(commands),
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors="replace",
                timeout=timeout,
                cwd=self.cwd,
            )
        except subprocess.TimeoutExpired:
            return []
        return [line.rstrip("\r\n") for line in completed.stdout.splitlines()]

    def stream(self, commands: list[str]) -> subprocess.Popen:
        return subprocess.Popen(
            self.command(commands),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            cwd=self.cwd,
        )


def probe(ocd: OpenOcd, timeout: float) -> bool:
    lines = ocd.run([f'echo "PROBE [read_memory {PROBE_ADDRESS} 32 1]"'], timeout)
    return any(line.startswith("PROBE 0x") for line in lines)


def poll_session(
    ocd: OpenOcd,
    regions: list[tuple[int, int]],
    period_ms: int,
    iterations: int,
    deadline: float | None,
    out,
) -> tuple[int, float | None]:
    reads = " ".join(f"| [read_memory {a} 32 {n}]" for a, n in regions)
    loop = (
        f"for {{set i 0}} {{$i < {iterations}}} {{incr i}} "
        f'{{ echo "S {reads}"; sleep {period_ms} }}'
    )
    process = ocd.stream([loop])
    written = 0
    first: float | None = None
    assert process.stdout is not None
    for raw in process.stdout:
        line = raw.rstrip("\r\n")
        if not line.startswith("S |"):
            continue
        now = time.time()
        if first is None:
            first = now
        out.write(f"{now:.3f} {line[2:]}\n")
        out.flush()
        written += 1
        if deadline is not None and now >= deadline:
            break
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
    return written, first


def dump_memory(ocd: OpenOcd, address: int, size: int, path: Path) -> bool:
    words = size // 4
    commands = []
    for start in range(0, words, DUMP_CHUNK_WORDS):
        count = min(DUMP_CHUNK_WORDS, words - start)
        chunk_address = address + start * 4
        commands.append(
            f'echo "D {chunk_address} [read_memory {chunk_address} 32 {count}]"'
        )
    lines = ocd.run(commands, timeout=120)
    data = bytearray()
    for line in lines:
        if line.startswith("D "):
            for value in line.split()[2:]:
                data += int(value, 0).to_bytes(4, "little")
    if len(data) != words * 4:
        log(f"转储 0x{address:08x} 失败：只得到 {len(data)} 字节")
        return False
    path.write_bytes(bytes(data))
    log(f"已转储 0x{address:08x}+0x{size:x} → {path}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--openocd", default=os.environ.get("OPENOCD", "openocd"))
    parser.add_argument("--cwd", help="OpenOCD 的工作目录（WSL 调 Windows exe 时用）")
    parser.add_argument("--region", type=parse_region, action="append", required=True)
    parser.add_argument("--rate", type=float, default=5.0, help="采样频率 Hz")
    parser.add_argument("--duration", type=float, default=60.0, help="记录时长 s")
    parser.add_argument("--wait-power-cycle", action="store_true")
    parser.add_argument("--max-wait", type=float, default=7200.0, help="等待上电上限 s")
    parser.add_argument("--probe-interval", type=float, default=3.0)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--dump", type=parse_dump, action="append", default=[])
    args = parser.parse_args()

    ocd = OpenOcd(args)
    period_ms = max(1, round(1000.0 / args.rate))
    if args.wait_power_cycle:
        log("等待目标断电……")
        while probe(ocd, timeout=20):
            time.sleep(args.probe_interval)
        log("目标已断电，等待重新上电……")

    give_up = time.time() + args.max_wait
    started: float | None = None
    with args.out.open("w", encoding="utf-8", newline="\n") as out:
        out.write(f"# swd_poll {time.strftime('%Y-%m-%d %H:%M:%S %z')}\n")
        for address, words in args.region:
            out.write(f"# region 0x{address:08x} words={words}\n")
        out.write(f"# rate_hz={args.rate} duration_s={args.duration}\n")
        out.flush()
        while True:
            if started is None and time.time() > give_up:
                log("等待上电超时")
                return 1
            deadline = None if started is None else started + args.duration
            remaining = args.duration if started is None else deadline - time.time()
            if remaining <= 0:
                break
            iterations = int(remaining * args.rate) + 1
            written, first = poll_session(
                ocd, args.region, period_ms, iterations, deadline, out
            )
            if started is None and first is not None:
                started = first
                log(f"开始记录（{args.duration:.0f} s）")
            elif not written:
                time.sleep(0.2)
    log("记录结束")
    ok = all(dump_memory(ocd, address, size, path) for address, size, path in args.dump)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
