#!/usr/bin/env python3
"""导出或比较 ELF 的符号集合（结构调整阶段的对照判据，评审报告 §7.3）。

P1–P2.5 只搬结构、不改行为：链接顺序会变，镜像的 SHA 不再相同，
判据改为"函数与对象符号的名称、大小集合一致"。nm 的文本输出跨平台不能直接比较
（排序与 _sdata 的差异，见 docs/ai/decisions/2026-10-25-迁移基线与对照.md §7），
这里改用 readelf -sW 归一化：只取有大小的 FUNC/OBJECT 符号，记录
（输出段、类型、绑定、大小、名称），不含地址，按多重集合比较。

用法：
    symbols.py dump <ELF>                     # 输出归一化的符号表
    symbols.py compare <基准> <ELF>           # 基准可以是 ELF 或 dump 的输出；不一致时返回 1
    symbols.py compare --renames <改名表> <基准> <ELF>
                                              # 先按改名表换算基准中的名字（只改名、不改行为的提交）
    symbols.py sections <ELF>                 # 输出各段大小（辅助参考）

改名表是 TSV（# 开头为注释），每行"旧名<TAB>新名"，是 C++ 源码中的标识符（如命名空间
alg_math → math）；比较前把基准修饰名里的"长度+旧名"（8alg_math）换成"长度+新名"（4math）。
大小、类型等其余字段不换算，仍须逐项一致。

readelf 取自 --readelf、环境变量 READELF 或 PATH（Arm GNU 工具链的 arm-none-eabi-readelf，
或 binutils 的 readelf，两者对 ARM ELF 的输出一致）。
"""

from __future__ import annotations

import argparse
import collections
import os
import re
import subprocess
import sys
from pathlib import Path

SYMBOL = re.compile(
    r"^\s*\d+:\s+[0-9a-fA-F]+\s+(?P<size>\d+)\s+(?P<type>\w+)\s+(?P<bind>\w+)"
    r"\s+\w+\s+(?P<ndx>\w+)\s+(?P<name>\S.*)$"
)
SECTION = re.compile(
    r"^\s*\[\s*(?P<index>\d+)\]\s+(?P<name>\S+)\s+\S+\s+[0-9a-fA-F]+\s+[0-9a-fA-F]+"
    r"\s+(?P<size>[0-9a-fA-F]+)\s"
)
KEPT_TYPES = ("FUNC", "OBJECT")
HEADER = "# 输出段\t类型\t绑定\t大小\t名称"


def readelf(tool: str, *args: str) -> list[str]:
    result = subprocess.run(
        [tool, "-W", *args],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return result.stdout.splitlines()


def sections(tool: str, elf: Path) -> dict[int, tuple[str, int]]:
    table = {}
    for line in readelf(tool, "-S", str(elf)):
        match = SECTION.match(line)
        if match:
            table[int(match["index"])] = (match["name"], int(match["size"], 16))
    return table


def symbols(tool: str, elf: Path) -> collections.Counter[tuple[str, ...]]:
    names = {index: name for index, (name, _) in sections(tool, elf).items()}
    table: collections.Counter[tuple[str, ...]] = collections.Counter()
    for line in readelf(tool, "-s", str(elf)):
        match = SYMBOL.match(line)
        if not match or match["type"] not in KEPT_TYPES or match["size"] == "0":
            continue
        ndx = match["ndx"]
        section = names.get(int(ndx), ndx) if ndx.isdigit() else ndx
        table[
            (section, match["type"], match["bind"], match["size"], match["name"])
        ] += 1
    return table


def load(tool: str, path: Path) -> collections.Counter[tuple[str, ...]]:
    """读取基准：ELF 直接解析；文本按 dump 的格式读取。"""
    with path.open("rb") as handle:
        is_elf = handle.read(4) == b"\x7fELF"
    if is_elf:
        return symbols(tool, path)
    table: collections.Counter[tuple[str, ...]] = collections.Counter()
    for line in path.read_text("utf-8").splitlines():
        if line and not line.startswith("#"):
            table[tuple(line.split("\t"))] += 1
    return table


def load_renames(paths: list[Path]) -> list[tuple[re.Pattern[str], str]]:
    """读取改名表，转成修饰名中的替换规则。"""
    rules = []
    for path in paths:
        for line in path.read_text("utf-8").splitlines():
            if not line.strip() or line.startswith("#"):
                continue
            old, new = line.split("\t")
            pattern = re.compile(rf"(?<![0-9]){len(old)}{re.escape(old)}")
            rules.append((pattern, f"{len(new)}{new}"))
    return rules


def renamed(
    table: collections.Counter[tuple[str, ...]],
    rules: list[tuple[re.Pattern[str], str]],
) -> collections.Counter[tuple[str, ...]]:
    result: collections.Counter[tuple[str, ...]] = collections.Counter()
    for key, count in table.items():
        name = key[4]
        for pattern, replacement in rules:
            name = pattern.sub(replacement, name)
        result[(*key[:4], name)] += count
    return result


def format_rows(table: collections.Counter[tuple[str, ...]]) -> list[str]:
    rows = []
    for key in sorted(table, key=lambda key: (key[0], key[4], key[1], key[2], key[3])):
        rows.extend(["\t".join(key)] * table[key])
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--readelf", default=os.environ.get("READELF", "readelf"))
    commands = parser.add_subparsers(dest="command", required=True)
    dump = commands.add_parser("dump", help="输出归一化的符号表")
    dump.add_argument("elf", type=Path)
    compare = commands.add_parser("compare", help="与基准比较，不一致时返回 1")
    compare.add_argument(
        "--renames",
        type=Path,
        action="append",
        default=[],
        help="改名表（可重复）：比较前换算基准中的名字",
    )
    compare.add_argument("reference", type=Path, help="基准 ELF 或 dump 的输出")
    compare.add_argument("elf", type=Path)
    section_command = commands.add_parser("sections", help="输出各段大小")
    section_command.add_argument("elf", type=Path)
    args = parser.parse_args()

    if args.command == "dump":
        print(HEADER)
        print("\n".join(format_rows(symbols(args.readelf, args.elf))))
        return 0
    if args.command == "sections":
        for _, (name, size) in sorted(sections(args.readelf, args.elf).items()):
            if name and size:
                print(f"{name}\t{size}")
        return 0

    reference = load(args.readelf, args.reference)
    rules = load_renames(args.renames)
    if rules:
        before = reference
        reference = renamed(reference, rules)
        changed = sum(count for key, count in before.items() if key not in reference)
        print(f"按改名表换算了基准中 {changed} 个符号的名字")
    current = symbols(args.readelf, args.elf)
    missing = reference - current
    extra = current - reference
    total = sum(reference.values())
    for row in format_rows(missing):
        print(f"- {row}")
    for row in format_rows(extra):
        print(f"+ {row}")
    if missing or extra:
        print(
            f"符号集合不一致：基准 {total} 个，缺少 {sum(missing.values())} 个，"
            f"多出 {sum(extra.values())} 个（- 只在基准中，+ 只在 {args.elf.name} 中）",
            file=sys.stderr,
        )
        return 1
    print(f"符号集合一致：{total} 个（名称、大小、类型、绑定、输出段）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
