#!/usr/bin/env python3
"""按根目录 .clang-format 检查或格式化本仓的手写 C/C++ 代码。

文件范围由本脚本统一定义，CI 与本地共用：git 跟踪的 .c/.h/.cpp/.hpp，
排除 CubeMX 生成区、厂商代码与 vendor 的生成头（EXCLUDE）。
clang-format 18 的 .clang-format-ignore 不支持 **、* 也不跨目录，
排除不了整棵目录树，因此不用它（见 decision.md Decision-20261026-04）。

用法：
    format.py --check                 # 全量检查，不合规时返回 1
    format.py --fix                   # 原地格式化
    format.py --check --since <提交>  # 只检查相对该提交改动过的文件
    format.py --list                  # 列出纳入范围的文件

clang-format 取自 --clang-format 参数、环境变量 CLANG_FORMAT 或 PATH，
版本须为 18（与 CI 一致）。
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys

SUFFIXES = ("*.c", "*.h", "*.cpp", "*.hpp")
EXCLUDE = re.compile(
    r"^legacy/Platform/STM32H7/DM_MC02/(Core|Drivers|Middlewares|USB_DEVICE)/"
    r"|^platform/.*/cubemx/"
    r"|^src/third_party/"
)
BATCH = 40


def git(*args: str) -> list[str]:
    result = subprocess.run(
        ["git", "-c", "core.quotePath=false", *args],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )
    return [line for line in result.stdout.splitlines() if line]


def files(since: str | None) -> list[str]:
    if since:
        changed = git("diff", "--name-only", "--diff-filter=ACMR", f"{since}...HEAD")
        candidates = [
            path for path in changed if path.endswith((".c", ".h", ".cpp", ".hpp"))
        ]
    else:
        candidates = git("ls-files", "--", *SUFFIXES)
    return sorted(path for path in candidates if not EXCLUDE.search(path))


def check_version(clang_format: str) -> None:
    output = subprocess.run(
        [clang_format, "--version"],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    print(output)
    if "version 18." not in output:
        print("需要 clang-format 18（与 CI 一致）", file=sys.stderr)
        sys.exit(2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--fix", action="store_true")
    mode.add_argument("--list", action="store_true")
    parser.add_argument("--since", help="只处理相对该提交改动过的文件")
    parser.add_argument(
        "--clang-format", default=os.environ.get("CLANG_FORMAT", "clang-format")
    )
    args = parser.parse_args()

    root = git("rev-parse", "--show-toplevel")[0]
    os.chdir(root)
    targets = files(args.since)
    if args.list:
        print("\n".join(targets))
        return 0
    if not targets:
        print("没有需要处理的 C/C++ 文件。")
        return 0

    check_version(args.clang_format)
    options = ["--dry-run", "--Werror"] if args.check else ["-i"]
    failed = False
    for start in range(0, len(targets), BATCH):
        batch = targets[start : start + BATCH]
        result = subprocess.run([args.clang_format, *options, *batch], check=False)
        failed = failed or result.returncode != 0
    action = "检查" if args.check else "格式化"
    print(f"{action} {len(targets)} 个文件：{'有不合规文件' if failed else '完成'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
