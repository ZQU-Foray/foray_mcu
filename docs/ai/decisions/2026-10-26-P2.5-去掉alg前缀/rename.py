#!/usr/bin/env python3
"""src/lib 子目录与命名空间去掉 alg_ 前缀（Decision-20261026-09）。

用法（仓库根执行，按顺序，每步单独提交）：
    rename.py --move       # git mv 子目录（文件内容不变）
    rename.py --rewrite    # 改写命名空间、限定名、include 路径、头文件保护宏与 CMake 源文件路径

改写只做两种整词替换，见 RENAMES：
- alg_<旧名> → <新名>：命名空间声明、限定名（alg_math::Vector3）、include 与 CMake 中的路径；
- LIB_ALG_<旧名>_ → LIB_<新名>_：头文件保护宏（由 include 路径大写得到）。
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

# 旧名（去掉 alg_ 后）→ 新名。controller 改为 control，与 target 名 lib_control、
# tree.md 的目标分类一致；odometry 只是命名空间（在 kinematics 目录里），没有同名目录。
RENAMES = {
    "controller": "control",
    "crc": "crc",
    "estimate": "estimate",
    "filter": "filter",
    "fsm": "fsm",
    "kinematics": "kinematics",
    "math": "math",
    "odometry": "odometry",
    "slope": "slope",
}
DIRECTORIES = [name for name in RENAMES if name != "odometry"]
SUFFIXES = (".c", ".h", ".cpp", ".hpp")
EXTRA_FILES = ("src/CMakeLists.txt",)
NAME = re.compile(r"\balg_(" + "|".join(RENAMES) + r")\b")
GUARD = re.compile(r"\bLIB_ALG_(" + "|".join(name.upper() for name in RENAMES) + r")_")


def git(*args: str) -> list[str]:
    result = subprocess.run(
        ["git", "-c", "core.quotePath=false", *args],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )
    return [line for line in result.stdout.splitlines() if line]


def move() -> None:
    for name in DIRECTORIES:
        git("mv", f"src/lib/alg_{name}", f"src/lib/{RENAMES[name]}")
    print(f"已移动 {len(DIRECTORIES)} 个目录")


def rewrite() -> None:
    files = [
        path
        for path in git("ls-files")
        if (path.endswith(SUFFIXES) or path in EXTRA_FILES) and "/cubemx/" not in path
    ]
    changed = 0
    for path in files:
        text = Path(path).read_text("utf-8")
        new = NAME.sub(lambda match: RENAMES[match[1]], text)
        new = GUARD.sub(lambda match: f"LIB_{RENAMES[match[1].lower()].upper()}_", new)
        if new != text:
            Path(path).write_text(new, "utf-8", newline="\n")
            changed += 1
    print(f"改写了 {changed} 个文件")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--move", action="store_true")
    mode.add_argument("--rewrite", action="store_true")
    args = parser.parse_args()
    if args.move:
        move()
    else:
        rewrite()
    return 0


if __name__ == "__main__":
    sys.exit(main())
