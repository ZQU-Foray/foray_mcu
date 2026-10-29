#!/usr/bin/env python3
"""P2 机械搬迁：按规则生成移动表，执行 git mv，并机械改写 include 与头文件保护宏。

用法（仓库根执行，按顺序）：
    move.py --table 移动表.tsv     # 只生成移动表
    move.py --move                 # git mv / git rm（文件内容不变，单独提交）
    move.py --rewrite              # 改写 include 与保护宏（在 --move 之后执行）

规则（docs/ai/decisions/2026-10-26-P2-机械搬迁/README.md）：
- 文件名改为 snake_case；C++ 头文件统一 .hpp（迁移源 Algorithm 下的 .h 是 C++），C 头文件保持 .h；
- 被 CubeMX 生成代码按名字 include 的头文件保持原名（Port/Init.h；UsbCdc.h、Entry.h 在 P2.5）；
- include 一律写成相对层根目录的路径：src/ 下为 "hal/can.hpp"，app/ 下为 "tasks/imu_task.hpp"，
  测试为 "support/mock_motor.hpp"，影子头相对影子根，port 内部为同目录文件名；
- 头文件保护宏由新的 include 路径大写得到（"hal/detail/can.h" → HAL_DETAIL_CAN_H）。
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

LEGACY = "legacy"
BOARD_OLD = "legacy/Platform/STM32H7/DM_MC02"
BOARD_NEW = "platform/st/stm32h7/board_dm"
CUBEMX_NEW = f"{BOARD_NEW}/cubemx"
PORT_NEW = f"{BOARD_NEW}/port"
SHADOW = "test/host/support/shadow"

# 被 CubeMX 生成代码按名字 include，不能改名（生成区不手改）。
KEEP_NAME = {f"{BOARD_OLD}/Port/Init.h"}
# 迁移源的旧构建文件：根工程已取代，删除。
DELETE = [
    f"{BOARD_OLD}/CMakeLists.txt",
    f"{BOARD_OLD}/CMakePresets.json",
    f"{BOARD_OLD}/cmake/gcc-arm-none-eabi.cmake",
    "legacy/Platform/Interface/CMakeLists.txt",
    "legacy/Platform/PortKit/CMakeLists.txt",
]
# 留到 P2.5 的文件（评审报告 §7.2.1）。
STAY = (
    "legacy/Application/Entry.",
    "legacy/Application/Imu/",
    "legacy/Application/Task/TaskManager.",
    "legacy/Platform/PortKit/",
    f"{BOARD_OLD}/Port/UsbCdc.",
    "legacy/README.md",
    "legacy/.docs/",
)
CUBEMX_DIRS = ("Core/", "Drivers/", "Middlewares/", "USB_DEVICE/", "cmake/")
CUBEMX_FILES = (
    "mc02_cpp.ioc",
    ".mxproject",
    "startup_stm32h723xx.s",
    "STM32H723xG_flash.ld",
)
TASK_FILES = {
    "0_imu/ImuTask.cpp": "app/tasks",
    "0_imu/ImuTask.hpp": "app/tasks",
    "0_imu/ImuDataReadyMailbox.hpp": "app/tasks",
    "1_chassis/ChassisTask.cpp": "app/tasks",
    "1_chassis/ChassisTask.hpp": "app/tasks",
    "1_chassis/ChassisController.cpp": "app/chassis",
    "1_chassis/ChassisController.hpp": "app/chassis",
    "2_gimbal/GimbalTask.cpp": "app/tasks",
    "2_gimbal/GimbalTask.hpp": "app/tasks",
    "4_communication/RemoteReceiver.cpp": "app/remote",
    "4_communication/RemoteReceiver.hpp": "app/remote",
}
SOURCE_SUFFIXES = (".c", ".h", ".cpp", ".hpp")
INCLUDE = re.compile(r'^(\s*#\s*include\s*)"([^"]+)"', re.MULTILINE)
GUARD = re.compile(r"^\s*#\s*ifndef\s+(\w+)\s*\n\s*#\s*define\s+\1\b", re.MULTILINE)


def snake(stem: str) -> str:
    return re.sub(r"(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])", "_", stem).lower()


def renamed(path: str, cpp_header: bool) -> str:
    """目录不变，文件名改为 snake_case；cpp_header 时 .h 改为 .hpp。"""
    directory, name = path.rsplit("/", 1)
    if name.endswith(".md"):
        return path
    stem, suffix = name.rsplit(".", 1)
    if cpp_header and suffix == "h":
        suffix = "hpp"
    return f"{directory}/{snake(stem)}.{suffix}"


def target_of(path: str) -> str | None:
    """返回搬迁后的路径；不搬迁返回 None。"""
    if path in DELETE or path.startswith(STAY):
        return None
    if path == f"{BOARD_OLD}/.clangd":
        return ".clangd"
    if path.startswith(f"{BOARD_OLD}/"):
        rest = path[len(BOARD_OLD) + 1 :]
        if rest.startswith(CUBEMX_DIRS) or rest in CUBEMX_FILES:
            return f"{CUBEMX_NEW}/{rest}"
        if rest.startswith("Port/"):
            if path in KEEP_NAME:
                return f"{PORT_NEW}/{rest[5:]}"
            return renamed(f"{PORT_NEW}/{rest[5:]}", cpp_header=False)
        return None
    if path == "legacy/Libraries/README.md":
        return "src/README.md"
    if path.startswith("legacy/Libraries/Algorithm/"):
        return renamed("src/lib/" + path[len("legacy/Libraries/Algorithm/") :], True)
    if path.startswith("legacy/Libraries/Component/"):
        return renamed("src/lib/component/" + path.rsplit("/", 1)[1], True)
    if path.startswith("legacy/Libraries/Device/"):
        return renamed("src/driver/" + path[len("legacy/Libraries/Device/") :], True)
    if path.startswith("legacy/Libraries/Protocol/"):
        rest = path[len("legacy/Libraries/Protocol/") :]
        return renamed("src/driver/protocol/" + rest, True)
    if path.startswith("legacy/Platform/Interface/Detail/"):
        return renamed("src/hal/detail/" + path.rsplit("/", 1)[1], False)
    if path.startswith("legacy/Platform/Interface/"):
        return renamed("src/hal/" + path.rsplit("/", 1)[1], True)
    if path.startswith("legacy/Application/Task/"):
        rest = path[len("legacy/Application/Task/") :]
        if rest in TASK_FILES:
            return renamed(f"{TASK_FILES[rest]}/{rest.rsplit('/', 1)[1]}", True)
        return None
    if path.startswith("legacy/Application/Module/"):
        return renamed("app/gimbal/" + path.rsplit("/", 1)[1], True)
    if path.startswith("test/host/") and path.endswith(SOURCE_SUFFIXES):
        if path.startswith(f"{SHADOW}/Libraries/"):
            old = path[len(f"{SHADOW}/Libraries/Device/") :]
            return renamed(f"{SHADOW}/driver/{old}", True)
        if path.startswith(f"{SHADOW}/"):
            return None  # cmsis_os2.h 替代第三方头，保持原名
        return renamed(path, True)
    return None


def git(*args: str) -> list[str]:
    result = subprocess.run(
        ["git", "-c", "core.quotePath=false", *args],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )
    return [line for line in result.stdout.splitlines() if line]


def mapping(files: list[str]) -> dict[str, str]:
    table = {}
    for path in files:
        new = target_of(path)
        if new and new != path:
            table[path] = new
    return table


def include_root(path: str) -> str:
    """新位置的文件被 include 时，路径相对哪个根目录书写。"""
    for prefix in (f"{SHADOW}/", "src/", "app/", "test/host/"):
        if path.startswith(prefix):
            return path[len(prefix) :]
    if path.startswith(f"{PORT_NEW}/"):
        return path.rsplit("/", 1)[1]
    raise ValueError(f"没有约定 include 根：{path}")


def guard_of(path: str) -> str:
    if path.startswith("platform/"):
        spelled = path[len("platform/") :]
    else:
        spelled = include_root(path)
    return re.sub(r"[^A-Za-z0-9]", "_", spelled).upper()


def search_dirs(old_path: str) -> list[str]:
    """迁移前编译这个文件时 "..." include 的查找顺序（同目录优先）。"""
    return [
        old_path.rsplit("/", 1)[0],
        LEGACY,
        "legacy/Platform/Interface",
        "test/host",
        f"{BOARD_OLD}/Port",
        "legacy/Platform/PortKit",
    ]


def rewrite(table: dict[str, str]) -> int:
    inverse = {new: old for old, new in table.items()}
    current = git("ls-files", "--", *(f"*{suffix}" for suffix in SOURCE_SUFFIXES))
    old_files = {inverse.get(path, path) for path in current} | set(table)
    changed = 0
    for path in current:
        generated = "/cubemx/" in path or (
            path.startswith(f"{BOARD_OLD}/")
            and not path.startswith(f"{BOARD_OLD}/Port/")
        )
        if generated:
            continue  # 生成区不改
        old_path = inverse.get(path, path)
        text = Path(path).read_text("utf-8")
        original = text

        def replace(match: re.Match[str], old_path: str = old_path) -> str:
            spelled = match.group(2)
            for directory in search_dirs(old_path):
                candidate = str(Path(directory, spelled).as_posix())
                candidate = re.sub(r"/\./", "/", candidate)
                if candidate in old_files:
                    new = table.get(candidate)
                    if new is None:
                        return match.group(0)  # 指向未搬迁的文件，写法不变
                    return f'{match.group(1)}"{include_root(new)}"'
            return match.group(0)  # 第三方或生成区的头文件

        text = INCLUDE.sub(replace, text)
        guard = GUARD.search(text)
        if guard and path in inverse and path.endswith((".h", ".hpp")):
            text = re.sub(rf"\b{guard.group(1)}\b", guard_of(path), text)
        if text != original:
            Path(path).write_text(text, "utf-8", newline="\n")
            changed += 1
    return changed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--table", type=Path, help="把移动表写入该文件（TSV）")
    mode.add_argument("--move", action="store_true")
    mode.add_argument("--rewrite", action="store_true")
    args = parser.parse_args()

    if args.rewrite:
        # --move 之后：由当前文件反推移动表（规则是确定的，用搬迁前的文件清单重新计算）。
        base = git("rev-parse", "HEAD~1")[0]
        table = mapping(git("ls-tree", "-r", "--name-only", base))
        print(f"改写了 {rewrite(table)} 个文件")
        return 0

    files = git("ls-files")
    table = mapping(files)
    if args.table:
        with args.table.open("w", encoding="utf-8", newline="\n") as handle:
            handle.write("原路径\t新路径\n")
            for old, new in sorted(table.items()):
                handle.write(f"{old}\t{new}\n")
            for path in DELETE:
                handle.write(f"{path}\t（删除）\n")
        print(f"移动 {len(table)} 个文件，删除 {len(DELETE)} 个")
        return 0

    for old, new in sorted(table.items()):
        Path(new).parent.mkdir(parents=True, exist_ok=True)
        git("mv", old, new)
    git("rm", "-q", *DELETE)
    print(f"已移动 {len(table)} 个文件，删除 {len(DELETE)} 个")
    return 0


if __name__ == "__main__":
    sys.exit(main())
