#!/usr/bin/env python3
"""P2.5 搬迁：把 legacy/ 中剩下的源码搬到目标位置，并机械改写 include 与头文件保护宏。

用法（仓库根执行，按顺序，每步单独提交）：
    move.py --move       # git mv（文件内容不变）
    move.py --rewrite    # 改写 include 与保护宏（只改这两类行）

规则同 P2（Decision-20261026-08）：文件名 snake_case；被生成代码按名字 include 的头文件保持原名
（"Application/Entry.h"、"UsbCdc.h"）；include 写成相对层根目录的路径；保护宏由 include 路径
（platform 下为 platform/ 之后的路径）大写得到。去向与理由见同目录 README.md 与 Decision-20261026-10。
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

APPLICATION = "legacy/Application"
PORT_OLD = "legacy/Platform/STM32H7/DM_MC02/Port"
PORT_NEW = "platform/st/stm32h7/board_dm/port"

# 原路径 → 新路径
MOVES = {
    f"{APPLICATION}/Entry.cpp": "config/assembly/entry_legacy.cpp",
    f"{APPLICATION}/Entry.h": "config/assembly/Application/Entry.h",  # 生成区按此路径 include
    f"{APPLICATION}/Task/TaskManager.cpp": "app/common/task_manager.cpp",
    f"{APPLICATION}/Task/TaskManager.hpp": "app/common/task_manager.hpp",
    f"{APPLICATION}/Imu/AttitudeEstimator.cpp": "app/imu/attitude_estimator.cpp",
    f"{APPLICATION}/Imu/AttitudeEstimator.hpp": "app/imu/attitude_estimator.hpp",
    f"{APPLICATION}/Imu/GyroBiasCalibration.cpp": "app/imu/gyro_bias_calibration.cpp",
    f"{APPLICATION}/Imu/GyroBiasCalibration.hpp": "app/imu/gyro_bias_calibration.hpp",
    f"{APPLICATION}/Imu/ImuDiagnostics.cpp": "app/imu/imu_diagnostics.cpp",
    f"{APPLICATION}/Imu/ImuDiagnostics.hpp": "app/imu/imu_diagnostics.hpp",
    f"{APPLICATION}/Imu/ZeroRateBiasObserver.cpp": "app/imu/zero_rate_bias_observer.cpp",
    f"{APPLICATION}/Imu/ZeroRateBiasObserver.hpp": "app/imu/zero_rate_bias_observer.hpp",
    "legacy/Platform/PortKit/PortKit.c": f"{PORT_NEW}/port_kit.c",
    "legacy/Platform/PortKit/PortKit.h": f"{PORT_NEW}/port_kit.h",
    f"{PORT_OLD}/UsbCdc.c": f"{PORT_NEW}/usb_cdc.c",
    f"{PORT_OLD}/UsbCdc.h": f"{PORT_NEW}/UsbCdc.h",  # 生成区按名字 include
}

# include 的旧写法 → 新写法（按搬迁前的查找规则逐个确认过，只有这些写法指向被搬的文件）
INCLUDES = {
    "Application/Imu/AttitudeEstimator.hpp": "imu/attitude_estimator.hpp",
    "Application/Imu/GyroBiasCalibration.hpp": "imu/gyro_bias_calibration.hpp",
    "Application/Imu/ImuDiagnostics.hpp": "imu/imu_diagnostics.hpp",
    "Application/Imu/ImuDiagnostics.cpp": "imu/imu_diagnostics.cpp",  # 宿主测试直接编译
    "Application/Imu/ZeroRateBiasObserver.hpp": "imu/zero_rate_bias_observer.hpp",
    "Application/Task/TaskManager.hpp": "common/task_manager.hpp",
    "Entry.h": "Application/Entry.h",  # entry_legacy.cpp 不再与头文件同目录
    "PortKit.h": "port_kit.h",
}
SUFFIXES = (".c", ".h", ".cpp", ".hpp")
INCLUDE = re.compile(r'^(\s*#\s*include\s*)"([^"]+)"', re.MULTILINE)
GUARD = re.compile(r"^\s*#\s*ifndef\s+(\w+)\s*\n\s*#\s*define\s+\1\b", re.MULTILINE)


def git(*args: str) -> list[str]:
    result = subprocess.run(
        ["git", "-c", "core.quotePath=false", *args],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )
    return [line for line in result.stdout.splitlines() if line]


def guard_of(path: str) -> str:
    """头文件保护宏：include 路径（platform 下为 platform/ 之后的路径）大写。"""
    for root in ("app/", "config/assembly/", "platform/"):
        if path.startswith(root):
            spelled = path[len(root) :]
            return re.sub(r"[^A-Za-z0-9]", "_", spelled).upper()
    raise ValueError(f"没有约定 include 根：{path}")


def move() -> None:
    for old, new in MOVES.items():
        Path(new).parent.mkdir(parents=True, exist_ok=True)
        git("mv", old, new)
    print(f"已移动 {len(MOVES)} 个文件")


def rewrite() -> None:
    moved_headers = {new for new in MOVES.values() if new.endswith((".h", ".hpp"))}
    changed = 0
    for path in git("ls-files"):
        if not path.endswith(SUFFIXES) or "/cubemx/" in path:
            continue  # 生成区不改
        text = Path(path).read_text("utf-8")
        new = INCLUDE.sub(
            lambda match: f'{match[1]}"{INCLUDES.get(match[2], match[2])}"', text
        )
        guard = GUARD.search(new)
        if guard and path in moved_headers:
            new = re.sub(rf"\b{guard[1]}\b", guard_of(path), new)
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
