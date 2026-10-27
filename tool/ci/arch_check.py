#!/usr/bin/env python3
"""架构门禁：检查 target 依赖图与源码文本是否符合 docs/ai/architecture.md 的依赖规则。

用法：
    arch_check.py graph <targets.dot> --mode firmware|host   # cmake --graphviz 导出的依赖图
    arch_check.py scan                                       # 文本扫描（仓库根执行）

graph 检查（规则 1、2、5、6、15 与评审报告 §5.2）：
  - 每个 target 名都要符合命名白名单（新增一类 target 先改规则与本脚本）；
  - 每条边都要在分层允许的依赖表内；迁移期放行的边单独列出并注明例外编号；
  - 传递可达：app 到不了 port/board/生成区/assembly；框架到不了平台实现与产品；
    测试到不了 port/board/生成区/assembly；只有固件可执行文件能链接 assembly；
  - 无环；port、board 与生成区必须是 OBJECT 库；必需的 target 都存在。
scan 检查（规则 3、8、9、12、15）：契约头不出现厂商与 RTOS 名；CMake 不用 file(GLOB) 与
  全局 include_directories()；app/ 与 src/ 不用 #if 做裁剪（头文件保护与 __cplusplus 除外）；
  一级目录白名单；框架代码不引用产品的路径或 target。
迁移源 legacy/ 不在扫描范围内（例外 X-12）。
"""

from __future__ import annotations

import argparse
import collections
import re
import subprocess
import sys
from pathlib import Path

# ---- 依赖图 ----

NODE = re.compile(
    r'^\s*"(?P<id>node\d+)"\s*\[\s*label\s*=\s*"(?P<name>[^"]+)",\s*shape\s*=\s*(?P<shape>\w+)'
)
EDGE = re.compile(r'^\s*"(?P<source>node\d+)"\s*->\s*"(?P<target>node\d+)"')
EXTERNAL_SHAPE = "septagon"  # 外部库（libm、预编译的 CMSIS-DSP 等），不参与检查
IMPORTED = "::"  # 导入的 target（如 Threads::Threads）属于外部依赖，不参与检查
OBJECT_SHAPE = "hexagon"

# (层, 名称模式)。顺序有意义：先匹配先用。
LAYERS = [
    ("firmware", r"firmware_\w+"),
    ("assembly", r"assembly_\w+"),
    ("profile", r"profile_\w+"),
    ("hal_detail", r"hal_detail"),
    ("contract", r"hal|hal_os|boot"),
    ("utils", r"utils_\w+"),
    ("msg", r"msg"),
    ("lib", r"lib_\w+"),
    ("driver", r"driver_\w+"),
    ("middleware", r"middleware_\w+"),
    ("app_common", r"app_common"),
    ("app_tasks", r"app_tasks"),
    ("app", r"app_\w+"),
    ("os", r"os_freertos_\w+"),
    ("port", r"port_\w+"),
    ("board", r"board_\w+"),
    ("vendor", r"(mx_generated|rtos|usb)_\w+"),
    ("mock", r"platform_mock"),
    ("test", r"test_\w+"),
    ("legacy", r"legacy_build_options"),
]
FRAMEWORK = {"contract", "hal_detail", "utils", "msg", "lib", "driver", "middleware"}
PRODUCT = {"firmware", "assembly", "profile", "app", "app_common", "app_tasks"}
PLATFORM = {"os", "port", "board", "vendor", "mock"}
APP = {"app", "app_common", "app_tasks"}
FORBIDDEN_FOR_APP = {"port", "board", "vendor", "assembly"}
FORBIDDEN_FOR_TEST = {
    "port",
    "board",
    "vendor",
    "assembly",
    "os",
    "firmware",
}
APP_DEPENDENCIES = {"contract", "utils", "msg", "lib", "driver", "middleware", "legacy"}

# 每一层可以直接依赖的层（docs/ai/architecture.md §2.3、§3）。
ALLOWED = {
    "firmware": {"assembly", "port"},
    "assembly": APP | APP_DEPENDENCIES | {"profile", "os", "board"},
    "profile": set(),
    "contract": {"legacy"},
    "hal_detail": set(),
    "utils": set(),
    "msg": {"utils", "contract"},
    "lib": {"lib", "legacy", "utils"},
    "driver": {"contract", "lib", "driver", "utils", "legacy"},
    # 架构 v2 §2.3：middleware 只依赖 utils 与契约（hal/os；传输类服务另可依赖 hal），不依赖 driver、lib。
    "middleware": {"middleware", "contract", "utils", "legacy"},
    "app": APP_DEPENDENCIES | {"app_common"},
    "app_tasks": APP_DEPENDENCIES | {"app", "app_common", "os"},
    "app_common": APP_DEPENDENCIES | {"os"},
    "os": set(),
    "port": {"vendor", "contract", "hal_detail", "utils"},
    "board": {"contract", "hal_detail"},
    "vendor": set(),
    "mock": {"contract", "hal_detail", "utils"},
    "test": FRAMEWORK | {"mock", "app", "app_common"},
    "legacy": set(),
}

# 迁移期放行的具体边：(起点, 终点) → 依据。到期后删除对应条目。
TRANSITIONAL_EDGES = {
    ("hal", "hal_detail"): "X-07 迁移期 C ABI 层，P3a 删除",
    (
        "lib_estimate",
        "driver_bmi088",
    ): "已知结构债：ImuBodyMapping 依赖 BMI088 驱动，P3b 偿还",
    (
        "app_common",
        "app_tasks",
    ): "TaskManager 含任务表，P3b 把任务表拆到 assembly 后消除",
}
TRANSITIONAL_TARGETS = {
    # 生成区接缝：USB_DEVICE 调用 port 私有 C ABI（UsbCdc.h 引用 hal/detail）。
    # main.c/freertos.c 调用 Application_*（X-09）只是 include 路径，由组合根注入，不是依赖图上的边。
    "vendor": ({"hal_detail"}, "X-01 生成区接缝，P4 消除"),
}

REQUIRED = {
    "firmware": [
        "firmware",
        "assembly",
        "profile",
        "contract",
        "utils",
        "port",
        "board",
        "vendor",
        "os",
        "app",
        "lib",
        "driver",
    ],
    "host": ["contract", "mock", "test", "lib", "driver"],
}


def layer_of(name: str) -> str | None:
    for layer, pattern in LAYERS:
        if re.fullmatch(pattern, name):
            return layer
    return None


def parse_graph(path: Path) -> tuple[dict[str, str], list[tuple[str, str]]]:
    shapes: dict[str, str] = {}
    names: dict[str, str] = {}
    edges = []
    for line in path.read_text("utf-8").splitlines():
        if "legendNode" in line:
            continue
        node = NODE.match(line)
        if node:
            names[node["id"]] = node["name"]
            shapes[node["name"]] = node["shape"]
            continue
        edge = EDGE.match(line)
        if edge:
            edges.append((edge["source"], edge["target"]))
    targets = {
        name: shape
        for name, shape in shapes.items()
        if shape != EXTERNAL_SHAPE and IMPORTED not in name
    }
    # 自环是 $<TARGET_OBJECTS:...> 生成器表达式在 graphviz 中的表现（CMake 不允许 target 依赖自己），忽略。
    resolved = [
        (names[source], names[target])
        for source, target in edges
        if source != target and names[source] in targets and names[target] in targets
    ]
    return targets, sorted(set(resolved))


def reachable(start: str, graph: dict[str, set[str]]) -> set[str]:
    seen: set[str] = set()
    stack = [start]
    while stack:
        for nxt in graph.get(stack.pop(), ()):
            if nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    return seen


def find_cycle(graph: dict[str, set[str]]) -> list[str] | None:
    state: dict[str, int] = {}
    path: list[str] = []

    def visit(node: str) -> list[str] | None:
        state[node] = 1
        path.append(node)
        for nxt in sorted(graph.get(node, ())):
            if state.get(nxt) == 1:
                return [*path[path.index(nxt) :], nxt]
            if state.get(nxt) is None:
                found = visit(nxt)
                if found:
                    return found
        path.pop()
        state[node] = 2
        return None

    for node in sorted(graph):
        if node not in state:
            found = visit(node)
            if found:
                return found
    return None


def check_graph(path: Path, mode: str) -> list[str]:
    targets, edges = parse_graph(path)
    errors = []
    layers = {}
    for name in sorted(targets):
        layer = layer_of(name)
        if layer is None:
            errors.append(f"target 名不在命名白名单内：{name}")
        else:
            layers[name] = layer
    graph: dict[str, set[str]] = collections.defaultdict(set)
    notes = []
    for source, target in edges:
        graph[source].add(target)
        if source not in layers or target not in layers:
            continue
        source_layer, target_layer = layers[source], layers[target]
        if target_layer in ALLOWED[source_layer]:
            continue
        if (source, target) in TRANSITIONAL_EDGES:
            notes.append(
                f"{source} -> {target}：{TRANSITIONAL_EDGES[(source, target)]}"
            )
            continue
        allowed_targets, reason = TRANSITIONAL_TARGETS.get(source_layer, (set(), ""))
        if target_layer in allowed_targets:
            notes.append(f"{source} -> {target}：{reason}")
            continue
        errors.append(
            f"不允许的依赖：{source}（{source_layer}）-> {target}（{target_layer}）"
        )

    for name, layer in layers.items():
        closure = {layers[n] for n in reachable(name, graph) if n in layers}
        if layer in APP and closure & FORBIDDEN_FOR_APP:
            errors.append(
                f"规则 1：{name} 可以到达 {sorted(closure & FORBIDDEN_FOR_APP)}"
            )
        if layer in FRAMEWORK and closure & (PLATFORM | PRODUCT):
            errors.append(
                f"规则 2/15：框架 {name} 可以到达 {sorted(closure & (PLATFORM | PRODUCT))}"
            )
        if layer == "test" and closure & FORBIDDEN_FOR_TEST:
            errors.append(
                f"规则 6：测试 {name} 可以到达 {sorted(closure & FORBIDDEN_FOR_TEST)}"
            )
        if layer in {"port", "board", "vendor"} and targets[name] != OBJECT_SHAPE:
            errors.append(
                f"{name} 必须是 OBJECT 库（覆盖 HAL 弱符号的回调不能放进静态库）"
            )
    for source, target in edges:
        if layers.get(target) == "assembly" and layers.get(source) != "firmware":
            errors.append(
                f"规则 5：只有固件可执行文件可以链接 {target}，{source} 也链接了它"
            )

    cycle = find_cycle(graph)
    if cycle:
        errors.append("依赖成环：" + " -> ".join(cycle))
    present = set(layers.values())
    missing = [layer for layer in REQUIRED[mode] if layer not in present]
    if missing:
        errors.append(f"{mode} 构建缺少必需的 target 层：{missing}")

    print(f"依赖图：{len(targets)} 个 target，{len(edges)} 条边（{path}）")
    for note in sorted(set(notes)):
        print(f"  迁移期放行：{note}")
    return errors


# ---- 文本扫描 ----

WHITELIST = {
    "app": {
        "common",
        "imu",
        "chassis",
        "gimbal",
        "remote",
        "link",
        "launch",
        "debug_comm",
        "tasks",
    },
    "src": {
        "hal",
        "boot",
        "lib",
        "driver",
        "msg",
        "middleware",
        "utils",
        "third_party",
    },
    "platform": {"st", "ti", "mock"},
}
# ST HAL 的名字都是 HAL_ 后跟字母（HAL_GPIO_WritePin、HAL_OK）。文件自己的头文件保护宏
# （如 HAL_OS_CLOCK_HPP）扫描前先去掉，见 header_guard()。
VENDOR_NAMES = re.compile(r"(?i:stm32|cmsis|freertos)|\bHAL_[A-Za-z]")
GUARD = re.compile(r"^\s*#\s*ifndef\s+(\w+)\s*\n\s*#\s*define\s+\1\b", re.MULTILINE)
GLOBAL_INCLUDE = re.compile(r"(?<![\w])include_directories\s*\(", re.IGNORECASE)
FILE_GLOB = re.compile(r"\bfile\s*\(\s*GLOB", re.IGNORECASE)
CONDITIONAL = re.compile(r"^\s*#\s*(if|ifdef|ifndef|elif)\b(.*)$")
# 规则 9 迁移期放行的宏：迁移源的两个构建期开关（例外 X-14），P3b 收进 profile 后删除。
CONDITIONAL_EXCEPTIONS = ("IMU_ENABLE_DIAGNOSTICS", "BMI088_GYRO_BANDWIDTH_REGISTER")
PRODUCT_REFERENCE = re.compile(
    r"\b(app_\w+|assembly_\w+|profile_\w+|firmware_\w+)\b|(?<![\w/.])(app|config)/"
)
SOURCE_SUFFIXES = (".c", ".h", ".cpp", ".hpp")


def tracked(*patterns: str) -> list[Path]:
    result = subprocess.run(
        ["git", "-c", "core.quotePath=false", "ls-files", "--", *patterns],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )
    return [Path(line) for line in result.stdout.splitlines() if line]


def strip_comments(path: Path, text: str) -> str:
    if path.suffix in SOURCE_SUFFIXES:
        text = re.sub(
            r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.DOTALL
        )
        return re.sub(r"//[^\n]*", "", text)
    return re.sub(r"#[^\n]*", "", text)  # CMake 注释


def header_guard(text: str) -> str | None:
    match = GUARD.search(text)
    return match.group(1) if match else None


def conditional_violations(text: str) -> list[int]:
    """#if 只允许用于头文件保护与 __cplusplus（规则 9）。"""
    lines = text.splitlines()
    bad = []
    for number, line in enumerate(lines, 1):
        match = CONDITIONAL.match(line)
        if not match:
            continue
        keyword, rest = match.group(1), match.group(2).strip()
        if "__cplusplus" in rest or any(
            name in rest for name in CONDITIONAL_EXCEPTIONS
        ):
            continue
        if keyword == "ifndef":
            following = next(
                (line.strip() for line in lines[number:] if line.strip()), ""
            )
            if following == f"#define {rest}":
                continue
        bad.append(number)
    return bad


def check_scan() -> list[str]:
    errors = []
    for path in tracked("src/hal/*"):
        if path.suffix not in SOURCE_SUFFIXES:
            continue
        text = path.read_text("utf-8")
        guard = header_guard(text)
        if guard:
            text = re.sub(rf"\b{guard}\b", "", text)
        match = VENDOR_NAMES.search(text)
        if match:
            errors.append(
                f"规则 3：契约头 {path} 出现了厂商或 RTOS 名：{match.group(0)}"
            )
    for path in tracked("CMakeLists.txt", "*.cmake", "*/CMakeLists.txt"):
        if path.parts[0] == "legacy":
            continue
        code = strip_comments(path, path.read_text("utf-8"))
        if FILE_GLOB.search(code):
            errors.append(f"规则 8：{path} 使用了 file(GLOB)")
        if GLOBAL_INCLUDE.search(code):
            errors.append(f"规则 8：{path} 使用了全局 include_directories()")
    for path in tracked("app/*", "src/*"):
        if path.suffix in SOURCE_SUFFIXES:
            code = strip_comments(path, path.read_text("utf-8"))
            for number in conditional_violations(code):
                errors.append(
                    f"规则 9：{path}:{number} 用 #if 做条件编译（只允许头文件保护与 __cplusplus）"
                )
    for root, allowed in WHITELIST.items():
        for path in tracked(f"{root}/*"):
            if len(path.parts) > 2 and path.parts[1] not in allowed:
                errors.append(
                    f"规则 12：{root}/{path.parts[1]} 不在一级目录白名单内（{path}）"
                )
    for path in tracked("src/*", "platform/*", "cmake/*"):
        if "cubemx" in path.parts:
            continue  # 生成区由规则 7 与例外 X-09 管
        if (
            path.suffix in SOURCE_SUFFIXES
            or path.name == "CMakeLists.txt"
            or path.suffix == ".cmake"
        ):
            code = strip_comments(path, path.read_text("utf-8"))
            match = PRODUCT_REFERENCE.search(code)
            if match:
                errors.append(f"规则 15：框架文件 {path} 引用了产品：{match.group(0)}")
    print("文本扫描：规则 3、8、9、12、15")
    return sorted(set(errors))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    graph = commands.add_parser("graph", help="检查 cmake --graphviz 导出的依赖图")
    graph.add_argument("dot", type=Path)
    graph.add_argument("--mode", choices=sorted(REQUIRED), required=True)
    commands.add_parser("scan", help="文本扫描（仓库根执行）")
    args = parser.parse_args()

    errors = (
        check_graph(args.dot, args.mode) if args.command == "graph" else check_scan()
    )
    for error in errors:
        print(
            f"::error::{error}"
            if "GITHUB_ACTIONS" in __import__("os").environ
            else f"错误：{error}"
        )
    if errors:
        print(f"架构检查未通过：{len(errors)} 处", file=sys.stderr)
        return 1
    print("架构检查通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
