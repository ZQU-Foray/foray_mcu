#!/usr/bin/env python3
"""架构门禁：检查 target 依赖图与源码文本是否符合 docs/ai/architecture.md 的依赖规则。

用法：
    arch_check.py graph <targets.dot> --mode firmware|host|framework   # cmake --graphviz 导出的依赖图
    arch_check.py scan                                                 # 文本扫描（仓库根执行）
    arch_check.py atomics <构建目录> --nm <arm-none-eabi-nm>           # Cortex-M0+ 原子库调用
    arch_check.py callbacks <ELF> --nm <arm-none-eabi-nm> --port <目录>  # HAL 回调是否生效

graph 检查（规则 1、2、5、6、15 与评审报告 §5.2）：
  - 每个 target 名都要符合命名白名单（新增一类 target 先改规则与本脚本）；
  - 每条边都要在分层允许的依赖表内；迁移期放行的边单独列出并注明例外编号；
  - 传递可达：app 到不了 port/board/生成区/assembly；框架到不了平台实现与产品；
    测试到不了 port/board/生成区/assembly；只有固件可执行文件能链接 assembly；
  - 无环；port、board 与生成区必须是 OBJECT 库；必需的 target 都存在；
  - framework 模式（框架独立构建）：图里不得出现产品、板卡、RTOS 或测试，连未定义的同名库也不行。
scan 检查（规则 3、8、9、12、13、15、17）：契约头不出现厂商与 RTOS 名；CMake 不用 file(GLOB) 与
  全局 include_directories()；app/ 与 src/ 不用 #if 做裁剪（头文件保护与 __cplusplus 除外）；
  一级目录白名单；不用原子读改写；框架代码不引用产品的路径或 target；topic 只在 app/topics 定义、
  每个 topic 只在一处取得发布句柄、清单头只 include 标准库与 topic 机制。
atomics 检查（规则 13）：framework-m0plus 构建出的静态库里没有 __atomic_*、__sync_* 库调用。
  Cortex-M0+ 没有 LDREX/STREX，原子读改写只能编译成库调用；文本扫描查不到的写法（如对原子变量用 ++）在这里现形。
callbacks 检查（规则 16）：port 源码里定义的每个 HAL_*Callback，在固件里都是强符号（覆盖了 HAL 的弱实现）；
  固件里没有被 C++ 改名的 HAL 回调（.cpp 没有 include 对应 HAL 头时，定义得到 C++ 链接，弱实现照旧生效）。
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
    # boot 单独成层：只有生成区与组合根可以依赖它（规则 7，Decision-20261031-11）
    ("boot", r"boot"),
    ("contract", r"hal|hal_os"),
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
    ("check", r"check_\w+"),
    ("legacy", r"legacy_build_options"),
]
FRAMEWORK = {
    "contract",
    "boot",
    "hal_detail",
    "utils",
    "msg",
    "lib",
    "driver",
    "middleware",
}
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
    "assembly": APP | APP_DEPENDENCIES | {"profile", "os", "board", "boot"},
    "profile": set(),
    "boot": set(),
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
    # 生成区只调 boot_*（USER CODE 区）；X-01 的 USB 接缝见 TRANSITIONAL_TARGETS
    "vendor": {"boot"},
    "mock": {"contract", "hal_detail", "utils"},
    "test": FRAMEWORK | {"mock", "app", "app_common"},
    # 框架独立构建里的检查用 target（如 check_templates：实例化只有头文件的模板），只能依赖框架。
    "check": FRAMEWORK | {"legacy"},
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
        "boot",
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
    "framework": ["contract", "utils", "middleware", "lib", "driver", "mock", "check"],
}
# 框架独立构建（规则 2、15）中不得出现的层：产品、板卡与 RTOS、测试。
FORBIDDEN_IN_FRAMEWORK = PRODUCT | {"os", "port", "board", "vendor", "test"}


def layer_of(name: str) -> str | None:
    for layer, pattern in LAYERS:
        if re.fullmatch(pattern, name):
            return layer
    return None


def parse_graph(
    path: Path,
) -> tuple[dict[str, str], list[tuple[str, str]], set[str]]:
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
    # 外部库：链接了一个不存在的 target 名时，CMake 把它当成库文件名，图里显示为外部库
    externals = {
        name
        for name, shape in shapes.items()
        if shape == EXTERNAL_SHAPE and IMPORTED not in name
    }
    # 自环是 $<TARGET_OBJECTS:...> 生成器表达式在 graphviz 中的表现（CMake 不允许 target 依赖自己），忽略。
    resolved = [
        (names[source], names[target])
        for source, target in edges
        if source != target and names[source] in targets and names[target] in targets
    ]
    return targets, sorted(set(resolved)), externals


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
    targets, edges, externals = parse_graph(path)
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

    if mode == "framework":
        for name in sorted(set(targets) | externals):
            if layer_of(name) in FORBIDDEN_IN_FRAMEWORK:
                kind = "外部库" if name in externals else "target"
                errors.append(
                    f"规则 2/15：框架独立构建中出现了{kind} {name}（{layer_of(name)}）"
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
        "topics",
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
    "platform": {"dm_mc02", "mock"},
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

# 规则 13：std::atomic 的读改写（成员函数与 std::atomic_* 自由函数）。std::exchange 不是原子操作，不算。
ATOMIC_RMW = re.compile(
    r"(?:\.|->)\s*(?:fetch_\w+|exchange|compare_exchange_\w+|test_and_set)\s*\("
    r"|\bstd::atomic_(?:fetch_\w+|exchange\w*|compare_exchange_\w+|flag_test_and_set\w*)\s*\("
)
# 例外 X-08 放行的文件 → 到期。改掉后删除对应条目（文件里已经没有时，扫描会提示）。
ATOMIC_RMW_EXCEPTIONS = {
    "src/driver/motor/dji/dji_motor.cpp": "X-08，F3 驱动适配时改",
    "app/remote/remote_receiver.cpp": "X-08，F2 随旧 app 归档",
    "app/tasks/imu_data_ready_mailbox.hpp": "X-08，F2 随旧 app 归档",
    "app/tasks/imu_task.cpp": "X-08，F2 随旧 app 归档",
}
# 规则 17：topic 只在产品清单（app/topics）定义；topic 机制本身在 src/middleware/topic。
TOPIC_TYPE = re.compile(r"\bTopic\s*<")
TOPIC_HOMES = ("app/topics/", "src/middleware/topic/")
CLAIM_PUBLISHER = re.compile(r"([A-Za-z_][\w:]*)\s*(?:\.|->)\s*claim_publisher\s*\(")
# 清单头只能 include 标准库（不带 .h 的名字）、topic 机制、时基与清单里的其他头：消息不能用 lib 等层的类型。
INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]', re.MULTILINE)
TOPIC_QUOTED_INCLUDES = re.compile(
    r"middleware/topic/topic\.hpp|hal/os/clock\.hpp|topics/[\w/]+\.hpp"
)
STANDARD_HEADER = re.compile(r"[a-z_]+")
# 规则 7：生成区的 #include "…" 只能指向生成区自己的头，或下面这些（Decision-20261031-11）
GENERATED_EXTERNAL_INCLUDES = {
    "boot/boot.h": "启动入口（规则 7）",
    "UsbCdc.h": "X-01：USB 生成代码调用 port 的 UsbCdcPort_*",
}
# 厂商 HAL 的头：stm32h7xx_hal_conf.h 按模块开关列出全部 HAL 头，没启用的模块不在仓库里
VENDOR_HEADER = re.compile(r"stm32\w*\.h")
# Cortex-M0+ 上原子读改写编译成的库调用
ATOMIC_CALL = re.compile(r"\b__(?:atomic|sync)_\w+")
# 规则 16：port 源码里的 HAL 回调定义（有函数体，不是声明）；固件 nm 输出中的一行；被 C++ 改名的 HAL 回调
CALLBACK_DEFINITION = re.compile(r"\b(HAL_\w+Callback)\s*\([^;{]*\)\s*\{")
NM_LINE = re.compile(r"^(?:[0-9a-fA-F]+\s+)?(?P<kind>\w)\s+(?P<name>\S+)$")
MANGLED_CALLBACK = re.compile(r"^_Z\w*?\d+HAL_\w*Callback")


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
    errors += check_atomic_rmw()
    errors += check_topics()
    errors += check_generated_includes()
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
    print("文本扫描：规则 3、7（include）、8、9、12、13、15、17")
    return sorted(set(errors))


def handwritten_sources(*roots: str) -> list[Path]:
    """手写的 C/C++ 源码：排除生成区与 vendor 头。"""
    return [
        path
        for path in tracked(*(f"{root}/*" for root in roots))
        if path.suffix in SOURCE_SUFFIXES
        and "cubemx" not in path.parts
        and "third_party" not in path.parts
    ]


def check_generated_includes() -> list[str]:
    """规则 7：生成区（Drivers、Middlewares 之外）只 include 生成区自己的头与登记过的接口头。"""
    errors = []
    generated = tracked("platform/*/cubemx/*")
    roots = {path.parts[1] for path in generated}
    for board in sorted(roots):
        files = [path for path in generated if path.parts[1] == board]
        headers = {path.name for path in files if path.suffix == ".h"}
        for path in files:
            if path.suffix not in (".c", ".h") or {"Drivers", "Middlewares"} & set(
                path.parts
            ):
                continue
            code = strip_comments(path, path.read_text("utf-8", errors="replace"))
            for kind, header in INCLUDE.findall(code):
                name = Path(header).name
                if kind != '"' or name in headers or VENDOR_HEADER.fullmatch(name):
                    continue
                if header not in GENERATED_EXTERNAL_INCLUDES:
                    errors.append(
                        f'规则 7：生成区 {path.as_posix()} include 了 "{header}"：'
                        "生成区只能调用 boot_*（src/boot/boot.h）与登记过的接口"
                    )
    return errors


def check_atomic_rmw() -> list[str]:
    """规则 13：不用 std::atomic 的读改写，例外 X-08 的文件放行。"""
    errors = []
    seen = set()
    for path in handwritten_sources("src", "app", "config", "platform"):
        code = strip_comments(path, path.read_text("utf-8"))
        for match in ATOMIC_RMW.finditer(code):
            name = path.as_posix()
            if name in ATOMIC_RMW_EXCEPTIONS:
                seen.add(name)
                continue
            number = code.count("\n", 0, match.start()) + 1
            errors.append(
                f"规则 13：{name}:{number} 用了原子读改写 {match.group(0).strip()}"
                "（Cortex-M0+ 不支持，改用 utils/concurrency 的原语或 topic）"
            )
    for name in sorted(set(ATOMIC_RMW_EXCEPTIONS) - seen):
        print(f"  提示：例外 X-08 放行的 {name} 已经没有原子读改写，可以删掉这一条")
    return errors


def check_topics() -> list[str]:
    """规则 17：topic 只在 app/topics 定义；每个 topic 只在一处取得发布句柄；清单头只 include 允许的头。"""
    errors = []
    claims: dict[str, list[str]] = collections.defaultdict(list)
    for path in handwritten_sources("src", "app", "config", "platform"):
        name = path.as_posix()
        code = strip_comments(path, path.read_text("utf-8"))
        if not name.startswith(TOPIC_HOMES):
            for match in TOPIC_TYPE.finditer(code):
                number = code.count("\n", 0, match.start()) + 1
                errors.append(
                    f"规则 17：{name}:{number} 定义或使用了 topic::Topic，topic 只在 app/topics 定义"
                )
        for match in CLAIM_PUBLISHER.finditer(code):
            number = code.count("\n", 0, match.start()) + 1
            claims[match.group(1)].append(f"{name}:{number}")
        if name.startswith("app/topics/"):
            for kind, header in INCLUDE.findall(code):
                allowed = (
                    TOPIC_QUOTED_INCLUDES.fullmatch(header)
                    if kind == '"'
                    else STANDARD_HEADER.fullmatch(header)
                )
                if not allowed:
                    shown = f'"{header}"' if kind == '"' else f"<{header}>"
                    errors.append(
                        f"规则 17：清单头 {name} 不能 include {shown}：消息只用标准标量与定长数组"
                    )
    for topic, places in sorted(claims.items()):
        if len(places) > 1:
            errors.append(
                f"规则 17：{topic} 在 {len(places)} 处取得发布句柄（{'、'.join(places)}），每个 topic 只有一个发布者"
            )
    return errors


# ---- Cortex-M0+ 原子库调用 ----


def check_atomics(build: Path, nm: str) -> list[str]:
    """规则 13：framework-m0plus 构建出的静态库里不得有 __atomic_*、__sync_* 库调用。"""
    archives = sorted(
        path for path in build.rglob("*.a") if "CMakeFiles" not in path.parts
    )
    errors = []
    if not any(path.name == "libcheck_templates.a" for path in archives):
        errors.append(
            f"{build} 里没有 libcheck_templates.a：先用 framework-m0plus preset 构建"
        )
    for archive in archives:
        result = subprocess.run(
            [nm, "-u", str(archive)],
            check=True,
            stdout=subprocess.PIPE,
            text=True,
            encoding="utf-8",
        )
        calls = sorted(set(ATOMIC_CALL.findall(result.stdout)))
        if calls:
            errors.append(
                f"规则 13：{archive.relative_to(build)} 调用了 {'、'.join(calls)}"
                "（Cortex-M0+ 上原子读改写编译成库调用）"
            )
    print(f"原子库调用检查：{len(archives)} 个静态库（{build}）")
    return errors


# ---- HAL 回调符号 ----


def check_callbacks(elf: Path, nm: str, ports: list[Path]) -> list[str]:
    """规则 16：port 定义的 HAL 回调在固件里都是强符号；没有被 C++ 改名的 HAL 回调。"""
    defined: dict[str, str] = {}
    for port in ports:
        for path in sorted(port.rglob("*")):
            if path.suffix not in SOURCE_SUFFIXES:
                continue
            code = strip_comments(path, path.read_text("utf-8"))
            for match in CALLBACK_DEFINITION.finditer(code):
                number = code.count("\n", 0, match.start()) + 1
                defined[match.group(1)] = f"{path.as_posix()}:{number}"
    result = subprocess.run(
        [nm, str(elf)], check=True, stdout=subprocess.PIPE, text=True, encoding="utf-8"
    )
    kinds: dict[str, set[str]] = collections.defaultdict(set)
    for line in result.stdout.splitlines():
        match = NM_LINE.match(line.strip())
        if match:
            kinds[match["name"]].add(match["kind"])
    errors = []
    for name, place in sorted(defined.items()):
        if "T" in kinds.get(name, set()):
            continue
        state = "弱实现仍在生效" if kinds.get(name) else "没有进入固件"
        errors.append(
            f"规则 16：{place} 定义了 {name}，但固件里{state}"
            "（C++ 文件要 include 对应 HAL 头；port 必须是 OBJECT 库）"
        )
    for name in sorted(kinds):
        if MANGLED_CALLBACK.match(name):
            errors.append(
                f"规则 16：固件里有被 C++ 改名的 HAL 回调 {name}（定义它的 .cpp 没有 include 对应 HAL 头）"
            )
    if not defined:
        errors.append(f"{'、'.join(map(str, ports))} 里没有找到 HAL 回调定义，检查路径")
    print(f"HAL 回调检查：port 定义 {len(defined)} 个（{elf}）")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    graph = commands.add_parser("graph", help="检查 cmake --graphviz 导出的依赖图")
    graph.add_argument("dot", type=Path)
    graph.add_argument("--mode", choices=sorted(REQUIRED), required=True)
    commands.add_parser("scan", help="文本扫描（仓库根执行）")
    atomics = commands.add_parser(
        "atomics", help="检查 framework-m0plus 构建的静态库没有原子库调用"
    )
    atomics.add_argument("build", type=Path)
    atomics.add_argument("--nm", required=True, help="arm-none-eabi-nm 的路径")
    callbacks = commands.add_parser(
        "callbacks", help="检查 port 的 HAL 回调在固件里生效"
    )
    callbacks.add_argument("elf", type=Path)
    callbacks.add_argument("--nm", required=True, help="arm-none-eabi-nm 的路径")
    callbacks.add_argument(
        "--port",
        type=Path,
        action="append",
        required=True,
        help="port 源码目录，可重复",
    )
    args = parser.parse_args()

    if args.command == "graph":
        errors = check_graph(args.dot, args.mode)
    elif args.command == "atomics":
        errors = check_atomics(args.build, args.nm)
    elif args.command == "callbacks":
        errors = check_callbacks(args.elf, args.nm, args.port)
    else:
        errors = check_scan()
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
