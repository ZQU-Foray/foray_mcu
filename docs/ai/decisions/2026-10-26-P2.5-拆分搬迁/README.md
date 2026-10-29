# P2.5 搬迁：legacy/ 剩余源码的去向、复现与验证（2026-10-26）

把 `legacy/` 中剩下的 16 个源码文件搬到目标位置，删除迁移期的 `legacy_include`、`legacy_library()` 与 `portkit`，不改行为。决策见 [decision.md](../../../../decision.md) Decision-20261026-10。同属 P2.5 的 `src/lib` 去掉 `alg_` 前缀见 [另一份记录](../2026-10-26-P2.5-去掉alg前缀/README.md)。

## 去向

| 迁移前（`legacy/` 下） | 搬迁后 | target |
|---|---|---|
| `Application/Entry.cpp` | `config/assembly/entry_legacy.cpp` | `assembly_<profile>` |
| `Application/Entry.h` | `config/assembly/Application/Entry.h`（路径与文件名保持） | 同上 |
| `Application/Task/TaskManager.{cpp,hpp}` | `app/common/task_manager.{cpp,hpp}` | `app_common` |
| `Application/Imu/*`（4 组） | `app/imu/`（snake_case） | `app_imu` |
| `Platform/PortKit/PortKit.{c,h}` | `platform/st/stm32h7/board_dm/port/port_kit.{c,h}` | `port_<board>` |
| `Platform/STM32H7/DM_MC02/Port/UsbCdc.c` | `.../port/usb_cdc.c` | `port_<board>` |
| `Platform/STM32H7/DM_MC02/Port/UsbCdc.h` | `.../port/UsbCdc.h`（文件名保持） | `port_<board>` |

`legacy/` 只剩迁移源 README 与架构图。

### 与计划不同的两处

**PortKit 整体并入 port，没有拆到 `src/utils` 与 `src/hal/os`**。计划写的是"队列/通知 → `src/utils`，临界区契约 → `src/hal/os`，实现 → port"，但原样拆分做不到：

- 队列与通知本身也用 CMSIS 的 `__DMB()`（`cmsis_compiler.h`），原样放进零平台的 `src/utils` 会违反规则 3；
- 把临界区拆到另一个文件会改变生成代码：Release 下 `PortKit_Critical_Enter/Exit` 被内联进 `PortKit_Notifier_Set`（34 字节），拆开后变成函数调用，符号大小随之改变。

P3a 本来就要把 PortKit 改写为 C++：队列与通知通用化为 `src/utils` 并发原语（`std::atomic`），临界区落到 `src/hal/os` 契约与各板 port 的实现。拆分放到那时一起做。

**`Entry.h` 放在 `config/assembly/Application/Entry.h`**。CubeMX 生成的 `main.c`、`freertos.c` 以 `"Application/Entry.h"` 引用启动入口，生成区不能手改，头文件只能放在名为 `Application/` 的目录下。`config/CMakeLists.txt` 把 `config/assembly` 注入 `mx_generated_<board>` 的 include 路径（例外 X-09）。这样写有两个原因：

- 平台层的 CMake 不出现产品路径，规则 15 仍然成立；
- 这条耦合只是 include 路径，不是依赖图上的边。

P3b 生成区改为只调 `src/boot` 的 boot ABI 后删除。

## 提交

| 提交 | 内容 |
|---|---|
| `b98b113` | `move.py --move`：16 个文件 `git mv`，全部 R100。本提交单独无法编译 |
| `11cea0b` | `move.py --rewrite`：21 个文件只改 `#include` 与头文件保护宏所在的行 |
| `7be2ed9` | 构建与工具适配：各层 CMakeLists、`cmake/legacy.cmake`、`.clang-tidy`、`arch_check.py` |

`11cea0b` 登记在 `.git-blame-ignore-revs`。

## 复现

在 `src/lib` 去掉 `alg_` 前缀的最后一个提交（`bee9ca8`）上执行，每步单独提交：

```sh
python docs/ai/decisions/2026-10-26-P2.5-拆分搬迁/move.py --move
python docs/ai/decisions/2026-10-26-P2.5-拆分搬迁/move.py --rewrite
```

构建与工具的改动是手工完成的，见提交 `7be2ed9`。

## 验证

| 项 | 结果 |
|---|---|
| 符号集合（带 `alg_` 改名表，与 P1 存档比较） | Debug 1426 个、Release 1038 个，一致；Windows xPack GCC 15.2.1 与 Linux Arm GNU 15.2.Rel1 各验证一次 |
| 搬迁文件的目标文件 | `port_kit`、`usb_cdc`、`entry_legacy`、`task_manager` 四个目标文件与搬迁前相比，反汇编（含重定位）与 `.rodata/.data/.bss` 内容逐项相同（Debug、Release） |
| 编译选项 | 112 个固件源文件，除 include 目录外与搬迁前相同；唯一的例外是 `port_kit.c` 随 port 带上严格告警（`-Wextra -Werror`），无告警，生成代码见上一行 |
| 依赖图 | 删除 `portkit`、`legacy_include` 两个节点及其边；生成区 → legacy 的放行随之删除；两张图检查通过 |
| 宿主测试 | 14/14（WSL g++ 11.4、MSYS2 g++ 16.2），零告警 |
| 命名、格式、架构检查 | 命名 0 处（`.clang-tidy` 头文件过滤已去掉 `legacy/Application`）；格式 131 个文件通过；文本扫描通过 |
| 可追溯性 | 16 个文件全部 R100，`git log --follow` 可追溯到迁移源最早的提交 |
