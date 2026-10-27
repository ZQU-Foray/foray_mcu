# P2.5 第一部分：src/lib 去掉 alg_ 前缀（2026-10-26）

负责人要求"取消掉 alg_ 这样的前缀"。`src/lib` 的子目录与命名空间一起改，不改行为。决策见 [decision.md](../../../../decision.md) Decision-20261026-09。

## 对照表

目录名、命名空间名、target 名后缀三者一致：

| 原目录 / 命名空间 | 新目录 / 命名空间 | target（不变） |
|---|---|---|
| `alg_math` | `math` | `lib_math` |
| `alg_controller` | `control` | `lib_control` |
| `alg_crc` | `crc` | `lib_crc` |
| `alg_estimate` | `estimate` | `lib_estimate` |
| `alg_filter` | `filter` | `lib_filter` |
| `alg_fsm` | `fsm` | `lib_fsm` |
| `alg_kinematics`（另有命名空间 `alg_odometry`） | `kinematics`（`odometry`） | `lib_kinematics` |
| `alg_slope` | `slope` | `lib_slope` |

`alg_controller` 改为 `control`，不是字面去前缀的 `controller`：这样与已有 target 名 `lib_control`、[tree.md](../../../../tree.md) 目标结构的分类名一致。`component` 本来没有前缀，不变。机器可读的版本见 [改名表.tsv](改名表.tsv)。

## 提交

| 提交 | 内容 |
|---|---|
| `9f62a21` | `rename.py --move`：8 个目录 `git mv`，31 个文件全部 R100。本提交单独无法编译 |
| `ee2bcfc` | `rename.py --rewrite`：45 个文件 416 行整词替换（命名空间、限定名、include、保护宏、`src/CMakeLists.txt` 的源文件路径）。本提交可编译 |
| `bdada62` | `format.py --fix`：命名空间变短后的对齐调整，10 个文件 |
| `0b049d8` | `symbols.py compare --renames`；CI 带改名表对照 |

`ee2bcfc` 与 `bdada62` 登记在 `.git-blame-ignore-revs`。

## 复现

在 P2 的最后一个提交（`37bb46e`）上执行，每步单独提交：

```sh
python docs/ai/decisions/2026-10-26-P2.5-去掉alg前缀/rename.py --move
python docs/ai/decisions/2026-10-26-P2.5-去掉alg前缀/rename.py --rewrite
python tool/ci/format.py --fix
```

改写前已确认：代码中的 `alg_` 只出现在命名空间声明、限定名、include 路径与保护宏里，注释和字符串里没有。

## 对照方法

命名空间改名会改变修饰名（`_ZN8alg_math4normE...` → `_ZN4math4normE...`），符号集合不能直接与 P1 存档比较。`symbols.py compare --renames 改名表.tsv` 先把存档中修饰名里的"长度+旧名"换成"长度+新名"，再按原判据比较：名称、大小、类型、绑定、输出段逐项一致。

固件中没有大小随名字变化的符号（如 `__PRETTY_FUNCTION__` 的静态字符串），所以只换算名字、不放宽大小，判据仍然成立。

## 验证

| 项 | 结果 |
|---|---|
| 符号集合（带改名表，与 P1 存档比较） | Debug 1426 个（换算 70 个名字）、Release 1038 个（换算 47 个），一致；Windows xPack GCC 15.2.1 与 Linux Arm GNU 15.2.Rel1 各验证一次 |
| 反向对照（不带改名表） | 报不一致，说明对照确实覆盖了改名的符号 |
| 编译选项 | 与 P2 的 `compile_commands.json` 相比，只有 `lib` 下源文件路径不同 |
| 依赖图 | target 与边不变，两张图检查通过 |
| 宿主测试 | 14/14（WSL g++ 11.4、MSYS2 g++ 16.2），零告警 |
| 命名、格式、架构检查 | 命名 0 处；格式 131 个文件通过；文本扫描通过 |
