# 架构判据与依赖规则（foray_mcu）

> **状态**：📄 占位（P0 完善）。本文件将成为本仓"一段代码该落哪层/能不能 include 谁"的唯一人读入口。
> **权威方案**：[reviews/2026-10-25-foray_mcu-架构评审报告.md](reviews/2026-10-25-foray_mcu-架构评审报告.md) §5（目标结构、CMake target 图、依赖规则）。若本文件与评审报告冲突，以评审报告和 [decision.md](../decision.md) 的 Accepted 条目为准。

## 1. 本文件将要固定的内容（P0 任务）

1. **分层判据**：各目录/子层回答什么问题、禁止回答什么问题（从 ControllerCode 架构图附录 A 与组织规范恢复/重写）。
2. **target 依赖图**：完整 DAG（含 `foray_boot`、`foray_utils`、`foray_hal_os`、`foray_os_freertos` 的可见性；port/board 为 OBJECT 库），箭头方向图例。
3. **允许/禁止依赖表**：`app / config / platform / src（hal、lib、driver、msg、middleware、utils）/ test / tool` 两两关系。
4. **例外登记**：UsbCdc 边界（Q8）、链接脚本 overlay（Q9）、OS 抽象（Q10）、Dji 型号映射（Q11）、生成区 `--wrap` seam、`-Wdouble-promotion` 豁免清单（Decision-20261025-03）、C 链接的三处例外（评审报告 §5.11）等。
5. **机器检查映射**：CI/`tool/ci` 脚本如何实现每条规则（target 图断言、文本模式扫描、生成区/哈希检查、框架独立构建与裸机构建）。
6. **框架与产品边界、平台可移植性约束**：哪些代码属于可复用框架；Cortex-M0+/M4F/M7/R5F 的差异如何约束框架代码（评审报告 §5.8、§5.9）。
7. **命名与格式**：ROS 2 C++ 命名、组织格式规则，以及 `.clang-tidy` / `.clang-format` 的维护方式（评审报告 §5.12、Decision-20261025-04）。

## 2. 当前过渡规则

在 P0 完成前，代码/PR 以评审报告 §5.3 的 16 条依赖规则为准；违反其中任一条需要在 PR 描述中引用评审决议，并在 [decision.md](../decision.md) 登记例外。
