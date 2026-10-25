# Foray MCU（foray_mcu）

> **下位机（MCU）固件** · owner 角色：电控组
> `layer: MCU` —— **不在 L1–L5 分层内**，位于 L2 下行 HAL **之下**
> 当前状态：🟡 **结构重构 P0 进行中**（方案已确认；迁移源在 `legacy/`，新结构尚未开始构建）

## 1. 本仓是什么

RoboMaster 多兵种、多开发板的 MCU 下位机固件。本仓以 `ControllerCode`（GitHub `ZQU-Foray/foray_mcu_controller`）为迁移源（历史已导入，文件原样放在 `legacy/`），正在做结构重构：把"单板、单产品、单可执行文件"的工程改造成"产品线（兵种 × 开发板）"工程。

- 迁移源基线：`ControllerCode` `main` @ `a2cfd16`（2026-10-23，19 个提交，git 跟踪 333 个文件，项目自有代码约 2.3 万行）。
- 目标平台：STM32H723（DM-MC02，主力，先行）、STM32F407（RM C 板）、TI TM4C / AM26x（按需）；`src/` + `platform/` 作为可复用框架，另供 MSPM0 等小项目（大概率裸机）使用。
- 当前状态：已建 git，基线 tag `baseline/a2cfd16`；迁移源原样放在 `legacy/`，可在 `legacy/Platform/STM32H7/DM_MC02` 按原方式构建；目标结构目录（`app/ config/ platform/ src/ test/ tool/`）仍是空骨架；还没有根 CMakeLists 和测试。
- 结构方案与完整评审材料：**[docs/reviews/2026-10-25-foray_mcu-架构评审报告.md](docs/reviews/2026-10-25-foray_mcu-架构评审报告.md)**。
- 方案已确认（[decision.md](decision.md) Decision-20261025-01 至 -05），执行计划与进度见 [plan.md](plan.md)。

## 2. 目标结构（已确认，按阶段实施）

| 目录 | 职责 | 目标依赖方向 |
|---|---|---|
| `app/` | 业务能力实现：imu / chassis / gimbal / remote / link / launch … 与任务薄壳；**每个能力一个 target** | → `src/` 的契约与公共库；**禁止**依赖 `platform/` 实现与 `config/assembly` |
| `config/` | 装配根，拆两个角色：`profiles/<profile>/` 是纯数据（能力集合、参数、板卡，INTERFACE）；`assembly/` 是组合根（任务表、对象构造、启动入口，STATIC） | `assembly` → `profile` + `app` + `platform/board`；`profile` 为叶子，不依赖任何代码 |
| `platform/` | 平台层：`<vendor>/<mcu>/<board>/{cubemx,port,board}`；`mock/` 供宿主测试 | → `src/hal` 契约（用 C++ 实现）；**禁止**依赖 app/config；**禁止**把 HAL/CMSIS/FreeRTOS 类型放进公共头 |
| `src/` | 通用框架：`hal`（C++ 契约 + 最小 OS 契约 + boot 契约）、`lib`（纯算法/组件）、`driver`（设备与设备协议）、`msg`（最新值邮箱/SPSC 队列）、`middleware`、`utils`（含并发原语）、`third_party` | 层内依赖见评审报告 §5.7；**禁止**依赖 platform/app/config/RTOS/HAL；必须能独立构建、能裸机构建 |
| `test/` | 宿主单元/集成测试（ctest），链接 `platform/mock`；**必须入库** | → 被测 target + `platform/mock`；**不进固件链接图** |
| `tool/` | 烧录、调试、数据回放、代码生成、CI 脚本 | 无构建期依赖；不得再被 `.gitignore` 排除 |
| `docs/` | 架构、评审、迁移、决策与证据链 | — |

目标 CMake target 图与"禁止依赖"规则见评审报告 §5。**`src/bsp` 是早期骨架命名，已确认改为 `src/hal`（不设 `abi/` 子层）。**

**语言**：主语言 C++17（`-fno-rtti -fno-exceptions -fno-threadsafe-statics`，无堆分配）；C 只用于三处外部强制的接缝：CubeMX 生成区调用的启动入口、覆盖 HAL 弱符号的回调、生成区里的中断入口（评审报告 §5.11）。命名按 ROS 2 C++ 风格，格式按组织规则（4 空格、LF、UTF-8），分别由根目录 `.clang-tidy` 与 `.clang-format` 执行（评审报告 §5.12）。

## 3. 构建（计划，尚未落地）

重构完成后，切换产品只用一个开关：

```bash
cmake --preset arm-debug            # 默认 profile：infantry_v1 × board_dm × Debug
cmake --preset arm-release
cmake --preset host-tests && ctest --preset host-tests
cmake -S . -B build/hero-debug -DFORAY_PROFILE=hero_v1 ...   # 其他兵种/机器人
```

- 唯一入口为 `FORAY_PROFILE`；板卡由 profile 声明；工具链在 `cmake/toolchains/`；本机路径放 `CMakeUserPresets.json`（不入库）。
- 迁移验收用的基线对照（ControllerCode 可复现构建实测）：Debug `.bin` **238,320 B**，Release `.bin` **100,664 B**；SHA-256、工具链版本与对照判据见 [docs/decisions/2026-10-25-迁移基线与对照.md](docs/decisions/2026-10-25-迁移基线与对照.md)（证据来源已转存 [docs/assets/controller_code-固件恢复可行性评估.md](docs/assets/controller_code-固件恢复可行性评估.md)）。

## 4. 依赖

| 类型 | 依赖 |
|---|---|
| 组织自研仓 | `foray_interfaces` —— **仅消费**生成的 `generated/lower_link.hpp`（vendor，非构建期依赖）；**禁止手写双侧编解码** |
| 第三方 | STM32 HAL/LL、FreeRTOS（CMSIS-RTOS2 仅迁移期使用）、ST USB Device、CMSIS（随板级 `cubemx/` 一起提交） |

## 5. 下位机安全职责（硬路径）

链路失联后的安全必须由本仓自主保证，不依赖上位机。以下职责继承自 `ControllerCode`，**当前尚未实现**（列入 [plan.md](plan.md) P4）：

| 失联时长 | 动作 |
|---|---|
| 100 ms | 底盘速度归零 |
| 200 ms | 撤销开火授权，摩擦轮停 |
| 500 ms | 云台停止接受指令，保持当前位置 |

开火硬上限：连发数上限 + 发射间隔下限，独立于上位机，不接受任何来源覆盖。

## 6. 上游 / 下游

| 方向 | 对象 |
|---|---|
| **上游**（本仓依赖谁） | `foray_interfaces`（仅消费生成头文件）；历史基线 `ControllerCode` |
| **下游**（谁依赖本仓） | 无 —— 最底层硬件侧固件 |

```text
上位机（算法组）
   foray_platform / foray_decision / ...
        │   USB CDC
        │   （契约：foray_interfaces/protocol/lower_link.md）
        ▼
下位机（电控组）
   foray_mcu  ← 本仓
        │   CAN / UART / SPI / PWM
        ▼
   电机 · 云台 · 发射机构 · IMU
```

## 7. 仓库规范（组织五件套）

| 文件 | 内容 |
|---|---|
| [README.md](README.md) | 本仓定位、目标结构、构建与依赖 |
| [AGENTS.md](AGENTS.md) | AI 协作规范 |
| [plan.md](plan.md) | 分阶段执行计划与进度 |
| [tree.md](tree.md) | 目录结构说明与关键位置 |
| [decision.md](decision.md) | 关键工程决策日志（含本次重构的决策） |

尚未落地的组织合规件（P0 补齐）：`.foray-layer`、`CODEOWNERS`（`@ZQU-Foray/电控组`）、`.github/workflows/ci.yml`。

## 8. 参考

- [上位机 ↔ 下位机链路协议](https://github.com/ZQU-Foray/foray_interfaces/blob/dev/protocol/lower_link.md)
- [算法结构](https://github.com/ZQU-Foray/foray_docs/blob/main/algorithm_structure.md) / [仓库结构](https://github.com/ZQU-Foray/foray_docs/blob/main/repository_structure.md)
- [组织贡献指南](https://github.com/ZQU-Foray/.github/blob/main/CONTRIBUTING.md)
- 结构评审报告：[docs/reviews/2026-10-25-foray_mcu-架构评审报告.md](docs/reviews/2026-10-25-foray_mcu-架构评审报告.md)
- ControllerCode 既有架构图（PNG 内嵌 5 页，含附录 A–D 评审清单）：[docs/assets/controller_code-架构图.png](docs/assets/controller_code-架构图.png)
