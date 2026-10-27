# 关键工程决策日志（foray_mcu）

> 记录 fork patch、换版本、**破坏性接口变更**、架构取舍。
> 格式见 `foray_docs/repository_structure.md` §7.2。
> 状态约定：`Accepted` = 已生效；`Proposed` = 待评审/未生效；`Superseded` = 被新决策替代。

---

## Decision-20260219-01（继承自 ControllerCode，Accepted）

- **Date**：2026-02-19
- **Status**：Accepted（随本仓迁移继承）
- **Source**：ControllerCode `origin/chore/align-org-structure` @ `f1f46cd` 的 `decision.md`

**Context**：MCU 下位机固件此前未对齐组织仓库规范——无五件套、无 `CODEOWNERS`、无层级声明；同时上位机 ↔ 下位机链路协议已在 `foray_interfaces` 落定 v0.1（USB CDC）。

**Decision**：

1. 本仓层级声明为 **`layer: MCU`**，**不在 L1–L5 分层内**——下位机位于 L2 下行 HAL 之下。
2. 依赖方向：`depends_on_layers: [X2]`，但**仅消费生成的头文件**（vendor），非构建期依赖。
3. 应用层协议编解码**必须使用** `foray_interfaces/generated/lower_link.hpp`，禁止手写。
4. `CODEOWNERS` 指向 **`@ZQU-Foray/电控组`**。
5. 链路失联的安全降级**由本仓自主实现**（硬路径），不依赖上位机。

**Reason**：

- 算法结构的 6 层 + 装配根全部是上位机；下位机固件不在这套分层里，但 CI 的依赖方向校验需要一个显式声明，故用 `MCU` 而非硬塞进 `L0`。
- 手写两侧编解码必然在字节序 / 字段偏移 / CRC 上漂移——生成是唯一可靠解法。
- 链路失联时上位机可能已经死了；安全不能依赖一个可能已经不在的进程。

**Alternatives / Rejected**：

- 把本仓标为 `L0` —— 否决：会让它看起来像"最底层软件库"，而它其实是硬件侧固件。
- 把协议规范放本仓 —— 否决：跨队契约应放 X2 契约仓（`foray_interfaces`）。
- 上位机与下位机各自维护一份编解码 —— 否决：漂移风险不可接受。
- 安全定时器依赖上位机心跳 —— 否决：链路断了就失效，与"硬路径"要求矛盾。

---

## Decision-20261025-01（结构重构方案，Accepted）

- **Date**：2026-10-25
- **Status**：**Accepted**（2026-10-25，负责人确认评审报告 **v1.0** §11.1 的 Q1–Q18 全部采纳建议；评审材料见 [docs/ai/reviews/2026-10-25-foray_mcu-架构评审报告.md](docs/ai/reviews/2026-10-25-foray_mcu-架构评审报告.md)）
- **影响范围**：目录结构、CMake target 模型、平台契约、启动接缝、迁移流程；**不影响迁移源代码行为（P0–P2.5 为不改行为阶段，P3a 只改接口不改行为）**

**Context**：

- 本仓当前为空骨架（无 `.git`、无源码）；迁移源为 ControllerCode（`main` @ `a2cfd16`），P0 才迁入 git 历史。迁移源现有结构为 Application / Libraries / Platform / CubeMX 四层，依赖倒置在 include 层成立（业务对 Platform 的 26 条依赖全部指向 `Platform/Interface/*.hpp`），但存在四个结构性问题：
  1. 整车装配知识分散在 5 处（`Entry.cpp`、`TaskManager.cpp`、`ChassisTask.cpp`/`GimbalTask.cpp`、板级 CMake）；`DjiMotorModelMap` 中是电机型号的厂商固有参数，建议留在驱动层（Q11 默认采纳）；
  2. 构建为单 executable + `file(GLOB)` + 全仓 include，层没有链接期边界；
  3. CubeMX 生成代码反向 include 应用头（`main.c`/`freertos.c` → `Application/Entry.h`），启动权在生成区（`Entry.h` 已是纯 C 启动契约，收口工作量小）；
  4. 平台契约枚举是"本机器人能力集"（CAN 3 通道、SPI 3 设备、UART 2 端点、PWM 2 通道、GPIO 8 脚，Interface 与 Detail 双份硬编码），多板复用会污染公共契约。
- 新结构提案（app 按兵种 / config 装配根 / platform 按厂商和开发板 / src 通用框架 + bsp/topic/算法/组件）方向正确，但存在 `config` 双角色成环、`app` 按兵种复制分叉、`src/bsp` 命名与 os/boot 契约缺失（v0.4 起不再要求 C ABI 层）、CubeMX 边界破口四处必须先修正的边界。
- 另：ControllerCode 的 host 测试（`fd1edbb` 的 `Tests/` 26 文件 / 2,979 行；该提交共 28 文件 / 3,008 行）已被后续提交删除且被 `.gitignore` 屏蔽；`.docs/`、`Tools/`、`*.py`、`*.cfg`、`.clang-format/.clang-tidy` 等排除/删除导致测试、工具、证据链丢失。迁移基线数据（Debug/Release `.bin` 与 SHA）所在的《固件恢复可行性评估》也未入 git，本仓已转存到 `docs/assets/` 与 `docs/ai/decisions/`。本仓必须先把这些资产恢复，再谈结构迁移。
- 负责人确认的前提见 Decision-20261025-02（平台范围与开发前提）与 Decision-20261025-03（双精度浮点暂不处理）。

**Decision**：

1. 采用 `app / config / platform / src / test / tool / docs` 七块结构作为重构目标；`test`、`tool`、`docs` 与源码同为一级资产，必须入库。
2. **`config` 拆两个角色**：`profiles/<profile>/` 为纯数据（INTERFACE 叶子：能力集合、参数、板卡）；`assembly/` 为组合根（STATIC：任务表、对象构造、启动入口）。`app` 不依赖 `assembly`；参数由 assembly 注入。
3. **`app` 按能力/角色划分**（imu/chassis/gimbal/remote/link/launch…），兵种（`robot_type`，避免与 git branch 混淆）与机器人（robot）只作为 profile 的属性/实例；能力用链接期 target 组合，不用 `#ifdef`。
4. **`src/bsp` 调整为 `src/hal`**：契约统一用 **C++**（取消原 `Detail/*.h` 的 C ABI 层，P3a 合并；C 链接只留在生成区启动入口、HAL 弱符号回调、中断入口三处）+ **最小 OS 契约**（C++ RAII 临界区 + 单调时基；线程、队列、事件不进框架）+ boot 契约（`extern "C"` 声明头，实现为 C++）；契约头不再枚举板级逻辑设备，设备 ID 由 board 头提供、经 `Config` 注入。`platform` 拆 `cubemx/`（生成物）+ `port/`（手写 C++，P3a 由现有 C 文件改写）+ `board/`（pinmap/能力清单/启动胶水），port 与 board 用 OBJECT 库。
5. **生成区隔离 + boot ABI**：`platform/**/cubemx/**` 只放生成物；生成代码只调用稳定的 `foray_boot_init / init_rtos / run` C 契约（由 `Entry.h` 演进；裸机构建只用 Init + Run），不再 include app 头。
6. **迁移分 P0–P5**：P0 基线与测试恢复（迁移源整体放在 `legacy/`）→ P0.5 格式与命名统一（不改行为，见 Decision-20261025-04）→ P1 原地建 target 骨架（零源码改动）→ P2 机械搬迁（1:1 项）→ P2.5 混合文件拆分搬迁（不改行为）→ P3a 框架化与契约 v1（改接口不改行为，F407 验证）→ P3b 装配根/能力拆分（在新结构中重建丢失业务）→ P4 链路协议/UsbCdc/CI 矩阵（TI port 按需）→ P5 MSPM0 与框架提取（第一个小项目启动时）。每阶段可构建、可回滚、有对照数据。
7. **框架与产品分开**：框架（`src/`、`platform/`、`cmake/`）不得引用产品（`app/`、`config/`），必须能独立构建、能裸机构建；同仓开发，第一个小项目启动时用 `git subtree split` 提取。框架代码不直接用 `std::atomic` 读改写（用 `src/utils` 并发原语），并开启 `-Wdouble-promotion`。
8. 已于 2026-10-25 生效：Q1–Q18 按评审报告 v1.0 §11.1 的建议执行；仓库、环境与布局见 Decision-20261025-05。

**Reason**：

- 现有代码的分层哲学（依赖倒置、Libraries 零 RTOS、任务层薄壳、宿主测试）是资产；重构应"重新切片"而不是重写。
- `config` 单 target 同时"被 app 依赖"又"装配 app"在 CMake 图上成环；`app` 按兵种会导致同能力代码复制分叉。
- 生成区破界与契约枚举耦合是多板/多供应商复用的根障碍，必须先从构建与物理边界上隔离。
- 没有 target 边界与宿主测试，目录调整的收益接近零（现状单 executable + GLOB，任何文件都能 include 任何文件）。

**Alternatives（详见评审报告 §10）**：

- **方案 A（最小改造）**：保留 Application/Libraries/Platform 三目录，只新增 `config/{profiles,assembly}`、根 CMake/presets、test/tool/docs、CI；1–2 周、低风险，解决"能切兵种 + 能回归"约 60–70%，但 Libraries 混杂、Algorithm→Device 违规、平台契约与框架复用问题保留。v0.3 起只作为临近比赛节点时的退路。
- **方案 B（原提案照抄）**：`app/{兵种}` + `config/{机器人}` + 空 `src/bsp`；4–6 周、高风险（复制分叉、#ifdef 蔓延、契约污染），**不建议**。
- **方案 C（本决策 2–8 条，推荐）**：完整修正方案；P0–P4 约 40–75 人日（1 人约 8–15 周，不含 TI port、P5 与丢失业务重建）、中风险。负责人已确认后续多人、多平台且当前时间充裕，因此直接走 C。
- **P0 期间可先做**：恢复 Tests、root CMake/presets、CI 第一版（无论 A/C 都需要）。

**Rejected**：

- 一次性大重构（把路径/分层/构建/配置/协议五类变更叠在一个 PR）——回归无法定位，且当前基线不唯一（见评审报告 §1.4）。
- `app` 下按兵种建目录、复制 Application —— 业务分叉后修复无法汇总。
- 用 `#ifdef FORAY_CAP_*` / `USE_XXX` 做能力裁剪 —— 组合爆炸、不可枚举、无法 CI 矩阵验证。
- 把 `config` 做成单个"被 app 依赖的库" —— 构建循环/职责错位。
- 把 CubeMX 生成文件搬进"通用框架"、或让 Port include 应用头 —— 违反依赖倒置与生成区隔离。
- 用 `file(GLOB)` 收集业务源码、把仓库根加进全局 include —— 取消所有层边界。
- 在 `src/lib` 或 app 里手写链路协议编解码 —— 违反组织 X2 约束。

---

## Decision-20261025-02（平台范围与开发前提，Accepted）

- **Date**：2026-10-25
- **Status**：Accepted（负责人在评审讨论中确认；作为 Decision-20261025-01 的输入前提）
- **Source**：评审报告 v0.3 §1.5

**Context**：结构重构需要知道团队规模、目标平台与运行模型，才能决定框架要做到多通用。

**Decision**：

1. 当前单人开发，时间充裕；后续确定多人协作。因此现在就建立 target 边界、能力所有权边界与 CI；评审流程在单人阶段轻量化。
2. 主力平台 STM32H723（DM-MC02），先按 H723 开发；第二块板为 STM32F407（RM C 板），在 P3a 用来验证契约 v1。
3. TI 平台限定为 Cortex-M（TM4C）或 Cortex-R（AM26x），**不含 C2000**；契约设计沿用（v0.4 起统一为 C++ 契约，见 Decision-20261025-01 第 4 条）。
4. MSPM0（Cortex-M0+）不是团队开发主力，用于其他小型项目，**大概率裸机**；框架必须能裸机构建，MSPM0 port 在第一个小项目启动时（P5）再做。

**Reason**：

- 多人加入后再补边界和 CI，返工成本远高于现在建立。
- Cortex-R 没有 PRIMASK/BASEPRI、M0+ 没有无锁原子读改写（评审报告 §5.8 实测），这两点决定了最小 OS 契约与并发原语必须进框架。
- C2000 的 16 位 `char` 会影响全部字节级契约；排除它之后，现有契约设计可以沿用。

**Alternatives / Rejected**：

- 等第二个人加入再建边界 —— 否决：多人基于无边界的结构并行开发，后期分层成本更高。
- 现在就做 MSPM0 port —— 否决：没有真实项目驱动，容易抽象错位。

---

## Decision-20261025-03（双精度浮点暂不处理，Accepted）

- **Date**：2026-10-25
- **Status**：Accepted（负责人确认；两道防护随 P3a 落地）
- **Source**：评审报告 v0.3 §5.10（Q14）

**Context**：IMU 姿态解算（VQF、`Matrix3`、巴特沃斯低通、零偏标定）内部使用 double，热路径为陀螺 2000 Hz、加速度计 1600 Hz。H723 有硬件双精度；F407、TM4C（Cortex-M4F）上 double 会编译为软件库调用。double 是有意设计（`ButterworthLowPass.hpp:19`），且没有扩散：`-Wdouble-promotion` 只在 `Vqf.cpp` 报警；接口均为 float。

**Decision**：

1. 先按 H723 开发，双精度暂不处理。
2. 防护一：框架代码开启 `-Wdouble-promotion`，IMU 姿态解算文件豁免；新增豁免需在本文件登记。
3. 防护二：`Vqf.cpp` 增加编译期检查，目标无双精度 FPU（`__ARM_FP` 不含 0x8）时 `#error`。
4. 触发条件：第一个要在 M4F 板上链接 IMU 姿态解算能力的 profile 出现时，先实测耗时，再优先选混合精度方案，并用 `test/data` 的 IMU 基准数据对照。

**Reason**：

- 低性能平台不一定需要这类解算；改动只在 IMU 能力内部，推迟处理的成本低。
- 两道防护让问题在第一次编译时暴露，而不是上板后才发现 CPU 占用异常。

**Alternatives / Rejected**：

- 现在全部改为 float —— 否决：上游 VQF 明确指出 float 状态在长时运行下有数值问题，且需要精度验证。
- 现在按标量类型模板化 —— 否决：没有需求驱动，增加复杂度。

---

## Decision-20261025-04（命名与格式，Accepted）

- **Date**：2026-10-25
- **Status**：Accepted（负责人确认；配置已在 P0 提交，代码在 P0.5 统一）
- **Source**：评审报告 v1.0 §5.12（Q18）

**Context**：迁移源的命名是 LLVM 风格（函数 PascalCase、变量 camelBack），历史上两份 `.clang-format` 缩进不一致（`Libraries/Algorithm` 为 4，`Platform/STM32H7/DM_MC02` 为 2）。组织 CONTRIBUTING 规定 4 空格缩进、LF、UTF-8、文件末尾换行，C/C++ 命名为变量/函数 `snake_case`、类 `PascalCase`、宏/枚举 `UPPER_SNAKE_CASE`。

**Decision**：

1. 命名按 ROS 2 C++ 风格：类型 PascalCase；函数、方法、变量、参数 snake_case；私有与受保护成员加尾下划线；全局变量加 `g_` 前缀；命名空间 snake_case；枚举值、宏、编译期常量 UPPER_SNAKE_CASE；文件 snake_case，C++ 用 `.hpp`/`.cpp`。由根 `.clang-tidy`（readability-identifier-naming）执行。
2. 格式按组织规则：根 `.clang-format` 原样沿用历史 `Libraries/Algorithm/.clang-format`（缩进 4），追加 `LineEnding: LF` 与 `InsertNewlineAtEOF: true`；clang-format 统一用 18。
3. ROS 2 与组织规则冲突的排版项（缩进 2、行宽 100 等）以组织为准。
4. 手写代码在 P0.5 一次性格式化与改名，格式化与改名分开提交，并写入 `.git-blame-ignore-revs`；文件改名随 P2 搬迁进行；C ABI 在 P3a 合并时直接按新规则重写。

**Reason**：

- 组织已经规定了排版与基本命名；ROS 2 风格补齐了组织规定没覆盖的细节（成员后缀、全局前缀、文件扩展名），也与组织其他 ROS 2 仓库一致。
- 一次性统一并单独提交，可以用 `.bin` 的 SHA 不变来证明没有改变行为。

**Alternatives / Rejected**：

- 全部按 ROS 2（含 2 空格缩进）—— 否决：与组织的 4 空格规则冲突。
- 沿用迁移源的 LLVM 命名 —— 否决：与组织规定的 `snake_case` 不一致。

---

## Decision-20261025-05（仓库、开发环境与迁移源布局，Accepted）

- **Date**：2026-10-25
- **Status**：Accepted（负责人确认；第 1–5 项已执行，2026-10-26 完成 GitHub 首次推送；第 7 项的芯片存档已完成）
- **Source**：评审报告 v1.0 §1.4、§1.5、§7.4

**Decision**：

1. 本仓已建 git（`main`）。迁移源 ControllerCode（GitHub `ZQU-Foray/foray_mcu_controller`）作为 `legacy` remote 导入 main、dev、chore/align-org-structure 的历史，基线 tag 为 `baseline/a2cfd16`。
2. 迁移源文件整体放在 `legacy/`（一个纯重命名提交），避免 `Platform/` 与 `platform/` 在不区分大小写的文件系统上冲突；P0–P2.5 期间 `legacy/` 只做机械搬出，不在其中开发新功能。
3. `.gitattributes` 设为 `* -text`，不做行尾自动转换（迁移源有 47 个 CRLF 文件）；手写代码在 P0.5 统一为 LF。
4. 根 `.gitignore` 重写，只忽略构建产物、IDE 目录与本机配置。
5. 远程仓库在 GitHub ZQU-Foray 组织下新建（负责人有完整权限，Actions 可用），待创建后首次推送。
6. 开发以 Windows 为主，WSL 可选；基线在 Windows 上用 xPack GCC 15.2.1 复现；CI 在 GitHub Actions（Linux）上用同版本 GCC，跨环境对照以符号名与大小集合为准。
7. 迁移基线风险由负责人接受：`a2cfd16` 之后的进度（新电机控制、启动时间优化、上下位机通信、云台逻辑）找不回，在新结构中重做；芯片内固件在烧录新固件前先读出存档，作为行为参考。

**Reason**：

- `legacy/` 让迁移源保持可构建、零改动，同时与目标结构完全隔开；后续搬迁都是可追溯的重命名。
- 团队以 Windows 为主，仓库布局与行尾策略必须在 Windows 上不出问题。

---

## Decision-20261026-01（组织合规件、CI 第一版与分支模型，Accepted）

- **Date**：2026-10-26
- **Status**：Accepted（负责人授权推进 P0；CI 在首个 PR 上运行验证）
- **Source**：组织 `foray_docs/repository_structure.md` §6.4、§6.5、§7.3；组织 `ZQU-Foray/.github` 的 `workflows/ci-template.yml`；迁移源 `chore/align-org-structure` @ `f1f46cd` 的合规件

**Context**：组织要求每个仓库有 `.foray-layer`、`CODEOWNERS`，并把组织 CI 模板复制为 `.github/workflows/ci.yml`。模板的 lint-c 会对全部 C/C++ 文件做 clang-format 检查，而本仓的 `legacy/` 手写代码与从 `fd1edbb` 恢复的测试要到 P0.5 才统一格式，CubeMX 生成区与厂商代码则永远不应格式化。clang-format 18 的 `.clang-format-ignore` 不支持 `**`，`*` 也不跨目录（2026-10-26 实测），排除不了整棵目录树。

**Decision**：

1. `.foray-layer` 沿用迁移源 `f1f46cd` 的版本（`layer: MCU`、`depends_on_layers: [X2]`，与 Decision-20260219-01 一致）。
2. `CODEOWNERS` 放在仓库根目录，内容为 `* @ZQU-Foray/电控组`。链路协议的算法组 Review 规则等 P4 vendor 生成头时再加（届时需要先给算法组 Write 权限）。
3. `.github/workflows/ci.yml` 从组织模板复制：lint-python、test-python 保持原样（无 Python 文件时自动跳过），只把 ruff 改为 `--output-format=github` 输出，让错误直接标在 PR 的文件行上（2026-10-26 补充）；删除 build-ros2（本仓不是 ROS 2 工作空间）。
4. **lint-c 改为增量检查**：只检查相对 PR 基准（或推送前提交）改动的 C/C++ 文件；用 CI 脚本中的正则排除 `legacy/Platform/STM32H7/DM_MC02/{Core,Drivers,Middlewares,USB_DEVICE}/`、`platform/**/cubemx/`、`src/third_party/`；clang-format 通过 pip 固定为 18.1.8。P0.5 一次性格式化之后改为全量检查（排除项保留）。
5. 新增 **host-tests** 作业：构建并运行 `test/host`（ctest）。
6. 新增 **firmware-legacy** 作业：用 xPack GCC 15.2.1-1.1 linux-x64（发布包 SHA-256 固定在 CI 中）构建 `legacy/` 的 Debug 与 Release。**Release 镜像必须与基线 SHA-256 逐字节一致**（`docs/ai/decisions/baseline-a2cfd16/SHA256SUMS`）；Debug 镜像内含工具链安装路径，只要求构建成功。该闸门在 P0–P0.5 有效，P1 起改为根工程构建 + 符号名与大小集合对照。
7. **分支模型按组织规范 §6.1**（负责人 2026-10-26 确认）：从 `main`（`28a5533`）建 `dev`；日常 PR 一律合到 `dev`，功能分支从 `dev` 切出；`main` 只承载里程碑快照，每个阶段（如 P0）验收后由 `dev` 合入 `main`。

**Reason**：

- 增量检查沿用迁移源已经跑通的做法：新代码不放过，存量代码也不会让 CI 长期红灯（组织模板的设计原则同样是"长期红灯等于没有 CI"）。
- Release SHA 闸门把"P0–P0.5 不改变 legacy 行为"从约定变成了自动检查，P0.5 的格式化与改名尤其需要它；2026-10-26 已在 Linux 上验证 Release 能逐字节复现基线。

**Alternatives / Rejected**：

- 照搬模板的全量 lint-c：P0.5 之前必然红灯。否决。
- 用 `.clang-format-ignore` 排除生成区：clang-format 18 的模式语法排除不了整棵目录树。否决（P0.5 再评估按深度展开的写法）。
- 在生成区目录放 `DisableFormat: true` 的 `.clang-format`：要往 `legacy/` 与生成区里加文件，违反冻结约定。否决。
- 对 Debug 也做 SHA 闸门：Debug 镜像随工具链安装路径变化，CI 上不可能复现。否决。

**待负责人在 GitHub 上操作**：把默认分支改为 `dev`；给 `电控组` 本仓 Write 权限（否则 CODEOWNERS 静默失效）；`main` 与 `dev` 的分支保护（组织规范：PR + 1 approve、禁强推、禁删除）。

---

## Decision-20261026-02（docs/ai/architecture.md v1 与例外登记表，Accepted）

- **Date**：2026-10-26
- **Status**：Accepted（PR 合并即负责人评审通过）
- **Source**：评审报告 v1.0 §3.2–§3.3、§5、§8.4；Decision-20260219-01、Decision-20261025-01 至 -05、Decision-20261026-01

**Context**：评审报告 §8.4 要求 `docs/ai/architecture.md` 在 P0 创建并与 §5 的依赖规则一致。规则、例外与白名单此前分散在评审报告、多条决策与 plan.md 中，评审 PR 时没有一张可对照的清单。

**Decision**：

1. `docs/ai/architecture.md` v1 作为分层判据、16 条依赖规则、一级目录白名单、生成区治理与例外登记的唯一人读入口；规则条文与评审报告 §5.3 一致，另为每条规则标注检查方式与生效阶段（与 plan.md 的阶段对齐）。
2. 已批准的例外集中登记为 X-01–X-13；其中 X-10（`test/host` 在 P0 作为独立 CMake 工程直接编译 `legacy/` 源码，不链接被测 target 与 `platform/mock`）为本条新登记，P1 改由根工程引入后取消。
3. 违反目标规则但不属于例外的既有问题列为"已知结构债"；评审报告 §2.3 缺口 1（SPI 可选能力被当作必选）此前未排期，补入 plan.md P3a。
4. 此后新增或修改规则、白名单、例外：先在本文件登记，再改 `docs/ai/architecture.md` 与 CI 检查。

**Reason**：

- 评审 PR 与 AI 代理需要一份可直接对照的规则表；把"何时由什么检查执行"写清楚，避免把尚未生效的规则当作已执行。

**Alternatives / Rejected**：

- 只在 `docs/ai/architecture.md` 中引用评审报告章节：评审报告是历史记录，会随阶段推进与决策变化而过时，否决。

---

## Decision-20261026-03（AI 撰写的文档放入 docs/ai/，Accepted）

- **Date**：2026-10-26
- **Status**：Accepted（负责人要求）
- **Source**：负责人 2026-10-26 指示

**Context**：`docs/` 下的评审报告、架构判据、基线与决策证据都由 AI 撰写，与人工撰写的文档混在一起，读者无法区分来源与可信程度。

**Decision**：

1. AI 撰写的文档统一放在 `docs/ai/`：`architecture.md`、`README.md`（改为 docs/ai 索引）、`reviews/`（含 `archive/`）、`decisions/`（含 `baseline-a2cfd16/`）、`assets/controller_code-固件恢复可行性评估.md`（从迁移源转存，按写作时间与风格判断为 AI 撰写）。
2. 人工制作的材料留在 `docs/` 的其他位置：`docs/assets/controller_code-架构图.png`（迁移源既有架构图）。
3. 此后 AI 代理新写的文档一律放 `docs/ai/`（写入 `AGENTS.md`）；根目录五件套与各目录内 `README.md` 的位置由组织规范与工具约定固定，不受此限。
4. 移动用 `git mv` 完成（`git log --follow` 可追溯），全仓引用同步更新；评审报告历史版本（`archive/`）只修正链接目标、不改正文（其中 24 条链接在移动前就是断的，一并修复）。

**Reason**：区分 AI 产出与人工产出，便于评审时判断哪些内容需要人逐条核对。

---

## Decision-20261026-04（P0.5 格式化范围与全量格式检查，Accepted）

- **Date**：2026-10-26
- **Status**：Accepted（按 plan.md P0.5 执行，"二选一并记入 decision.md"）
- **Source**：评审报告 §5.12；Decision-20261025-04；Decision-20261026-01 第 4 项

**Context**：P0.5 要一次性格式化手写代码并排除生成区与厂商代码。clang-format 18 的 `.clang-format-ignore` 不支持 `**`，`*` 也不跨目录（2026-10-26 实测），只能按目录深度展开模式；CI 的增量检查此前把排除正则写在 workflow 里。

**Decision**：

1. 格式化范围由 `tool/ci/format.py` 统一定义，CI 与本地共用：git 跟踪的 `.c/.h/.cpp/.hpp`，排除 `legacy/Platform/STM32H7/DM_MC02/{Core,Drivers,Middlewares,USB_DEVICE}/`、`platform/**/cubemx/`、`src/third_party/`。不使用 `.clang-format-ignore`。
2. 一次性格式化 126 个手写文件，单独提交（`c23eb5e`），并去除块注释内残留的 CR（手写文件统一为 LF）；提交登记在 `.git-blame-ignore-revs`。
3. CI 的 lint-c 改为全量检查（`tool/ci/format.py --check`），关闭 Decision-20261026-01 第 4 项的增量检查与 `docs/ai/architecture.md` 例外 X-13。

**验证**：legacy Debug/Release `.bin` 与基线 SHA-256 逐字节一致（Windows，xPack GCC 15.2.1，同一安装路径）；`test/host` 13 项 ctest 全绿。

**Alternatives / Rejected**：

- `.clang-format-ignore` 按深度展开（如 `Drivers/*`、`Drivers/*/*`……）：生成区目录很深、写法冗长，新增层级会漏，否决。
- 在生成区放 `DisableFormat: true` 的 `.clang-format`：要往生成区与 `legacy/` 里加文件，否决。

---

## Decision-20261026-05（P0.5 命名统一：工具、规则微调与改名结果，Accepted）

- **Date**：2026-10-26
- **Status**：Accepted（按 plan.md P0.5 执行；`.clang-tidy` 首版的微调按评审报告 §5.12 在此登记）
- **Source**：评审报告 §5.12；Decision-20261025-04；过程与证据见 `docs/ai/decisions/2026-10-26-P0.5-改名/`

**Context**：P0.5 要用 clang-tidy 把 C++ 标识符改为 ROS 2 命名。首次运行发现：

1. 同一个头文件会被多个翻译单元重复报告，并行 `clang-tidy --fix` 会互相覆盖；pip 版 clang-tidy 18 不带 `clang-apply-replacements`。
2. 迁移源固件工程的 include 目录写成 `DM_MC02/../../..`，clang 看到的头文件路径不规范化，原 `HeaderFilterRegex` 匹配不上，固件独有的头文件被整体漏掉；Windows 路径用 `\` 时同样匹配不上。
3. `Application_Init` 等 C ABI 函数、测试替身重新声明的 CMSIS-RTOS2 函数会被当成普通函数改名，导致链接失败。
4. 被 `#if` 关掉的代码（`IMU_ENABLE_DIAGNOSTICS` 只在 Debug 打开）、没有被任何翻译单元包含的头文件，对 clang-tidy 都不可见。

**Decision**：

1. 新增 `tool/ci/naming.py`，CI 与本地共用：
   - 输入多个编译数据库，向交叉编译器查询目标三元组与系统头目录后交给 clang-tidy 18；
   - 汇总各单元导出的修正，去重、查冲突，全部成功后才写盘；有任一单元无法自动修正的标识符整体跳过；
   - 用 `-H` 找出孤立头文件，借路径最近的翻译单元的编译参数单独检查。
2. `.clang-tidy` 微调：
   - `HeaderFilterRegex` 兼容 `/` 与 `\`，接受以 `../` 开头的写法，并纳入 `Platform/Interface/*.hpp`（C++ 包装层）；
   - 新增 `FunctionIgnoredRegexp`，不改名的函数包括"模块_函数"形式的 C ABI、`os*`（CMSIS-RTOS2）、`HAL_*`（覆盖 HAL 弱符号的回调）。
3. 命名检查覆盖三个编译数据库：固件 Debug、固件 Release、宿主测试。以后新增编译开关时，要把对应配置加入检查。
4. 补充一条命名约定（`.clang-tidy` 检查不出，评审时把关）——标识符里的数字：
   - "数值+单位"和序号单独成段，如 `ACCEL_ODR_100_HZ`、`GYRO_RANGE_2000_DPS`、`POWER_24V_0`、`CHANNEL_1`、`LEVEL_0`；C++ 包装层的枚举名因此与 C 层宏后缀一致（`CAN_PORT_CHANNEL_1`）；
   - 名字里固有的数字不拆：型号（`BMI088`、`GM6020_CURRENT`、`c610`）、引脚名（`INTERRUPT1`）、术语（`OSR2`、`crc8`）、数学常量（`SQRT2`）、标号（`WHEEL_M1`）。
5. 改名结果：
   - 自动改名 2190 处（9383 处修改）；
   - 手工修正模板依赖访问、未实例化模板里的调用、2 处同名遮蔽、注释中的旧名字 205 处；
   - 改名提交（`2b3fbf8`）与改名后的格式化提交（`0c51793`）分开，均登记在 `.git-blame-ignore-revs`。
6. CI 新增 `lint-naming` 作业（`naming.py --check`），存量已清零，此后全量检查。

**验证**：
- legacy Debug/Release `.bin` 与基线 SHA-256 逐字节一致（Windows，xPack GCC 15.2.1）；
- 符号"地址+大小+类型"集合与基线一致；
- 宿主测试 13/13 通过；
- 三种配置的命名检查均为 0 处。

**Consequences**：
- 芯片里旧固件的符号名仍是旧名，调试时按地址对照（改名后的符号表与基线地址一一对应）。
- `alg_math::PI` 与 CMSIS-DSP `arm_math.h` 的 `#define PI` 同名，同一翻译单元不能同时包含两者（P3a 注意）。
- 未实例化模板里依赖类型的旧名字，clang-tidy 与编译器都发现不了，只能靠残留扫描（`residual.py`）。P1 起新写的模板要有实例化测试。

**Alternatives / Rejected**：

- 直接并行 `clang-tidy --fix`：同一头文件被多个进程改写，结果不确定，否决。
- 用 `clang-apply-replacements`：pip 版不带，要另装完整 LLVM，且不处理"部分单元无法修正"的情况，否决。
- 只用固件 Release 数据库：会漏掉诊断代码与测试代码，否决。

---

## Decision-20261026-06（名字去掉 foray 前缀，Accepted）

- **Date**：2026-10-26
- **Status**：Accepted（负责人 2026-10-26 要求："所有原本有 foray 前缀的都不需要这些前缀，其它不变"）
- **Source**：负责人指示；适用于评审报告 §5、`docs/ai/architecture.md` 与此前决策中的命名

**Decision**：计划中凡是带 `foray`/`FORAY` 前缀的名字一律去掉前缀，含义与结构不变：

| 类别 | 原名 | 新名 |
|---|---|---|
| CMake target | `foray_hal`、`foray_hal_os`、`foray_boot`、`foray_utils`、`foray_msg`、`foray_lib_<pkg>`、`foray_driver_<device>`、`foray_middleware_<svc>`、`foray_app_<capability>`、`foray_app_common`、`foray_assembly_<profile>`、`foray_profile_<profile>`、`foray_firmware_<profile>`、`foray_os_freertos_<board>`、`foray_port_<board>`、`foray_board_<board>`、`foray_mx_generated_<board>`、`foray_rtos_<board>`、`foray_usb_<board>`、`foray_platform_mock`、`foray_test_<name>`、`foray_hal_detail`、`foray_legacy_include` | 去掉 `foray_`，如 `hal`、`port_<board>`、`legacy_include` |
| CMake 变量 | `FORAY_PROFILE`（以及示意 schema 中的 `FORAY_PROFILE_ID`、`FORAY_BOARD`、`FORAY_FEATURES` 等） | `PROFILE`、`PROFILE_ID`、`BOARD`、`FEATURES` |
| 启动入口（C 链接） | `foray_boot_init`、`foray_boot_init_rtos`、`foray_boot_run`，头文件 `src/boot/foray_boot.h` | `boot_init`、`boot_init_rtos`、`boot_run`，`src/boot/boot.h`（P3b 落地） |
| 文件与函数 | `cmake/foray_targets.cmake`、`foray_profile.hpp(.in)`、宿主测试的 `foray_host_test()` | `cmake/targets.cmake`、`profile.hpp(.in)`、`host_test()` |

**不改**（不是计划里的前缀）：仓库名 `foray_mcu`、组织规定的 `.foray-layer`、外部仓库 `foray_interfaces`、`foray_docs` 等。

**适用**：此前的 Accepted 决策与评审报告 v1.0 中的旧名一律按上表理解，不逐条改写历史记录；`docs/ai/architecture.md`、`plan.md`、`tree.md` 等现行文档已同步。

**Consequences**：
- target 名不再带项目标识。将来把框架提成独立仓库、被其他 CMake 工程引入时（P5），`hal`、`utils` 这类通用名可能与对方的 target 重名，届时用带命名空间的 ALIAS（如 `framework::hal`）或在提取时统一加前缀。
- 架构检查（`tool/ci/arch_check.py`）按新名字的命名白名单判定 target 所在的层。

---

## Decision-20261026-07（P1 构建骨架的设计与验收，Accepted）

- **Date**：2026-10-26
- **Status**：Accepted（按 plan.md P1 执行；过程与对照数据见 `docs/ai/decisions/2026-10-26-P1-构建骨架/`）
- **Source**：评审报告 §5.2、§5.4、§7.2 P1、§7.3；Decision-20261026-06

**Context**：P1 要在零源码改动的前提下建立根工程与 target 骨架，并以"符号名与大小集合与基线一致"验收。源码在 P2 之前仍在 `legacy/`。

**Decision**：

1. **profile 与板卡的 schema**（评审报告 §5.4 要求 P1 定稿）：
   - 唯一开关是 `-DPROFILE=<名称>`；
   - `config/profiles/<名称>/profile.cmake` 只含数据：`PROFILE_ID`（须与目录名一致）、`BOARD`（`platform/` 下的板卡目录）、`FEATURES`（P3b 起决定链接哪些能力）；
   - 板卡目录下的 `board.cmake` 声明 `BOARD_ID`（target 名后缀）、`BOARD_CPU`（配置时与工具链核对）、`BOARD_OPENOCD_CONFIG`；
   - 第一个 profile 是 `baseline`（a2cfd16 基线固件，DM-MC02，板标识 `dm_mc02`），P1–P2.5 用它做符号对照，P3b 起再按兵种建立正式 profile；
   - `profile_<名称>` 是生成头 `profile.hpp`（`profile::ID`、`profile::BOARD`），叶子 target。
2. **target 归属**：每个 legacy 文件按 P2/P2.5 的目标位置归属一个 target（头文件也列入），源码引用 `legacy/` 路径，P2 搬迁时只改路径。图中新增的节点有三类：
   - `app_tasks`：任务外壳，对应 `app/tasks`；
   - 迁移期过渡件：`legacy_include`、`legacy_build_options`、`portkit`；
   - `msg`、`middleware_<svc>` 暂不建立，P3b/P4 有内容时再建。
3. **OBJECT 库的目标文件传递**：CMake 只把直接链接的 OBJECT 库放进最终链接。`link_object_library()` 用 PRIVATE 链接保留依赖边，同时把 `$<TARGET_OBJECTS:…>` 放进链接接口。这样 port 下的生成区、assembly 下的 board 都能进入固件，依赖图也保持与 §5.2 一致。
4. **编译选项与迁移源一致**：工具链文件照抄迁移源的 CPU、优化、调试选项，Release 不定义 `NDEBUG`。C 用 gnu11，C++ 用 C++17。差异只限于对应文件不使用的宏，逐文件对照表见证据目录。两点经验：
   - **`-Wall` 加给全部代码**：GCC 15 在 -O0 下 `-Wall` 会改变生成代码，空死循环里多一条 `nop`。按 target 加会使 34 个函数大小不同，所以由工具链统一添加；
   - **严格告警**（`-Wextra -Werror`）按 target 添加，只加给手写代码。
5. **include 约定**（P2 起适用）：
   - 新代码以层的根目录为 include 根：框架为 `src/`（`#include "hal/os/clock.hpp"`），产品为 `app/`；两边一级目录不重名；
   - 不把仓库根加入 include 路径；
   - 迁移期的 `legacy_include` 与 `hal` 对 `legacy/Platform/Interface` 的暴露在 P2 删除。
6. **头文件保护宏**：路径大写，单下划线分隔，如 `HAL_OS_CLOCK_HPP`，与迁移源一致。`architecture.md` 原示例的双下划线写法通不过 `.clang-tidy` 的宏命名检查，而且是 C++ 保留标识符，予以更正。
7. **最小 OS 契约草案**：`hal::os::CriticalSection`（RAII，可嵌套，析构恢复进入前状态）、`hal::os::now_ticks()`、`hal::os::tick_frequency_hz()`。platform_mock 实现并有宿主测试；固件的 port 实现与契约 v1 冻结在 P3a。
8. **宿主测试**：
   - 由根工程在宿主构建中引入；框架的单元测试链接 `lib_*`、`driver_*` target；
   - 底盘集成测试要用影子头，仍在测试里直接编译 app 源码，例外 X-10 缩小为这一项，P3b 解决；
   - 测试可执行文件为 `test_<名称>`，ctest 名称不变。
9. **工具与 CI**：
   - `tool/ci/symbols.py`：readelf 归一化的符号对照。存档的符号表在 Windows xPack 与 Linux Arm GNU 两套工具链上逐项一致，解决了基线对照文档 §7 记录的 `nm` 跨平台问题；
   - `tool/ci/arch_check.py`：依赖图检查与文本扫描；
   - CI 的 `firmware` 作业比较根工程与迁移源构建的符号集合；`lint-arch` 做文本扫描；`host-tests` 与 `lint-naming` 改用根工程 preset；
   - 迁移源构建与 Release SHA 校验保留到 P2；
   - xPack 安装抽成 `.github/actions/setup-arm-gcc`。
10. **新增例外**：X-14（`legacy_build_options`，P3b）、X-15（`portkit`，P2.5）、X-16（`os_freertos_<board>` 只能整目录暴露 CubeMX 的 `Core/Inc`）。依赖图中迁移期放行的边：
    - `hal → hal_detail`（X-07）；
    - `mx_generated → hal_detail/legacy_include`（X-01、X-09）；
    - `lib_estimate → driver_bmi088`（已知结构债）；
    - `app_common → app_tasks`（TaskManager 含任务表，P3b）。

11. **CMake 文件按层组织**（负责人 2026-10-26 选定）：
    - 一层的全部 target 定义在该层的一个 `CMakeLists.txt` 里，不在子目录里各放一个；
    - 手写的共 7 个：根、`src/`、`app/`、`config/`、每块板一个（`platform/<vendor>/<mcu>/<board>/`）、`platform/mock`、`test/host`；
    - 某一层以后变大或由不同的人维护时，再拆出子目录的文件，拆分在本文件登记；
    - P1 初版曾按子目录各放一个（共 14 个），合并前后的依赖图（节点、边与可见性）和每个文件的编译命令逐字相同。

**验证**：
- 根工程固件与迁移源构建的符号集合一致：Debug 1426 个、Release 1038 个函数与对象符号，名称、大小、类型、绑定、输出段全部相同；Windows xPack GCC 15.2.1 与 Linux Arm GNU 15.2.Rel1 各验证一次；
- `.bin` 因对齐填充差几个字节，SHA 不同（预期内）；
- ctest 14/14（MSYS2 g++ 16.2、WSL g++ 11.4）；
- 依赖图检查与文本扫描通过，并做了反向验证；
- 命名检查 0 处，格式检查 131 个文件通过；
- `legacy/` 无任何改动。

**Alternatives / Rejected**：
- **固件直接链接全部 OBJECT 库**：依赖图会变成"固件 → 生成区"，与 §5.2 不符，检查也无法区分合法与违规的直连，否决。
- **把 port、board 改为 STATIC 加 WHOLE_ARCHIVE**：需要 CMake 3.24，且与规则"port 与 board 用 OBJECT"不符，否决。
- **只给手写代码加 `-Wall`**：会改变生成代码（第 4 点），否决。
- **每个子目录一个 CMakeLists**（P1 初版的做法）：搬迁前大部分目录只有一个指向 `legacy/` 的构建文件，本仓规模（手写源码约 100 个文件）用不着这么细，负责人选定按层组织（第 11 点），否决。

---

## Decision-20261026-08（P2 机械搬迁的规则与结果，Accepted）

- **Date**：2026-10-26
- **Status**：Accepted（按 plan.md P2 执行；移动表、脚本与验证见 `docs/ai/decisions/2026-10-26-P2-机械搬迁/`）
- **Source**：评审报告 §7.2、§7.2.1、附录 A；Decision-20261026-07

**Decision**：

1. **范围**：按附录 A 把 1:1 项搬出 `legacy/`：
   - Platform/Interface → `src/hal`（Detail → `src/hal/detail`）；
   - Libraries/Algorithm、Component → `src/lib`；
   - Libraries/Device → `src/driver`；Libraries/Protocol → `src/driver/protocol`；
   - 任务外壳 → `app/tasks`；ChassisController → `app/chassis`；RemoteReceiver → `app/remote`；
   - DM_MC02 的 CubeMX 生成物 → `platform/st/stm32h7/board_dm/cubemx`；Port → `port`。

   计划外的两处：
   - `Application/Module/GimbalModule` 计划未写阶段，它是 1:1 项，随 P2 搬到 `app/gimbal`；
   - 测试与支持文件名一并改为 snake_case。

   留到 P2.5：`Application/{Entry,Imu,Task/TaskManager}`、PortKit、`Port/UsbCdc`。
2. **文件名**：
   - 一律 snake_case；
   - 迁移源 Algorithm 下的 `.h` 是 C++ 头，统一改为 `.hpp`；C 头（`hal/detail/*.h`、port 头）保持 `.h`；
   - 被 CubeMX 生成代码按名字 include 的头保持原名：`port/Init.h`；P2.5 的 `UsbCdc.h`、`Entry.h` 同理。生成区不手改，改名会让大小写敏感的系统找不到文件。
3. **include 与保护宏**：
   - include 一律相对层根目录书写：`src/` 下如 `"hal/can.hpp"`，`app/` 下如 `"tasks/imu_task.hpp"`，测试如 `"support/mock_motor.hpp"`；
   - 去掉裸头名 include；
   - `hal/detail` 只能经 `src/` 根引用，其中的 `time.h`、`spi.h` 等与标准库、CubeMX 头同名，不得把该目录本身加入 include 路径；
   - 保护宏由新的 include 路径大写得到，如 `HAL_DETAIL_CAN_H`。
   - 改写由 `move.py --rewrite` 完成：按搬迁前的查找顺序解析到具体文件再换写法，只改 include 与保护宏所在的行。
4. **提交拆分**：
   - 纯 `git mv` 单独一个提交，323 个文件全部 R100；
   - include 改写、构建与工具适配各一个提交。

   这样即使是只有一行 include 的文件（`basic_math.cpp`、`fsm.cpp`），改写后合计相似度低于 50%，`git log --follow` 仍能逐提交追溯到迁移源最早的提交。
5. **迁移源旧构建**：
   - DM_MC02 的 `CMakeLists.txt`、`CMakePresets.json`（含本机路径）、`cmake/gcc-arm-none-eabi.cmake`，以及 Interface、PortKit 的 `CMakeLists.txt` 删除，由根工程取代；
   - CubeMX 每次都会重写的 `cmake/stm32cubemx/CMakeLists.txt` 与 `starm-clang.cmake` 随生成区保留（根工程不用，可对照源文件清单）；
   - CubeMX 再生成时附带产生的旧式构建文件写入 `.gitignore`；
   - `.clangd` 移到仓库根；`Libraries/README.md` → `src/README.md`，`motor/README.md` 随驱动搬迁，正文保留旧路径并加迁移说明，P3b 更新。
6. **legacy_include 推迟到 P2.5 删除**（修订 plan.md 与例外 X-11）：P2.5 的文件仍以 `"Application/..."` 被引用，P2 结束时只有它们还需要 `legacy/` include 根。
   - 框架 target 与已搬迁的 app target 已不再链接 `legacy_include`，依赖图少了 21 条指向它的边；
   - `.clang-tidy` 头文件过滤只保留 `legacy/Application` 一段，同样到 P2.5。
7. **工具与 CI**：
   - `naming.py` 排除 C ABI 头 `src/hal/detail`（X-07）；
   - `arch_check.py` 规则 9 放行 X-14 的两个构建开关（`IMU_ENABLE_DIAGNOSTICS`、`BMI088_GYRO_BANDWIDTH_REGISTER`），规则 15 跳过生成区；
   - CI `firmware` 作业删除迁移源构建与基线 SHA 校验，符号对照改用 P1 存档的符号表。

**验证**：
- 符号与 P1 存档一致：Debug 1426、Release 1038；Windows xPack 与 Linux Arm GNU 各验证一次；
- 112 个固件源文件除 include 目录外的编译选项与 P1 逐项相同；
- ctest 14/14（MSYS2 g++ 16.2、WSL g++ 11.4）；
- 依赖图与 P1 相比只少了指向 `legacy_include` 的边；
- 命名检查 0 处，格式检查 131 个文件通过，文本扫描通过。

**Alternatives / Rejected**：
- **搬迁与改写合成一个提交**：小文件的改名识别会失效，`git log --follow` 断链，否决。
- **把 `Init.h` 也改为 `init.h`，同时手改 `main.c` 的 USER CODE**：违反生成区不手改的规则，否决。等 P3b 的 boot ABI 落地后，生成区不再直接调用 `Port_Init`，届时再处理。
- **P2 一并搬走 P2.5 的项以便按原计划删除 `legacy_include`**：会把 P2.5 的拆分判断混进机械搬迁，否决。

---

## Decision-20261026-09（src/lib 去掉 alg_ 前缀，Accepted）

- **Date**：2026-10-26
- **Status**：Accepted（负责人 2026-10-26 要求："取消掉 alg_ 这样的前缀"）
- **Source**：负责人指示；修订 Decision-20261026-08 第 1 点"保留 `alg_*` 子目录名"与 tree.md 中"P3b 统一"的安排。过程与验证见 `docs/ai/decisions/2026-10-26-P2.5-去掉alg前缀/`

**Decision**：

1. **范围**：`src/lib` 的子目录与命名空间一起去掉 `alg_` 前缀。目录名、命名空间名、target 名后缀三者一致：

   | 原目录 / 命名空间 | 新目录 / 命名空间 | target（不变） |
   |---|---|---|
   | `alg_math` | `math` | `lib_math` |
   | `alg_controller` | `control` | `lib_control` |
   | `alg_crc`、`alg_estimate`、`alg_filter`、`alg_fsm`、`alg_slope` | `crc`、`estimate`、`filter`、`fsm`、`slope` | `lib_<同名>` |
   | `alg_kinematics`（另有命名空间 `alg_odometry`） | `kinematics`（`odometry`） | `lib_kinematics` |

   - `alg_controller` 改为 `control`，不是字面去前缀的 `controller`：与已有 target 名 `lib_control`、tree.md 目标结构的分类名一致；
   - `component` 本来没有前缀，不变。
2. **时机**：放在 P2.5 开头，单独一个 PR。它是改名类变更，按"一个 PR 只做一类变更"与 P2.5 的拆分搬迁分开。原计划 P3b 统一，提前的理由：P3b 同时改行为，改名混在其中难以审查；现在改，除 `legacy/Application/Imu` 外的引用都已在新位置，一次改完。
3. **提交拆分**（同 P2）：
   - 纯 `git mv`：8 个目录、31 个文件，全部 R100；
   - 整词替换：45 个文件 416 行，只改命名空间、限定名、include、保护宏与 `src/CMakeLists.txt` 的源文件路径；
   - 重新格式化：10 个文件。
   - 后两个提交登记到 `.git-blame-ignore-revs`。
4. **对照判据不变，基准不变**：命名空间改名会改变修饰名。
   - `tool/ci/symbols.py compare` 新增 `--renames`：按改名表把 P1 存档修饰名里的"长度+旧名"换成"长度+新名"，大小等其余字段仍须逐项一致；
   - CI `firmware` 作业带改名表对照，基准仍是 P1 存档；
   - 固件中没有大小随名字变化的符号（如 `__PRETTY_FUNCTION__`），只换算名字、不放宽大小。

**验证**：
- 符号集合与 P1 存档一致（带改名表）：Debug 1426、Release 1038；Windows xPack 与 Linux Arm GNU 各验证一次。不带改名表时报不一致，说明对照覆盖了改名的符号；
- 编译选项与 P2 相比只有源文件路径不同；依赖图不变；
- ctest 14/14；命名检查 0 处，格式检查 131 个文件通过，文本扫描通过。

**Consequences**：
- 新名字较通用（`math`、`filter`、`control`）。与局部变量、参数同名（如 `filter`、`crc`）不冲突，因为 `X::` 前的名字只按命名空间与类型查找。框架将来被其他工程引入时（P5）若与对方命名空间冲突，再整体加外层命名空间，与 Decision-20261026-06 的 target 名同样处理。
- 历史记录中的 `alg_*` 按上表理解，不逐条改写：评审报告、Decision-20261026-05（如 `alg_math::PI` 与 CMSIS-DSP 的 `PI` 同名，现为 `math::PI`）、-08、P2 移动表、`src/README.md` 正文。

**Alternatives / Rejected**：
- **只改目录、不改命名空间**：`lib/math/` 下是 `alg_math::`，同一模块两套名字，否决。
- **按原计划在 P3b 统一**：P3b 允许改行为，改名混在其中难以审查，否决。

---

## Decision-20261026-10（P2.5 搬迁的去向与结果，Accepted）

- **Date**：2026-10-26
- **Status**：Accepted（按 plan.md P2.5 执行；脚本与验证见 `docs/ai/decisions/2026-10-26-P2.5-拆分搬迁/`）
- **Source**：评审报告 §7.2.1、附录 A；Decision-20261026-08 第 6 点

**Decision**：

1. **去向**（16 个文件，全部 R100）：
   - `Application/Entry.cpp` → `config/assembly/entry_legacy.cpp`；
   - `Application/Task/TaskManager.*` → `app/common/task_manager.*`；
   - `Application/Imu/*` → `app/imu/`；
   - `Port/UsbCdc.c` → `platform/.../port/usb_cdc.c`。

   被生成代码按名字 include 的头保持路径与文件名：`Entry.h` 放在 `config/assembly/Application/Entry.h`，`UsbCdc.h` 放在 port。
2. **PortKit 整体并入 port，拆分推迟到 P3a**（修订 plan.md P2.5 与评审报告 §7.2.1 的这一项）：`PortKit.{c,h}` → `platform/st/stm32h7/board_dm/port/port_kit.{c,h}`，删除过渡 target `portkit`。原样拆分做不到：
   - 队列与通知也用 CMSIS 的 `__DMB()`，放进零平台的 `src/utils` 违反规则 3；
   - 临界区拆到另一个文件会改变 Release 生成代码：`PortKit_Critical_Enter/Exit` 原本内联进 `PortKit_Notifier_Set`，拆开后变成调用。

   P3a 把 PortKit 改写为 C++ 时一并拆分：队列与通知 → `src/utils` 并发原语，临界区 → `src/hal/os` 契约 + 各板 port 实现。
3. **生成区接缝的 include 路径由组合根注入**（例外 X-09）：`config/CMakeLists.txt` 对 `mx_generated_<board>` 追加 `config/assembly` 的 include 路径。
   - 平台层 CMake 不出现产品路径，规则 15 不变；
   - 这条耦合只是 include 路径，不在依赖图上；
   - P3b 生成区改为只调 boot ABI 后删除。
4. **删除迁移期过渡件**：
   - `legacy_include`、`legacy_library()`、`LEGACY_DIR`；
   - `.clang-tidy` 头文件过滤中的 `legacy/Application` 一段；
   - 例外 X-11、X-15 关闭。X-12 也关闭：`legacy/` 只剩迁移源 README 与架构图，没有代码与构建文件。

   `cmake/legacy.cmake` 只剩 `legacy_build_options`（X-14，P3b）。
5. **编译选项的一处变化**：`port_kit.c` 随 port 带上严格告警（`-Wextra -Werror`）。它没有告警，目标文件的反汇编与重定位与搬迁前逐项相同。其余 111 个固件源文件除 include 目录外不变。
6. **工具**：`arch_check.py` 删去 `portkit` 层、`legacy_include`、生成区 → legacy 的放行；测试不再允许依赖 legacy 层。

**验证**：
- 符号与 P1 存档一致（带 `alg_` 改名表）：Debug 1426、Release 1038；Windows xPack 与 Linux Arm GNU 各验证一次；
- 搬迁的 4 个目标文件反汇编（含重定位）与数据段逐项相同；
- ctest 14/14（WSL g++ 11.4、MSYS2 g++ 16.2）；
- 命名检查 0 处，格式检查 131 个文件通过，文本扫描与两张依赖图检查通过；
- `git log --follow` 可追溯到迁移源最早的提交。

**Alternatives / Rejected**：
- **按计划把队列/通知原样放进 `src/utils`**：`src/` 会 include CMSIS 头，违反规则 3；把 `__DMB()` 换成 C11 `atomic_thread_fence` 会改变生成的指令（`dmb ish` 与 `dmb sy`），不属于不改行为的阶段。否决。
- **保留 `portkit` 过渡 target，原地不动**：`legacy/` 搬不空，`legacy_include` 与例外 X-15 都要继续保留。否决。
- **把 `Entry.h` 放进 `src/boot`，让生成区链接 `boot`**：这是 P3b boot ABI 的形态，但现有 `Application_*` 入口是产品的组合根，提前放进框架会让框架含产品接口。否决。
- **在平台 CMake 里直接写 `config/assembly` 路径**：违反规则 15（框架不得引用产品路径）。否决。

---

## Decision-20261027-01（按负责人骨架重建：架构 v2，Accepted）

- **Date**：2026-10-27
- **Status**：Accepted（负责人 2026-10-27 逐条答复 15 个骨架问题）。其中第 4 点"topic 清单放在 `app/topics`"与"不支持多发布者"是负责人提出疑问后 AI 给出的建议，随架构 v2 一并待负责人审阅确认。
- **Source**：负责人对 P2.5 结果的意见："我以为你的迁移是以原工程为参考，以我的想法为骨架然后重新构建，但是你现在的现状是充满了对原项目的妥协和新架构的适配"。负责人最初的提案见评审报告 §3.1。
- **修订**：Decision-20261025-01 第 6 点的迁移方式（P3a–P5）及其 Reason 中的"重新切片而不是重写"；评审报告 §5.7 的"topic 第一版不引入"；评审报告中 platform 的厂商/芯片/板三级布局；Decision-20261026-10 第 2 点的 PortKit 拆分时机。其余 Accepted 条目不变。

**Context**：P0–P2.5 采用渐进迁移，每一步与旧固件逐符号对照，可验证、可回滚。代价有两个：
- 旧结构的形状被一路带过来：C 接口层、`Application_*` 入口、PortKit 放在 port、`config/assembly/Application/Entry.h`。
- 目标架构本身是从分析旧工程推出来的，几处偏离了负责人的提案：topic 被推迟，lib/utils 的划分，platform 分三级。

另外，被保护的基线 `a2cfd16` 本身不完整：遥控 → 底盘/云台的链路缺失，底盘从不使能输出。对它做到逐符号一致，价值有限。

**Decision**（编号对应 15 个问题）：

1. **app 按能力划分**；兵种 = 能力组合 + 参数（profile）。
2. **依赖方向**：config（组合根）依赖 app，参数经 `Config` 注入；app 不依赖 config。
3. **profile = 兵种 × 板**；一个兵种只有一台车，不需要单车覆盖。
4. **topic 进入框架**，机制在 `src/middleware/topic`（负责人审阅时提出：topic 是给能力用的系统服务，放在 middleware；middleware 只收与产品无关的运行期服务，新增先登记）：
   - 语义：最新值，不做队列；
   - 中断里可以发布；订阅者可以阻塞等待新数据；
   - 清单集中一处；框架验证好用后改为从描述文件生成。
   - AI 建议（待确认）：
     - 清单放在产品侧 `app/topics/`，不放 `src/`：具体 topic（云台指令等）是 RM 产品的数据，放进框架会让框架含产品内容；
     - 每个 topic 只有一个发布者，多个来源由仲裁能力合并；
     - topic 的单位是"一种数据"，归发布它的能力所有，清单按能力分组；一个能力可以发布多个 topic。
5. **硬件契约名为 `hal`**，风格为自由函数 + 不透明句柄，链接期绑定。
6. **platform 按板一级**：`platform/<板标识>/{cubemx,port,board}`。
7. **utils 与 lib 的边界**：
   - 没有领域含义、哪一层都可能用的放 `src/utils`：SPSC 队列、最新值邮箱、fsm、crc；
   - 领域算法放 `src/lib`：slope 归入 `lib/control`；轴级执行器骨架（component）留在 lib。
8. **线程归组合根**：能力只提供 `init()`/`step()`，零 RTOS；任务表在 assembly，由 `app/common` 的任务运行器执行。
9. **RTOS 接口**：任务层直接用 FreeRTOS 原生 API；框架只经 `hal/os`（临界区、时基、信号）。
10. **允许修改 CubeMX 的 USER CODE 区**，但只能写 `boot_*` 调用；改动行登记在 decision.md，由 CI 检查。
11. **profile 参数**用 C++ `constexpr` 头文件（`params.hpp`）。
12. **重建期间不需要能上车的固件**：当前目标是搭建可行的框架。
13. **先框架、后能力**：框架建成并经负责人确认好用后，才重建能力。
14. **F407 C 板**放在 DM-MC02 重建完成之后。
15. **验收方式**：宿主单元测试 + `test/data` 录制数据回放 + 实机检查清单（实机部分由负责人执行）。

**由此而来的安排**：
- 迁移方式改为"以负责人骨架为准重建，迁移源只作参考与零件来源"。写得好的算法与驱动直接搬来，按新契约适配。
- 架构文档重写为 `docs/ai/architecture.md` v2。
- plan.md 的 P3a–P5 改为 F1–F4（框架）、A（能力重建）、B（F407）、E（框架提取与 MSPM0），原任务的去向见 plan.md 的对照表。
- F2 开始时，旧能力代码打 tag `archive/pre-rebuild` 后移出构建；能力重建时从 tag 取参考。
- "与旧固件逐符号对照"的判据止于 P2.5，CI 的符号对照在 F2 退役。

**Alternatives / Rejected**：
- **继续按原 P3a/P3b 改造旧代码**：每一步都要兼容旧结构，过渡件会延续，负责人的骨架要到很晚才成形。否决。
- **推倒重来，不复用旧代码**：PID、VQF、EKF、BMI088、DJI 协议等经过实车使用，重写没有收益。否决。
- **topic 回调式订阅**：回调在发布者的上下文执行，中断里发布时不可控。否决。
- **topic 支持多发布者**：最新值语义下多个来源交替写入，"最新"失去意义；单写者 seqlock 也无法保持无锁。否决（待确认）。

---

## 待登记

- 分支保护的具体设置（组织规范 §6.1、§10，负责人在 GitHub 上操作后补记）。
- `-Wdouble-promotion` 豁免清单（P3a）。
- 契约 v1 冻结记录（P3a 结束时）；TM4C、AM26x port 的启动条件与编译器选择（Q13）。
