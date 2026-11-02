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

## Decision-20261027-02（F1 第 1 步：utils、middleware、lib 的归类，Accepted）

- **Date**：2026-10-27
- **Status**：Accepted（负责人动手完成并确认；fsm 放 middleware 为负责人决定）
- **Source**：Decision-20261027-01 第 7 点；修订其中"fsm 归 utils、math 留 lib"的划分

**Decision**：

1. **math → `src/utils/math`**：向量、四元数、矩阵没有领域含义，按"无领域含义、哪层都可能用"的标准归 utils。命名空间仍为 `math`。lib 的算法改为依赖 `utils_math`。
2. **crc → `src/utils/crc`**（同 -01）。
3. **fsm → `src/middleware/fsm`**（负责人决定）：
   - 状态机作为给能力组织模式与流程的运行期机制，与 topic 同属 middleware；
   - middleware 的定义相应改为"给能力用的运行期机制与服务"；
   - driver 与 lib 仍不得依赖 middleware，驱动内部的少量状态自己处理。
4. **slope 并入 `src/lib/control`**：目录名即命名空间，`slope::` 改为 `control::`；`lib_slope` 并入 `lib_control`。
5. **target 与构建**：
   - utils 每个子目录一个 target：`utils_<名>`；
   - utils 用新的 `utils_library()`，不链接迁移期的构建开关 `legacy_build_options`，因为 utils 只依赖标准库；
   - 删除 P1 留下的空 target `utils`。
6. **依赖图检查**：
   - target 名规则 `utils_\w+`；
   - lib 可以依赖 utils；
   - middleware 收紧为只依赖 utils、契约与 middleware 内部（迁移期另允许 `legacy_build_options`）。

**验证**：
- 宿主 ctest 14/14；
- arm-debug、arm-release 构建通过，固件符号与 P1 存档一致：被移动或改名的代码（math 之外）固件都没有链接，math 的命名空间未变；
- 两张依赖图检查、文本扫描、格式、命名检查通过；
- 纯移动单独一个提交，只有一行的 `basic_math.cpp`、`fsm.cpp` 也能被识别为改名。

**Alternatives / Rejected**：
- **fsm 放 utils**（-01 原方案）：驱动也能用状态机。负责人选择把它作为能力层的运行期机制，放 middleware。
- **utils 也链接 `legacy_build_options` 并在检查中放行**：utils 会背上产品的迁移期宏，与"只依赖标准库"矛盾。否决。

---

## Decision-20261028-01（template 声明单独一行，Accepted）

- **Date**：2026-10-28
- **Status**：Accepted（负责人决定）
- **Source**：负责人在 F1 编写 `spsc_queue` 时提出；修订 Decision-20261025-04 第 2 点（根 `.clang-format` 原样沿用历史配置）

**Context**：根 `.clang-format` 基于 LLVM 风格，`AlwaysBreakTemplateDeclarations` 取默认值 `MultiLine`：模板声明与后面的类或函数能放进一行（行宽 120）时合并成一行，如 `template <typename T, std::size_t Capacity> class SpscQueue`，放不下时才换行。同一个文件里两种写法混用，模板参数与类名、函数名挤在一行，不好读。

**Decision**：

1. `.clang-format` 增加 `AlwaysBreakTemplateDeclarations: Yes`：`template <...>` 总是单独一行，类、函数、成员模板、前置声明与 `template <>` 特化都适用。clang-format 19 起该选项改名为 `BreakTemplateDeclarations`，旧名仍可用；本仓固定 18.1.8，用旧名。
2. 已有代码按新配置重新格式化，单独一个 `style` 提交，登记在 `.git-blame-ignore-revs`。
3. 组织规则（4 空格、LF、UTF-8、文件末尾换行）不涉及模板换行，不冲突。

**验证**：
- 重新格式化 12 个已跟踪文件，拆开 25 处模板声明；去掉全部空白后与改动前相同，只改了换行；
- 格式检查 131 个文件通过；宿主 ctest 14/14。

**Alternatives / Rejected**：
- **保持默认 `MultiLine`**：短声明并成一行、长声明换行，同一文件里写法不统一。否决。
- **只在新代码里手动换行**：clang-format 会把它合并回去，CI 的格式检查不通过。否决。

---

## Decision-20261029-01（utils 并发原语放 src/utils/concurrency，Accepted）

- **Date**：2026-10-29
- **Status**：Accepted（目录 `concurrency` 由负责人建立，AI 建议保留；`latest_value` 的实现由 AI 按负责人的骨架补齐）
- **Source**：plan.md F1"`src/utils`：spsc_queue、latest_value"；修订架构 v2 §2.1、tree.md 中 `src/utils/spsc_queue` 的写法

**Context**：架构 v2 与 tree.md 把 F1 的并发原语写成 `spsc_queue`、`latest_value` 两项，没有定目录。负责人编写队列时建了 `src/utils/concurrency/`。按 Decision-20261027-02，utils 每个子目录一个 target，命名空间与子目录同名。

**Decision**：

1. 并发原语放 `src/utils/concurrency/`，命名空间 `concurrency`，target `utils_concurrency`：
   - `SpscQueue<T, Capacity>`（`spsc_queue.hpp`）：单生产者、单消费者环形队列，由 PortKit 的队列改写，acquire/release，不依赖 CMSIS；容量是 2 的幂，不超过 2^31；
   - `LatestValue<T>`（`latest_value.hpp`）：单写者、多读者的最新值邮箱（seqlock）。数据按 32 位字做 relaxed 原子读写，用内存屏障排序（Boehm 2012 的写法），读写同时发生也不是数据竞争，ThreadSanitizer 不会误报。版本号 31 位，按 2^31 回绕，求差用 `VERSION_MASK`；
   - 读者不能在写入中途抢占写者，见 Decision-20261029-02。
2. **不用原子读改写**（规则 13），只用原子读、原子写与内存屏障。也不用 `static_assert(is_always_lock_free)`：它要求全部原子操作都无锁，Cortex-M0+ 没有 LDREX/STREX，GCC 在那里报 false。实测 M0+、M4、M7 上的原子读写都是单条 LDR/STR 加 DMB，没有库调用。这条约束改由 CI 检查"Cortex-M0+ 编译后不出现 `__atomic_*`、`__sync_*` 库调用"，随 F1 的 CI 项加入。
3. **只有头文件**：用现有的 `framework_interface()` 建 INTERFACE 库。它不链接迁移期构建开关，符合 utils"只依赖标准库"，所以不新增辅助函数。头文件的告警由使用方的编译检查（宿主测试带 `-Werror`）。依赖图检查已按 `utils_\w+` 归类，脚本不用改。
4. **宿主测试**：`spsc_queue`、`latest_value`，含多线程压力与计数器回绕，用 `-O2`。ThreadSanitizer 只在本地跑，CI 不跑。

**验证**：
- 宿主 ctest 17/17（WSL g++ 11.4）；两个并发测试另用 MSYS2 g++ 16.2 加 `-Werror` 编译运行通过；
- ThreadSanitizer 无报告；
- 反向验证，每一项都被测试抓到：
  - 读者去掉第二次比对、写者不先写奇数：压力测试报出数十万次不一致；
  - 数据改成普通 `memcpy`：ThreadSanitizer 报数据竞争；
  - 队列的 `size()` 改成不处理回绕的写法：回绕测试失败；
- 交叉编译：Cortex-M0+/M4/M7 × `-O0`/`-Os`/`-O2` 加 `-Werror` 通过，没有 `__atomic_*`、`__sync_*` 库调用；全局对象放在 `.bss`，不需要运行期初始化（topic 要求常量初始化）；
- 格式、命名（0 处）、文本扫描、两张依赖图检查通过；固件 Debug/Release 构建通过，符号与 P1 存档一致（1426、1038）。

**Alternatives / Rejected**：
- **按 tree.md 分成 `utils/spsc_queue/`、`utils/latest_value/`**：变成两个各只有一个头文件的 target，命名空间也要各取一个名字。否决。
- **新增 `utils_interface()`**：与 `framework_interface()` 完全相同。否决。

---

## Decision-20261029-02（seqlock 的读写竞态与 topic 发布，Accepted）

- **Date**：2026-10-29（2026-10-31 决定）
- **Status**：Accepted（2026-10-31，负责人选定 AI 建议的方案 1）
- **Source**：编写 `LatestValue` 时发现；补充架构 v2 §3.4"中断里不订阅"的理由

**Context**：

- `LatestValue` 的读者遇到写入中途（序号是奇数），或拷贝期间有新的写入（前后两次序号不同），会重试，直到读到一份完整的数据。
- 写者打断读者没有问题：读者恢复运行后发现序号变了，重读一次即可。
- 反过来不行。单核上，读者在写入中途抢占写者后，写者要等读者让出 CPU 才能写完，而读者在等写者写完：读者无限重试，系统卡死。
- 架构 v2 §3.4 只考虑了中断："中断不能等被它打断的写端完成，所以中断里不订阅"。同样的问题也出在任务之间：
  - **高优先级任务读低优先级任务发布的 topic 会卡死**，如控制任务读低频的裁判系统数据；
  - 优先级相同时，时间片轮转会让读者空转一个时间片后写者继续，不会卡死，但浪费时间。
- 只有读者恰好在写入的几十个时钟周期里抢占写者才会触发。测试很难覆盖，现场表现为偶发卡死。
- 跨核共享（如 AM26x 的两个核）不在本决策范围。

**可选方案**：

1. **topic 发布时用 `hal::os::CriticalSection` 包住 `write`（AI 建议）**：
   - 写入期间不会被任何读者打断，上面的限制全部消失，在中断里读也安全（中断里仍不能阻塞等待）；
   - 代价：每次发布关一次中断，时长等于一次消息拷贝。可以给消息大小设上限（如 `static_assert(sizeof(T) <= 64)`）来限制关中断的时长，数值 F2 在板上实测后定；
   - 依赖：`CriticalSection` 可以在中断里用、可以嵌套（P1 草案已经这样约定）。如果它只屏蔽 RTOS 管理的中断（BASEPRI），优先级高于阈值的中断既不能发布也不能读 topic；这一点在 hal/os 定稿时一并确定。
   - hal/os 定稿结果（2026-10-31，Decision-20261031-03）：临界区只屏蔽框架中断。选本方案时，优先级高于框架线的中断既不能发布也不能读 topic；这与"线以上的中断不得使用框架"的规则一致，不是新增的限制。
2. **规定优先级**：每个 topic 的发布者优先级必须高于全部订阅者，中断里不订阅。
   - 不需要关中断；
   - 编译期查不出来。任务表或优先级一改就可能静默违反，后果是偶发卡死。
3. **读者重试若干次后让出 CPU**：`hal/os` 要增加"让出"操作；仍会空转，最坏延迟不确定。
4. **多缓冲**：读者不用重试。但三缓冲的交换要用原子读改写，规则 13 不允许；改成每个读者各有一个槽位，内存与复杂度都更高。

**Decision**（2026-10-31）：

1. 采用方案 1：topic 发布时用 `hal::os::CriticalSection` 包住 `LatestValue::write` 与发布计数；叫醒等待者（`Signal::notify()`）放在临界区外。实现见 Decision-20261031-05。
2. 结果：
   - 写入期间框架中断与任务切换都被屏蔽，读者不会在写入中途抢占写者，也就不会无限重试。任务之间不再有优先级限制；框架中断里可以用 `update()` 读 topic，`wait()` 仍只在任务里用；
   - 框架线以上的中断既不能发布，也不能读 topic，与 Decision-20261031-03 的规则一致；
   - 消息大小上限 `topic::MAX_MESSAGE_SIZE` 暂定 64 字节，由 `static_assert` 检查，F2 在板上实测关中断的时长后再定。
3. `LatestValue` 不改。它是通用原语，在 topic 之外直接使用时，仍按类注释里的限制使用，或者同样在写入时关中断。
4. 跨核共享仍不在本决策范围。

**Alternatives / Rejected**：

- 方案 2（规定优先级）：编译期查不出来，任务表一改就可能静默违反。否决。
- 方案 3（重试后让出 CPU）：要扩充 `hal/os`，最坏延迟不确定。否决。
- 方案 4（多缓冲）：三缓冲违反规则 13；每个读者一个槽位，内存与复杂度更高。否决。

---

## Decision-20261031-01（hal/os 时基固定为 1 µs，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（负责人选定 AI 建议的方案 B）
- **Source**：plan.md F1 第 3 步；修订 P1 草案 `src/hal/os/clock.hpp`（`now_ticks()` 加 `tick_frequency_hz()`）与架构 v2 §3.2

**Context**：

- P1 草案按平台频率计数，频率在运行时由 `tick_frequency_hz()` 给出。
- DM-MC02 上旧的 `platform::Time` 数的是 DWT 周期（480 MHz），32 位计数约 8.9 s 回绕一次；F407（168 MHz）约 25.6 s；AM26x、MSPM0 没有 DWT（架构 §8.2）。
- 回绕周期太短，已经造成两类问题：
  - `src/hal/time.hpp` 的 `has_elapsed_ms` 遇到超过约 8.9 s 的超时，永远返回"未到期"；
  - 9 s 没更新的数据，用 32 位周期差算出来只"过了" 0.05 s，超时检测失灵。
- 频率因板而异，使用方都要在运行时换算。`app/imu/gyro_bias_calibration.hpp`、`zero_rate_bias_observer.hpp` 把 480000000 写成了默认值。

**Decision**：

1. 契约只有一个时基 `hal::os::now_us()`：
   - 单位固定为 1 µs，类型 `hal::os::Microseconds`（`std::uint32_t`），按 2^32 回绕，约 71.6 分钟一次；
   - 任务、中断里都能调用，不阻塞，不进临界区；
   - `now_ticks()`、`tick_frequency_hz()` 删除。
2. 换算辅助写在契约头里（`constexpr`，平台无关）：
   - `elapsed_us`、`has_elapsed`、`to_seconds`，常量 `MICROSECONDS_PER_MILLISECOND`、`MICROSECONDS_PER_SECOND`；
   - 两个时刻一律用无符号差比较，间隔必须小于一个回绕周期。
3. topic 的时间戳、`Signal` 的超时都用这个单位（Decision-20261031-02）。
4. 不进 v1 的两项：
   - 周期级计数器（性能分析用）：需要时作为板级可选能力再加（选项 D）；
   - 忙等延时：现有代码没有调用 `Time::delay_us` 的地方。
5. 实现：
   - **DM-MC02（F2）**：
     - TIM2 是 32 位定时器，现在没有使用。APB1 定时器时钟 240 MHz，分频值 239，得到 1 MHz 自由计数，自动重装值 0xFFFFFFFF，不开中断；`now_us()` 读一次 `TIM2->CNT`。
     - TIM2 在 `.ioc` 里配置（生成区变更，负责人用 CubeMX 6.17.0 操作）。计数器由 port 在 `hal::init()` 里启动，USER CODE 区不加代码。
     - 加入 TIM2 会改变固件符号，所以随 F2 第一批提交进行，那时符号对照正好退役；不放进 F1。
     - 调试暂停时是否冻结 TIM2（`DBGMCU`）在 F2 定。建议冻结：SysTick 在调试暂停时也停，冻结后两者一致，断点恢复后不会集体超时。
   - **F407（阶段 B）**：用 TIM2 或 TIM5（都是 32 位），做法相同。
   - **其他平台**：届时按板选计数源，契约不变。
   - **宿主**：`platform/mock` 的时间由测试推进。
6. 旧的 `src/hal/time.hpp`（`platform::Time`）与 `hal/detail/time.h` 只剩 bmi088 驱动在用，驱动改用 `hal::os::now_us()` 后删除（F3）。hal 契约 v1 不再单列 time。

**Alternatives / Rejected**：

- **A 保持按平台频率计数**：精度最高（约 2 ns），但回绕短且各板不同，每处超时与时间戳都要按运行时频率换算。否决。
- **C 1 µs、64 位**：实际上永不回绕，但时间戳占 8 字节；32 位 CPU 读不了原子的 64 位数，每个 port 都要在中断里数溢出，再"高位、低位、高位"拼起来。否决。
- **D 另加周期计数器**：以后需要时再加，见第 4 点。

---

## Decision-20261031-02（hal/os 的 Signal：语义与存储，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（负责人选定 AI 的建议；"等待者怎么确定"在实现时由 AI 细化，见第 2 点与 Alternatives 第 1 条）
- **Source**：plan.md F1 第 3 步；架构 v2 §3.2"`Signal`：二值信号"

**Context**：

- 架构 v2 只定了：`Signal` 是二值信号，`wait(timeout)` 在任务里阻塞，`notify()` 在任务或中断里都能调用。没有定：等待者怎么确定、早到的通知怎么处理、超时的单位、对象里存什么。
- 用到 `Signal` 的地方：
  - topic 的阻塞等待；
  - 任务表里"等某个 topic 再运行"的任务；
  - 驱动等 DMA 完成。等待的是调用驱动的那个任务，它可能同时也在等 topic，所以一个任务要能先后等多个 `Signal`。
- FreeRTOS V10.6.2 支持带下标的任务通知：
  - CMSIS-RTOS2 的线程标志（`osThreadFlags*`，现有 IMU 任务在用）和 stream buffer 用 0 号；
  - `FreeRTOSConfig.h` 没有设置 `configTASK_NOTIFICATION_ARRAY_ENTRIES`，默认只有 1 个槽。

**Decision**：

1. 接口（`src/hal/os/signal.hpp`）：
   - `void notify() noexcept`；
   - `bool wait(Microseconds timeout) noexcept`：true 表示取走了一个通知，false 表示超时；
   - 常量 `NO_WAIT`（0，只检查不睡眠）、`WAIT_FOREVER`（0xFFFFFFFF）。
2. 语义：
   - **通知不丢**：`notify()` 早于 `wait()` 时，下一次 `wait()` 立即返回 true；
   - **合并**：上一次 `wait()` 返回之后的多次 `notify()` 只算一次，需要计数时用队列；
   - **一次一个等待者**：每次 `wait()` 时登记当前任务，返回时注销。不同任务可以先后等同一个 `Signal`，一个任务可以先后等多个 `Signal`；两个任务同时等同一个 `Signal` 是误用；
   - **返回 true 不等于有新数据**：true 只说明"上次取走之后有过 `notify()`"，对应的事件可能已经被调用方处理过。调用方醒来后要重新检查条件（如 topic 的版本号）。这是 10-29 建议里"允许虚假唤醒"的具体含义；
   - **超时**：实际等待不少于 `timeout`，按实现的调度粒度向上取整（DM-MC02 上是 1 ms 的 RTOS 节拍）；
   - **上下文**：`notify()` 在任务与框架中断里都能调用（框架中断见 Decision-20261031-03），由实现区分上下文；`wait()` 只在任务里调用（裸机板为主循环）；两者都不能在 `CriticalSection` 内调用；
   - **常量初始化**：`constexpr` 默认构造，不可拷贝、不可移动，作全局对象时不依赖构造顺序。
3. 存储：契约头里只有两个与 RTOS 无关的私有字段，由实现在临界区内读写：
   - `void *waiter_`：正在等待的任务，FreeRTOS 上存任务句柄；
   - `bool pending_`：有没有待处理的通知。

   待处理状态放在 `Signal` 里，不放在任务通知里。这样一个任务先后等几个 `Signal` 也互不干扰：任务通知只用来叫醒任务，醒来后各自检查自己的 `pending_`。
4. FreeRTOS 实现（F2）：
   - `wait()`：在临界区内检查 `pending_`，有就清掉并返回 true；没有就登记当前任务，出临界区后 `ulTaskNotifyTakeIndexed(1, pdTRUE, 剩余节拍)`，醒来再检查。剩余时间按 `now_us()` 计算。
   - `notify()`：在临界区内置 `pending_`、取出 `waiter_`；出临界区后，有等待者就叫醒它：任务里用 `xTaskNotifyGiveIndexed`，中断里用 `vTaskNotifyGiveIndexedFromISR` 加 `portYIELD_FROM_ISR`。用 IPSR 判断是否在中断里。
   - 用 1 号通知槽，避开 0 号（CMSIS 线程标志、stream buffer），`configTASK_NOTIFICATION_ARRAY_ENTRIES` 改为 2。`FreeRTOSConfig.h` 属于生成区：F2 先查 CubeMX 6.17.0 的 FreeRTOS 参数里有没有这一项；没有的话只能写在 `USER CODE BEGIN Defines` 区，超出规则 7 的范围，要另行登记例外，由负责人决定。
     - 2026-10-31 补充：CubeMX 6.18.1 没有这一项，改用板级 CMake 的编译宏统一设置，不改生成区，见 Decision-20261031-10。
   - 裸机板（MSPM0）：`pending_` 加 WFI，不用 `waiter_`。
5. 宿主替身（`platform/mock`）：
   - 用条件变量等待，超时按测试推进的时间计算；
   - 检查误用并 `abort()`：在临界区内调用 `wait()`、`notify()`；两个线程同时等同一个 `Signal`；
   - 真实时间 10 s 还没被叫醒也 `abort()`，说明测试漏了 `notify()` 或推进时间，避免测试挂起。

**Alternatives / Rejected**：

- **第一次 `wait()` 时绑定，以后固定不变**（10-29 的建议）：驱动等 DMA 完成用的 `Signal` 由调用驱动的任务来等，绑定死以后就不能换任务。每次 `wait()` 时登记的代价相同（本来就要进一次临界区）。改为每次登记。
- **待处理状态只存在任务通知里**（`Signal` 里不放 `pending_`）：一个任务等多个 `Signal` 时，A 的通知会被 `wait(B)` 取走，A 的通知就丢了。否决。
- **FreeRTOS 二值信号量**：任意任务都能等，也支持多个等待者；但对象必须在运行时创建，中断里不能创建，`Signal` 就不能常量初始化；对象约 80 字节，大小还因平台而异，契约头没法预留空间。否决。
- **多个等待者**：要用信号量或事件组，架构已定不进框架。否决。

---

## Decision-20261031-03（hal/os 临界区只屏蔽框架中断，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（负责人选定 AI 建议的方案 B）
- **Source**：plan.md F1 第 3 步；架构 v2 §3.2"`CriticalSection`：构造时关中断"；Decision-20261029-02 方案 1 的依赖

**Context**：

- Cortex-M 有两种关中断的方式（优先级数字越小越高）：
  - PRIMASK：一刀切，关掉全部可屏蔽中断；
  - BASEPRI：只关优先级数值大于等于阈值的中断，阈值以上的照常响应。
- FreeRTOS 的阈值是 5（`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`）：优先级 5–15 的中断可以调用 `FromISR` 函数，0–4 的不能调用 FreeRTOS，FreeRTOS 自己的临界区也从不屏蔽它们。
- DM-MC02（`.ioc` 的 NVIC 配置）：外设中断（FDCAN、SPI、DMA、BDMA、MDMA、串口、USB、EXTI、OCTOSPI、TIM4/5）全部是 5；SysTick、PendSV、HAL 时基 TIM1 是 15；没有中断用 0–4。
- 旧 PortKit 的临界区用 PRIMASK。按 DM-MC02 现在的配置，两种方式效果完全一样；只有将来把某个不能被耽误的中断（如高频电流环、编码器捕获）放到 0–4 时才有区别。

**Decision**：

1. 临界区屏蔽全部"框架中断"，不屏蔽框架线以上的中断：
   - **框架线**每块板一条。FreeRTOS 板与 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 相同，DM-MC02 是优先级 5；
   - **框架中断**：优先级不高于框架线的中断（DM-MC02 为 5–15），可以使用框架（`hal::os`、topic，以及用到它们的驱动）；
   - **线以上的中断**（DM-MC02 为 0–4）不得调用 RTOS，也不得使用框架，只能自己处理数据，再经框架中断或任务转交；
   - 各平台：M3/M4/M7 用 BASEPRI；M0+ 没有 BASEPRI，用 PRIMASK，没有线以上的中断；R5 屏蔽 IRQ，FIQ 在线以上。
2. 实现要求：
   - 构造时保存原状态再屏蔽，析构时恢复原状态；任务与框架中断里都能用；可以嵌套；
   - 不能用 `taskENTER_CRITICAL`：它只能在任务里用，而且按嵌套计数开关中断，不保存原状态；
   - DM-MC02 的做法等价于 FreeRTOS 的 `portSET_INTERRUPT_MASK_FROM_ISR` / `portCLEAR_INTERRUPT_MASK_FROM_ISR`：读出旧 BASEPRI，用 `BASEPRI_MAX` 抬到框架线，析构时写回。
3. 临界区内不得阻塞，不得调用 RTOS API，也不得调用 `Signal`。原因：FreeRTOS 的任务级 API 退出它自己的临界区时，嵌套计数归零就把 BASEPRI 清零，会在我们的临界区里提前打开中断。
4. 宿主替身（`platform/mock`）：一把进程内的递归锁，同一时刻只有一个线程在临界区内，对应单核上关中断的效果。
5. 对 Decision-20261029-02：topic 若选方案 1（发布时用临界区包住写入），线以上的中断既不能发布也不能读 topic，与第 1 点的规则一致。

**Alternatives / Rejected**：

- **A 全部屏蔽（PRIMASK）**：任何中断都能使用框架；但最高优先级的中断也会被推迟，每次推迟一个临界区的时长（发布 topic 时是一次消息拷贝）。否决。

---

## Decision-20261031-04（DM-MC02 启用 TIM2 作时基计数源；CubeMX 改用 6.18.1，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（负责人在 CubeMX 中配置并生成；提交方式与合入时机由负责人选定 AI 的建议）
- **Source**：Decision-20261031-01 第 5 点；修订其中的 CubeMX 版本（6.17.0 → 6.18.1）

**Context**：

- Decision-20261031-01 定了 DM-MC02 用 TIM2 分频到 1 MHz 作时基，`.ioc` 由负责人配置。
- 2026-10-31 起开发环境从 Windows（WSL）换到 Ubuntu 24.04。负责人用 Linux 上的 STM32CubeMX 6.18.1（数据库 DB.6.0.181）生成，上一版是 Windows 上的 6.17.0（DB.6.0.170）。
- 生成结果与上一版的差别分两类：
  - **格式**：`.ioc` 行尾 CRLF → LF（953 行）；`usbd_conf.c` 行尾 LF → CRLF（665 行，文件末尾补了换行）；`.mxproject` 路径分隔符 `\` → `/`，头文件列表换了顺序。去掉这些差异后内容相同。
  - **内容**：TIM2（`.ioc` 48 行；`tim.c`、`tim.h`；`main.c` 多一行 `MX_TIM2_Init()`）。`tim.c` 里 `HAL_TIM_PWM_Msp*` 与 `HAL_TIM_Base_Msp*` 换了先后顺序，函数内容不变。
- 固件包仍是 FW_H7 V1.13.0，`Drivers/`、`Middlewares/` 没有变化；时钟树没变，TIM2 的时钟仍是 240 MHz。

**Decision**：

1. TIM2 配置：
   - 内部时钟，分频 240−1，向上计数，自动重装 0xFFFFFFFF，时钟不分频，不预装载，TRGO Reset；
   - 不开中断，不接 DMA；
   - `MX_TIM2_Init()` 只初始化、不启动。计数器由 port 在 `hal::init()` 里启动（F2），USER CODE 区不加代码。
2. CubeMX 的基准版本改为 6.18.1（DB.6.0.181），以后在 Linux 上用这个版本生成。`docs/ai/decisions/2026-10-25-迁移基线与对照.md` 里的 6.17.0 是迁移源的历史记录，不改。
3. 生成物按 Linux 上 CubeMX 的输出入库，以后在本机重新生成就不会再出现格式差异。分两次提交：
   - 第一次只含格式变化（上述三个文件，去掉行尾与分隔符差异后内容不变），登记到 `.git-blame-ignore-revs`；
   - 第二次是 TIM2 的实际变化；
   - 本分支合入时用 merge 提交，不要 rebase 或 squash，否则 `.git-blame-ignore-revs` 里的提交号会失效。
4. 固件符号的变化（对照 P1 存档，其他符号不变）：
   - 新增 `htim2`（76 B）、`MX_TIM2_Init`（Debug 156 B、Release 108 B）；
   - `HAL_TIM_Base_MspInit` 变大（Debug 144 → 184 B，Release 120 → 152 B）；
   - `main` 变大（Debug 232 → 236 B，Release 184 → 188 B）。
5. 合入时机：CI 的 `firmware` 作业还在对照 P1 存档，上面的变化会让它失败。分支 `f2/tim2-timebase` 等 F2 让符号对照退役的 CI 改动合入后再开 PR。

**验证**：固件 Debug、Release 构建无告警；符号差异与第 4 点完全一致。

**Alternatives / Rejected**：

- **把生成物手工转回 Windows 上的格式**：等于手工修改生成区；而且下次在 Linux 上重新生成又会变回来。否决。
- **继续用 6.17.0**：Linux 上要另装旧版本；6.18.1 这次生成的代码除 TIM2 外与上一版相同，没有理由留在旧版本。否决。
- **现在就合入，并在 CI 的符号对照里加"预期变化"清单**：要为一个提前一个阶段的改动修改 `symbols.py` 和 CI。否决。

---

## Decision-20261031-05（topic 机制 v1：接口与实现，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（落实架构 v2 §3.4 与 Decision-20261029-02 方案 1；接口细节由 AI 拟定，负责人在 F4 框架验收时评估用起来是否顺手）
- **Source**：plan.md F1 第 4 步；架构 v2 §3.4
- **编号**：Decision-20261031-04 已在分支 `f2/tim2-timebase` 上使用（TIM2 时基），本条顺延为 -05

**Context**：

- 架构 v2 §3.4 定了 topic 的语义：最新值、单发布者、中断里可发布、`update()` 与 `wait()`、观测计数、常量初始化。
- 没有定的：发布句柄怎么取得和保管、等待槽有几个、`wait()` 什么时候返回、丢失次数从什么时候算、调试通道怎么列出全部 topic、target 的形式。

**Decision**：

1. **位置与 target**：`src/middleware/topic/topic.hpp`，命名空间 `topic`。只有头文件，INTERFACE target `middleware_topic`，依赖 `hal_os`、`utils_concurrency`。
2. **`Topic<T>`**：
   - 用 `LatestValue<T>` 保存最新值；构造函数是 `constexpr`，名字字符串只存指针；
   - 只在 `app/topics` 定义；
   - 消息约束用 `static_assert` 检查：可平凡拷贝，`sizeof(T) <= MAX_MESSAGE_SIZE`（暂定 64 字节，Decision-20261029-02）。
3. **`Publisher<T>`**：
   - 由 `Topic::claim_publisher()` 取得。每个 topic 只有第一次调用成功（在临界区内检查标志），以后都返回无效句柄，初始化时检查，失败则初始化失败；
   - 可移动、不可拷贝；无效句柄的 `publish()` 什么也不做；
   - `publish()` 的步骤：先读 `now_us()`；在临界区内写入消息，并记录发布次数与时刻；出临界区后逐个叫醒等待者。任务与框架中断里都能调用。
4. **`Subscriber<T>`**：
   - 构造函数是 `constexpr`，可以常量初始化；一个订阅者对象只给一个上下文用；
   - `update(out)`：版本号变了才拷贝；错过的发布计入 `lost_count()`。订阅者构造后还没读过时，此前的发布都算错过。任务与框架中断里都能用；
   - `wait(out, timeout)`：先 `update()`，有没读过的数据就立即返回；没有才睡眠，醒来再 `update()`。通知是旧的（数据已被读过）就接着等；超时返回 false。只在任务里用；
   - `enable_wait()`：在 topic 上占一个等待槽，登记自己的 `Signal`。初始化阶段调用，失败则初始化失败；重复调用只占一个槽。
5. **等待槽**：每个 topic 固定 `WAIT_SLOTS = 4` 个，只增不减，在临界区内写。发布时在临界区外用 acquire 读取。没有调用 `enable_wait()` 就 `wait()` 的，发布叫不醒它，只能睡到超时，不会空转。
6. **观测**：
   - 非模板基类 `TopicBase` 提供 `name()`、`publish_count()`（按 2^32 回绕）、`last_publish_us()`；订阅者提供 `lost_count()`；
   - 产品在 `app/topics/all.hpp` 用 `const topic::TopicBase *` 的数组列出全部 topic，给调试通道用。它是编译期常量，不需要运行期登记。
7. **不用原子读改写**（规则 13）：每个计数都只有一个写者，用"读一次、写一次"更新。
8. **清单约定**：写在 `app/topics/README.md`。`app_topics` target 与 `arch_check.py` 的 topics 层随第一个 topic 加入（F2）；topic 实例不加 `g_`，与 `.clang-tidy` 的全局变量规则冲突，届时一并处理。`arch_check.py` 的 app 目录白名单现在补上 `topics`，与架构 §5.4 一致。
9. **留给以后**：
   - 任务运行器按 topic 触发任务（F2）时，可能要让运行器直接在 `TopicBase` 上登记自己的 `Signal`。现在只有订阅者能登记；
   - 等待槽数与消息上限在 F2 实测后复核。

**验证**：
- 宿主 ctest 22/22（Ubuntu 24.04，g++ 13.3）：新增 `topic` 测试；另加 3 项配置期检查，64 字节的消息能编译，65 字节的和不可平凡拷贝的被 topic 的 `static_assert` 拒绝；
- 本地 ThreadSanitizer 跑 3 次无报告（g++ 13.3 加 `-Wno-tsan`）；clang++ 18 加 `-Wall -Wextra -Wshadow -Werror` 编译运行通过；正常构建连续 50 次通过；
- 反向验证，下面 10 处故意改坏都被测试抓到：
  - 发布不进临界区；写入挪到临界区外；在临界区内叫醒等待者；
  - 丢失计数少减 1；
  - 超时后不再检查新数据；把旧通知当作新数据；等待前不先检查；
  - 发布者不检查；等待槽不去重；只叫醒第一个等待者；
- 交叉编译：Cortex-M0+/M4/M7 × `-O0`/`-Os`/`-O2` 加 `-Werror` 通过，没有 `__atomic_*`、`__sync_*` 库调用；全局的 `Topic`、`Subscriber`、`Publisher` 在 `.data`、`.bss`，没有运行期构造；
- 格式、命名（0 处）、文本扫描、两张依赖图检查通过；固件 Debug/Release 构建通过，符号与 P1 存档一致（1426、1038）。topic 还没有进固件。

**Alternatives / Rejected**：
- **每个 topic 用模板参数指定等待槽数**：订阅者的类型要带上槽数，或者多一层基类；预计每个 topic 只有 1–2 个等待者。否决，固定 4 个，不够时改常量。
- **`wait()` 严格等下一次发布**：两次 `wait()` 之间到来的发布会被跳过，任务处理慢一点就整拍丢数据。否决，改为"有没读过的就立即返回"，与 `update()` 的判断一致。
- **没有 `enable_wait()` 时 `wait()` 立即返回 false**：任务循环会空转，高优先级任务会饿死其他任务。否决，改为睡到超时。
- **第一次 `wait()` 时自动登记**：槽满要到运行时才发现，违背"初始化阶段发现"。否决。
- **发布句柄可拷贝**：同一个 topic 容易出现两个发布者。否决。

---

## Decision-20261031-06（F1 的 CI：框架独立构建、Cortex-M0+ 原子检查、规则 13 与 17 的文本扫描，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（负责人要求直接推进 F1 的 CI 项；做法由 AI 拟定）
- **Source**：plan.md F1"CI"项；架构 v2 §5.2 规则 2、13、15、17，§5.3；Decision-20261029-01 第 2 点（用 M0+ 编译检查代替 `is_always_lock_free`）

**Context**：

- 规则 2、15 要求框架（`src/` + `platform/` + `cmake/`）不引用产品。现在只有依赖图检查与文本扫描，没有在"产品不存在"的条件下构建过框架：
  - 链接一个不存在的 target 名时，CMake 只把它当成库文件名，静态库照常生成；
  - 产品目录还在，include 产品路径也能编译通过。
- 规则 13（不用原子读改写）还没有检查。现有的原子读改写正好是 X-08 登记的那些：dji 电机 2 处，旧 app 6 处。文本扫描只能查函数名，查不到对原子变量直接用 `++`、`+=` 的写法。
- Cortex-M0+ 没有 LDREX/STREX，原子读改写只能编译成 `__atomic_*` 库调用。在 M0+ 上编译，就能把所有写法都查出来。
- 规则 17 只有 topic 的 `static_assert`。"只在 `app/topics` 定义"和"一个发布者"还没有静态检查。
- 只有头文件的模板（`utils_concurrency`、`middleware_topic`）没有使用方就不会被编译。框架独立构建与 M0+ 检查都要先把它们实例化。

**Decision**：

1. **框架独立构建**：根 `CMakeLists.txt` 加选项 `FRAMEWORK_ONLY`，新增两个 preset：
   - `framework`（宿主）：配置 `src` 与 `platform/mock`；
   - `framework-m0plus`：用新工具链文件 `cmake/toolchains/arm-gcc-cortex-m0plus.cmake` 交叉编译 `src`，不选板卡，只编译不链接；
   - 两者都不含 `app/`、`config/`、`test/host` 与任何板卡，都另编 `check_templates`（见第 2 点）；
   - 固件构建仍只有 `-DPROFILE` 一个开关；`FRAMEWORK_ONLY` 只用于这两个 preset，区分构建种类，与 host-tests 用工具链区分是同一类做法。
2. **模板实例化**：`test/framework/template_check.cpp` 显式实例化 `SpscQueue`、`LatestValue`、`Topic`、`Publisher`、`Subscriber`，target 是 `check_templates`（STATIC），定义在根 `CMakeLists.txt` 的 `FRAMEWORK_ONLY` 分支里。
   - 不为它新建 CMakeLists：它只有一个文件，只在框架独立构建里出现；
   - `arch_check.py` 新增 `check` 层（`check_\w+`），只能依赖框架；
   - 以后新增只有头文件的并发原语或 topic 模板时，在这里补实例化。
3. **CI 作业 `framework`**：
   - 用 `git archive` 只取出 `CMakeLists.txt`、`CMakePresets.json`、`cmake/`、`src/`、`platform/mock/`、`test/framework/`，在这个副本里构建；
   - 宿主：配置后用 `arch_check.py graph --mode framework` 检查依赖图，再构建。framework 模式下，图里不得出现产品、板卡、RTOS 或测试，连"外部库"形式的同名节点也不行，这样链接不存在的产品 target 也会被拦下；
   - Cortex-M0+：构建后用新增的 `arch_check.py atomics` 检查所有静态库没有 `__atomic_*`、`__sync_*` 调用。
   - 放在单独的作业里，不放进 host-tests：它要 Arm 工具链，而且必须在副本里构建。架构 §5.3 原来写的是"host-tests 增加框架独立构建"，这里改为单独作业。
4. **M0+ 的范围**：plan 原来只写 utils 与 topic，现在扩大为整个框架的默认构建，新写的框架代码自动纳入检查。暂时排除 3 个 target，登记为**例外 X-17**，F3 驱动适配后解除：
   - `driver_bmi088`、`driver_motor_dji`：用 `static_assert(is_always_lock_free)` 要求原子读改写无锁，M0+ 上编译失败，后者另有 X-08；
   - `lib_estimate`：经 `imu_body_mapping` 依赖 `driver_bmi088`（已知结构债）。

   排除的写法是在根 `CMakeLists.txt` 里给这 3 个 target 设 `EXCLUDE_FROM_ALL`，只在交叉编译的框架构建里生效。其余 11 个静态库都能为 M0+ 编译，也没有原子库调用。
5. **文本扫描**（`arch_check.py scan`，lint-arch）：
   - **规则 13**：扫描 `src/`、`app/`、`config/`、`platform/` 的手写代码（生成区与 vendor 头除外），查 `.fetch_*(`、`.exchange(`、`.compare_exchange_*(`、`.test_and_set(` 与 `std::atomic_*` 的读改写自由函数。`std::exchange` 不是原子操作，不算。X-08 的 4 个文件放行；放行的文件改掉以后，扫描会提示删掉这一条；
   - **规则 17**：`topic::Topic<` 只能出现在 `app/topics/` 与 `src/middleware/topic/`；同一个 topic 的 `claim_publisher()` 只能出现在一处；`app/topics/` 的头文件只能 include 标准库头（不带 `.h` 的名字）、`middleware/topic/topic.hpp`、`hal/os/clock.hpp` 与清单里的其他头，消息因此用不了 `lib` 等层的类型。
6. **命名检查**：lint-naming 的编译数据库加上 `framework` preset，否则 `test/framework` 不会被检查。

**验证**：
- 只含框架文件的副本里：`framework` 构建通过，framework 模式的依赖图检查通过（24 个 target）；`framework-m0plus` 构建通过，11 个静态库没有原子库调用；
- 反向验证，下面每一项都被拦下：
  - 依赖图：`lib_control` 链接 `app_imu`；
  - 构建：框架 include 产品头；
  - M0+：utils 里对 `std::atomic` 用 `++`（文本扫描查不到的写法），报 `__atomic_fetch_add_4`；
  - 规则 13：`fetch_add`、`compare_exchange_weak`。`std::exchange` 不误报；
  - 规则 17：框架里定义 `topic::Topic`；同一 topic 两处 `claim_publisher()`（只出现一次的不误报）；清单头 include `lib` 头与 `<stdint.h>`（`<array>`、topic 头、清单头不误报）；
  - X-08 放行文件改掉原子读改写后，扫描给出删除提示；
- 现有构建不变：宿主 ctest 22/22；固件 Debug/Release 构建通过，符号与 P1 存档一致（1426、1038）；固件与宿主依赖图检查通过；格式检查通过；命名检查用 4 个编译数据库，0 处。

**Alternatives / Rejected**：
- **只为 M0+ 编 utils 与 topic**（plan 原文）：新写的驱动和 lib 代码要等到阶段 E 做 MSPM0 时才会被检查。否决，改为全框架 + 例外。
- **用 `static_assert(is_always_lock_free)` 检查**：它要求全部原子操作都无锁，M0+ 上恒为 false（Decision-20261029-01）。否决。
- **在完整仓库里配置框架构建**：产品目录还在，include 产品路径照样能编译，证明不了规则 15。否决，改为在副本里构建。
- **为 `check_templates` 单独建 `test/framework/CMakeLists.txt`**：多一个 CMake 文件（Decision-20261026-07 要求先登记），换来的只是一个只有一个源文件的 target。否决。

---

## Decision-20261031-07（F2 的顺序：第 1–4 步按外设无关的部分先做，第 5 步待定，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（第 1–4 步：负责人同意 AI 建议的顺序）；第 5 步 Proposed（负责人考虑中）
- **Source**：plan.md F2、F3；待登记中"旧包装类与 `src/hal/detail` 的删除时机"（2026-10-31 登记）

**Context**：

- plan 的 F2 先归档旧 app，再建契约 v1、用 C++ 重写 port，最后删除 `src/hal/detail`。驱动改用 hal v1 放在 F3。
- 2026-10-31 登记的问题：F2 一删旧接口，4 个驱动就编不过。当时 AI 建议方案 A，F2 先保留旧接口到 F3。
- 补充发现：现在的 C port 自己定义了 9 个 HAL 回调（`HAL_FDCAN_RxFifo0Callback`、`HAL_SPI_TxCpltCallback` 等）。同一个回调只能有一份实现，所以同一个外设的新 C++ port 不能和旧 C port 同时存在。方案 A 要新旧并存，就得多写一层用完即弃的转接代码。

**Decision**：

F2 先做与具体外设无关的部分，每一步固件都可构建：

1. CI 的固件符号对照退役，改为段大小报告，加入 HAL 回调符号检查（Decision-20261031-08）；合入 TIM2 分支（Decision-20261031-04）。
2. 板级目录从 `platform/st/stm32h7/board_dm` 搬到 `platform/dm_mc02`（按板一级），只移动不改内容。
3. hal/os 的 port 实现：TIM2 时基、BASEPRI 临界区、`Signal` 用 1 号任务通知槽（Decision-20261031-01～03）。
4. 启动入口 `boot_*`、`app/common` 任务运行器、profile `bringup_dm_mc02` 就位后，归档旧 app 与 profile `baseline`，固件改为 bringup。

第 5 步（外设）待负责人决定，见"待登记"。AI 建议按外设纵向推进，把 F2 与 F3 合起来排：

- 每个外设依次做：契约 v1 + mock → port 用 C++ 重写这一块 → 改用它的驱动 → 删除这一块的旧包装与 `hal/detail`；
- 顺序：gpio/pwm → can → spi → uart → usb_cdc。

**本地分支**（PR 暂不开，负责人要求先在本地推进）：
- 第 1 步在 `f2/firmware-ci`，从 `f1/ci` 切出；
- 第 2 步的分支先用 merge 合入 `f2/tim2-timebase`，再搬目录；
- 以后开 PR 的顺序：`f1/topic` → `f1/ci` → `f2/firmware-ci` → `f2/tim2-timebase`（merge 提交，保留提交号）→ 第 2 步起的分支。

**Alternatives / Rejected**：
- **原方案 A（新旧接口并存到 F3）**：要多写一层转接代码，port 的 C++ 重写也要推迟。否决。
- **原 plan 的顺序（F2 开头就归档旧 app）**：bringup 就位之前没有可构建的固件。否决，改为第 4 步先建 bringup 再归档。

---

## Decision-20261031-08（固件 CI：符号对照退役，改为段大小报告；HAL 回调符号检查，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（plan.md F2 的 CI 项；负责人同意 F2 第 1 步）
- **Source**：plan.md F2"CI"项；架构 v2 §5.2 规则 16、§5.3；AGENTS.md §4、§6；Decision-20261031-04

**Context**：

- P1–P2.5 的判据是"固件符号与迁移源一致"，基准是 P1 存档的符号表。F2 起固件行为开始改变（TIM2、port 重写、归档旧 app），这个判据不再适用。TIM2 的提交也一直在等它退役。
- 规则 16：覆盖 HAL 弱符号的回调写在 `.cpp` 里却没有 include 对应 HAL 头时，定义会得到 C++ 链接（改名），HAL 的弱实现照旧生效，编译、链接都不报错。port 改用 C++ 之前，需要先有检查。

**Decision**：

1. firmware 作业删除符号对照。`tool/ci/symbols.py` 与 P1 存档保留：纯移动、纯改名的提交（如 F2 第 2 步）仍可在本地用 `symbols.py compare` 自证符号不变。
2. 新增 `symbols.py report`：段大小报告（Markdown），写入作业摘要，并附 `.bin` 的 sha256。
   - 存储区取自 map 文件的 Memory Configuration；
   - 用量与链接器的 `--print-memory-usage` 一致：用到的最高地址减去起始地址；段按运行地址归入存储区，LOAD 段按装载地址归入（`.data` 的初值在 FLASH）；
   - 只是报告，不设门槛。
3. 新增 `arch_check.py callbacks`（规则 16）：
   - 从 port 源码找出全部 `HAL_*Callback` 定义，每个在固件里都必须是强符号（nm 的 `T`）。仍是弱实现（`W`）或没有进入固件的，都报错；
   - 固件里出现被 C++ 改名的 HAL 回调（`_Z…HAL_…Callback`）也报错；
   - CI 对 arm-debug、arm-release 两个固件都检查，port 路径随第 2 步搬目录更新。

**验证**：
- 报告与链接器一致：Release FLASH 100652、RAM_D1 17408、DTCMRAM 91584 字节；Debug FLASH 238096、RAM_D1 86112 字节；
- 回调检查：现有固件 Debug、Release 都是 9 个回调全部生效。反向验证：
  - 把 GPIO 回调挪到不 include HAL 头的 `.cpp`：报"弱实现仍在生效"。被改名的定义没人引用，被 `--gc-sections` 回收了，所以靠强符号检查发现；
  - 把 CAN 错误回调标成 `weak`：报"弱实现仍在生效"。

**Alternatives / Rejected**：
- **段大小设门槛**（如与 `dev` 比较，增长超过阈值即失败）：F2–A 阶段固件变化大，门槛只会频繁误报。暂不设，需要时再加。
- **回调清单写死在 CI 里**：port 增减回调时容易漏改。否决，改为从 port 源码提取。

---

## Decision-20261031-09（板级目录按板一级：`platform/dm_mc02`，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（F2 第 2 步，负责人同意 Decision-20261031-07 的顺序）
- **Source**：架构 v2 §2.1、§3.7、§5.4；plan.md F2 第 2 步

**Context**：

- 架构 v2 规定板级目录按板一级，目录名就是板标识（`platform/dm_mc02`）。现在的目录还是 P2 时的 `platform/st/stm32h7/board_dm`，是"厂商 / 芯片 / 板"三级。
- `platform/st/stm32f4/board_c` 与 `platform/ti` 只是空目录，没有任何入库文件。

**Decision**：

1. `git mv platform/st/stm32h7/board_dm platform/dm_mc02`，生成区（`cubemx/`）随之整体移动，内容不变：
   - `.ioc`、`.mxproject` 里只有相对路径，`ProjectManager.UnderRoot=false`；
   - 板目录里的 CMake 都以 `CMAKE_CURRENT_SOURCE_DIR` 为基准。

   负责人以后用 CubeMX 打开 `platform/dm_mc02/cubemx/mc02_cpp.ioc` 重新生成。
2. 只改 4 处路径：
   - profile `baseline` 的 `BOARD` 改为 `dm_mc02`，与 `board.cmake` 的 `BOARD_ID` 相同；
   - CI 回调检查的 port 路径；
   - `arch_check.py` 的 `platform/` 一级目录白名单由 `st`、`ti`、`mock` 改为 `dm_mc02`、`mock`，与架构 §5.4 一致；
   - OpenOCD 配置里的注释。
3. 验证方式：纯移动，固件的函数与对象符号集合应与移动前完全相同，用 `symbols.py compare` 对比移动前后的 dump（基准是本分支合入 TIM2 之后、移动之前的固件）。

**Alternatives / Rejected**：
- **保留"厂商 / 芯片 / 板"三级**：与架构 v2 不符；同一芯片出现第二块板时再决定共用代码放哪里（§3.7）。否决。

---

## Decision-20261031-10（hal/os 在 DM-MC02 上的实现：TIM2、BASEPRI、任务通知槽，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（F2 第 3 步，负责人同意 Decision-20261031-07 的顺序；第 2、3 点按此前登记的建议执行，负责人可以否决）
- **Source**：Decision-20261031-01 第 5 点、-02 第 4 点、-03 第 2 点、-04；plan.md F2 第 3 步

**Context**：

- hal/os 契约 v1 在 F1 定稿，固件上还没有实现。
- Decision-20261031-02 第 4 点：`Signal` 用 1 号任务通知槽，`configTASK_NOTIFICATION_ARRAY_ENTRIES` 要改成 2。先查 CubeMX 有没有这一项；没有的话只能写进 `FreeRTOSConfig.h` 的 USER CODE 区，超出规则 7，要登记例外，由负责人决定。
- 查得：CubeMX 6.18.1 的整个参数数据库（`db/`）里都没有 `ARRAY_ENTRIES`。`FreeRTOS.h` 在没有定义时用 `#ifndef` 取默认值 1。
- Decision-20261031-01 第 5 点：调试暂停时是否冻结 TIM2 在 F2 定，当时建议冻结。

**Decision**：

1. 实现放在 `platform/dm_mc02/port/os/`（C++），每个契约头对应一个文件：
   - `clock.cpp`：`now_us()` 读一次 `TIM2->CNT`；
   - `critical_section.cpp`：保存 BASEPRI，用 `BASEPRI_MAX` 抬到 `configMAX_SYSCALL_INTERRUPT_PRIORITY`（优先级 5），然后 `isb`、`dsb`；析构时写回；
   - `signal.cpp`：`pending_`、`waiter_` 在临界区内读写，用 1 号通知槽叫醒任务（中断里用 FromISR 版本加 `portYIELD_FROM_ISR`，按 IPSR 判断上下文）；醒来后以 `pending_` 为准，超时按 µs 核对；误用由 `configASSERT` 拦下（在中断里或临界区内调用 `wait()`、在临界区内调用 `notify()`、两个任务同时等）；
   - port 私有的 `os.hpp` 声明 `port::os::start_clock()`。
2. **通知槽的个数用编译宏统一设置，不改生成区**：板级 CMake 定义 `rtos_definitions`（`configTASK_NOTIFICATION_ARRAY_ENTRIES=2`），同时加进 `cubemx_definitions`（生成代码、内核、USB、port）与 `os_freertos_<板>` 的接口。
   - 它决定任务控制块与 `StaticTask_t` 的大小，所有 include FreeRTOS 头的编译单元都必须一致；
   - `signal.cpp` 用 `static_assert` 检查至少有 2 个槽；
   - 这样不需要第 -02 第 4 点说的 USER CODE 例外。
3. **调试暂停时冻结 TIM2**（`__HAL_DBGMCU_FREEZE_TIM2()`），按 -01 的建议：SysTick 在内核暂停时也停，两者一致，断点恢复后不会集体超时。
4. TIM2 的计数器由 `port::os::start_clock()` 启动，F2 第 4 步由 `hal::init()` 调用。在那之前固件里没有调用方，这一步的函数会被 `--gc-sections` 回收，要到第 4 步的 bringup 才在板上运行。
5. port 链接 `hal_os` 契约（依赖图允许 port → 契约）。

**验证**：
- 固件 Debug、Release 构建无告警（port 带 `-Wextra -Werror`）；
- 用 ninja 的依赖记录核对：Debug、Release 各有 16 个编译单元 include `FreeRTOS.h`，全部带 `-DconfigTASK_NOTIFICATION_ARRAY_ENTRIES=2`；
- 反汇编核对：
  - 临界区是 `mrs BASEPRI` → `msr BASEPRI_MAX, 0x50` → `isb` → `dsb`，析构是 `msr BASEPRI`；
  - `now_us()` 是一次读 `0x40000024`（TIM2_CNT）；
  - `start_clock()` 置 DBGMCU `APB1LFZ1` 的 TIM2 位再启动 TIM2；
  - `notify()` 调用 `vTaskGenericNotifyGiveFromISR` / `xTaskGenericNotify`，下标都是 1；
- 固件符号的变化（Release）只有：`Idle_TCB`、`Timer_TCB`（cmsis_os2.c 里的 `StaticTask_t`）由 92 变为 96 字节，与内核的任务控制块一致；`xTaskGenericNotify`、`xTaskGenericNotifyFromISR`、`xTaskGenericNotifyWait` 变大；
- HAL 回调检查、依赖图、文本扫描、格式、命名（0 处）通过。

**Alternatives / Rejected**：
- **写进 `FreeRTOSConfig.h` 的 USER CODE 区**（-02 第 4 点的原方案）：要为生成区登记例外，而且宏的位置不显眼。改用编译宏，效果相同，不碰生成区。
- **改用 0 号通知槽**：会与 CMSIS-RTOS2 的线程标志、stream buffer 冲突。旧 app 归档后线程标志虽然没人用，但 stream buffer 仍可能被引入。否决。
- **不冻结 TIM2**：断点停住后 TIM2 继续走，SysTick 停着，恢复运行时 µs 超时会集体到期，调试时的行为与正常运行不同。否决。

---

## Decision-20261031-11（启动入口 `boot_*` 与 `hal::init()`；生成区只调 `boot_*`，X-09 关闭，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（F2 第 4 步的第一部分；负责人同意 Decision-20261031-07 的顺序）
- **Source**：架构 v2 §3.1"初始化"、§3.3、§6；AGENTS.md §2"USER CODE 区里对 `boot_*` 的调用（改动行登记在 decision.md）"；plan.md F2 第 4 步

**Context**：

- 现在 `main.c` 的 USER CODE 区先调 port 的 `Port_Init()`，再调 `Application_Init()`；`freertos.c` 调 `Application_InitRtos()`，默认任务循环调 `Application_Start()`。头文件是 `config/assembly/Application/Entry.h`，由 config 把 assembly 目录注入生成区的 include 路径（例外 X-09）。
- 架构 v2：生成区只调 `src/boot/boot.h` 的 `boot_*`，`hal::init()` 由 port 实现，assembly 在 `boot_init()` 里调用；生成区不直接调 port。
- 第 4 步的任务运行器与 bringup 需要先确认设计，旧 app 要到 bringup 就位后才归档。启动入口可以先换，旧 app 照常运行。

**Decision**：

1. `src/boot/boot.h`（C 头，`extern "C"`）：
   - `bool boot_init(void)`：外设初始化之后、内核初始化之前；
   - `bool boot_init_rtos(void)`：内核初始化之后、启动之前，创建任务；
   - `void boot_run(void)`：默认任务每拍调用一次，可以为空。

   `boot` target 列出这个头。
2. `src/hal/init.hpp`：`[[nodiscard]] bool hal::init() noexcept`，初始化本板外设与 hal/os 的时基，在 `boot_init()` 里调用一次。
   - DM-MC02 的实现在 `platform/dm_mc02/port/hal_init.cpp`：先 `port::os::start_clock()`，再调迁移期的 C port `Port_Init()`；
   - `Port_Init()` 第 5 步随外设逐个改写后去掉。
3. **生成区的改动行**（只在 USER CODE 区，CRLF 行尾不变）：
   - `Core/Src/main.c`：
     - `USER CODE BEGIN Includes`：`#include "Application/Entry.h"`、`#include "Init.h"` 两行改为 `#include "boot/boot.h"`；
     - `USER CODE BEGIN 2`：`Port_Init()`、`Application_Init()` 两段改为一段 `if (!boot_init()) { Error_Handler(); }`；
   - `Core/Src/freertos.c`：
     - `USER CODE BEGIN Includes`：`#include "Application/Entry.h"` 改为 `#include "boot/boot.h"`；
     - `USER CODE BEGIN RTOS_THREADS`：`Application_InitRtos()` 改为 `boot_init_rtos()`；
     - `USER CODE BEGIN StartDefaultTask`：循环里的 `(void)Application_Start();` 改为 `boot_run();`，`osDelay(1)` 不变。
4. 迁移期的 assembly `entry_legacy.cpp` 改为实现 `boot_*`：
   - `boot_init()` = `hal::init()` + 原 `Application_Init()` 的内容；
   - `boot_init_rtos()` = 原 `Application_InitRtos()`；
   - `boot_run()` = 原 `Application_Start()`，返回值不再需要。

   删除 `Application/Entry.h` 与 config 向生成区注入 include 路径的那一行，**例外 X-09 关闭**。生成区经 `boot` target 取得 `src/` 的 include 路径。
5. `arch_check.py`：
   - `boot` 从契约层分出来单独成层，只有生成区（vendor）与 assembly 可以依赖它；
   - 文本扫描加入规则 7 的 include 部分：生成区（`cubemx/`，`Drivers/`、`Middlewares/` 除外）的 `#include "…"` 只能指向生成区自己的头，或 `boot/boot.h`，或 X-01 的 `UsbCdc.h`。规则 7 中"USER CODE 区只调 `boot_*`"的部分靠本条登记的改动行与评审把关。
6. 行为变化：启动时多启动了 TIM2（hal/os 时基），其余与原来相同。负责人上板确认旧固件照常运行。

**上板验证**（2026-10-31，记录见 `docs/ai/decisions/2026-10-31-F2-启动入口上板验证/`）：
- 烧录前先只读备份板上固件：与 2026-10-26 的存档逐字节相同。备份放在仓库外的 `foray_mcu_archive/2026-10-31_DM-MC02_flash_before_f2/`；
- 烧录 arm-debug 固件，`Verified OK`；
- 不停核读取：
  - TIM2 1 s 增加约 100 万，与 RTOS 节拍的增量一致；调试冻结位已置；
  - 调度器运行，共 6 个任务；TaskManager 为 RUNNING，任务在内核启动前创建，3 个任务全部创建成功；
  - 栈余量充足；
  - PC 采样不在 `Error_Handler`；
  - BMI088 陀螺、加速度计的采样序号按 2 kHz、1.6 kHz 增长。

  旧固件照常运行。

**Alternatives / Rejected**：
- **等任务运行器与 bringup 一起换**：一次改动太大，出问题时不好定位。否决，先单独换入口。
- **`hal::init()` 等第 5 步的契约写法定了再加**：三种写法里它都是同一个自由函数，不受写法影响。否决。

---

## Decision-20261031-12（任务运行器、bringup 能力与 profile，第一个 topic，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（F2 第 4 步的第二部分；负责人选定 AI 推荐的三点：step 用函数指针、任务栈静态分配、公开 `TopicBase::add_waiter()`）
- **Source**：架构 v2 §3.4、§3.5、§3.6；plan.md F2 第 4 步；Decision-20261031-05 第 9 点、`app/topics/README.md`"待第一个 topic 加入时处理"

**Context**：

- 架构 v2 §3.5 只定了任务表的内容（名称、栈、优先级、触发方式、按顺序调用的 step）与"运行器用 FreeRTOS 原生 API、巡检栈水位"，没有定写法。
- 按 topic 触发的任务要在 topic 上登记自己的 `Signal`，而现在只有订阅者能登记（-05 第 9 点）。
- bringup profile 不含 RM 能力，但它的行为（发布、订阅、统计）按架构属于能力，不能写进组合根；它用的 topic 也必须定义在 `app/topics`（规则 17）。

**Decision**：

1. **topic**：`TopicBase::add_waiter(hal::os::Signal &)` 改为公开，供任务运行器登记。语义不变：占一个等待槽，重复登记不占新槽，槽满返回 false。
2. **任务运行器**（`app/common/task_runner.hpp`，命名空间 `common`，只有头文件，用 FreeRTOS 原生 API）：
   - `using Step = void (*)(hal::os::Microseconds now)`。组合根用普通函数（或不捕获的 lambda）调能力对象；同一个任务的各 step 拿到同一个 `now`（本轮开始的时刻）；
   - `TaskSpec{名称, 栈（字）, 优先级, 触发方式, step 列表}`，每个任务最多 `MAX_STEPS`（8）个 step，第一个 `nullptr` 之后的忽略。任务表是组合根里的 `constexpr std::array<TaskSpec, N>`；
   - 触发方式：
     - `common::every_ms(周期)`：`xTaskDelayUntil`；错过周期计入 `overrun_count`；
     - `common::on_topic(topic)`：运行器的 `Signal` 经 `add_waiter()` 登记到 topic，`wait(WAIT_FOREVER)` 醒来就跑一轮。多次发布可能合并成一轮，step 用 `update()` 读最新值；
   - `TaskRunner<TASKS>`：任务表作模板实参（`template <const auto &Tasks>`）：
     - 任务数与栈总字数在编译期求出；
     - 任务表在编译期检查（`static_assert`）：有名字和 step、栈不小于 `configMINIMAL_STACK_SIZE`、优先级在范围内、周期换成节拍后不为 0、按 topic 触发时给了 topic；
     - 栈、任务控制块、`Signal` 都是成员，用 `xTaskCreateStatic`，不用堆；
     - 全部成员初值为零，作全局对象时放在 `.bss`，不占 Flash 的初值空间。最初的写法是构造时传入任务表的指针，结果整个对象（含 2 KB 栈）进了 `.data`，所以改为模板实参；
   - 三个入口，分别对应 `boot_*`：
     - `init()`（`boot_init`）：登记 topic 等待者。等待槽满是唯一的运行期失败；
     - `start()`（`boot_init_rtos`）：创建任务；
     - `sample_stack_health(now)`（`boot_run`，1 Hz）：读各任务的栈高水位；
   - 观测：每个任务的运行轮数、单轮最长耗时、错过周期次数、剩余栈（字），调试器可读。
3. **bringup 能力**（`app/bringup`，target `app_bringup`，新增一级目录 `bringup`，命名空间 `bringup`）：
   - `Heartbeat`：每轮发布一次 `topics::bringup::heartbeat`（序号 + 发布时刻）；
   - `Monitor`：每轮用 `update()` 读心跳，统计收到次数、丢失次数（`lost_count()`）、乱序、延迟（本轮时刻 − 发布时刻）的最大值和超过 `Config::latency_limit_us` 的次数；
   - 不依赖 RTOS。宿主测试暂时在测试里直接编译 `app/bringup` 的源码（与 X-10 相同的做法），旧 app 归档后 `app/` 再加入宿主构建。
4. **第一个 topic**（`app/topics`）：
   - `bringup.hpp`（`Heartbeat` 消息与 `heartbeat` 实例）、`all.hpp`（`topics::ALL`，全部 topic 的 `const TopicBase *`）；
   - INTERFACE target `app_topics`，只依赖 `middleware_topic` 与 `hal_os`。`arch_check.py` 新增 `topics` 层，只能依赖 middleware 与契约，能力、运行器与组合根都可以依赖它；
   - **命名**：`tool/ci/naming.py` 不对 `app/topics/` 下的全局变量要求 `g_` 前缀，topic 实例按清单命名（架构 §9.2）。清单头里的全局变量只能是 topic 实例，`ALL` 是编译期常量，按常量规则大写。
5. **profile `bringup_dm_mc02`**：
   - `profile.cmake`：板 `dm_mc02`，能力 `bringup`；
   - `params.hpp`：`Monitor` 的配置，只含数据；
   - 组合根 `config/assembly/bringup_dm_mc02.cpp`：构造 `Heartbeat`、`Monitor` 与运行器，定义任务表，实现 `boot_*`；
   - 任务表：
     - `heartbeat`：1 ms 周期，优先级 25；
     - `monitor`：按 `bringup.heartbeat` 触发，优先级 26。

     读者优先级高于写者，正好是 Decision-20261029-02 修好的情形。CubeMX 的默认任务是 24，只调 `boot_run`。
   - `config/CMakeLists.txt` 按 profile 选组合根：`baseline` 仍用 `entry_legacy.cpp`，其他 profile 用 `assembly/<profile>.cpp`，链接 `CAPABILITIES` 列出的 `app_<能力>`。
6. **preset**：`arm-debug`、`arm-release` 改为 `bringup_dm_mc02`。`baseline` 在第 4 步归档前仍可用 `-DPROFILE=baseline` 构建。CI 的固件路径改为 `firmware_*.elf`，不写死 profile。
7. **待定**：F2 验收的"中断里发布的 topic 能被阻塞等待的任务收到"。现在没有不改生成区就能挂接的空闲中断源（TIM1 的周期回调在生成的 `main.c` 里，SysTick 钩子要改 CubeMX 配置，EXTI 只给 BMI088 的两个引脚）。方案另行提出，由负责人决定。

**验证**：
- 宿主：ctest 23/23（新增 `bringup`；`topic` 补"直接登记的 Signal"）；
- 编译期检查：任务表写错的 5 种情况（没有 step、栈太小、优先级等于空闲任务、优先级越界、周期为 0）都编译失败，报错指明原因；
- 固件 Debug、Release 无告警；`g_runner` 在 `.bss`；依赖图、HAL 回调、文本扫描、格式、命名检查通过。关掉 naming.py 的豁免时，`heartbeat` 会被报出来；
- `baseline` 仍可用 `-DPROFILE=baseline` 构建；
- **上板**（2026-10-31，记录见 `docs/ai/decisions/2026-10-31-F2-bringup上板验证/`）：
  - 心跳 1 kHz，没有错过周期；
  - monitor 的运行轮数与 heartbeat 相同，收到全部心跳，丢失、乱序都是 0，最大延迟 9 µs；
  - 两个任务的栈各剩一半以上；
  - PC 采样都在空闲任务里。

**Alternatives / Rejected**：
- **step 用"函数指针 + 上下文指针"**：任务表里要多一列 `void *`，组合根还要做类型转换。否决。
- **任务栈用 FreeRTOS 的堆**（`xTaskCreate`）：违背"无堆分配"。否决。
- **运行器内部建一个类型擦除的订阅者来等 topic**：要为每种消息类型实例化，或者另写一套等待接口。公开 `add_waiter()` 一处就够了。否决。
- **bringup 的行为写在组合根里**：组合根只做构造、注入、任务表与入口（§3.6），topic 也只能在 `app/topics` 定义。否决。

---

## Decision-20261031-13（归档旧能力代码：tag `archive/pre-rebuild`，X-04、X-10 关闭，Accepted）

- **Date**：2026-10-31
- **Status**：Accepted（F2 第 4 步的最后一部分；负责人同意"按 AI 的方案做 4c"）
- **Source**：plan.md F2 第 4 步"归档旧能力代码"；Decision-20261027-01（能力在阶段 A 重建，从 tag 取参考）

**Context**：

- bringup profile 已经就位，preset 默认构建它（Decision-20261031-12），并已上板验证。旧 app（迁移源的 imu、chassis、gimbal、remote、tasks、TaskManager）只剩 profile `baseline` 在用。
- 旧 app 带着三项例外：X-04（任务外壳用 CMSIS-RTOS2）、X-08 的 6 处原子读改写、X-10（集成测试直接编译 app 源码）。它们还让 `app_common` 依赖 `app_tasks`，让 `app/` 无法加入宿主构建。

**Decision**：

1. **打 tag**：在分支 `f2/bringup` 的末端（`9c11546`，旧 app 与 `baseline` 仍可构建）打附注 tag `archive/pre-rebuild`。阶段 A 重建能力时从 tag 取参考。
   - PR 都用 merge 提交合入，所以这个提交会留在 `dev` 的历史里；
   - tag 随这批分支一起推送。
2. **删除**：
   - `app/{imu,chassis,gimbal,remote,tasks}`；
   - `app/common/task_manager.*`；
   - `config/assembly/entry_legacy.cpp`；
   - `config/profiles/baseline/`；
   - `config/CMakeLists.txt` 里 `baseline` 的分支；
   - 宿主测试：底盘集成测试（`chassis_controller`、`controller_equivalence`、`mock_*` 共 6 项）及其专用的支持文件（`mock_motor.*`、`task_runtime_stub.cpp`、`shadow/`），以及 `bmi088_acquisition` 中测旧 app 的两段。

   `bmi088_acquisition` 删掉的两段：
   - `ImuDataReadyMailbox`（`app/tasks`）：被 `utils` 的 `LatestValue` 与 topic 取代；
   - `ImuDiagnostics` 的采集（`app/imu`）：随 IMU 能力在阶段 A 重建。

   BMI088 驱动本身的测试全部保留。
3. **`app/` 加入宿主构建**：
   - `app_topics`、`app_bringup` 在宿主上编译；
   - `app_common`（任务运行器，要 RTOS）只在固件构建里定义，改为 INTERFACE（只有头文件）；
   - `bringup` 测试改为链接 `app_bringup`，不再直接编译源码。
4. **迁移期构建开关（X-14）**：
   - `IMU_ENABLE_DIAGNOSTICS` 已经没有使用者，删除；
   - `BMI088_GYRO_BANDWIDTH_REGISTER` 留给 BMI088 驱动，驱动改写时改为 `Config` 参数；
   - `app_library()` 不再链接 `legacy_build_options`。
5. **`arch_check.py`**：
   - 删除 `app_tasks` 层与"`app_common` → `app_tasks`"的放行边；
   - 能力（`app`）不得依赖 `app_common`：任务运行器带 RTOS 头，规则 10；
   - app 白名单去掉 `tasks`、`debug_comm`；
   - 原子读改写的放行只剩 `dji_motor.cpp`（X-08）；
   - 条件编译的放行只剩 `BMI088_GYRO_BANDWIDTH_REGISTER`；
   - 文本扫描加入规则 10：`src/`、`app/` 中只有 `app/common` 可以 include FreeRTOS 与 CMSIS-RTOS 的头。
6. **例外**：X-04、X-10 关闭；X-08 只剩 `dji_motor.cpp` 的 2 处；X-14 只剩陀螺带宽。
7. **验证方式**：bringup 固件不链接旧 app，归档前后它的函数与对象符号应完全相同，用 `symbols.py compare` 对比。

**Alternatives / Rejected**：
- **保留旧 app，只移出构建**：留在树里的旧代码会被误用，也让 CI 的检查规则一直带着放行。否决，删除并以 tag 保存。
- **保留 `bmi088_acquisition` 里的邮箱与诊断两段，把头文件挪进 test/**：等于在测试目录里维护一份旧 app 代码。否决。

---

## Decision-20261101-01（hal 契约 v1 用句柄类写法；F2 第 5 步按外设纵向推进，Accepted）

- **Date**：2026-11-01
- **Status**：Accepted（负责人 2026-11-01："全部按照你认为的最优的方式就行，我想象中的架构就是文档那样"，由 AI 按建议推进，负责人事后学习、审阅）。2026-11-01 已上板验证（`docs/ai/decisions/2026-11-01-F2-外设上板验证/README.md`）
- **Source**：待登记中的"F2 第 5 步（外设）怎么做"与"hal 契约的写法与谁来写"；Decision-20261031-07 第 5 步；Decision-20261031-12 第 7 点（中断里发布的验收）
- **修订**：架构 §3.1 的示意写法（命名空间 + 自由函数）；Decision-20261027-01 第 5 点"自由函数 + 不透明句柄"中的"自由函数"改为"句柄类的成员函数"。不透明句柄、链接期绑定、没有虚函数不变。**这是对负责人骨架答复的一处偏离**，理由见 Reason，负责人可以改回甲。

**Context**：

- 第 5 步要定稿 `src/hal` 契约 v1，用 C++ 重写 port，删除 `src/hal/detail`（X-07）。三种写法（甲：命名空间 + 自由函数；乙：静态类；丙：句柄类）的对比见待登记的原文，运行开销、mock、链接方式都相同，差别在调用形式与句柄能否被随意读写。
- 负责人的草稿 `src/hal/temp/can.hpp` 把类型都收进 `class Can`，方向与乙、丙一致。
- 旧 C port 自己定义了 9 个 HAL 回调，同一外设的新旧 port 不能并存（-07）。
- F2 验收要求"中断发布的 topic 能被阻塞等待的任务收到"，不改生成区就能挂接的中断只有 BMI088 的两个数据就绪 EXTI（-12 第 7 点）。

**Decision**：

1. **写法：丙，句柄本身是类**：
   - 每类外设一个句柄类：`hal::Gpio`、`hal::Pwm`、`hal::Can`、`hal::Spi`、`hal::Uart`、`hal::UsbCdc`，头文件 `src/hal/<外设>.hpp`；
   - 句柄只有一个私有编号 `index_`（`std::uint8_t`），`constexpr explicit` 构造。编号由 `board.hpp` 给出，驱动与能力读不到也改不了；句柄可拷贝，驱动按值存进自己的 `Config`；
   - 成员函数由 port 实现，宿主测试由 `platform/mock` 实现，一次构建只链接其中一份；
   - 全板只有一份、不需要句柄的用自由函数：`hal::init()`、`hal::os::now_us()`。规律与 hal/os 相同：多实例的用类（`Signal`），单例的用自由函数；
   - 结果、帧、统计等类型收在类里（`hal::Can::Frame`、`hal::Can::SendResult`），枚举只定义一份；
   - 中断回调统一为 `hal::Callback`（`void (*)(void *context) noexcept`，`src/hal/callback.hpp`），在框架中断里执行，只做取数据、发布 topic、`notify()`；
   - 时间参数用 `hal::os::Microseconds`；`src/hal/time.hpp` 随驱动改用 `hal::os::now_us()` 删除（Decision-20261031-01 第 4 点）；
   - 每个成员函数注明：在任务还是中断里调用、是否阻塞、必选还是可选。
2. **可选能力**（架构 §3.1"能力分级"）：
   - 可选的成员函数在不支持时返回 `UNSUPPORTED`（或 `false`），板级用 `board.hpp` 的 `constexpr` 常量声明，如 `board::SPI_ASYNC`；
   - 要求可选能力的驱动给出编译期常量（如 `device::Bmi088::REQUIRES_SPI_ASYNC`），组合根 `static_assert`；能回退的驱动在运行期回退（WS2812 遇到 `UNSUPPORTED` 改用阻塞发送）。
3. **`board.hpp`**：`platform/<板>/board/board.hpp`，命名空间 `board`，只 include `hal/*.hpp`：
   - 物理资源句柄（`board::CAN_1`、`board::SPI_IMU_GYRO`、`board::PWM_BUZZER` 等）、每类外设的个数与能力常量；
   - 只有组合根与本板的 port 能看到（`board_<板>` 的 include 路径只开到 `board/`），规则 1 的 `board.hpp` 部分由依赖图与文本扫描检查；
   - port 用 `static_assert` 核对自己的外设表与 `board.hpp` 的个数一致。
4. **第 5 步按外设纵向推进，F2 与 F3 合排**。每个外设依次做完：契约 v1 + mock → port 用 C++ 重写 → 改用它的驱动 → 删除旧包装与 `hal/detail` 的对应文件 → 宿主测试。顺序与本地分支：
   - `f2/gpio-pwm`：gpio、pwm；驱动 buzzer、BMI088 加热；
   - `f2/can`：can；驱动 DJI 电机（X-08 的原子读改写一并去掉）；
   - `f2/spi`：spi；驱动 BMI088、WS2812（X-14 的陀螺带宽改为 `Config` 参数）；
   - `f2/uart`、`f2/usb-cdc`：uart、usb_cdc（X-01 的 `UsbCdcPort_Handle*` 保留为 port 的 C 接口）；
   - `f2/port-cleanup`：删除 PortKit、`hal/time.hpp` 与 DWT 时基、`hal/detail`、`Init.h`（X-07 关闭）。
5. **port**：C++，每个外设一个 `.cpp`，覆盖 HAL 回调的文件 include 对应 HAL 头（规则 16）。PortKit 拆分：
   - 队列 → `concurrency::SpscQueue`；
   - 临界区 → `hal::os::CriticalSection`。已核对生成代码：全部外设中断（FDCAN、SPI、UART、DMA、BDMA、EXTI、OTG_HS）都在优先级 5，即框架线上，BASEPRI 能屏蔽它们；
   - 中断通知 → port 私有的 `Notifier`（回调 + 上下文，在临界区内设置）；
   - D-cache 维护 → port 私有的 `cache.hpp`。
6. **mock**：`platform/mock` 实现全部 hal 契约，测试控制接口在 `mock_hal.hpp`：按编号注入输入（CAN 帧、UART 字节、GPIO 电平与中断）、检查输出（发出的帧、PWM 设定、写出的字节），SPI 由测试登记设备应答函数，异步传输由测试决定何时完成。驱动的宿主测试改用 mock，删除测试里的 C ABI 桩。
7. **中断里发布的验收**：用 BMI088 陀螺仪的数据就绪中断（2 kHz）。bringup 能力在 GPIO 中断回调里发布 topic，按 topic 触发的任务统计条数、丢失与延迟。BMI088 要先由驱动配置好才会产生中断，所以随 `f2/spi` 之后的 bringup 检查一起做。
8. **谁来写**：AI 写，负责人事后审阅与学习（负责人 2026-11-01）。负责人的草稿 `src/hal/temp/can.hpp`（未入库）由 `src/hal/can.hpp` 取代，草稿里的 `QUEUED_FULL` 按旧代码改回 `QUEUE_FULL`，`MAX_DATA_LENGTH` 收进类里，避免以后 uart 等外设的同名常量在 `hal` 命名空间冲突。

**Reason**（为什么不按 Decision-20261027-01 第 5 点的字面写成自由函数）：
- 句柄的不透明由编译器保证：编号是私有的，甲、乙的句柄编号是公开字段；
- 与已经定稿的 hal/os 是同一套规律（`Signal` 是类、`now_us()` 是自由函数），以后加外设不用再选写法；
- 调用最短（`config_.bus.send(frame)`），IDE 能直接列出一个外设能做什么；
- 与负责人草稿的方向一致；
- 代价：类不能再打开，可选能力只能写在类里（不支持的板返回 `UNSUPPORTED`）；成员函数经 `this` 取编号，多一次内存读取，可以忽略。

**Alternatives / Rejected**：
- **甲（命名空间 + 自由函数，架构原文）**：完全可行，可选能力能拆到单独的头文件；但句柄编号是公开字段。负责人更看重这两点时可以改回，改动是机械的（成员函数改为以句柄为第一个参数的自由函数）。
- **乙（静态类）**：没有甲的灵活，也没有丙的封装。否决。
- **新旧接口并存到 F3（原方案 A）**：要多写一层转接代码，原因见 -07。否决。
- **中断发布用软件触发的 EXTI 或改 CubeMX 配置腾出一个定时器**：前者要为测试加一条只在 bringup 用的路径，后者要改生成区。BMI088 的数据就绪中断是阶段 A 的 IMU 能力本来就要用的路径。否决。

---

## Decision-20261101-02（F3：lib/estimate 去掉对 BMI088 驱动的依赖，X-17 关闭；规则 14 生效，Accepted）

- **Date**：2026-11-01
- **Status**：Accepted（负责人 2026-11-01 授权 AI 按最优方式推进，同 Decision-20261101-01）
- **Source**：plan.md F3"`src/lib/estimate` 去掉对 BMI088 驱动的依赖（POD 入参）；框架开启 `-Wdouble-promotion`"；架构 §7"框架阶段必须处理的已知问题"；例外 X-17、X-06

**Context**：

- `lib/estimate/imu_body_mapping` include BMI088 驱动的头，把驱动的样本记录换算成机体系的角速度、加速度，是 lib → driver 的反向依赖（依赖图检查单独放行）；它也是 Cortex-M0+ 框架构建跳过 `lib_estimate` 的原因（X-17）。这些函数目前没有使用者。
- 规则 14（框架开启 `-Wdouble-promotion`）还没有生效。

**Decision**：

1. `imu_body_mapping` 只保留与传感器无关的部分：`AxisMapping`、`is_valid_axis_mapping`、`map_sensor_vector_to_body`（三维 float 向量），新增 `IDENTITY_AXIS_MAPPING`。删除：
   - `gyro_record_to_body`、`accel_record_to_body`（以驱动的样本记录为参数）；
   - `BMI088_SENSOR_TO_BODY`：安装方向是整车参数，阶段 A 由 IMU 能力的 `Config` 接收，profile 给值；
   - 关于 BMI088 量程的两条 `static_assert`：EKF 标度假设由阶段 A 的 IMU 能力检查。
   陀螺的 rad/s 换算移到驱动：`Bmi088Gyro::to_radians_per_second()`（与原实现逐位一致：先 `to_dps` 再乘 `DEG_TO_RAD`）。
2. `lib_estimate` 不再链接 `driver_bmi088`，依赖图检查删除这条放行边；Cortex-M0+ 框架构建编译全部框架 target，**X-17 关闭**。
3. **规则 14 生效**：`framework_library()`、`utils_library()` 加 `-Wdouble-promotion`（随严格告警即为错误）。实测只有 `vqf.cpp` 有隐式提升（18 处，都是有意的双精度计算），按 X-06 单独对这个文件关掉。
4. X-06 要求的"目标没有双精度 FPU 就报错"的检查推迟到阶段 B（第一块没有双精度 FPU 的板，F407）：现在的写法要么在 `src/` 里写 `#if`（规则 9），要么让 Cortex-M0+ 的框架构建失败，而现有的唯一固件目标（Cortex-M7）有双精度 FPU。阶段 B 在板级 CMake 声明 FPU 能力，由组合根检查。

**验证**：宿主测试新增 `axis_mapping`（合法与不合法的映射、原地映射）；Cortex-M0+ 构建 14 个框架静态库，没有原子读改写的库调用；三套构建里 `-Wdouble-promotion` 只报 `vqf.cpp`。

**Alternatives / Rejected**：
- **把 BMI088 专用的换算留在 lib，改成以 POD 为参数**：换算本身依赖驱动的量程常量，放进 lib 仍要复制驱动的知识。换算属于驱动，组合属于 IMU 能力。否决。
- **规则 14 的豁免按 target（整个 `lib_estimate`）**：会把 EKF、滤波器里的意外提升一起放过。否决，按文件豁免。

---

## Decision-20261101-03（bringup 固件加入外设检查；中断里发布 topic 的验收；任务运行器的 topic 超时，Accepted）

- **Date**：2026-11-01
- **Status**：Accepted（负责人 2026-11-01 授权 AI 按最优方式推进，同 Decision-20261101-01）。2026-11-01 已按检查清单上板验证（`docs/ai/decisions/2026-11-01-F2-外设上板验证/README.md`）：中断里发布的 topic 叫醒阻塞等待的任务，F2 验收项满足
- **Source**：plan.md F2 验收"中断发布的 topic 能被阻塞等待的任务收到"、F3"bringup 固件加入各驱动的检查"、F4"用框架写一个最小示范：IMU 驱动 → 姿态 topic → USB 输出"；Decision-20261101-01 第 7 点；架构 §3.5、§3.6

**Decision**：

1. **bringup 能力加入外设检查**（`app/bringup`，仍是框架验证用，不含 RM 业务）：
   - `ImuCheck`：板载 BMI088。两个数据就绪中断（GPIO 外部中断）与 SPI 事务完成中断在**框架中断里发布** `bringup.imu_event`（累计次数与时刻）；IMU 任务按它触发，把事件交给驱动、收割样本、换算到机体坐标系、喂 VQF，限频发布 `bringup.imu`（角速度、加速度、四元数、温度）；加热器按温度恒温（PWM）。报告事件数、样本数、中断发布到任务读到的延迟（最大值与超限次数）。这就是 F2 验收的"中断里发布"，也是 F4 的最小示范与阶段 A IMU 能力的起点；
   - `CanCheck`：每条总线一个，**只收不发**（接在整车上也不会让电机动），统计帧数、见过的 DJI 反馈标识符、port 的计数；
   - `SbusCheck`：UART → SBUS 解析 → 发布 `bringup.sbus`；
   - `UsbReport`：USB 虚拟串口回显收到的字节，10 Hz 输出一行状态（心跳、IMU、SBUS），是 F4 示范的输出端；
   - `Indicator`：上电蜂鸣器响两声（PWM），板载 RGB 灯按状态闪烁（SPI 发送）：IMU 在更新为绿、只有心跳为黄、都停为红。
2. **能力的 `Config` 分两部分**：组合根注入的依赖（驱动对象的指针、引脚等句柄）与 profile 给的 `Parameters`（只含数据，写在 `params.hpp`）。`params.hpp` 不能 include `board.hpp` 与驱动对象，依赖由组合根组装（架构 §3.6）。
3. **任务运行器**：`common::on_topic(topic, timeout_ms)` 新增可选的超时：超过这么久没有发布也运行一轮（`TaskStatus::timeout_count` 计数）。IMU 任务用 1 ms：数据就绪中断开始之前，驱动的初始化要按时间推进。
4. **任务表**（组合根）：`imu`（按 `bringup.imu_event`，1 ms 超时，优先级 28）、`monitor`（26）、`heartbeat`（1 ms，25）、`io`（5 ms，20：CAN、SBUS、USB 报告、指示）。
5. **规则 4 的检查生效**：`config/profiles/*/params.hpp` 只能 include 能力的头（`app/` 下除 `topics`、`common` 外的一级目录）与标准库头（`arch_check.py scan`）。
6. 能力也加 `-Wdouble-promotion`（`app_library()`），与框架一致。

**影响**：
- Flash 与 RAM：Release 的 `.text` 约 87 KB；`.data` 由约 0.4 KB 增至约 5 KB——几个全局对象（BMI088 驱动、`ImuCheck` 里的 VQF、加热器等）有非零初值，整个对象带 Flash 初值。1 MB Flash 上可以接受；以后在意时可改为 `init()` 里赋值。
- USB 在内核启动后才由生成区的默认任务初始化（`MX_USB_DEVICE_Init`），`UsbReport::init()` 不要求 USB 就绪。

**验证**：宿主测试新增 `bringup_imu`（SPI 替身扮演 BMI088、GPIO 替身触发数据就绪：初始化、中断里发布、样本换算与 VQF、限频、加热器、延迟超限）与 `bringup_io`（CAN 只收、SBUS 跨段拼帧与失控保护、USB 状态行格式与回显、蜂鸣器两声与灯的三种状态、定点格式化）；固件 Debug、Release 构建、依赖图、回调、文本扫描、格式、命名通过。上板验证见 2026-11-01 的记录：首轮发现 `ImuCheck` 的延迟在任务醒来后、读 topic 前又有新发布时得负值并计入 late，改为用 `bringup.imu_event` 里的发布时刻（`published_us`），宿主测试补上这个情形；探测脚本的速率与 monitor 判定一并修正。修正后探测 20/20。

**Alternatives / Rejected**：
- **IMU 任务按 1 kHz 周期运行**：2 kHz 的陀螺数据就绪会被合并一半，也验证不了"中断里发布"。否决。
- **SPI 完成中断直接叫醒任务的 `Signal`，不经 topic**：任务只能等一个触发源；三个中断都发布同一个 topic 最简单，发布在临界区内完成，多个中断共用一个发布者是安全的。否决。
- **CAN 检查发送探测帧**：没有其他节点时帧得不到应答，接在整车上又可能干扰电调。否决，只收。

---

## Decision-20261101-04（板级目录改为"厂商 / 板"两级：`platform/st/dm_mc02`，Accepted）

- **Date**：2026-11-01
- **Status**：Accepted（负责人 2026-11-01："dm_mc02 移动到 st 里面就行"）
- **Source**：负责人要求；修订 Decision-20261031-09（按板一级）与架构 §2.1、§3.7、§5.4

**Context**：

- Decision-20261031-09 把 `platform/st/stm32h7/board_dm`（厂商 / 芯片 / 板三级）改成了 `platform/dm_mc02`（按板一级）。
- 负责人要求板级目录放在厂商目录下。以后的板（F407 C 板、TI 的板）按厂商归类，同一厂商的共用代码也有地方放。

**Decision**：

1. `git mv platform/dm_mc02 platform/st/dm_mc02`，生成区（`cubemx/`）随之整体移动，内容不变（`.ioc`、`.mxproject` 只有相对路径；板目录里的 CMake 都以 `CMAKE_CURRENT_SOURCE_DIR` 为基准）。负责人以后用 CubeMX 打开 `platform/st/dm_mc02/cubemx/mc02_cpp.ioc`。
2. 布局为"厂商 / 板"两级，不再有芯片一级：`platform/<厂商>/<板标识>/{cubemx,port,board}`。板标识（`BOARD_ID`，target 名的后缀）仍是 `dm_mc02`；profile 的 `BOARD` 是 `platform/` 下的路径 `st/dm_mc02`。同一厂商的板之间要共用代码时，再登记放在 `platform/<厂商>/` 下的哪里。
3. 路径改动：
   - profile `bringup_dm_mc02` 的 `BOARD` 改为 `st/dm_mc02`；
   - CI 回调检查的 port 路径；
   - `arch_check.py`：`platform/` 一级目录白名单改为厂商（`st`）与 `mock`，另加二级白名单（`st` 下只有 `dm_mc02`）；`board.hpp`、`extern "C"`、生成区 include 的路径匹配改为允许厂商一级；
   - 板内头文件的保护宏随路径改名（`PLATFORM_ST_DM_MC02_…`；`UsbCdc.h` 的保护宏还是 P2 前的旧路径，一并改正）；
   - OpenOCD 配置里的注释。
4. 验证方式：纯移动加路径改写，固件的函数与对象符号集合应与移动前完全相同（`symbols.py compare`），Release 的 `.bin` 应逐字节相同。

**Alternatives / Rejected**：
- **恢复"厂商 / 芯片 / 板"三级**（`platform/st/stm32h7/dm_mc02`）：负责人只要求放进厂商目录；芯片一级在只有一块 H7 板时是空层级。否决。

---

## Decision-20261101-05（BMI088 加热改为模型预测：两级预热 + 平台校正加热器强度 + 前馈 PI 恒温；丢失功能 L6 提前到 F3，Accepted）

- **Date**：2026-11-01
- **Status**：Accepted（负责人 2026-11-01："提升陀螺仪的初始化速度……之前实测启动慢的原因就是因为陀螺仪温度稳定太慢了"；做法按 Decision-20261101-01 的授权由 AI 决定）。2026-11-01 已上板验证（`docs/ai/decisions/2026-11-01-L6-加热提速上板验证/README.md`）
- **Source**：丢失功能 L6（`docs/ai/decisions/2026-10-26-丢失功能与恢复依据.md` §2）。原计划在阶段 A 的 `app/imu` 能力里做，负责人要求提前；加热控制在驱动层，不需要等能力重建

**Context**：

- 陀螺零偏校准要等温度稳定（旧 app：进入 50 ±0.5 ℃、带内极差 ≤ 0.25 ℃ 并保持 20 s 才开窗），陀螺初始化的时间主要花在加热到稳。
- 现有 `device::Bmi088Heater`（沿用 `a2cfd16`）：45 ℃ 以下固定 200‰ 预热，之后 PI（Kp 10、Ki 1）。2026-11-01 实测（接 24 V，从 36.9 ℃ 复位启动）：3.6 s 就到 50 ℃，但冲到 **56.9 ℃（+6.9 ℃）**，再回落到 48.0 ℃，28 s 才进入并保持 ±0.25 ℃，49 s 才满足旧 app 的校准条件。
- 原因：加热器很强、传感器很慢。200‰ 时读数一个更新周期就涨 9.5 ℃，而 BMI088 的温度寄存器 1.28 s 才更新一次、分辨率 0.125 ℃；只看读数的 PI 看到温度时早已冲过。
- 原固件（源码已丢）的冷启动：24.1 s 到 50 ℃、过冲 +0.375 ℃、111 s 进入并保持 ±0.25 ℃；它在内部估计热源侧温度（升到约 63 ℃ 后回落），据此提前收功率。
- 实测辨识（`foray_mcu_archive/2026-11-01_heater_warmup/`）：从占空比到读数，等价于两级一阶滞后串联——被加热区（时间常数 15.5 s，稳态增益 0.967 ℃/‰）→ 传感器（滞后 0.95 s）→ 每 1.28 s 采样、0.125 ℃ 截断。用这个模型仿真现有算法：过冲 6.0 ℃、±0.25 ℃ 稳定 27.3 s、校准条件 48.7 s，与实测（6.9 ℃、28 s、49.4 s）吻合。
- 加热功率与供电电压的平方成正比：6S 电池 25.2 V → 21 V，功率差 44%。同一组参数必须能承受 ±25% 的加热器强度误差，只按模型一次性开环预热不行（仿真：强度 +25% 时过冲 8 ℃）。

**Decision**：

1. **算法放 `src/lib/control/heater_control.{hpp,cpp}`**（`control::HeaterControl`）：纯算法，输入目标、读数、读数距今的时间、周期，输出占空比，不依赖 hal，宿主测试可以接热模型仿真。`device::Bmi088Heater` 改用它，不再用 PID；BMI088 特有的读数周期（1.28 s）、分辨率（0.125 ℃）由驱动给出。
2. **模型**：`dx1/dt = (K·u + Ta − x1)/τ1`（被加热区），`dx2/dt = (x1 − x2)/τ2`（传感器），读数是 x2 在寄存器更新时刻的截断采样。第一个读数到来时认为各处温度相同（x1 = x2 = Ta = 读数）。
3. **观测器**：由读数跳变锁定寄存器的更新相位，之后每 1.28 s 一次，在更新时刻用"读数 − 模型在那一刻的 x2"修正被加热区与传感器的估计。
4. **两级预热**：
   - 探测级：用预热功率（300‰）加热；每个控制周期向前推演"之后保持在本级目标所需功率时，传感器的最高温度"，到本级目标就收功率，最后一个周期按插值只补到恰好。本级目标 = 当前 + 剩余温差 × 0.7，加热器偏强 25% 也不越过目标。
   - 收功率后等 4 次读数更新，温度停在平台上；**实际温升 / 预测温升**就是加热器强度的修正系数（电池电压、个体差异），之后的预测都乘上它。
   - 逼近级：用修正后的模型瞄准目标下方 0.8 ℃，等 3 次读数更新；最多再补一级。剩余温差不到 0.5 ℃ 时直接恒温。
5. **恒温**：维持功率（模型前馈）+ 比例（16 ‰/℃，对模型估计的传感器温度）+ 积分（1.0 ‰/(℃·s)，对读数，限幅 30‰），上限 100‰，输出四舍五入到 1‰。读数低于目标 2 ℃ 以上（例如加热电源断开又恢复）时按当前读数重新预热，积分不在没电时累积。
6. **接口**：`Bmi088Heater` 在构造时接收 `Bmi088Heater::Config`（本板的热模型 + 可选的控制参数），由组合根从 profile 的 `params.hpp` 注入；DM-MC02 的模型参数写在 `config/profiles/bringup_dm_mc02/params.hpp`。65 ℃ 硬保护、500 ms 样本超时、100 Hz PWM、128 ms 控制周期不变。
7. **整定依据**：Python 原型（不入库）在 144 种情况下搜索参数——加热器强度 ±25%、两个时间常数各 ±10%/±25%、4 种寄存器相位、3 种起点（25 ℃ 冷启动且板子自热升温、37 ℃ 板子降温、37 ℃ 板子升温）。所选参数：标称从 37 ℃ 起过冲 0.125 ℃、14.6 s 进入并保持 ±0.25 ℃；从 25 ℃ 起过冲 0.25 ℃、14.6 s；最坏过冲 0.5 ℃、34.7 s。现有算法在同样情况下过冲 6–9 ℃。
   **上板实测**（`docs/ai/decisions/2026-11-01-L6-加热提速上板验证/README.md`，从 39.5 ℃ 复位）：最高 50.125 ℃（+0.125 ℃）、15.3 s 进入并保持 ±0.25 ℃、35.4 s 满足旧 app 的校准条件；旧 PI 实测为 +6.875 ℃、28.0 s、49.4 s，原固件为 +0.375 ℃、111 s。加热电源断开时控制器判定没电、不修正强度，来电后不冲过头。
8. **范围**：只改加热控制。陀螺校准的 20 s 稳定门控属于阶段 A 的 `app/imu`，温度稳定变快之后再评估缩短。

**Alternatives / Rejected**：
- **保留 PI 只调参数**：读数 1.28 s 一次、升温最快约 7.5 ℃/s，只看读数的反馈避免不了过冲或慢爬。否决。
- **一次性按模型开环预热到目标**：标称很好（过冲 0.25 ℃），加热器强度 +25% 时过冲 8 ℃。否决。
- **二自由度（平滑参考曲线 + 模型前馈 + 反馈）**：仿真试过一轮，最坏情况下稳不进 ±0.25 ℃。否决。
- **测供电电压补偿加热功率**：要先确认板上有没有电压采样并加 ADC 契约；平台校正已经覆盖这部分误差。以后有电压采样时可以作为补充。
- **照搬原固件的做法**：源码已丢，按约定只作行为参考，不做逆向。

**回滚**：revert 本决策的提交，`Bmi088Heater` 回到 PID 实现；`params.hpp` 去掉 `HEATER`，组合根改回只传 PWM 句柄。

---

## Decision-20261101-06（bringup 上电打开板上两路 24 V 输出，Accepted）

- **Date**：2026-11-01
- **Status**：Accepted（负责人 2026-11-01："帮我常态打开板载 24v"）
- **Source**：bringup 接电机做检查（GM6020、DM-J4310）时，电机由板上两路可控 24 V 输出供电；此前每次复位后都要经 SWD 写 `GPIOC_BSRR` 打开

**Context**：DM-MC02 的两路 24 V 输出由 PC14、PC13 控制（`board::POWER_24V_0/1`，高电平打开，2026-11-01 实测），CubeMX 初始化为低（关）。bringup 不碰它们，接在上面的电机没电。

**Decision**：
1. 新增 bringup 能力 `bringup::PowerOutputs`：`init()` 按 profile 参数把组合根注入的输出一次写好（打开或关闭），没有 `step()`。打开的电平由组合根给出（DM-MC02 为高），能力不假定极性。
2. profile `bringup_dm_mc02` 的 `POWER_OUTPUTS` 为打开；组合根在 `hal::init()` 之后第一个初始化它，电机尽早上电（达妙电机上电自检约 1 s）。5 V 输出不动。
3. 安全：达妙电机上电默认失能，大疆电调没有控制帧不出力，bringup 只收不发，打开电源不会让电机动。

**Alternatives / Rejected**：
- **在 port 的 `hal::init()` 里打开**：电源开不开是产品决定，不归平台层。否决。
- **在组合根里直接写 GPIO**：组合根只装配、不含行为（架构 §3.6）。否决。

---

## Decision-20261102-01（达妙电机驱动：启动回读核对后才允许使能；电机组契约加入使能 / 失能，DJI 也要使能才出力，Accepted）

- **Date**：2026-11-02
- **Status**：Accepted（负责人 2026-11-02："你开始写驱动吧"；做法按 Decision-20261101-01 的授权由 AI 决定）
- **Source**：plan.md F3"达妙电机"的驱动一项；协议层 `protocol::DmEsc`（`f3/dm-protocol`）；寄存器回读排查（`docs/ai/decisions/2026-11-01-达妙电机寄存器回读/README.md`）；《DM-J4310-2EC V1.2 减速电机使用说明书》V1.4

**Context**：

- 达妙电机的控制帧按量程（PMAX、VMAX、TMAX）线性映射，量程是电机里的寄存器，可以被调试助手改掉；驱动用的量程与电机里的不一致时，指令与反馈都会被等比例缩放，控制环不会察觉。控制模式同理：MIT 帧发给速度模式的电机，含义完全不同。
- 达妙电机上电默认失能，要发"使能"才出力；出错（过温、欠压、通讯丢失等）后自己退出使能，要"清除错误"再使能。使能、失能、清除错误的帧 ID 跟随电机当前的控制模式（说明书"使能"一节）。
- 电机只在收到发给自己的帧时回一帧反馈（问询式），反馈与寄存器应答都发到 MST_ID。负责人 2026-11-01 已把 ESC_ID 1 的电机的 MST_ID 改为 0x11（约定 MST_ID = ESC_ID + 0x10），回读确认应答从 0x011 回来。
- 反馈帧 D0 是 `ID | 状态 << 4`：ESC_ID 只有低 4 位进反馈，说明书建议 ESC_ID 小于 16。
- 通讯超时保护（寄存器 TIMEOUT，0x09，以 50 µs 计数）只在使能时生效，出厂值未知；写寄存器只改 RAM，不写 flash。
- 现有电机组契约（`device::MotorGroup`）只有 `init`、`process`、`read_state`、`set_torque`、`clear_commands`，没有使能的概念：DJI 电调收到控制帧就出力。

**Decision**：

1. **驱动 `device::DmMotor`**（`src/driver/motor/dm/`，target `driver_motor_dm`）：一条 CAN 总线上最多 4 台达妙电机，用 MIT 模式。只用 `protocol::DmEsc` 编解码；构造时接收 `Config`（总线句柄、型号档案、每台电机的 ESC_ID 与 MST_ID）。
2. **启动核对**（`init()` 不阻塞，`process()` 里逐台推进，请求 20 ms 没有应答就重发，一直重试，电机晚上电也能接上）：
   1. 读控制模式（0x0A）；
   2. 按读到的模式发"失能"，等到一帧反馈：单片机复位时电机可能还在使能状态，先让它停下；
   3. 读 PMAX、VMAX、TMAX（0x15–0x17）；
   4. 写通讯超时（0x09）= 档案值（J4310 为 50 ms，即 1000 个计数），核对回显；
   5. 模式是 MIT、三个量程与档案一致（相对误差 1e-5）、超时回显一致 → `READY`；否则 `MISMATCH`，不允许使能，重新 `init()` 才会再核对。读到的值都在快照里，便于排查。
3. **使能**：`enable(i)` 只在 `READY` 时有效。之后的 `process()` 先发一次"清除错误"，再每周期发"使能"，直到反馈的状态为"使能"才开始发 MIT 帧（Kp = Kd = 0，只给力矩，或 `set_command()` 给完整的 MIT 指令）。"清除错误"之后 2 ms 内收到的反馈不算确认，避免把之前在路上的反馈当成使能成功。50 ms 内没确认就放弃，记为使能失败。
4. **运行中**：
   - 没有请求出力时，每周期发"失能"作保活，用来取得反馈并确保电机不出力；
   - 使能后反馈出现故障状态 → 撤销请求、转为发"失能"；要再次 `enable()` 才会清错重试，驱动不自动重试；
   - 使能后反馈变成"失能"（电机复位）或 50 ms 没有反馈（离线）→ 撤销请求并重新核对（电机掉电重启后，RAM 里写的超时值会丢）。
5. **一帧属于哪台电机**：帧 ID 等于它的 MST_ID，且寄存器应答的 D0–D1 等于 ESC_ID，或反馈 D0 的低 4 位等于 ESC_ID。所以几台电机共用 MST_ID 也能分开；推荐仍按 ESC_ID + 0x10 设置。
6. **配置检查**（`init()` 拒绝）：
   - ESC_ID 在 1–15 之内且互不相同；
   - MST_ID 不是 0x7FF，低 8 位不等于本组任何 ESC_ID：电机按帧 ID 低 8 位认帧，否则别的电机会把反馈当成发给自己的帧；
   - 数量 1–4，量程有效，超时至少一个计数。
   带宽：每台电机每周期一帧指令加一帧反馈。1 Mbps、1 kHz 时每台约占 26% 总线负载，一条总线建议不超过 3 台。
7. **位置**：反馈位置在 ±PMAX 处回绕（说明书的 float_to_uint 按 16 位截断）。驱动按 65536 个计数累计多圈；`MotorState::angle_degrees` 仍按契约从核对后的第一帧反馈起算。电机掉电重启后，它的位置从 [−π, π] 重新开始，多圈计数可能差 2π 的整数倍；离线期间的运动不做推断。
8. **电机组契约**（`src/driver/motor/`）：
   - `MotorGroup` 加 `enable() -> bool`（全组请求出力：任何一台还不能使能就一台都不使能，返回 false）和 `disable()`（全组撤销请求并清零指令）。`init()` 之后默认不出力；
   - `set_torque()` 在没有请求出力时返回 false，不暂存；
   - `MotorState` 加 `enabled`（已经在出力）与 `fault`（核对不符、电机报故障或使能失败）；
   - `MotorConnection` 加 `master_id`，DJI 不用；
   - `MotorModel` 加 `DM_J4310`（档案：MIT，量程 12.5 rad / 30 rad/s / 10 N·m，峰值扭矩 12.5 N·m，通讯超时 50 ms；最大力矩取峰值与 TMAX 的较小者，即 10 N·m）。
9. **DJI 适配器同样要 `enable()` 才出力**：没有使能时 `set_torque()` 返回 false，但仍发全零保活帧（C610/C620 要收到控制帧才回反馈）；`disable()` 清零指令。`DjiMotor` 本身不变。这样"失能"对所有品牌含义相同，能力可以统一处理失控保护。

**偏离负责人原先说法之处**：plan.md 只写了"回读控制模式与量程，核对后才允许使能"。本决策另外做了三件事：核对前先失能电机；把通讯超时写进电机 RAM；DJI 适配器也改为要使能才出力。改 DJI 的原因：阶段 A 还没开始，现在统一契约代价最小。

**Alternatives / Rejected**：
- **只核对、不写通讯超时**：出厂值或调试助手里的值未知，为 0 时单片机死机后电机会一直保持最后的力矩。写进 RAM 不磨损 flash，掉电恢复原值，代价只是多一次往返。否决只核对。
- **启动时直接按档案的模式发"失能"**：电机若处在别的模式，帧 ID 不对，失能不生效。先读模式。否决。
- **使能失败或故障后驱动自动清错重试**：过流、过温这类故障会被反复清除、反复出力。重试与否是能力的策略。否决。
- **要求每台电机的 MST_ID 各不相同**：按内容区分已经足够，不额外限制已有接线。否决。
- **DJI 适配器保持"收到指令就出力"，`enable()` 为空操作**：两种品牌的"失能"含义不同，能力写失控保护时要区分品牌。否决。

**回滚**：revert 本决策的提交。`driver_motor_dm` 删除；电机组契约回到没有 `enable`/`disable` 的版本；DJI 适配器恢复为收到指令就出力。

---

## Decision-20261102-02（bringup 加入达妙电机检查：取代 CAN1 的只收检查，上电后让电机转一小段，Accepted）

- **Date**：2026-11-02
- **Status**：Accepted（负责人 2026-11-02："先验证 dm 的驱动的正确吧……电机可自由转动无刚性连接，放心测试就行"）
- **Source**：plan.md F3"达妙电机"的 bringup 检查一项；Decision-20261102-01

**Context**：

- `device::DmMotor` 只在宿主机上用模拟电机测过。上板要证明四件事：
  - 回读核对、使能状态机、MIT 指令在真电机上成立；
  - 写进电机 RAM 的通讯超时真的生效；
  - 驱动换算出的速度、位置方向与指令一致；
  - CAN 发送正常（bringup 至今只收不发，CAN 发送没有验证过）。
- 电机在 DM-MC02 丝印 CAN1 上（ESC_ID 1、MST_ID 0x11），由板上 24 V 输出供电。负责人确认电机能自由转动，没有刚性连接。

**Decision**：

1. 新增 bringup 能力 `bringup::DmCheck`（`app/bringup/dm_check.{hpp,cpp}`），持有组合根注入的 `device::DmMotor`（CAN1，一台）。每轮 `step()` 先调驱动的 `process()`，再按阶段推进，上电后只跑一遍：
   1. 核对：等驱动 READY（5 s 内；电机上电自检约 1 s）；
   2. 失能保活 0.5 s：没有使能时也在线；
   3. 使能：200 ms 内确认；
   4. 零力矩 1 s：MIT 全零指令，电机保持使能，记录位置漂移；
   5. 转动 2 s：Kd 1.0 N·m·s/rad、力矩 1.0 N·m，稳态转速不超过力矩 / Kd = 1 rad/s，2 s 最多转 2 rad。看反馈的速度、位置是否朝正方向变。最初用 0.3 N·m、Kd 0.3，上板时减速箱的静摩擦（约 0.1–0.3 N·m）让电机时转时不转，于是按比例加大，限速不变；
   6. 刹车 0.5 s：只留 Kd；
   7. 通讯超时：300 ms 不调 `process()`。电机应按写进去的 50 ms 超时自己失能，报"通讯丢失"（0xD）；驱动看到故障后撤销出力。电机没有复位，RAM 里的设置还在，驱动不必重新核对。停发期间留在接收队列里的旧反馈，会让驱动在恢复后第一次 `process()` 里还认为电机在线，所以不会因离线而重新核对；
   8. 再使能（先清错）后失能：确认反馈回到"失能"。
   之后一直发失能保活。必过项是第 1、3、4、5、7、8 步的状态机与时限；转动方向另记一项（`motion_ok`），通讯丢失状态也另记，不影响必过项。
2. 电机接线（ESC_ID、MST_ID）与各阶段参数写在 profile 的 `params.hpp`（`DmCheck::Parameters`），转动可以关掉（`spin`）。
3. 原来 CAN1 上的只收检查 `g_can_1` 删去：CAN1 的接收由驱动占住，收发都由本检查验证；CAN2、CAN3 的只收检查不变。
4. 新增任务 `dm`：1 ms 周期，优先级 23：在心跳（25）与 CubeMX 默认任务（24）之下、io（20）之上。
5. 上板探测脚本 `tool/debug/bringup_probe.py` 读 `g_dm_check.report_`，判定结果并列出各阶段测得的值。

**安全**：这一条改变了 Decision-20261101-06 第 3 点的前提。从此 bringup 固件上电后，会让 CAN1 上 ESC_ID 1、MST_ID 0x11 的达妙电机使能并转一小段（≤ 2 rad，≤ 1 rad/s）。总线上没有这台电机时，检查停在核对阶段，不发任何控制帧。bringup 固件装到整车上之前，要把 `params.hpp` 里的 `spin` 关掉，或者去掉本检查。

**Alternatives / Rejected**：
- **只做"使能 → 零力矩 → 失能"**：证明不了指令真的起作用、方向对不对。否决。
- **纯力矩、不加阻尼地转**：空载电机会一直加速到最高转速。加 Kd 后转速有上限。否决纯力矩。
- **停核来模拟通讯中断**：停核会冻住加热 PWM（可能停在高电平，见加热测试的经验），还会停掉别的任务。改为检查自己暂停调用 `process()`。否决停核。
- **经电机组（`MotorGroup<DM_J4310, 1>`）来测**：电机组只给力矩，做不了阻尼转动，也读不到核对值。适配器很薄，已有宿主测试。否决。

**回滚**：revert 本决策的提交，组合根恢复 `g_can_1`，删掉任务 `dm`。

---

## Decision-20261102-03（电机架构：能力只用关节 Joint，品牌驱动挂在每条总线的 CanBus 上，型号做成数据，Accepted）

- **Date**：2026-11-02
- **Status**：Accepted（负责人 2026-11-02："那我没有什么问题了，就按照你自己推荐的来改吧"）。2026-11-02 先以 Proposed 写出方案；负责人追问 CanBus 的适用范围与能否放进 `src/hal` 后补了 D9，随后 D1–D9 全部按推荐通过
- **Source**：负责人 2026-11-02 提出：电机品牌、型号多，管理麻烦、用法不统一；控制要稳定 1 kHz，问引入方便开发的架构会不会降低控制性能
- **方案全文**：[docs/ai/decisions/2026-11-02-电机架构方案/README.md](docs/ai/decisions/2026-11-02-电机架构方案/README.md)

**Context**：现在的电机组（`MotorGroup<型号, N>` + `MotorAdapterFor` + `MotorModel` 枚举）有六个问题：
- 一条总线只能挂一组同型号电机；
- 型号写在类型里，能力代码要知道型号；
- 加型号要改通用代码；
- 只有力矩接口，单位是度、rpm；
- 位置一律从第一帧起算，丢了达妙、GM6020 的绝对位置；
- 安全策略分散在各能力里。

现在没有能力用电机组，改的代价最小。

**Decision（提议）**：
1. 能力只读写关节 `device::Joint`：状态与指令，SI 单位，含方向、外接传动、零点。
2. 每条总线一个 `device::CanBus`：独占接收，按帧 ID 分给登记的品牌驱动，装配时检查 ID 冲突。
3. 品牌驱动（`DjiMotor`、`DmMotor`）改为登记到 `CanBus`、读写关节，各自的状态机保留。
4. 型号做成 `constexpr` 数据（型号目录），删去通用枚举与模板特化。
5. 一条总线只归一个任务，控制环在同一任务里按"收 → 算 → 发"排 step，不经 topic。
6. 组合根用 `static_assert` 检查各总线负载。
7. 集中的安全门能力：急停 / 失控与指令超时。
8. 性能用上板基准验收：重构前后的单轮耗时、周期抖动、反馈到下发延迟，都不劣于重构前。

**待负责人决定**（方案第 5 节，括号里是推荐；D9 为负责人 2026-11-02 追问后补充）：
- D1 指令模式（力矩 + 电机内阻抗）；
- D2 控制计算所在任务（各能力自己的任务）；
- D3 总线能否跨任务共用（不能）；
- D4 位置含义（有绝对编码器的给绝对位置）；
- D5 单位（SI）；
- D6 DJI 混接产品线（能，装配时查冲突）；
- D7 安全（集中：安全门 + 指令超时；急停后人工确认才恢复）；
- D8 名字（Joint）；
- D9 `CanBus` 放在哪一层（`src/driver/can_bus`，hal 契约不变；负责人问过能否放进 `src/hal`，比较见方案）。

**结论**：D1–D9 全部按推荐：
- D1 力矩 + 电机内阻抗；
- D2 控制在各能力自己的任务；
- D3 一条总线只归一个任务；
- D4 有绝对编码器的给绝对位置；
- D5 SI 单位；
- D6 DJI 混接、装配时查冲突；
- D7 集中安全门 + 指令超时、急停后人工确认才恢复；
- D8 Joint；
- D9 `src/driver/can_bus`，hal 契约不变。

**实施时的两处调整**（不改变结论）：
1. 步骤 M1（`hal::Can::Frame` 加接收时刻）提前到 M0（性能基线）之前。基线要测"收到反馈 → 下发指令"的延迟，需要接收时刻才准；加时间戳本身不改变要对比的结构。
2. 型号目录不集中成一个文件：每个品牌的型号写在该品牌目录里（如 `device::dji::M3508`、`device::dm::DM_J4310`）。通用代码因此不必知道有哪些品牌，原来的边界检查继续有效。

**Alternatives**：见方案第 5 节各项的其他选项。

**回滚**：按步骤分支逐个 revert（方案第 7 节）。

---

## 待登记

- 分支保护的具体设置（组织规范 §6.1、§10，负责人在 GitHub 上操作后补记）。
- `-Wdouble-promotion` 豁免清单（P3a）。
- 契约 v1 冻结记录（P3a 结束时）；TM4C、AM26x port 的启动条件与编译器选择（Q13）。
