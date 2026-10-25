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
- **Status**：**Accepted**（2026-10-25，负责人确认评审报告 **v1.0** §11.1 的 Q1–Q18 全部采纳建议；评审材料见 [docs/reviews/2026-10-25-foray_mcu-架构评审报告.md](docs/reviews/2026-10-25-foray_mcu-架构评审报告.md)）
- **影响范围**：目录结构、CMake target 模型、平台契约、启动接缝、迁移流程；**不影响迁移源代码行为（P0–P2.5 为不改行为阶段，P3a 只改接口不改行为）**

**Context**：

- 本仓当前为空骨架（无 `.git`、无源码）；迁移源为 ControllerCode（`main` @ `a2cfd16`），P0 才迁入 git 历史。迁移源现有结构为 Application / Libraries / Platform / CubeMX 四层，依赖倒置在 include 层成立（业务对 Platform 的 26 条依赖全部指向 `Platform/Interface/*.hpp`），但存在四个结构性问题：
  1. 整车装配知识分散在 5 处（`Entry.cpp`、`TaskManager.cpp`、`ChassisTask.cpp`/`GimbalTask.cpp`、板级 CMake）；`DjiMotorModelMap` 中是电机型号的厂商固有参数，建议留在驱动层（Q11 默认采纳）；
  2. 构建为单 executable + `file(GLOB)` + 全仓 include，层没有链接期边界；
  3. CubeMX 生成代码反向 include 应用头（`main.c`/`freertos.c` → `Application/Entry.h`），启动权在生成区（`Entry.h` 已是纯 C 启动契约，收口工作量小）；
  4. 平台契约枚举是"本机器人能力集"（CAN 3 通道、SPI 3 设备、UART 2 端点、PWM 2 通道、GPIO 8 脚，Interface 与 Detail 双份硬编码），多板复用会污染公共契约。
- 新结构提案（app 按兵种 / config 装配根 / platform 按厂商和开发板 / src 通用框架 + bsp/topic/算法/组件）方向正确，但存在 `config` 双角色成环、`app` 按兵种复制分叉、`src/bsp` 命名与 os/boot 契约缺失（v0.4 起不再要求 C ABI 层）、CubeMX 边界破口四处必须先修正的边界。
- 另：ControllerCode 的 host 测试（`fd1edbb` 的 `Tests/` 26 文件 / 2,979 行；该提交共 28 文件 / 3,008 行）已被后续提交删除且被 `.gitignore` 屏蔽；`.docs/`、`Tools/`、`*.py`、`*.cfg`、`.clang-format/.clang-tidy` 等排除/删除导致测试、工具、证据链丢失。迁移基线数据（Debug/Release `.bin` 与 SHA）所在的《固件恢复可行性评估》也未入 git，本仓已转存到 `docs/assets/` 与 `docs/decisions/`。本仓必须先把这些资产恢复，再谈结构迁移。
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
- **Status**：Accepted（负责人确认；P0 已执行前 4 项）
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

## 待登记

- GitHub 仓库创建与首次推送；CODEOWNERS / `.foray-layer` / CI 的启用时间（P0）。
- P0.5 首次运行 `.clang-tidy` 后的规则微调记录；`-Wdouble-promotion` 豁免清单（P3a）。
- 契约 v1 冻结记录（P3a 结束时）；TM4C、AM26x port 的启动条件与编译器选择（Q13）。
