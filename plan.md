# 开发计划（foray_mcu 结构重构）

> 当前状态：**P0 已验收（2026-10-26），进入 P0.5 格式与命名统一**（方案已确认，[decision.md](decision.md) Decision-20261025-01 至 -05 均为 Accepted；里程碑合并入 `main` 暂缓）。
> 权威方案见 [docs/ai/reviews/2026-10-25-foray_mcu-架构评审报告.md](docs/ai/reviews/2026-10-25-foray_mcu-架构评审报告.md)；本文件只记录执行计划、进度与验收标准。
> 迁移源 ControllerCode 原样放在 `legacy/`；各阶段的动作、验收与回滚以下文为准，不跨阶段混做。

## 任务列表

### R0 评审与文档（评审已完成）

- [x] 补齐组织五件套初版：README / AGENTS / plan / tree / decision（2026-10-25）
- [x] 结构重构方案评审报告初稿 v0.1 与 v0.2（`docs/ai/reviews/`，2026-10-25；原文存档于 `docs/ai/reviews/archive/`）
- [x] 评审报告 v0.3：吸收负责人确认的前提（后续多人、多平台；当前单人、时间充裕；TI 为 Cortex-M/R；MSPM0 裸机；双精度暂不处理），补充实测证据（2026-10-25）
- [x] 评审报告 v0.4：契约语言统一为 C++，取消 C ABI 层，C 链接只留三处外部强制的接缝（2026-10-25）
- [x] 负责人确认 Q1–Q18 全部采纳建议（2026-10-25）；Decision-20261025-01 改为 Accepted，新增 Decision-20261025-04（命名与格式）、-05（仓库、环境与迁移源布局）
- [x] 评审报告定稿为 v1.0（v0.1–v0.4 原文存档于 `docs/ai/reviews/archive/`）
- [x] 完善 `docs/ai/architecture.md`：分层判据、16 条依赖规则（含检查方式与生效阶段）、一级目录白名单、生成区治理、例外登记表（X-01–X-13，含 `-Wdouble-promotion` 豁免清单）、框架边界与可移植性、命名与格式（v1，2026-10-26，Decision-20261026-02）

### P0 基线与资产恢复（2–4 人日，✅ 2026-10-26 验收）

- [x] 建 git 仓库（`main`，提交身份 9isl，仓库级配置）；以 `legacy` remote（GitHub `ZQU-Foray/foray_mcu_controller`）导入 main、dev、chore/align-org-structure 的历史；tag `baseline/a2cfd16`（2026-10-25）
- [x] 迁移源整体移入 `legacy/`：一个纯重命名提交，333 个文件全部 100% 相似（2026-10-25，`c3490b2`）
- [x] 从 ControllerCode `fd1edbb` 恢复 host 测试到 `test/host/`：电机、协议、底盘与 IMU 采集测试共 13 项 ctest 全绿（WSL g++ 11.4、Windows MSYS2 g++ 16.2）；先原样放入（blob 哈希一致）再单独提交适配，未改 `legacy/`；P0 期间为独立 CMake 工程，P1 接入根工程；`Tests/ImuStream` 的被测模块（`ImuStream.hpp`、`ImuFrame.hpp`）在 `a2cfd16` 中已不存在，列为待重写（2026-10-26，适配记录见 `test/host/README.md`）
- [x] 确立根 `.clang-format`（沿用历史 `Libraries/Algorithm/.clang-format`，缩进 4，追加 LF 与文件末尾换行）与 `.clang-tidy`（ROS 2 命名），已用 clang-format/clang-tidy 18.1.8 校验（2026-10-25，`2026319`）
- [x] **重写** `.gitignore`（只忽略构建产物、IDE 目录与本机配置），删除 `legacy/.gitignore`；新增 `.gitattributes`（`* -text`）（2026-10-25，`3e2e536`）
- [x] 在 GitHub ZQU-Foray 组织下新建 `foray_mcu` 仓库并首次推送：负责人建仓（公开），推送 `main` 与 tag `baseline/a2cfd16`，remote 名 `origin`（2026-10-26）
- [x] **烧录任何新固件之前**，用 OpenOCD 读出 DM-MC02 的 Flash 存档，作为丢失功能重做时的行为参考：CMSIS-DAP（Vllink Basic2）只读、未停核；RDP 0；有效内容 285,412 B，判断为 Debug 构建，含迁移源中没有的链路协议与遥控任务；镜像存于仓库外（公开仓库不入 git），负责人已另行备份，SHA-256 与说明见基线对照文档 §8（2026-10-26）
- [x] 在 Windows 上、原结构（`legacy/Platform/STM32H7/DM_MC02`）中复现基线构建（xPack GCC 15.2.1）：Debug `.bin` 238,320 B / Release `.bin` 100,664 B，SHA-256 均与基线一致；`nm -S` 符号表与段表入档 `docs/ai/decisions/baseline-a2cfd16/`（见基线对照文档 §7；2026-10-26）
- [x] 录制一段真实 IMU 数据存入 `test/data/`，作为重构与精度调整的回归对照（2026-10-26，P0 范围内完成）：不烧录、只读 SWD，从原固件取得两份 50 ℃ 静态陀螺采集（各 2 万样本统计 + 1024 个原始样本，结果高度一致；噪声来自达妙小云台电机）→ `test/data/imu/`；原固件加热冷启动曲线（到 50 ℃ 24.1 s、过冲 0.375 ℃、111 s 进入 ±0.25 ℃）→ `test/data/heater/`。**移交 P3b**：姿态解算回归所需的长时间加速度计 + 陀螺原始序列，现有诊断录不出，待新结构实现 IMU 数据流后补录（见 P3b 任务）
- [x] 补齐组织合规件：`.foray-layer`（layer: MCU / depends_on_layers: [X2]，沿用迁移源 `chore/align-org-structure`）、`CODEOWNERS`（`* @ZQU-Foray/电控组`）、`.github/workflows/ci.yml`（第一版：lint-c 增量格式检查 + 宿主测试 + legacy 固件构建且 Release SHA 与基线一致；设计见 Decision-20261026-01）。本机已模拟各作业（2026-10-26）；GitHub 上首次运行结果见 PR。待负责人：给电控组本仓 Write 权限
- [x] 确认迁移基线：负责人确认最新进度找不回，以 `a2cfd16` 为基线，丢失的新电机控制、启动时间优化、上下位机通信、云台逻辑在新结构中重做（2026-10-25）；2026-10-26 负责人补充：加热启动性能与超调优化也已丢失。完整清单与证据见 [丢失功能与恢复依据](docs/ai/decisions/2026-10-26-丢失功能与恢复依据.md)
- [x] 验收（2026-10-26）：`git log --follow` 可用（迁移源文件追溯到原始提交，`docs/ai/` 下文档追溯到移动前）；`ctest` 绿（CI `host-tests`）；基线 SHA 与符号表入档（`docs/ai/decisions/baseline-a2cfd16/`）；Q4 证据表填毕。里程碑合并（`dev` → `main`）按负责人要求暂缓

### P0.5 格式与命名统一（不改行为，3–5 人日）

- [x] 排除 CubeMX 生成区与厂商代码：clang-format 18 的 `.clang-format-ignore` 不支持 `**`、`*` 不跨目录，改由 `tool/ci/format.py` 统一定义范围（CI 与本地共用），记入 Decision-20261026-04（2026-10-26）
- [x] 一次性格式化手写代码：126 个文件（`legacy/` 的 Application、Libraries、Platform/Interface、PortKit、Port 与 `test/host/`），122 个有变化，块注释内残留的 CR 一并去除，单独提交 `c23eb5e`；CI 的 lint-c 改为全量检查。验证：legacy Debug/Release `.bin` 与基线 SHA 逐字节一致，13 项 ctest 全绿（2026-10-26）
- [ ] 用 `clang-tidy --fix` 把 C++ 标识符改为 ROS 2 命名，手工补齐模板、宏等；同步更新测试与调试脚本中的符号名（如 `stackHealthArchive`）；C ABI 不改名；单独提交
- [ ] 把以上两类提交写入 `.git-blame-ignore-revs`（格式化提交 `c23eb5e` 已登记，改名提交待补）；`.clang-tidy` 的规则微调记入 decision.md
- [ ] 验收：同工具链下 `.bin` SHA 与 P0 复现的基线一致（改名若使 SHA 变化，以函数大小集合一致为准并逐项说明）；宿主测试绿
- [ ] 回滚：revert 对应提交

### P1 构建骨架（零源码改动，2–4 人日）

- [ ] 根 `CMakeLists.txt` + `CMakePresets.json` + `cmake/toolchains/`（首批 arm-gcc-cortex-m7、arm-gcc-cortex-m4f、host）
- [ ] 建立 `src/hal`（INTERFACE，C++ 契约）+ 过渡 target `foray_hal_detail`（现有 `Detail/*.h`，P3a 删除）+ `src/boot`（INTERFACE）+ `src/hal/os`（最小契约头）+ `src/utils`；`platform/mock` 最小实现；最小 host test target
- [ ] 顶层 executable 链接 assembly/port/board（assembly/board 可先空实现）；**port 与 board 用 OBJECT 库**（避免 HAL 弱符号回调被丢弃）；设置链接脚本、hex/bin 后处理、烧录目标
- [ ] 每个 target 私有 include；严格告警按 target 设置；删除全局 include 与 `file(GLOB)`
- [ ] 源码此时仍按迁移源根路径 include，设过渡 target `foray_legacy_include` 暴露 `legacy/`，并标注"P2 结束时删除"
- [ ] 符号对照工具（放 `tool/ci/`）：`nm` 文本跨平台不能直接 diff（排序与 `_sdata` 差异，见基线对照文档 §7），需按 `readelf -sW` 归一化；CI 的 `firmware-legacy` 作业改为根工程构建 + 符号集合对照
- [ ] 验收：**不改任何业务源码**；**`nm -S` 的符号名与大小集合与基线一致**（`.bin` SHA 预计因链接顺序改变而不同，只作参考）；`ctest` 可运行；`cmake --graphviz` 的节点/边集合与评审报告 §5.2 一致（允许空实现）
- [ ] 回滚：revert P1 单个提交，回到基线

### P2 机械搬迁（1:1 项，不改行为，2–3 人日）

- [ ] 从 `legacy/` 逐目录 `git mv`（文件名同时改为 snake_case，头文件保护宏随路径更新）+ include 机械改写（下列来源路径都在 `legacy/` 下）：`Application/Task/{0_imu,1_chassis,2_gimbal,4_communication}`→`app/tasks/`+能力目录；`Libraries/Algorithm|Component`→`src/lib/`（先保留子目录名）；`Libraries/Device`→`src/driver/`（含 `DjiMotorModelMap.hpp`，Q11 默认不移动）；`Libraries/Protocol`→`src/driver/protocol/`；`Platform/Interface`→`src/hal/`；`Platform/STM32H7/DM_MC02`→`platform/st/stm32h7/board_dm/`
- [ ] 拆分 `cubemx/`（生成物）与 `port/`（手写实现，此阶段仍是原 C 文件）、`board/`（pinmap/能力清单/启动胶水）；`Platform/Interface/Detail/` 暂放 `src/hal/detail/`
- [ ] 去掉裸头名 include（`Can.hpp`/`Time.hpp` 等）；**删除 `foray_legacy_include`**，此后层边界由构建保证
- [ ] 验收：构建绿；符号名与大小集合对照基线；`git diff -M` 为 rename + include/CMake 机械行（无逻辑改动）；`ctest` 绿；`git log --follow` 可追溯
- [ ] 回滚：revert P2 分支，P1 基线仍在

### P2.5 混合文件拆分搬迁（不改行为，2–4 人日）

> 处理评审报告 §7.2.1 的"非一对一映射"项：只移位/拆文件，不改逻辑。

- [ ] `Application/Entry.*` → `config/assembly/entry_legacy.*`（P3b 再收敛为 `foray_boot_*`）
- [ ] `Application/Task/TaskManager.*` → `app/common/task_manager.*`（P3b 再拆任务表到 `config/assembly`）
- [ ] `Application/Imu/*` → `app/imu/`（P3b 再拆 `ImuBodyMapping` 与时钟注入）
- [ ] `Platform/PortKit/*` → `src/utils/`（队列/通知）+ `src/hal/os/`（临界区契约）+ `platform/.../port/critical.c`（实现）
- [ ] `Port/UsbCdc.{c,h}` 与 USB 生成桩 → `platform/.../port/usb_cdc.c`（边界重划按 Q8，P4 做）
- [ ] 验收：构建绿；行为不变（host 对照/实机对照）；`git diff` 只含移位/拆文件/路径；ctest 绿
- [ ] 回滚：revert P2.5 分支，P2 基线仍在

### P3a 框架化与契约 v1（改接口不改行为，11–20 人日）

- [ ] 契约统一为 C++：合并 C ABI 层、删除 `src/hal/detail/`，枚举只保留 C++ 一份；port、PortKit 与 `platform/mock` 改写为 C++（约 2,700 行；`Spi.c`、`Uart.c` 的指定初始化器需改写）；历史测试的 C ABI 桩改为 C++ mock
- [ ] CI 增加 HAL 回调符号检查（`nm` 确认 `HAL_*Callback` 未被 C++ 改名）

- [ ] 设备 ID 移出公共契约头：契约头只定义 API 与不透明 ID 类型，ID 常量由 board 头提供，驱动经 `Config` 接收（BMI088 的 4 个文件、Buzzer、WS2812，共 16 处）
- [ ] 契约能力分级要有执行机制：SPI 异步接口在契约注释中是可选（MAY），BMI088、WS2812 却当作必选且无回退（评审报告 §2.3 缺口 1），F407 若用阻塞 SPI 会让 IMU 驱动进入故障态；契约 v1 中明确必选/可选并给出回退或编译期能力检查
- [ ] 最小 OS 契约实体（C++ RAII 临界区守卫 + 单调时基）在 H723 port 与 `platform/mock` 中实现
- [ ] `src/utils` 并发原语：计数器、最新值邮箱（seqlock）、SPSC 环形队列；在 M4/M7/R5 上用 `std::atomic`，M0+ 上退回临界区实现
- [ ] 框架代码开启 `-Wdouble-promotion`（IMU 姿态解算文件豁免）；`Vqf.cpp` 增加无双精度 FPU 时的 `#error` 检查
- [ ] CI 增加框架独立构建与裸机构建（只含 `src/` + `platform/mock`，不提供 RTOS 头）
- [ ] F407 C 板 bring-up：`platform/st/stm32f4/board_c`（cubemx/port/board），F407 验证 profile 不链接 IMU 姿态解算
- [ ] 验收：H723 行为不变（宿主测试 + 实机对照）；F407 上 CAN/SPI/UART/GPIO/PWM/时基契约测试通过；框架独立构建与裸机构建在 CI 绿；契约 v1 冻结并记入 [decision.md](decision.md)
- [ ] 回滚：按项 revert

### P3b 装配根与能力拆分（允许改行为，10–20 人日，丢失业务重建另计）

- [ ] 拆分 `config/profiles/<profile>/profile.cmake`（纯数据）与 `config/assembly`（任务表/对象构造/启动入口）；`IMU_ENABLE_DIAGNOSTICS`、`IMU_GYRO_BANDWIDTH` 收进 profile
- [ ] 落地 boot ABI：`src/boot/foray_boot.h` + `foray_boot_init/init_rtos/run`；生成区改为只调这三个符号
- [ ] `app` 按能力拆分 target：imu / chassis / gimbal / remote / link / launch；msg v1（最新值邮箱/SPSC 队列）与能力拆分同步落地
- [ ] 任务层从 `cmsis_os2` 改用 FreeRTOS 原生 API（约 15 处调用）
- [ ] 在新结构中重建丢失的业务链（遥控 → 底盘/云台）与其他丢失功能（L1–L6，见 [丢失功能与恢复依据](docs/ai/decisions/2026-10-26-丢失功能与恢复依据.md)）；其中加热控制（L6）以原固件冷启动实测曲线为验收下限
- [ ] IMU 原始数据流（加速度计 + 陀螺，长时间、带时间戳），补录姿态解算回归数据到 `test/data/imu/`
- [ ] 还既有结构债：`ImuBodyMapping` 反向依赖 Device；`Application/Imu` 的时钟读取/设备类型例外；任务表外 `defaultTask`；死端点检查与 Entry 漏检 Pwm；原子读改写迁移到并发原语；`TaskSpec::periodMs`（**条件项**：仅当迁移基线包含该字段时执行）
- [ ] 至少 2 个 profile 构建成功（`infantry_v1` × DM-MC02 + F407 验证 profile）；host test 覆盖装配/任务表/控制模块
- [ ] 验收：CI 分层检查绿；体积预算不超基线 +5%（超预算需 decision 记录）；行为对照（等价性测试/仿真/实机）
- [ ] 回滚：按能力 PR 粒度 revert，不影响其他能力

### P4 链路协议、UsbCdc 与 CI 全矩阵（8–15 人日，不含 TI port）

- [ ] vendor `foray_interfaces` 生成头（固定版本 + sha256）；`app/link` + `src/middleware/link` 接入链路协议（禁止手写编解码）
- [ ] 实现安全硬路径：100 / 200 / 500 ms 失联降级与开火硬上限（独立于上位机）
- [ ] UsbCdc 边界按 Q8 落地；链接 overlay 按 Q9 落地（再生成后自定义段仍在）
- [ ] CI 矩阵：profile × board × Debug/Release + host-tests + 框架独立构建 + arch-check + vendor-hash + format
- [ ] TM4C、AM26x port **按需启动**：先确认编译器（Q13，GCC 与 TI Arm Clang 双编译器）与 RTOS（Q12）
- [ ] 验收：每个 board 有产物；链路握手/编解码 host 测试绿；实机回归清单完成；组织 CI 绿灯

### P5 MSPM0 与框架提取（按需：第一个小项目启动时）

- [ ] MSPM0 裸机 port（`platform/ti/mspm0`）+ 最小示例；工具链 `arm-gcc-cortex-m0plus`；确认启动文件调用全局构造（`__libc_init_array`）
- [ ] `git subtree split` 把框架（`src/`、`platform/`、`cmake/`）提成独立仓库并保留历史；本仓与小项目通过 submodule 或 `FetchContent` 引用
- [ ] 验收：示例在 MSPM0 上运行；框架仓独立 CI 绿

## 当前 Agent State

`EXECUTING`（P0 已于 2026-10-26 验收，里程碑合并入 `main` 按负责人要求暂缓；P0.5 格式化已完成（Release/Debug 与基线逐字节一致），下一步是 ROS 2 命名改名）。
状态词表：`REVIEW_PENDING`（等待评审）/ `PLANNING`（方案已批准、编制细节）/ `EXECUTING`（实施中）/ `BLOCKED`（外部阻塞）/ `DONE`（阶段完成）。

## Before Snapshot

- 迁移源：ControllerCode，`main` @ `a2cfd16`，19 个提交，git 跟踪 333 文件，项目自有代码约 2.3 万行（本机 clone 路径因机器而异）。
- 目标仓（P0 开始前）：本仓无 `.git`、无 CMakeLists、无源码，只有五件套、评审报告与归档证据。
- 本次变更（P0 第一部分，2026-10-25）：建 git 仓库并导入迁移源历史；迁移源整体移入 `legacy/`（纯重命名）；重写 `.gitignore`、新增 `.gitattributes`；新增 `.clang-format`、`.clang-tidy`；提交五件套与评审材料（评审报告定稿 v1.0）。无源码内容改动、无构建、无生成区改动。
- 本次变更（P0 第二部分，2026-10-26）：推送到 GitHub `ZQU-Foray/foray_mcu`；只读存档芯片 Flash（仓库外）；在 Windows 上复现基线构建并入档符号表。只新增/修改文档与符号表文本，无源码改动、无生成区改动、未烧录。
- 本次变更（P0 第三部分，2026-10-26，分支 `p0/restore-host-tests`）：从 `fd1edbb` 恢复宿主测试到 `test/host/`（原样放入与适配分两个提交），新增独立 CMake 工程。无 `legacy/` 改动、无生成区改动。
- 本次变更（P0 第四部分，2026-10-26，分支 `p0/compliance-ci`）：新增 `.foray-layer`、`CODEOWNERS`、`.github/workflows/ci.yml`、`docs/ai/decisions/baseline-a2cfd16/SHA256SUMS`；更正基线工具链说明并补 Linux 复现结论。无源码改动、无 `legacy/` 改动。
- 本次变更（P0 第五部分，2026-10-26，分支 `p0/architecture-doc`）：`docs/ai/architecture.md` 从占位改为 v1；P3a 补 SPI 能力分级任务。只改文档。
- 本次变更（P0 第六部分，2026-10-26，分支 `p0/imu-data-heater`）：新增 `tool/debug/`（SWD 轮询、IMU 采集解码）、`test/data/imu/` 首个数据集、丢失功能清单；只读访问芯片，未烧录。
- 本次变更（P0 第七部分，2026-10-26，分支 `p0/docs-ai-split`）：AI 撰写的文档移入 `docs/ai/`（15 个文件，`git mv`），全仓引用同步更新，修复评审报告历史版本中原有的 24 条断链。只改文档与路径。
- 风险级别：**L1（仓库结构与配置，可整体回滚）**。
- 分支：`dev`（日常开发，PR 目标）+ `main`（里程碑快照）；基线 tag：`baseline/a2cfd16`；本仓提交见 `git log baseline/a2cfd16..dev`。

## 模糊点与待确认项

Q1–Q18 已全部确认（见评审报告 v1.0 §11.1 与 decision.md）。仍待落实的事项：

1. Windows 上 clang-format / clang-tidy 18 的安装方式（建议 LLVM 18 官方安装包或 `pip install clang-format==18.1.8`），保证与 CI 同版本。
2. `src/bsp/hardware/` 下 7 个 0 字节占位文件未纳入 git，P1 建 `src/hal` 时删除。
3. `a2cfd16` 中没有任何代码调用 `ChassisTask::SetEnabled(true)`（底盘上电默认失能，`Run` 不再自行使能），该版本底盘不会输出力矩；丢失的遥控任务可能承担使能职责。P0 不修，P3b 重做遥控/决策任务时明确使能职责。
4. 仓库设置（负责人在 GitHub 上操作）：默认分支改为 `dev`（已于 2026-10-26 建立，Decision-20261026-01 第 7 项）；给 `电控组` 本仓 Write 权限，否则 CODEOWNERS 静默失效；`main`、`dev` 分支保护（PR + 1 approve）。
5. 组织规范文件格式需组织负责人确认（`foray_docs/repository_structure.md`、`algorithm_structure.md` 已可从 GitHub 读取）。

## Vector Backend Status

- Backend: Markdown
- Status: ready
- Environment: 仓库内 Markdown（无外部向量后端）
- Index: `README.md` / `plan.md` / `tree.md` / `decision.md` / `docs/ai/reviews/` / `docs/ai/decisions/`
- Initialization: 2026-10-25
- Commit: `main` 分支（`origin` = GitHub `ZQU-Foray/foray_mcu`）；基线 tag `baseline/a2cfd16`（迁移源历史 + 本仓提交）

## Acceptance Criteria

- [x] 评审通过：Q1–Q18 有明确结论，[decision.md](decision.md) 已更新状态（2026-10-25）
- [ ] host 测试入库并全绿（`ctest`）（P0：历史测试 13 项已恢复并全绿；`ImuStream` 待重写）
- [ ] 至少 2 个 profile 构建成功（含 F407 验证 profile）；`platform/mock` 下 host 集成测试绿
- [ ] 框架独立构建与裸机构建在 CI 绿
- [ ] CI 架构门禁全绿：include/依赖方向、生成区、禁止 `file(GLOB)`、禁止手写协议、vendor 哈希
- [ ] 体积预算不超基线 +5%（例外需 decision 记录）
- [ ] 文档与代码一致：[tree.md](tree.md)、README 的"当前状态"与实际构建状态同步

## 进度记录

| 日期 | 阶段 | 内容 | 结果 |
|---|---|---|---|
| 2026-10-25 | R0 | 五件套初版 + 结构评审报告初稿落盘 | 待评审 |
| 2026-10-25 | R0 | 评审报告 v0.3：吸收负责人确认的前提、补充实测证据（附录 F）；同步五件套与 docs 索引；v0.1/v0.2 原文存档 | 待负责人确认默认采纳项 |
| 2026-10-25 | R0 | 评审报告 v0.4：契约语言统一为 C++（§5.11、Q17），P3a 工时调整为 11–20 人日，总计 37–70 人日；同步五件套 | 待负责人确认 |
| 2026-10-25 | R0 | 负责人确认 Q1–Q18；评审报告定稿 v1.0（新增命名与格式、P0.5、`legacy/` 布局）；decision.md 新增 -04、-05 | 已确认 |
| 2026-10-25 | P0 | 建 git 仓库并导入迁移源历史（tag `baseline/a2cfd16`）；迁移源整体移入 `legacy/`；重写 `.gitignore`、新增 `.gitattributes`；确立 `.clang-format` 与 `.clang-tidy`；提交五件套与评审材料 | 进行中 |
| 2026-10-26 | P0 | 推送到 GitHub（`main` + tag）；只读存档芯片 Flash（RDP 0，两遍一致，含丢失的链路协议与遥控任务）；Windows + xPack GCC 15.2.1 复现基线构建，Debug/Release SHA-256 均一致，符号表入档 | 进行中 |
| 2026-10-26 | P0 | 从 `fd1edbb` 恢复宿主测试到 `test/host/`（13 项 ctest 全绿，WSL 与 Windows 均验证；适配 5 处，未改 `legacy/`；`ImuStream` 待重写）；发现 `a2cfd16` 底盘从不使能 | 进行中 |
| 2026-10-26 | P0 | 补齐 `.foray-layer`、`CODEOWNERS`、CI 第一版（lint-c 增量、宿主测试、legacy 固件 Release SHA 闸门）；Linux 上 Release 逐字节复现基线；更正"基线用 Arm 官方工具链"的说法 | 进行中 |
| 2026-10-26 | P0 | 合并 CI 第一版（PR #2，CI 首次运行 5 个作业全绿），建 `dev` 分支；`docs/ai/architecture.md` v1：判据、规则表（检查方式 + 生效阶段）、白名单、例外登记表 | 进行中 |
| 2026-10-26 | P0 | 只读 SWD 取得原固件静态陀螺采集并入库；定位原固件加热器状态（算法与 `a2cfd16` 不同）；建立丢失功能清单（新增 L6 加热优化）；新增 `tool/debug/` | 进行中 |
| 2026-10-26 | P0 | 记录原固件加热冷启动曲线（到 50 ℃ 24.1 s、过冲 +0.375 ℃、111 s 进入 ±0.25 ℃），作为 L6 重做的验收下限；第二份陀螺采集验证可重复；新增 `heater_log_decode.py`；`swd_poll.py` 改用单调时钟 | P0 任务完成，待验收 |
| 2026-10-26 | P0 | AI 撰写的文档移入 `docs/ai/`（Decision-20261026-03），全仓 105 条相对链接全部可解析 | P0 任务完成，待验收 |
| 2026-10-26 | P0 | P0 验收：`git log --follow`、`ctest`、基线 SHA 与符号表、Q4 证据表均满足；`dev` → `main` 里程碑合并暂缓 | ✅ 已验收 |
| 2026-10-26 | P0.5 | 新增 `tool/ci/format.py`；一次性格式化 126 个手写文件（`c23eb5e`，登记于 `.git-blame-ignore-revs`）；CI lint-c 改为全量检查。legacy Debug/Release 与基线逐字节一致，ctest 13/13 | 格式化完成，改名待做 |
