# 开发计划（foray_mcu 结构重构）

> 当前状态：**P0–P2.5 已验收（2026-10-26）；2026-10-27 起按架构 v2 重建，下一步 F1 框架基础件**（Decision-20261027-01；里程碑合并入 `main` 暂缓）。
> 权威方案见 [docs/ai/architecture.md](docs/ai/architecture.md) v2 与 [decision.md](decision.md)；[评审报告 v1.0](docs/ai/reviews/2026-10-25-foray_mcu-架构评审报告.md) 是历史记录。本文件只记录执行计划、进度与验收标准。
> 各阶段的动作、验收与回滚以下文为准，不跨阶段混做。

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
- [x] 用 `clang-tidy --fix` 把 C++ 标识符改为 ROS 2 命名，手工补齐模板、宏等；同步更新测试与调试脚本中的符号名（如 `stackHealthArchive`）；C ABI 不改名；单独提交。由新增的 `tool/ci/naming.py` 汇总固件 Debug、Release 与宿主测试三个编译数据库的修正：自动改名 2190 处，手工修正模板依赖访问、未实例化模板里的调用与注释中的旧名字；改名 `2b3fbf8` 与改名后格式化 `0c51793` 分开提交。仓库内的调试脚本不按名字读符号，无需改动（评审报告提到的 `read_benchmark.sh` 等没有随迁移源进入本仓）。过程与复现见 [P0.5 改名记录](docs/ai/decisions/2026-10-26-P0.5-改名/README.md)（2026-10-26）
- [x] 把以上两类提交写入 `.git-blame-ignore-revs`（`c23eb5e`、`2b3fbf8`、`0c51793`）；`.clang-tidy` 的规则微调记入 decision.md（Decision-20261026-05）；CI 新增 `lint-naming` 全量命名检查
- [x] 验收（2026-10-26）：同工具链下 legacy Debug/Release `.bin` SHA 与基线逐字节一致；符号"地址+大小+类型"集合一致（改名后的符号表已入档，供 P1 对照）；宿主测试 13/13 绿；命名检查 0 处
- [ ] 回滚：revert 对应提交

### P1 构建骨架（零源码改动，2–4 人日，✅ 2026-10-26 验收）

> 过程、对照数据与发现见 [P1 构建骨架记录](docs/ai/decisions/2026-10-26-P1-构建骨架/README.md)，设计决定见 Decision-20261026-07；target 名去掉项目前缀见 Decision-20261026-06。

- [x] 根 `CMakeLists.txt` + `CMakePresets.json`（arm-debug、arm-release、host-tests）+ `cmake/toolchains/`（arm-gcc-cortex-m7、arm-gcc-cortex-m4f、host）；唯一开关 `-DPROFILE`，profile `baseline`（a2cfd16 基线固件）的 `profile.cmake` 声明板卡 `st/stm32h7/board_dm`，板卡的 `board.cmake` 声明板标识 `dm_mc02` 与 CPU，配置时与工具链核对
- [x] 建立 `src/hal`（INTERFACE，迁移期为 `legacy/Platform/Interface/*.hpp`）+ 过渡 target `hal_detail`（`Detail/*.h`，P3a 删除）+ `src/boot`（INTERFACE，P3b 才有内容）+ `src/hal/os`（最小契约头草案：`CriticalSection` RAII 守卫、`now_ticks()`/`tick_frequency_hz()`）+ `src/utils`（INTERFACE，P3a 才有内容）；`platform/mock` 实现 hal_os（时间与临界区可由测试控制）；新增宿主测试 `hal_os_mock`
- [x] 顶层 executable `firmware_baseline` 链接 `assembly_baseline` 与 `port_dm_mc02`；port、board 与生成区（`mx_generated_dm_mc02`、`rtos_dm_mc02`、`usb_dm_mc02`）、迁移期的 `portkit` 均为 OBJECT 库，间接依赖的目标文件经 `link_object_library()` 进入固件；board 带链接脚本，port 带 `--wrap`；生成 `.map/.hex/.bin`，`flash`/`openocd-server` 目标（OpenOCD + CMSIS-DAP，`tool/flash/openocd/`）
- [x] 每个 target 显式声明 include 与依赖；严格告警（`-Wextra -Werror`）按 target 设置，`-Wall` 与迁移源一样加给全部代码（GCC 15 在 -O0 下 `-Wall` 会改变生成代码）；新工程没有全局 include 与 `file(GLOB)`（`arch_check.py scan` 检查）
- [x] 源码仍在 `legacy/` 原地、按迁移源根路径 include，过渡 target `legacy_include` 暴露 `legacy/`，标注"P2 结束时删除"；迁移源的两个构建期开关由 `legacy_build_options` 提供（P3b 删除）
- [x] 符号对照工具 `tool/ci/symbols.py`（`readelf -sW` 归一化，名称、大小、类型、绑定、输出段，不含地址）；依赖图检查 `tool/ci/arch_check.py`；CI 的 `firmware-legacy` 改为 `firmware`：根工程构建 + 与迁移源逐配置比较符号集合 + 依赖图检查，迁移源 Release SHA 校验保留到 P2；`host-tests`、`lint-naming` 改用根工程 preset；新增 `lint-arch`
- [x] 宿主测试改由根工程引入：框架单元测试链接 `lib_*`/`driver_*`；底盘集成测试仍直接编译 app 源码（例外 X-10，P3b）
- [x] 验收（2026-10-26）：**业务源码零改动**（`legacy/` 无任何变更）；**符号集合与迁移源构建一致**——Debug 1426 个、Release 1038 个函数与对象符号的名称、大小、类型、绑定、输出段全部相同（Windows xPack GCC 15.2.1 与 Linux Arm GNU 15.2.Rel1 各验证一次）；`.bin` SHA 如预期不同（`.text` 等段因链接顺序有几个字节的对齐填充差异），唯一的符号表差异是 `QuaternionEkf.cpp` 的 FILE 符号（该文件的代码在迁移源中也被 `--gc-sections` 全部回收）；`ctest` 14/14（MSYS2 g++ 16.2、WSL g++ 11.4）；依赖图检查通过（与评审报告 §5.2 一致，迁移期放行的边逐条注明），并做了反向验证；命名检查 0 处
- [x] 回滚：revert P1 的合并提交即回到 P0.5 状态（`legacy/` 自带的构建仍可用）

### P2 机械搬迁（1:1 项，不改行为，2–3 人日，✅ 2026-10-26 验收）

> 移动表、脚本与验证见 [P2 机械搬迁记录](docs/ai/decisions/2026-10-26-P2-机械搬迁/README.md)，规则见 Decision-20261026-08。

- [x] 从 `legacy/` 逐目录 `git mv`（文件名同时改为 snake_case，C++ 头统一 `.hpp`，头文件保护宏随路径更新）+ include 机械改写：`Application/Task/{0_imu,1_chassis,2_gimbal,4_communication}`→`app/tasks/`+`app/chassis`、`app/remote`；`Application/Module/GimbalModule`→`app/gimbal/`（计划未写阶段，1:1 项随 P2 搬）；`Libraries/Algorithm|Component`→`src/lib/`（保留子目录名）；`Libraries/Device`→`src/driver/`（含 `dji_motor_model_map.hpp`，Q11 默认不移动）；`Libraries/Protocol`→`src/driver/protocol/`；`Platform/Interface`→`src/hal/`；`Platform/STM32H7/DM_MC02`→`platform/st/stm32h7/board_dm/`。被生成代码按名字 include 的 `port/Init.h` 保持原名
- [x] 拆分 `cubemx/`（生成物，含 CubeMX 每次重写的 cmake 文件）与 `port/`（手写实现，仍是 C）；`board/` 暂无手写内容（pinmap/能力清单在 P3a），target 只带链接脚本；`Platform/Interface/Detail/` 暂放 `src/hal/detail/`；迁移源旧构建文件删除
- [x] 去掉裸头名 include（`Can.hpp`/`Time.hpp` 等）；框架与已搬迁的 app target 不再依赖 `legacy_include`。**`legacy_include` 的删除推迟到 P2.5**：P2.5 的文件（Entry、TaskManager、Imu）仍以 `"Application/..."` 被引用（Decision-20261026-08）
- [x] 验收（2026-10-26）：构建绿（零告警）；符号集合与 P1 存档一致（Debug 1426、Release 1038，xPack 与 Arm GNU 两套工具链）；112 个固件源文件除 include 目录外的编译选项与 P1 相同；改动分三个提交：纯 `git mv`（323 个文件全部 R100）、include/保护宏改写（只改这两类行）、构建与工具适配；`ctest` 14/14；`git log --follow` 可追溯到迁移源最早的提交（含只有一行的文件）；CI 的 `firmware` 作业改用存档符号表作基准，删除迁移源 SHA 校验
- [x] 回滚：revert P2 的合并提交，P1 基线仍在

### P2.5 混合文件拆分搬迁（不改行为，2–4 人日，✅ 2026-10-26 验收）

> 处理评审报告 §7.2.1 的"非一对一映射"项：只移位/拆文件，不改逻辑。过程、对照与验证见 [P2.5 搬迁记录](docs/ai/decisions/2026-10-26-P2.5-拆分搬迁/README.md)，去向与偏离计划之处见 Decision-20261026-10。

- [x] `src/lib` 子目录与命名空间去掉 `alg_` 前缀（负责人要求，原定 P3b，单独一个 PR；`alg_controller` → `control`，其余去掉前缀）：纯 `git mv` + 整词替换 + 重新格式化三个提交；`symbols.py compare --renames` 按改名表换算 P1 存档后，Debug 1426、Release 1038 一致（Decision-20261026-09，[记录](docs/ai/decisions/2026-10-26-P2.5-去掉alg前缀/README.md)）

- [x] `Application/Entry.*` → `config/assembly/entry_legacy.cpp` + `config/assembly/Application/Entry.h`（生成代码按 `"Application/Entry.h"` include，路径保持；由组合根把 `config/assembly` 注入生成区的 include 路径，例外 X-09；P3b 再收敛为 `boot_*`）
- [x] `Application/Task/TaskManager.*` → `app/common/task_manager.*`（P3b 再拆任务表到 `config/assembly`）
- [x] `Application/Imu/*` → `app/imu/`（P3b 再拆 `ImuBodyMapping` 与时钟注入）
- [x] `Platform/PortKit/*` → **整体并入** `platform/.../port/port_kit.{c,h}`，删除过渡 target `portkit`（X-15 关闭）。原计划的拆分（队列/通知 → `src/utils`，临界区契约 → `src/hal/os`，实现 → port）**推迟到 P3a**：队列与通知用 CMSIS 的 `__DMB()`，不能原样进零平台的 `src/utils`；临界区拆到别的文件会改变 Release 生成代码（Decision-20261026-10）
- [x] `Port/UsbCdc.{c,h}` → `platform/.../port/usb_cdc.c` + `UsbCdc.h`（被 USB 生成代码按名字 include，保持原名；边界重划按 Q8，P4 做）
- [x] 搬完后删除 `legacy_include`、`legacy_library()` 与 `.clang-tidy` 头文件过滤中的 `legacy/Application` 一段（P2 推迟到此，Decision-20261026-08）；`legacy/` 只剩迁移源 README 与架构图（X-11、X-12 关闭）
- [x] 验收（2026-10-26）：构建绿（零告警）；符号集合按 `alg_` 改名表换算后与 P1 存档一致（Debug 1426、Release 1038，xPack 与 Arm GNU 两套工具链）；搬迁的 4 个目标文件反汇编（含重定位）与数据段逐项相同；编译选项除 include 目录外不变（`port_kit.c` 随 port 带上严格告警，无告警）；改动分纯 `git mv`（16 个文件 R100）、include/保护宏改写、构建与工具适配三个提交；`ctest` 14/14；命名 0 处
- [x] 回滚：revert P2.5 的两个合并提交（搬迁、去掉 `alg_` 前缀），P2 基线仍在

### 重建阶段（架构 v2，2026-10-27 起）

> 2026-10-27 起，迁移方式改为"按负责人的骨架重建，迁移源只作参考与零件来源"（Decision-20261027-01，[架构 v2](docs/ai/architecture.md)）。顺序：先建框架（F1–F4），负责人验收框架后再重建能力（A），DM-MC02 完成后再做 F407 C 板（B）。验证按模块进行，不再与旧固件逐符号对照。原 P3a、P3b、P4、P5 的任务按文末对照表并入新阶段，不再单独执行。工时为粗估。

### F1 框架基础件（宿主可测，纯新增，4–6 人日）

- [x] 第 1 步 归类调整（2026-10-27，Decision-20261027-02）：`crc`、`math` → `src/utils`（target `utils_crc`、`utils_math`，用 `utils_library()`，不带迁移期构建开关）；`fsm` → `src/middleware/fsm`（负责人决定）；`slope` 并入 `src/lib/control`（命名空间 `control`）；依赖图检查加入 lib → utils，middleware 按架构 v2 收紧
- [x] 第 2 步 `src/utils` 补齐（2026-10-29，Decision-20261029-01）：并发原语放 `src/utils/concurrency`（target `utils_concurrency`，只有头文件）——`SpscQueue`（由 PortKit 的队列改写，acquire/release，不用原子读改写）、`LatestValue`（单写者 seqlock）；`crc` 补 crc16（CRC-16/MCRF4XX，裁判系统帧尾）；宿主测试 `spsc_queue`、`latest_value`、`crc16`
- [ ] `src/hal/os` 契约定稿：`CriticalSection`、单调时基、`Signal`（等待/唤醒，中断里可唤醒）；`platform/mock` 实现（宿主线程，测试控制时间）
- [ ] `src/middleware/topic`（target `middleware_topic`）：最新值、单发布者、中断里可发布、阻塞等待、观测计数（架构 v2 §3.4）；`app/topics` 目录与清单约定；先决定 seqlock 读写竞态的处理（Decision-20261029-02，Proposed）
- [ ] 宿主测试：utils 与 topic 的单元测试，含多线程压力（发布线程 × 订阅线程、模拟中断发布、等待超时、单发布者检查）。utils 部分已完成（第 2 步）
- [ ] CI：框架独立构建（只配置 `src` + `platform/mock`，无 RTOS 头）；文本扫描加入规则 13、17；Cortex-M0+ 编译 utils 与 topic，检查不出现 `__atomic_*`、`__sync_*` 库调用（代替 `is_always_lock_free` 的静态检查，Decision-20261029-01）
- [ ] 验收：宿主测试全绿；框架独立构建绿；现有固件仍可构建（F1 只新增，不改旧代码的行为）

### F2 契约 v1 与 DM-MC02 板级（6–10 人日）

- [ ] 旧能力代码归档：打 tag `archive/pre-rebuild`，移出构建并删除 `app/{imu,chassis,gimbal,remote,tasks,common}` 的旧代码、`config/assembly/entry_legacy.cpp` 与 `Application/Entry.h`、profile `baseline`、依赖旧 app 的宿主测试（底盘集成测试；`bmi088_acquisition` 引用 `imu_diagnostics.cpp` 的部分在 F3 改写）（例外 X-04、X-09、X-10 关闭）；能力重建时从 tag 取参考
- [ ] `src/hal` 契约 v1：自由函数 + 不透明句柄 + 能力分级（can、spi、uart、gpio、pwm、time、usb_cdc、`hal::init`）；`platform/mock` 实现（可注入输入、检查输出）
- [ ] `platform/dm_mc02/`（按板一级）：port 用 C++ 重写，实现 hal v1 与 hal/os（FreeRTOS 任务通知）；PortKit 拆分完成；`board/board.hpp`（物理资源句柄、能力常量）；删除 `src/hal/detail`（X-07 关闭）
- [ ] 启动入口：`src/boot/boot.h`；CubeMX USER CODE 区改为调用 `boot_*`（改动行登记在 decision.md）
- [ ] `app/common` 任务运行器（FreeRTOS 原生 API，任务表驱动，栈水位巡检）
- [ ] profile `bringup_dm_mc02` 与对应的 assembly：启动 → 任务表 → topic → LED/蜂鸣器，固件可构建、可烧录
- [ ] CI：`firmware` 作业的符号对照退役，改为构建 + 段大小报告；加入 HAL 回调符号检查；依赖图与文本扫描加入规则 1（`board.hpp`）、4、7、10、16、18
- [ ] 验收：bringup 固件在板上启动，任务按表运行，中断发布的 topic 能被阻塞等待的任务收到；宿主测试全绿

### F3 驱动适配（4–7 人日）

- [ ] 驱动改用 hal v1，经 `Config` 接收句柄与参数：bmi088（陀螺带宽改为 `Config` 参数，X-14 关闭）、motor/dji（去掉原子读改写，X-08 关闭）、buzzer、ws2812、protocol/{dji,sbus}；用 `platform/mock` 做单元测试
- [ ] SPI 异步按能力分级使用：BMI088、WS2812 对可选能力做编译期检查或回退
- [ ] `src/lib/estimate` 去掉对 BMI088 驱动的依赖（POD 入参）；框架开启 `-Wdouble-promotion`（`vqf.cpp` 豁免并加 FPU `#error` 检查）
- [ ] bringup 固件加入各驱动的检查：CAN 收发（电机反馈）、SPI 读 BMI088、UART 收 SBUS、PWM/GPIO、USB CDC 回显、时基精度
- [ ] 验收：驱动单元测试全绿；bringup 固件上各驱动的检查通过

### F4 框架验证（2–4 人日）

- [ ] 实机检查清单（负责人执行）：启动、任务、topic（发布次数、丢失计数、等待延迟）、各外设
- [ ] 用框架写一个最小示范：IMU 驱动 → 姿态 topic → USB 输出，评估新增驱动、topic、任务的写法是否顺手
- [ ] 指南：怎样新增外设接口、驱动、topic、能力、profile（`docs/`）
- [ ] 验收：负责人确认框架"稳定好用"，再进入阶段 A

### A 能力重建（F4 验收后细化）

- 在框架上逐个重建能力：imu、chassis、gimbal、remote、launch、link（vendor `foray_interfaces`，禁止手写链路编解码）、加热；补回丢失功能 L1–L6（[丢失功能与恢复依据](docs/ai/decisions/2026-10-26-丢失功能与恢复依据.md)）
- 第一个兵种 profile（DM-MC02）；IMU 原始数据补录到 `test/data/imu/`，姿态解算用录制数据回归
- topic 清单改为从描述文件生成（框架验证好用后）
- 安全硬路径：失联降级与开火硬上限（独立于上位机）

### B F407 C 板（DM-MC02 重建完成后）

- `platform/` 下新增 C 板目录（板标识届时确定）：cubemx、port、board；契约测试（CAN/SPI/UART/GPIO/PWM/时基）
- 第二个 profile；CI 矩阵：profile × Debug/Release + host-tests + 框架独立构建

### E 框架提取与 MSPM0（按需：第一个小项目启动时）

- MSPM0 裸机 port + 最小示例；`git subtree split` 把框架提成独立仓库并保留历史

### 原 P3a–P5 任务的去向

| 原计划项 | 去向 |
|---|---|
| P3a 契约统一为 C++、删除 `hal/detail`、port 改写为 C++、设备 ID 移出契约、SPI 能力分级 | F2（契约、port、board）；F3（驱动侧） |
| P3a HAL 回调符号检查 | F2 |
| P3a 最小 OS 契约实体 | F1（契约 + mock）、F2（port 实现） |
| P3a `src/utils` 并发原语 | F1 |
| P3a `-Wdouble-promotion` 与 `vqf.cpp` 的 FPU 检查 | F3 |
| P3a 框架独立构建与裸机构建 CI | F1 |
| P3a F407 C 板 bring-up | B |
| P3b profile 与 assembly 拆分、boot ABI、任务层改 FreeRTOS 原生 API | F2 |
| P3b 能力拆分与 msg v1 | F1（topic）、A（能力） |
| P3b 重建丢失业务、IMU 原始数据补录、≥2 个 profile | A、B |
| P3b 结构债（`ImuBodyMapping` 反向依赖等） | F3；其余随旧代码归档，在 A 中按新结构重做 |
| P4 vendor `foray_interfaces`、`app/link`、安全硬路径 | A |
| P4 UsbCdc 边界（Q8）、链接 overlay（Q9） | A（链路能力）；overlay 在第一个自定义段出现时 |
| P4 CI 全矩阵；TM4C、AM26x port | B 之后；TI 平台按需 |
| P5 MSPM0 与框架提取 | E |

## 当前 Agent State

`PLANNING`（P0–P2.5 已验收；架构 v2 与重建计划待负责人审阅，审阅后进入 F1 框架基础件；里程碑合并入 `main` 按负责人要求暂缓）。
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
- [ ] 框架验收（F4）：负责人确认框架稳定好用
- [ ] 至少 2 个 profile 构建成功（DM-MC02 兵种 profile + F407，阶段 A、B）；`platform/mock` 下 host 测试绿
- [ ] 框架独立构建在 CI 绿（F1）
- [ ] CI 架构门禁全绿：include/依赖方向、生成区、禁止 `file(GLOB)`、禁止手写协议、vendor 哈希
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
| 2026-10-26 | P0.5 | 新增 `tool/ci/naming.py` 与 CI `lint-naming`；`.clang-tidy` 微调（Decision-20261026-05）；C++ 标识符按 ROS 2 命名改名（`2b3fbf8`，自动 2190 处 + 手工修正）与改名后格式化（`0c51793`），均登记于 `.git-blame-ignore-revs`。legacy Debug/Release 与基线逐字节一致，符号地址/大小集合一致，ctest 13/13，命名检查 0 处 | P0.5 验收 |
| 2026-10-26 | P0.5 | 修复：宿主测试的编译期检查漏改 `GM6020_CURRENT`（旧名不存在导致检查空过），补正向对照 | 已修复 |
| 2026-10-26 | P1 | 根工程构建骨架（target 按目标结构组织，源码仍在 `legacy/`）；`symbols.py`、`arch_check.py`；CI 改用根工程（`firmware`、`lint-arch`）；target 名去掉项目前缀（Decision-20261026-06、-07）。根工程固件与迁移源符号集合一致（Debug 1426、Release 1038），ctest 14/14，依赖图检查通过 | P1 验收 |
| 2026-10-26 | P1 | CMake 文件按层合并为 7 个（负责人选定，Decision-20261026-07 第 11 点）；合并前后依赖图与编译命令逐字相同 | 已完成 |
| 2026-10-26 | P2 | 机械搬迁：323 个文件 `git mv`（R100）+ include/保护宏改写 + 构建与工具适配；GimbalModule 随 P2 搬；`legacy_include` 删除推迟到 P2.5（Decision-20261026-08）。符号与 P1 存档一致，ctest 14/14 | P2 验收 |
| 2026-10-26 | P2.5 | `src/lib` 子目录与命名空间去掉 `alg_` 前缀（负责人要求，Decision-20261026-09）；`symbols.py` 支持改名表，CI 按改名表换算 P1 存档。符号一致（Debug 1426、Release 1038），ctest 14/14 | 已完成 |
| 2026-10-26 | P2.5 | 搬迁 `legacy/` 剩余 16 个源码文件（R100）+ include/保护宏改写 + 构建与工具适配；PortKit 整体并入 port，拆分推迟到 P3a；删除 `legacy_include`、`legacy_library()`、`portkit`（X-11、X-12、X-15 关闭，Decision-20261026-10）。符号一致，搬迁文件的目标文件逐项相同，ctest 14/14 | P2.5 验收 |
| 2026-10-27 | v2 | 迁移方式改为按负责人骨架重建（Decision-20261027-01）：`docs/ai/architecture.md` 重写为 v2；原 P3a–P5 改为 F1–F4（框架）、A（能力）、B（F407）、E（提取） | 待审阅 |
| 2026-10-27 | F1 | 第 1 步：crc、math 移入 `src/utils`，fsm 移入 `src/middleware`，slope 并入 `lib/control`（Decision-20261027-02）。纯移动、改写、构建三个提交；ctest 14/14，固件符号与 P1 存档一致 | 已完成 |
| 2026-10-28 | F1 | template 声明单独一行（Decision-20261028-01）：配置、格式化、登记 blame 忽略三个提交 | 已完成 |
| 2026-10-29 | F1 | 第 2 步：crc16；`utils_concurrency`（`SpscQueue`、`LatestValue`）（Decision-20261029-01）。ctest 17/17，ThreadSanitizer 无报告，M0+/M4/M7 无原子读改写库调用，固件符号与 P1 存档一致。发现 seqlock 读写竞态在任务之间同样存在，登记为待定决策（Decision-20261029-02） | 已完成 |
