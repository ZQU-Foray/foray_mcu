# 开发计划（foray_mcu 结构重构）

> 当前状态：**P0–P2.5 已验收（2026-10-26）；2026-10-27 起按架构 v2 重建。F1 完成待验收；F2 第 1–5 步与 F3 的代码完成并已上板验证（2026-11-01），待负责人验收；F4 的示范与指南已交付**（Decision-20261027-01、Decision-20261101-01～03；里程碑合并入 `main` 暂缓）。2026-11-03 负责人指示开始阶段 A 的上位机链路（RoboLink v0.2，Decision-20261103-01），见下文"上位机链路"；同日负责人确认 F1–F4 验收通过。
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
- [x] 第 3 步 `src/hal/os` 契约定稿（2026-10-31，Decision-20261031-01～03）：时基 `now_us()`（1 µs、32 位，去掉 `now_ticks()`、`tick_frequency_hz()`）；`Signal`（新增，通知不丢、多次合并、每次 `wait()` 登记等待者、可常量初始化）；`CriticalSection` 只屏蔽框架中断；`platform/mock` 改为线程安全（临界区为进程内锁，`Signal` 用条件变量，超时按测试推进的时间，检查误用）；宿主测试 `hal_os_mock`、`signal`、`signal_misuse`
- [x] 第 4 步 `src/middleware/topic`（2026-10-31，Decision-20261029-02、Decision-20261031-05）：target `middleware_topic`（只有头文件）——`Topic`、`Publisher`（`claim_publisher()` 只成功一次）、`Subscriber`（`update`、`wait`、`enable_wait`、`lost_count`）、`TopicBase`（观测）；发布时用临界区包住写入（负责人选定方案 1），消息上限暂定 64 字节，每个 topic 4 个等待槽；`app/topics/README.md` 定清单约定，`app_topics` target 随第一个 topic（F2）建立
- [x] 宿主测试：utils 与 topic 的单元测试，含多线程压力（发布线程 × 订阅线程、模拟中断读、等待超时、单发布者检查）。utils 在第 2 步、topic 在第 4 步完成（`topic`，另有消息约束的配置期检查）
- [x] CI（2026-10-31，Decision-20261031-06）：新增作业 `framework`，在只含框架文件的副本里做框架独立构建（preset `framework`：`src` + `platform/mock`，无 RTOS 头，依赖图 `--mode framework`）；Cortex-M0+ 构建整个框架（preset `framework-m0plus`），检查不出现 `__atomic_*`、`__sync_*` 库调用（代替 `is_always_lock_free`，Decision-20261029-01），范围从 utils 与 topic 扩大到全框架，`driver_bmi088`、`driver_motor_dji`、`lib_estimate` 暂不编译（X-17，F3 解除）；`test/framework/template_check.cpp` 实例化只有头文件的模板；文本扫描加入规则 13（X-08 放行）、17
- [x] 验收：宿主测试全绿；框架独立构建绿；现有固件仍可构建（F1 只新增，不改旧代码的行为）。负责人 2026-11-03 确认通过

### F2 契约 v1 与 DM-MC02 板级（6–10 人日）

> 顺序（Decision-20261031-07）：先做与具体外设无关的第 1–4 步，每一步固件都可构建；第 5 步（外设）按外设纵向推进、与 F3 合起来排（Decision-20261101-01，负责人 2026-11-01 授权 AI 按最优方式推进）。旧 C port 定义了 HAL 回调，同一外设的新旧 port 不能并存。
> 本地分支（一个分支对应以后的一个 PR，依次叠在前一个上）：`f1/topic` → `f1/ci` → `f2/firmware-ci`（第 1 步 CI）→ `f2/tim2-timebase`（merge 合入）→ `f2/board-dir`（第 2 步）→ `f2/hal-os-port`（第 3 步）→ `f2/boot`（第 4 步：启动入口）→ `f2/bringup`（第 4 步：任务运行器与 bringup）→ `f2/archive`（第 4 步：归档旧 app）→ `f2/peripheral-plan`（第 5 步的决策）→ `f2/gpio-pwm` → `f2/can` → `f2/spi` → `f2/uart` → `f2/usb-cdc` → `f2/port-cleanup` → `f3/estimate-decouple` → `f2/bringup-peripherals`（外设检查、上板探测脚本）→ `f2/docs-sync`（文档同步、框架使用指南）→ `f2/board-vendor-dir`（板级目录移到厂商目录下）→ `f2/onboard-peripherals`（上板验证：修正 IMU 延迟的测量与探测脚本，验证记录）→ `f3/heater-warmup`（BMI088 加热改为模型预测，Decision-20261101-05）→ `f2/onboard-can`（补记 CAN 接收的上板验证）→ `f3/dm-protocol`（达妙协议层与寄存器回读排查）→ `f2/bringup-power`（bringup 上电打开 24 V 输出）→ `f3/dm-driver`（达妙电机驱动与电机组使能）→ `f2/bringup-dm`（bringup 的达妙电机检查与上板验证）→ `f3/motor-arch-proposal`（电机架构方案）→ `f3/can-rx-timestamp`（M1：CAN 帧带接收时刻）→ `f3/motor-benchmark`（M0：1 kHz 控制环基准）→ `f3/can-bus-joint`（M2：`CanBus` 与 `Joint`）→ `f3/dm-joint`（M3：达妙驱动改用新结构）→ `f3/stamp-order-fix`（时间戳先后的修正与 M3 复测）→ `f3/dji-joint`（M4：DJI 驱动改用新结构）→ `f3/actuator-safety`（M5：安全门）→ `f3/motor-cleanup`（M6：删除电机组、文档同步；M7 基准复测）。2026-11-03 已按这个顺序快进合入 `dev`（提交号不变），分支已删除，整理前的分支在 `foray_mcu_archive/2026-11-03_branch_cleanup/` 的 bundle 里（Decision-20261103-05）。

- [x] 第 1 步 CI（2026-10-31，Decision-20261031-08）：`firmware` 作业的符号对照退役，改为段大小报告（`symbols.py report`，写入作业摘要）；加入 HAL 回调符号检查（规则 16，`arch_check.py callbacks`）。规则 1（`board.hpp`）、4、7、10、18 的检查随对应内容出现时加入
- [x] 第 1 步（续）合入 `f2/tim2-timebase`（2026-10-31，在分支 `f2/board-dir` 用 merge 提交合入，保留提交号）：TIM2 作 1 MHz 时基计数源，CubeMX 6.18.1（Decision-20261031-04）
- [x] 第 2 步 板级目录（2026-10-31，Decision-20261031-09）：`platform/st/stm32h7/board_dm` → `platform/dm_mc02`（按板一级），226 个文件纯移动（R100），只改 4 处路径；移动前后固件符号一致、`.bin` 逐字节相同。负责人以后从 `platform/dm_mc02/cubemx/mc02_cpp.ioc` 打开 CubeMX
- [x] 第 2 步（续）板级目录改为"厂商 / 板"两级（2026-11-01，Decision-20261101-04，负责人要求）：`platform/dm_mc02` → `platform/st/dm_mc02`，228 个文件纯移动（R100）；移动前后固件符号一致、Debug 与 Release 的 `.bin` 逐字节相同
- [x] 第 3 步 hal/os 的 port 实现（2026-10-31，Decision-20261031-10）：`platform/dm_mc02/port/os/`（C++，现在在 `platform/st/dm_mc02`）——时基读 TIM2，调试暂停时冻结，计数器由 `port::os::start_clock()` 启动（第 4 步由 `hal::init()` 调用）；临界区用 `BASEPRI_MAX` 抬到优先级 5；`Signal` 用 1 号任务通知槽。CubeMX 6.18.1 没有 `configTASK_NOTIFICATION_ARRAY_ENTRIES`，改用板级 CMake 的编译宏统一设为 2，不改生成区。要到第 4 步的 bringup 才在板上运行
- [x] 第 4 步 启动与 bringup：
  - [x] 启动入口（2026-10-31，Decision-20261031-11）：`src/boot/boot.h`、`hal::init()`（port 实现：启动 TIM2 + 迁移期的 `Port_Init()`）；CubeMX USER CODE 区改为调用 `boot_*`（改动行登记在 -11）；迁移期的 `entry_legacy.cpp` 实现 `boot_*`，旧 app 照常运行；X-09 关闭；规则 7 的 include 检查；
  - [x] `app/common` 任务运行器（2026-10-31，Decision-20261031-12）：`common::TaskRunner<TASKS>`，FreeRTOS 原生 API，任务表在编译期检查，栈静态分配，`every_ms` / `on_topic` 两种触发，栈水位巡检；`TopicBase::add_waiter()` 公开；
  - [x] profile `bringup_dm_mc02` 与对应的 assembly（2026-10-31，Decision-20261031-12）：bringup 能力（`Heartbeat` 1 kHz 发布、`Monitor` 按 topic 触发并统计），第一个 topic `bringup.heartbeat`，preset 改为 bringup；已上板验证（收到全部心跳，丢失 0，最大延迟 9 µs）。LED/蜂鸣器随第 5 步的 gpio/pwm 加入；"中断里发布"的验收待定中断源；
  - [x] 归档旧能力代码（2026-10-31，Decision-20261031-13）：tag `archive/pre-rebuild`（`9c11546`，可用 `-DPROFILE=baseline` 构建旧固件）；删除 `app/{imu,chassis,gimbal,remote,tasks}`、旧 TaskManager、`entry_legacy.cpp`、profile `baseline`、底盘集成测试与 `bmi088_acquisition` 中测旧 app 的两段；`app/` 加入宿主构建；X-04、X-10 关闭，X-08、X-14 缩小；规则 10、18 生效。bringup 固件归档前后 `.bin` 逐字节相同；能力重建时从 tag 取参考
- [x] 第 5 步 外设（2026-11-01，Decision-20261101-01）：`src/hal` 契约 v1 用句柄类（`hal::Gpio`、`Pwm`、`Can`、`Spi`、`Uart`、`UsbCdc`，`hal::Callback`，时间用 µs）；`platform/dm_mc02/board/board.hpp`（句柄、外设个数、能力常量，只给组合根与 port）；port 逐个外设用 C++ 重写，PortKit 拆分（队列 → `SpscQueue`，临界区 → `hal::os::CriticalSection`，通知 → `port::Notifier`）；`platform/mock` 实现全部外设（`mock_hal.hpp`）；驱动随外设改用（buzzer、BMI088 加热 → pwm；DJI 电机 → can，X-08 关闭；BMI088、WS2812 → spi，X-14 关闭）；删除 `src/hal/detail`、`hal/time.hpp`、DWT 时基、`Init.h`（X-07 关闭）；规则 1（`board.hpp`）、16（C 链接位置）的文本扫描生效。分支 `f2/peripheral-plan` → `f2/port-cleanup`
- [x] bringup 外设检查（2026-11-01，Decision-20261101-03）：IMU（数据就绪与 SPI 完成中断里发布 `bringup.imu_event`，按它触发的任务读样本、VQF 姿态、加热）、CAN（只收）、SBUS、USB 报告、指示；任务运行器的 `on_topic` 支持超时；规则 4 的检查生效；上板探测脚本 `tool/debug/bringup_probe.py`
- [x] 验收：bringup 固件在板上启动，任务按表运行，中断发布的 topic 能被阻塞等待的任务收到；宿主测试全绿（宿主 27/27 已绿；2026-11-01 按[检查清单](docs/ai/decisions/2026-11-01-F2-外设上板检查清单.md)上板验证，探测 20/20：中断里发布的 `bringup.imu_event` 叫醒 IMU 任务每秒约 6800 次，等待超时 0，最大延迟 73 µs，见[验证记录](docs/ai/decisions/2026-11-01-F2-外设上板验证/README.md)；负责人 2026-11-03 确认通过）

### F3 驱动适配（4–7 人日）

- [x] 驱动改用 hal v1，在构造时接收句柄（2026-11-01，随 F2 第 5 步）：bmi088（陀螺带宽改为 `Bmi088::Config`，X-14 关闭；`bmi088_acquisition` 改用 mock；时间改用 `hal::os::now_us()`，`src/hal/time.hpp` 已删除）、motor/dji（去掉快照的手写序列锁，X-08 关闭；`claim_receiver()` 取代全局占用表）、buzzer、ws2812；protocol/{dji,sbus} 不用 hal，不变；单元测试都用 `platform/mock`
- [x] SPI 异步按能力分级使用：BMI088 编译期要求（`REQUIRES_SPI_ASYNC`，组合根 `static_assert`），WS2812 运行期回退阻塞发送
- [x] `src/lib/estimate` 去掉对 BMI088 驱动的依赖（2026-11-01，Decision-20261101-02，X-17 关闭）；框架与能力开启 `-Wdouble-promotion`（`vqf.cpp` 豁免）。FPU 检查推迟到阶段 B（第一块没有双精度 FPU 的板）
- [x] bringup 固件加入各驱动的检查（Decision-20261101-03）：CAN 接收、SPI 读 BMI088、UART 收 SBUS、PWM（蜂鸣器、加热）、GPIO 外部中断、SPI 发送（WS2812）、USB CDC 回显与状态行、时基；CAN 只收不发，电机反馈要等阶段 A 的电机能力发控制帧
- [x] BMI088 加热改为模型预测（2026-11-01，Decision-20261101-05，负责人要求提前做丢失功能 L6）：`control::HeaterControl`（两级预热 + 平台校正加热器强度 + 前馈 PI 恒温），`Bmi088Heater` 不再用 PID，DM-MC02 的热模型由实测辨识、写在 profile；上板从 39.5 ℃ 起过冲 +0.125 ℃、15.3 s 进入并保持 ±0.25 ℃（旧 PI +6.875 ℃、28 s；原固件 +0.375 ℃、111 s），见[验证记录](docs/ai/decisions/2026-11-01-L6-加热提速上板验证/README.md)。分支 `f3/heater-warmup`
- [x] 达妙电机（负责人 2026-11-01 要求开始，DM-J4310）：
  - [x] 寄存器回读的上板排查（2026-11-01）：能读出，应答发到 MST_ID（默认 0x000）；控制模式 MIT、量程 12.5/30/10；之后负责人把 MST_ID 改为 0x11（约定 ESC_ID + 0x10），回读确认，见[记录](docs/ai/decisions/2026-11-01-达妙电机寄存器回读/README.md)
  - [x] 协议层 `protocol::DmEsc`（2026-11-01）：四种模式的控制帧与帧 ID、系统命令、寄存器读写与存储、应答与反馈解码，量程作参数；宿主测试 `dm_protocol`
  - [x] 驱动 `device::DmMotor`（2026-11-02，Decision-20261102-01）：启动时逐台读模式 → 失能 → 读量程 → 把通讯超时写进电机 RAM，与型号档案核对一致才允许使能；使能先清错、按反馈确认；故障、电机复位、离线时撤销出力；MIT 零 Kp/Kd 的力矩控制接入 `MotorGroup`（`MotorModel::DM_J4310`）。电机组契约加入 `enable()`/`disable()`，DJI 适配器也要使能才出力。宿主测试 `dm_motor`、`dm_adapter`（模拟电机按说明书帧格式应答），ctest 31/31。分支 `f3/dm-driver`；2026-11-02 已上板验证（见下一项）。电机组与适配器后来被电机架构的关节 `Joint` 取代（下面"电机架构"一项）
  - [x] bringup 检查 `bringup::DmCheck`（2026-11-02，Decision-20261102-02）：核对 → 失能保活 → 使能 → 零力矩 → 转动（1.0 N·m、Kd 1.0，≤ 1 rad/s）→ 刹车 → 停发 300 ms（通讯超时保护）→ 清错再使能 → 失能，取代 CAN1 的只收检查。上板 4 轮：核对、使能、通讯超时保护、失能保活、CAN1 收发每轮都通过；转动用 0.3 N·m 时被减速箱摩擦（约 0.2 N·m）卡住，改为 1.0 N·m 后两轮都通过。见[验证记录](docs/ai/decisions/2026-11-02-达妙电机驱动上板验证/README.md)。分支 `f2/bringup-dm`
- [x] 电机架构（负责人 2026-11-02 提出，Decision-20261102-03，D1–D9 按推荐通过）：能力只用关节 `Joint`，品牌驱动挂在每条总线的 `CanBus` 上，型号做成数据；控制环不跨任务，装配时检查总线负载，用上板基准验收性能。按[方案](docs/ai/decisions/2026-11-02-电机架构方案/README.md)第 7 节分步实现（M1 → M0 → M2–M7，约 6 人日），阶段 A 之前完成
  - [x] M1 `hal::Can::Frame` 带接收时刻 `received_us`（Decision-20261102-04）：port 在中断里记、mock 按模拟时间记；DJI、达妙按接收时刻判在线
  - [x] M0 bringup 达妙检查加 1 kHz 控制环基准（重构前）：单轮耗时、周期抖动、反馈 → 下发延迟，Debug 与 Release 各测一次，见[基准记录](docs/ai/decisions/2026-11-02-电机控制环基准/README.md)
  - [x] M2 `device::CanBus`（每条总线唯一的接收方，按帧 ID 范围分发，登记时查重叠，`can_load_percent()`）与关节 `device::Joint`（SI 单位、机构、使能、指令超时、`set_inhibited()`）
  - [x] M3 达妙驱动登记到 `CanBus`、经 `Joint` 控制（力矩与电机内阻抗），型号 `device::dm::DM_J4310`；bringup 达妙检查改用新接口，组合根 `static_assert` 检查 CAN1 负载。上板复测发现时间戳先后的 bug（帧的接收时刻晚于本轮 `now`，无符号差回绕），改为有符号差后整套检查与基准通过：反馈 → 下发延迟 1753 → 753 µs
  - [x] M4 DJI 驱动登记到 `CanBus`、经 `Joint` 控制，同一总线混接 C610/C620/GM6020（电流模式），反馈 ID 与控制帧格的冲突在 `init()` 拒绝；控制帧 ID 也登记，同一总线第二个 `DjiMotor` 被拒绝；GM6020 给绝对位置。只有宿主测试，上板要等 CAN2 接上 GM6020
  - [x] M5 安全门 `actuator::SafetyGate` 与急停 topic `safety.stop`：上电锁定，急停或超时锁定，人工确认（`arm_sequence` 变化）才放行；bringup 不接，阶段 A 的产品 profile 再接
  - [x] M6 删除电机组 `MotorGroup`、两个适配器与对应测试；同步框架使用指南、architecture（v2.12、§3.8、X-05）、电机 README、tree、宿主测试说明。ctest 31/31
  - [x] M7 上板基准复测（2026-11-02）：最终固件与 M3 的逐字节相同，改为 Release 交替重测（最终固件 3 轮、M0 固件 2 轮）。延迟 1753 → 753 µs，抖动不劣于 M0；单轮平均耗时稳定多 1.2 µs（11.2 → 12.4 µs，占 1 ms 的 0.12%），最长耗时不变。这一项按判据字面不算"不劣于"，交负责人判断（AI 建议接受），见[基准记录](docs/ai/decisions/2026-11-02-电机控制环基准/README.md)
- [x] bringup 上电打开板上两路 24 V 输出（2026-11-01，Decision-20261101-06，负责人要求）：`bringup::PowerOutputs`，接电机检查时不必再经 SWD 打开
- [x] 验收：驱动单元测试全绿（已绿）；bringup 固件上各驱动的检查通过（2026-11-01 上板：SPI 读 BMI088、GPIO 外部中断、PWM 加热闭环、SPI 发送 WS2812、USB CDC、UART 收 SBUS（含失控保护）、CAN 接收（GM6020 反馈 1 kHz）、时基通过；见[验证记录](docs/ai/decisions/2026-11-01-F2-外设上板验证/README.md)。2026-11-02 上板：CAN 发送与达妙电机驱动通过，见[达妙验证记录](docs/ai/decisions/2026-11-02-达妙电机驱动上板验证/README.md)；负责人 2026-11-03 确认通过）

### F4 框架验证（2–4 人日）

- [ ] 实机检查清单（负责人执行）：启动、任务、topic（发布次数、丢失计数、等待延迟）、各外设
- [x] 用框架写一个最小示范：IMU 驱动 → 姿态 topic → USB 输出（2026-11-01，`bringup::ImuCheck` → `bringup.imu` → `bringup::UsbReport`，Decision-20261101-03）
- [x] 指南：怎样新增外设接口、驱动、topic、能力、profile（2026-11-01，AI 撰写，放 [docs/ai/guide/框架使用指南.md](docs/ai/guide/框架使用指南.md)）
- [x] 验收：负责人确认框架"稳定好用"，再进入阶段 A。负责人 2026-11-03 确认 F1–F4 验收通过

### A 能力重建（F4 验收后细化）

- 在框架上逐个重建能力：imu、chassis、gimbal、remote、launch、pc_link（RoboLink v0.2，编解码只在 `src/middleware/robolink`，Decision-20261103-01）、加热；补回丢失功能 L1–L6（[丢失功能与恢复依据](docs/ai/decisions/2026-10-26-丢失功能与恢复依据.md)）
- 第一个兵种 profile（DM-MC02）；IMU 原始数据补录到 `test/data/imu/`，姿态解算用录制数据回归

#### 上位机链路 RoboLink v0.2（负责人 2026-11-02 提出、2026-11-03 指示开始，Decision-20261103-01）

> 目标：把 IMU 解算的姿态经 RoboLink 发给上位机，辅助视觉做手眼标定；控制快照第一版只记账、不执行。本地分支接在 `f3/motor-cleanup` 之后：`a/robolink-wire` → `a/robolink-session` → `a/imu-capability` → `a/pc-link` → `a/link-profile` → `a/heater-warm-start`（加热热启动修正、HELLO 版本说明、`foray_interfaces` 改动备好）→ `a/usb-seam`（X-01 关闭，之后是 VQF 调参与仓库整理的提交）。2026-11-03 已快进合入 `dev`，分支已删除（Decision-20261103-05）。

- [x] 第 1 步 编解码（`a/robolink-wire`）：规则 11 改写；标准帧生成器 `tool/robolink/make_golden.py`（37 个向量）与 PC 端对账工具；`middleware_robolink` 的帧层、槽位层与 8 条消息；宿主测试 `robolink_wire`、`robolink_messages`；与 PC 端 `foray_platform` 对账一致，差异 1 处（PC 端不接受 §7.1 的末尾追加字段），见[对账记录](docs/ai/decisions/2026-11-03-RoboLink-对账/README.md)
- [x] 第 2 步 连接会话（`a/robolink-session`）：`robolink::Session` 的握手、判活、序号、发送调度、计数；宿主测试 `robolink_session` 扮演 PC，故意改坏 13 处都被抓到；与 PC 端链路状态机在伪终端上联调通过（握手 2 ms，1 kHz 姿态与控制快照，两个方向的卡死与恢复），见[对账记录](docs/ai/decisions/2026-11-03-RoboLink-对账/README.md) §4
- [x] 第 3 步 IMU 能力（`a/imu-capability`）：`app/imu`（由 bringup 的 `ImuCheck` 改写），云台坐标系、上电零偏标定 `estimate::GyroBiasCalibration`（由归档的旧实现改写，时间改 µs、累加改 float）与 SETTLING/READY/DEGRADED、yaw 归零，`imu.event`、`imu.attitude`；宿主测试 `gyro_bias_calibration`、`imu_ready`/`imu_degraded`/`imu_reject`，故意改坏 14 处都被抓到
- [x] 第 4 步 链路能力（`a/pc-link`）：`app/pc_link`，USB 收发、`imu.attitude` → GIMBAL_ATTITUDE（无效时清零）、控制快照发布到 `pc_link.control`；宿主测试 `pc_link`，故意改坏 8 处都被抓到
- [x] 第 5 步 固件与上板（`a/link-profile`）：profile `link_dm_mc02`、组合根、PC 端探测脚本 `link_probe.py` 与 `pc_board_check.cpp`，CI 加构建 link 固件。2026-11-03 上板：姿态限频改按采样时刻（860 → 998 Hz，`fa3e0d3`）；冷启动 42 s 进入 READY；PC 端联调：握手 1 ms，60 s 无 CRC/帧头/槽位错误、帧序号连续，姿态 996 Hz、停留中位 52 µs，状态 200 Hz，控制快照序号回报正确，拔线重连通过；视觉组的 `RoboLinkHostLink` 直接连板子 20 s 不断链。见[对账记录](docs/ai/decisions/2026-11-03-RoboLink-对账/README.md) §5。发现 L6 加热控制器热启动会卡在 48 ℃，另行修复
- [x] USB 生成桩（例外 X-01）按 Q8 方案①归入板级实现、X-01 关闭，X-02（`--wrap`）改为长期例外（2026-11-03，Decision-20261103-04）：只改文档与注释，固件逐字节不变。原计划的"改为回调注册"核对后不成立（`--wrap` 管的是 DMA 访问不到 DTCM，与回调无关）
- [x] `foray_interfaces` 的改动备好（2026-11-03，Decision-20261103-01 第 12 点）：RoboLink v0.2 定稿、握手由 PC 发起、§7.1 接收方只查最小长度、标准帧生成器与 37 个标准帧入契约仓；3 个提交的补丁与 PR 说明、给 `foray_platform` 的 §7.1 问题描述在 `foray_mcu_archive/2026-11-03_foray_interfaces_pr/`，待负责人提交
- [x] L6 加热控制器热启动卡在目标下方（2026-11-03 上板发现，Decision-20261103-02，分支 `a/heater-warm-start`）：控制器把第一次读数当环境温度，热着复位时估高，恒温时低于目标 2 ℃ 立即重新预热又清积分，到不了目标。修正：读数比"加热器一直关着"的预测还低时下调环境估计；恒温积分长期偏大时转成环境修正；低于目标 2–4 ℃ 要持续 2 s 且读数不升才重新预热。仿真 768 个场景全部收敛（原来 242 个到不了目标），原有测试照旧通过；按负责人"行为不异常就行"，宿主测试加 4 个热启动场景与 132 个场景的宽松扫描（稳到目标、过冲 ≤ 2.5 ℃），故意改坏 5 处都被抓到；上板恒温 50 ℃ 时直接复位，21 s 回到 50 ℃、IMU 进入 READY（[记录](docs/ai/decisions/2026-11-03-加热热启动上板验证/README.md)）
- [x] yaw 漂移修正（2026-11-03，Decision-20261103-03）：恢复旧 `AttitudeEstimator` 的 VQF 板级参数（静止门限 0.35 rad/s、bias clip 0.05、标定不重置协方差），`Imu::Report` 补 bias/rest 诊断；宿主测试 39/39、link 固件构建通过；2026-11-03 上板 5 min 实测 yaw -0.05°（-0.010 °/min），rest_detected=1
- [x] 仓库整理（2026-11-03，Decision-20261103-05，负责人要求）：工作区按类提交；本地分支链快进合入 `dev`（提交号不变）；删除 GitHub 上 49 个与本地 50 个功能分支，丢弃已存为补丁的 stash；整理前的全部引用打包存档。以后每步一个分支，验证后快进合入 `dev` 即删
- [x] 代码注释删掉决策号（2026-11-03，Decision-20261103-06，负责人要求）：先是头文件（55 个文件 77 处），再是其余代码与配置文件（76 个），文档不动；只改注释，四个固件逐字节相同；以后代码与配置文件的注释不写决策号（AGENTS §1）
#### 输入与仲裁（遥控器为中心；Decision-20261105-03）

> 目标：功能的开关由遥控器决定。第一个功能是自瞄——大范围移动靠操作手，按住开火时自瞄才接管（有接管范围），
> 松开开火回到手动、从当前位置接着走。第一部分只用 SBUS 遥控；第二部分（图传、键鼠、裁判系统）等硬件到位再做。

- [x] 第 0 步 登记设计（`docs/input-design`）：decision.md 的 Decision-20261105-03（Proposed）、plan 本节、云台第 7 步改为指向这里
- [x] 第 1 步 remote 加发 `remote.input`（`feat/remote-input`）：`topics/operator_input.hpp`（统一的输入类型）、`topics/remote.hpp`、摇杆与拨杆的标定参数；`safety.stop` 先留着；宿主测试 `remote_input`；上板核对摇杆与拨杆读数待负责人做
- [x] 第 2 步 仲裁能力（`feat/command-arbiter`）：`app/command`（绑定、功能、`safety.stop` 与 `command.gimbal`）、`topics/command.hpp`；`safety.stop` 从 remote 搬过来；组合根把 `command_step` 放进 gimbal 任务、排在云台之前；白名单加 `command`；宿主测试 `arbiter_*` 12 个场景（Decision-20261105-03）；上板核对急停与放开待负责人做
- [x] 第 3 步 云台改订 `command.gimbal`（`feat/gimbal-command`）：手动 `RATE` 与目标转动速度上限、发布 `gimbal.state`、删掉 PC 目标的判断；仲裁加自瞄接管范围；宿主测试 `gimbal_*` 21 个场景（含手动 RATE、限位写回、目标速度上限、`gimbal.state`）与 `arbiter_engage_*` 3 个场景；`link_probe.py` 加 `--gimbal-yaw`/`--gimbal-pitch`/`--sweep`；上板核对手动瞄准与按住开火自瞄待负责人做
- [x] 第 4 步 收尾（`docs/input`）：decision 的实现与验证、architecture 白名单与安全那一条、tree、topics README、test README、丢失功能 L5
- 第二部分（图传与键鼠、裁判系统）等硬件到位：远程输入 `referee.operator_input`、来源切换规则、`command.chassis` / `command.shooter`

#### 控制律框架（LQR + ESO；Decision-20261106-01）

> 目标：做一套通用的控制律零件——线性 ESO（`src/lib/estimate`）、状态反馈（`src/lib/control`），以及电脑上的设计工具
> （模型 + Q、R → K）与辨识工具（录到的数据 → J、摩擦、重力、延迟）。云台先用 PID 点亮、上板时顺便录辨识数据，
> 辨识出模型后把 `AxisLoop` 换成 LQR + ESO（第二部分）；以后的底盘轮速、摩擦轮、大 yaw、轮腿都用同一套零件。
> 第一部分不上板；上板调参与取数改用 VS Code 插件 EmberProbe（经 SWD 不停核）。每步一个分支，验证完快进合入 `dev`、分支即删。

- [x] 第 0 步 登记设计（`docs/control-framework`）：Decision-20261106-01/02/03（Proposed）、plan 本节
- [x] 第 1 步 线性 ESO（`feat/linear-eso`）：`src/lib/estimate/eso.{hpp,cpp}`（类 `estimate::Eso<Order>`，1–2 阶、离散当前估计形式、按 ω₀ 配置增益），删掉没人用的 `extended_state_observer.hpp`；宿主测试 `eso`（增益即极点、收敛、b₀ 偏差、复位与断档、参数检查），故意改坏 5 处被抓到
- [x] 第 2 步 状态反馈（`feat/state-feedback`）：`src/lib/control/state_feedback.hpp`（`u = −K·(x − x_ref)`，K 作为数据）；宿主测试 `state_feedback`（手算、与朴素矩阵乘法对账、符号、无效参数、离散闭环）
- [x] 第 3 步 设计工具（`feat/lqr-design-tool`）：`tool/control/`（`models.py`、`lqr_design.py`、`README.md`、`requirements.txt`）；`--selftest` 复现设计表的四行；ruff/black
- [x] 第 4 步 链接脚本 overlay（`feat/axi-overlay`）：`platform/st/dm_mc02/board/overlay.ld`（`.axi_bss`，第二个 `-T`、不用 `INSERT`，例外 X-03 关闭）、board 的 CMake、architecture/AGENTS/tree；六个固件链接与超容量的反向验证
- [x] 第 4b 步 Release 带调试信息（`build/release-debug-info`）：`-Os -g0` → `-Os -g3`，三个 Release 固件的 `.bin` SHA-256 逐字节不变
- [x] 第 5 步 云台的调试接口（`feat/gimbal-debug-interface`）：录制（`CaptureSample`、`Capture`、组合根里放 `.axi_bss` 的存储）与在线调参（`Tuning`、`AxisLoop::retune()`，约定加两条）；宿主测试 `gimbal_capture`/`gimbal_capture_off`/`gimbal_tuning`/`gimbal_tuning_reject`；`link_probe.py --sweep-axis`；故意改坏 4 处被抓到
- [x] 第 6 步 辨识工具（`feat/axis-identify-tool`）：`tool/control/identify_axis.py`（回归式、Savitzky–Golay 求导、延迟搜索、`--simulate` 自检）；两个轴的 `--simulate` 通过
- 上板：指引 1 第 6 步 PID 点亮，用 EmberProbe 在线调参，**顺便按 §11 录辨识数据**（需要第 4、4b、5 步）
- 第二部分（录到数据以后）：第 7 步辨识结果与记录（`docs/gimbal-identification`）；第 8 步 `AxisLoop` 改写成 LQR + ESO + 前馈、仿真场景加噪声与正弦跟踪（`feat/gimbal-lqr-eso`）；第 9 步上板调 ω_n、ω₀、k_c（`params/gimbal-lqr-eso`）；第 10 步收尾（`docs/control-framework-close`）
- 第三部分（角加速度前馈，要算法组配合）：第 11 步 `foray_interfaces` 的协议（控制快照末尾追加 yaw、pitch 的角加速度）；第 12 步本仓接通（`feat/robolink-acceleration`：生成器与标准帧 → 编解码 → topic → 仲裁 → 云台）；第 13 步上板对比开、关前馈（`params/gimbal-acceleration-ff`）

#### 云台（两台 DM-J4310，小 yaw + pitch；Decision-20261105-01）

> 目标：按 IMU 稳住指向（世界系），能跟踪上位机的目标；顺便把框架从能力、profile、组合根到调参走一遍。
> 大 yaw 以后作为可选的第三个关节加进来；第 3 步的遥控只是 `safety.stop` 的临时来源，仲裁能力做好后改成发布 `remote.input`。
> 每步一个分支，验证完快进合入 `dev`、分支即删（Decision-20261103-05）。

- [x] 第 0 步 登记设计（`docs/gimbal-design`）：decision.md 的 Decision-20261105-01（Proposed）、plan 本节、tree 的 `app/gimbal`、`app/remote` 与 `infantry_dm_mc02`
- [x] 第 1 步 电源输出移出 bringup（`refactor/power-outputs`）：`PowerOutputs` 移到 `app/actuator`，bringup profile 的 `CAPABILITIES` 加 `actuator`；自证纯搬迁：bringup Debug 固件的符号集合只有 `bringup::PowerOutputs::init` → `actuator::PowerOutputs::init` 一处改名、大小同为 164 B，FLASH 同为 239,384 B；宿主测试 `power_outputs` 从 io_checks 拆出
- [x] 第 2 步 新 profile 与只读骨架（`feat/gimbal-profile`）：兵种定名步兵，`infantry_dm_mc02` profile 与组合根接两台达妙电机（ESC 1/2、MST 0x11/0x12，CAN1），`app/gimbal` 只读骨架填 `Report`（姿态、关节角、核对/在线/故障）；`config` 的 profile 增加 `DRIVERS`（组合根直接构造、能力不链接的驱动）；Debug 固件构建通过。**上板核对（两台电机的核对/方向/零点）待负责人做**
- [x] 第 3 步 遥控最小版（`feat/remote-safety`）：`app/remote`（SBUS → `safety.stop`，每步都发布、运行档 + 帧新鲜 + 无 failsafe、`arm_sequence` 只在拨上时加 1），安全门与遥控接进 `infantry_dm_mc02` 的任务表（收 → 云台 → 安全门 → 发）；宿主测试 `remote_run`/`remote_failsafe`/`remote_timeout`/`remote_reject`。**上板核对拨杆与超时待负责人做**
- [x] 第 4 步 单轴串级控制（`feat/gimbal-loop`）：`math::wrap_pi()`（宿主测试 `basic_math`）、`AxisLoop`（角度环 → 角速度环、`apply()` 的积分抗饱和、`compensation_newton_meter` 诊断）；宿主测试 `axis_loop` 带被控对象仿真（阶跃 0.3 rad 在 1 s 内收敛、稳态误差 < 0.005 rad；重力前馈；饱和与积分限幅；总限幅释放后无大超调；reset；非有限输入）
- [x] 第 5 步 状态机与出力（`feat/gimbal-output`）：DISABLED / ENABLING / HOLD / TRACK、两轴同进同出（`enable_all` / `disable_all` / 任一关节掉了）、yaw 回绕、限位按底座倾斜换算（限位总宽不超过 2π）、pitch 重力前馈 τg·cos(pitch)、总限幅、`Report` 的诊断；宿主测试 `gimbal_*` 17 个场景（两个真实达妙电机 + CanBus + DmMotor + 安全门，IMU 与 PC 目标由测试发布）；故意改坏 17 处都被抓到。之后按上板前的复查补了三处：目标被限位钳住的轴不加角速度前馈、采样时刻晚于本轮 `now` 不算过期（新鲜度按收到时刻算）、没有新姿态的那一轮不重复积分（重发上一拍力矩）；PC 的急停请求（flags bit3）只当目标无效
- [x] 控制律接口按评审调整（Decision-20261105-02）：`AxisLoop::Parameters`（控制律参数不透明）、`update(Measurement, Reference, dt)`、连续 yaw（`Report` 加 `yaw_continuous_rad`，复位时重新锚定；记圈数实现 `math::AngleUnwrapper`，带几十万拍与 double 真值对照的精度测试）、`reset()` 语义；测试拆成与控制器无关的 `axis_law_scenarios.hpp` 与 PID 特有的 `axis_loop_test.cpp`。PID 下行为等价、参数数值不变；通用场景不出现 `Gains`/`kp`/`integral`/`rate_target`；去掉 `apply()` 的回退、或把测量改回回绕的 yaw，通用场景与 `gimbal_yaw_cross_pi` 都会失败。**目标转动速度上限移到 `Gimbal` 不在其中**，与操作输入/仲裁那套的"按开火吸目标"一起做
- [x] 第 6 步 上板调参（`params/gimbal-tuning`）：参数、`tool/debug/gimbal_probe.py`（读模式、目标、力矩、补偿力矩、安全门、遥控、任务统计）与[调参清单](docs/ai/guide/云台上板调参清单.md)已备好，起步值写在 `params.hpp`（yaw 先不出力、pitch 角速度环 kp 0.05、总限幅 1 N·m）；**先 pitch 后 yaw、先角速度环后角度环，结果待负责人在板上做并写进 `docs/records/`**；两个轴的 --simulate 通过
- [x] 第 7 步 跟踪 PC 目标：由"输入与仲裁"第 3 步取代（云台改订 `command.gimbal`），`link_probe.py` 的扩展也在那里做；自检与 2 处改坏验证完成
- [ ] 第 8 步 收尾（`docs/gimbal`）：Decision-20261105-01 改 Accepted、tree、丢失功能 L4/L1/L5、CI 加 profile、architecture
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

`EXECUTING`（P0–P2.5 已验收；F1–F4 负责人 2026-11-03 确认验收通过；阶段 A 的上位机链路第一版已上板联调通过；PR 暂不开，每步验证后快进合入 `dev`、分支即删（Decision-20261103-05）；里程碑合并入 `main` 等负责人定）。
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
- [ ] CI 架构门禁全绿：include/依赖方向、生成区、禁止 `file(GLOB)`、链路编解码只在一处（规则 11）、vendor 哈希
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
| 2026-10-31 | F1 | 第 3 步：hal/os 契约 v1——1 µs 时基、`Signal`、临界区只屏蔽框架中断（Decision-20261031-01～03）；`platform/mock` 改为线程安全。ctest 21/21，ThreadSanitizer 无报告，故意改坏 mock 的四处都被测试抓到，M0+/M4/M7 上全局 `Signal` 常量初始化，固件符号与 P1 存档一致 | 已完成 |
| 2026-10-31 | F2 准备 | DM-MC02 的 `.ioc` 启用 TIM2 作 1 MHz 时基计数源，CubeMX 改用 6.18.1（Decision-20261031-04）。格式变化与 TIM2 分两次提交，在分支 `f2/tim2-timebase`，符号对照退役后合入 | 已合入（`f2/board-dir` 的 merge 提交） |
| 2026-10-31 | F1 | 第 4 步：`middleware_topic`——发布时用临界区包住写入（Decision-20261029-02 定为方案 1）、单发布者、`update`/`wait`、等待槽、观测计数（Decision-20261031-05）；`app/topics` 清单约定。ctest 22/22，ThreadSanitizer 无报告，故意改坏的十处都被测试抓到，M0+/M4/M7 无原子读改写库调用、全局对象常量初始化，固件符号与 P1 存档一致 | 已完成 |
| 2026-10-31 | F1 | CI：新增 `framework` 作业——只含框架文件的副本里做框架独立构建与依赖图检查；Cortex-M0+ 构建整个框架并检查原子库调用（X-17 的三个 target 暂不编译）；文本扫描加入规则 13、17（Decision-20261031-06）。反向验证七类违规都被拦下；ctest 22/22，固件符号与 P1 存档一致，命名检查 0 处 | 已完成，F1 待验收 |
| 2026-10-31 | F2 | F2 顺序调整（Decision-20261031-07）：先做与外设无关的第 1–4 步，第 5 步待定。第 1 步：固件 CI 的符号对照退役，改为段大小报告；加入 HAL 回调符号检查（Decision-20261031-08）。报告与链接器用量一致，回调检查反向验证两种失效都被抓到 | 第 1 步完成 |
| 2026-10-31 | F2 | 合入 TIM2 分支（merge 提交，符号变化与 Decision-20261031-04 一致）；第 2 步：板级目录改为 `platform/dm_mc02`（Decision-20261031-09），移动前后固件符号一致、`.bin` 逐字节相同 | 第 2 步完成 |
| 2026-10-31 | F2 | 第 3 步：hal/os 在 DM-MC02 上的实现（Decision-20261031-10）。16 个 include FreeRTOS 头的编译单元宏一致；反汇编核对临界区、时基、通知槽；`StaticTask_t` 与内核一致（92 → 96 字节）；板上运行待第 4 步 | 第 3 步完成（待上板） |
| 2026-10-31 | F2 | 第 4 步（启动入口）：生成区只调 `boot_*`，`hal::init()` 由 port 实现并启动 TIM2，X-09 关闭（Decision-20261031-11）。固件只有 `Application_*` 换成 `boot_*` 与新增的 `hal::init`、`start_clock`、`HAL_TIM_Base_Start`；旧 `main.c` 的两处 include 会被规则 7 拦下 | 启动入口完成；已上板确认旧固件照常运行（烧录前备份的板上固件与 10-26 存档相同） |
| 2026-10-31 | F2 | 第 4 步（任务运行器与 bringup）：`common::TaskRunner<TASKS>`、bringup 能力与 profile、第一个 topic（Decision-20261031-12）。ctest 23/23；任务表写错的 5 种情况编译期被拒绝；上板：心跳 1 kHz，monitor 收到全部心跳，丢失与乱序 0，最大延迟 9 µs | 完成；中断发布的验收待定，归档旧 app 待做 |
| 2026-10-31 | F2 | 第 4 步（归档旧 app）：tag `archive/pre-rebuild`，删除旧能力代码、`baseline` 与依赖它们的测试，`app/` 加入宿主构建，X-04、X-10 关闭，规则 10、18 生效（Decision-20261031-13）。bringup 固件归档前后 `.bin` 逐字节相同；ctest 17/17；从 tag 能构建旧 `baseline` 固件 | 第 4 步完成 |
| 2026-11-01 | F2 | 第 5 步的做法（Decision-20261101-01，负责人授权 AI 按最优方式推进）：hal 契约 v1 用句柄类，按外设纵向推进、与 F3 合排 | 已登记 |
| 2026-11-01 | F2 | 第 5 步：gpio、pwm、can、spi、uart、usb_cdc 的契约 v1、C++ port、mock 与驱动改用；删除 `hal/detail`、PortKit、DWT 时基；X-07、X-08、X-14 关闭；规则 1、16 的文本扫描生效。每个外设一个分支，宿主测试 19 → 25 项 | 第 5 步完成，待上板 |
| 2026-11-01 | F3 | `lib/estimate` 去掉对 BMI088 驱动的依赖，X-17 关闭，Cortex-M0+ 构建全部框架；`-Wdouble-promotion` 生效（Decision-20261101-02） | 完成 |
| 2026-11-01 | F2–F4 | bringup 外设检查与 IMU 最小示范、`on_topic` 超时、规则 4 的检查（Decision-20261101-03）；上板探测脚本与检查清单；ctest 27/27 | 完成，待上板 |
| 2026-11-01 | F2 | 板级目录改为"厂商 / 板"两级：`platform/st/dm_mc02`（负责人要求，Decision-20261101-04）。纯移动 + 路径改写两个提交；移动前后符号一致、`.bin` 逐字节相同 | 完成 |
| 2026-11-01 | F2、F3 | 上板验证（[记录](docs/ai/decisions/2026-11-01-F2-外设上板验证/README.md)，分支 `f2/onboard-peripherals`）：首轮 17/20，三项是测量问题——探测脚本按名义间隔算速率、两个计数先后读出，bringup IMU 检查的延迟在任务醒来后又有新发布时得负值（`978bec1` 修正并补宿主回归测试）；修正后 20/20，中断发布的 topic 叫醒 IMU 任务每秒约 6800 次、最大延迟 73 µs；接 24 V 后恒温 50 ±0.125 ℃；USB CDC 状态行与回显正常；SBUS 收帧、摇杆与失控保护正常（16:55 补测）；CAN2 收 GM6020 反馈 1 kHz、0x205、0 丢帧（19:16 补测，分支 `f2/onboard-can`） | F2、F3 待负责人验收 |
| 2026-11-02 | F3 | bringup 的达妙电机检查（Decision-20261102-02）与上板验证（[记录](docs/ai/decisions/2026-11-02-达妙电机驱动上板验证/README.md)）：DM-J4310 上核对、清错使能、MIT 力矩与方向、写进 RAM 的通讯超时、失能保活、CAN1 收发都通过；转动力矩从 0.3 改为 1.0 N·m（减速箱摩擦约 0.2 N·m）。ctest 32/32 | 完成 |
| 2026-11-02 | F3 | 电机架构 M0–M7（Decision-20261102-03、-04）：CAN 帧带接收时刻；1 kHz 控制环基准；`CanBus` 与关节 `Joint`；达妙、DJI 驱动改用新结构（DJI 可混接，冲突在装配时拒绝）；安全门与 `safety.stop`；删除电机组。M3 上板发现时间戳先后的 bug 并修正。M7 Release 交替重测 7 轮：反馈 → 下发延迟 1753 → 753 µs，抖动不劣于 M0，单轮平均耗时 11.2 → 12.4 µs、最长不变。ctest 31/31 | 完成；单轮平均耗时 +1.2 µs 待负责人判断 |
| 2026-11-02 | F3 | 达妙电机驱动 `device::DmMotor` 与适配器、型号 `DM_J4310`（Decision-20261102-01）：启动回读核对后才允许使能，电机组契约加入使能 / 失能（DJI 也要使能才出力）。ctest 31/31；驱动里 11 处关键逻辑逐一改坏，测试都失败；固件、框架独立构建、M0+、依赖图、格式与命名检查通过 | 完成，已上板（见下一行） |
| 2026-11-01 | F3（L6） | BMI088 加热改为模型预测（Decision-20261101-05）：实测旧 PI 冲到 56.9 ℃、28 s 才稳；由实测数据辨识两级一阶滞后热模型（0.967 ℃/‰、15.5 s、0.95 s），`control::HeaterControl` 两级预热 + 平台校正加热器强度 + 前馈 PI 恒温；仿真在加热器强度 ±25% 下整定，宿主测试 28/28；上板 +0.125 ℃、15.3 s 进入 ±0.25 ℃、校准条件 35.4 s；断电不误修正、来电不过冲（[记录](docs/ai/decisions/2026-11-01-L6-加热提速上板验证/README.md)） | 完成，待负责人验收 |
| 2026-11-03 | A（链路） | 上位机链路用 RoboLink v0.2，规则 11 改写为"编解码只在 `src/middleware/robolink` 一处、用独立生成的标准帧核对"（Decision-20261103-01）。第 1 步：`middleware_robolink` 的帧层、槽位层与消息；标准帧 37 个，MCU 端宿主测试逐字节通过，故意改坏 12 处都被抓到；PC 端 `foray_platform` 对账一致，差异 1 处 | 第 1 步完成 |
| 2026-11-03 | A（链路） | 第 2 步：`robolink::Session`（握手、判活、序号、发送调度、计数）；宿主测试 `robolink_session`，故意改坏 13 处都被抓到；与 PC 端 `RoboLinkHostLink` 在伪终端上联调通过 | 第 2 步完成 |
| 2026-11-03 | A（链路） | 第 3 步：IMU 能力 `app/imu`（云台姿态、上电零偏标定、yaw 归零，`imu.attitude`）；零偏标定移入 `lib/estimate`；宿主测试 4 项，故意改坏 14 处都被抓到 | 第 3 步完成 |
| 2026-11-03 | A（链路） | 第 4 步：链路能力 `app/pc_link`（RoboLink 经 USB，姿态上传，控制快照只记账发布到 `pc_link.control`）；宿主测试 `pc_link`，故意改坏 8 处都被抓到 | 第 4 步完成 |
| 2026-11-03 | A（链路） | 第 5 步：profile `link_dm_mc02` 与组合根、PC 端探测工具、CI 构建 link 固件；上板：IMU 与链路运行，姿态限频改按采样时刻（860 → 998 Hz）；加热器没接 24 V | PC 端联调待串口权限与 24 V |
| 2026-11-03 | A（链路） | 第 5 步上板联调：PC 端探测脚本 60 s（带 1 kHz 控制快照与拔线重连）全部通过，姿态 996 Hz；视觉组的 `RoboLinkHostLink` 直接连板子通过；冷启动 42 s 进入 READY，静止时 yaw 漂移约 0.3 °/min；负责人确认 F1、F4 验收通过 | 链路第一版完成 |
| 2026-11-03 | A（链路） | 加热控制器热启动修正（Decision-20261103-02）：环境温度估计的两处修正、恒温小温差时宽限重新预热；仿真 768 个场景全部收敛，原有测试照旧通过。上板：恒温 50 ℃ 时直接复位，环境估计 18 s 内从 50 ℃ 修正到 37 ℃，21 s 回到 50 ℃（最高 50.375 ℃），72 s IMU 进入 READY（[记录](docs/ai/decisions/2026-11-03-加热热启动上板验证/README.md)） | 完成 |
| 2026-11-03 | A（链路） | yaw 漂移修正（Decision-20261103-03）：恢复 VQF 板级静止零偏参数（静止门限 0.35 rad/s），标定后不重置协方差；上板 5 min yaw -0.05° | 完成 |
| 2026-11-03 | 整理 | 仓库整理（Decision-20261103-05）：本地分支链快进合入 `dev`（提交号不变），删除 49 个远端与 50 个本地功能分支，stash 存为补丁后丢弃，整理前的引用打包存档 | 完成 |
| 2026-11-03 | 整理 | 代码注释删掉决策号（Decision-20261103-06）：头文件 55 个、其余代码与配置文件 76 个，文档不动；四个固件逐字节相同，两个提交登记到 `.git-blame-ignore-revs` | 完成 |
