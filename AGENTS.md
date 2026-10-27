# AI 协作规范（foray_mcu）

> 本文件与 [README.md](README.md)、[plan.md](plan.md)、[tree.md](tree.md)、[decision.md](decision.md) 构成仓库五件套。
> 任何自动化代理或子代理在修改本仓之前，必须先读这五个文件，以及 `docs/ai/reviews/` 下最新评审结论。

## 0. 当前阶段

本仓 **P0（基线与资产恢复）、P0.5（格式与命名统一）、P1（构建骨架）已于 2026-10-26 验收，下一步是 P2（机械搬迁）**：方案已确认（[decision.md](decision.md) Decision-20261025-01 至 -05 均为 Accepted；评审报告 `docs/ai/reviews/2026-10-25-foray_mcu-架构评审报告.md` v1.0，旧版在 `docs/ai/reviews/archive/`，不再维护）。
迁移源 ControllerCode `a2cfd16` 原样放在 `legacy/`（基线 tag `baseline/a2cfd16`）；根工程（`CMakeLists.txt`、`CMakePresets.json`）已按目标结构建立 target，源码仍在 `legacy/` 原地编译，P2 起搬出；宿主测试由根工程的 host-tests preset 引入（见 `test/host/README.md`）。target、函数、变量名一律不加项目前缀（Decision-20261026-06）。**按 [plan.md](plan.md) 的阶段顺序推进，不跨阶段混做。**

## 1. 语言与格式

- 注释、文档、提交信息一律使用 **简体中文**；文件编码 **UTF-8（无 BOM）**。
- 主语言 **C++17**（`-fno-rtti -fno-exceptions -fno-threadsafe-statics`，无堆分配，热路径不用虚函数）。C 只用于三处外部强制的接缝：CubeMX 生成区调用的启动入口（`extern "C"` 声明，实现写在 C++ 里）、覆盖 HAL 弱符号的回调、生成区里的中断入口。覆盖 HAL 回调的 `.cpp` 必须 include 对应 HAL 头，否则回调会被 C++ 改名而静默失效（评审报告 §5.11）。
- 命名按 ROS 2 C++ 风格，格式按组织规则（4 空格、LF、UTF-8、文件末尾换行），分别由根目录 `.clang-tidy` 与 `.clang-format` 执行；clang-format/clang-tidy 统一用 18（评审报告 §5.12、Decision-20261025-04）。格式化与改名必须单独提交，并登记到 `.git-blame-ignore-revs`。
- `.clang-tidy` 查不出的命名约定（Decision-20261026-05）：标识符里"数值+单位"和序号单独成段（`ACCEL_ODR_100_HZ`、`POWER_24V_0`、`CHANNEL_1`），名字里固有的数字不拆（型号 `BMI088`、`GM6020_CURRENT`，术语 `crc8`，标号 `WHEEL_M1`）。C ABI 的"模块_函数"、`os*`、`HAL_*` 回调不按 C++ 规则改名。
- 不提交编译产物（`build/`、`*.elf`、`*.hex`、`*.bin`、`*.map`）、IDE 配置、临时文件与日志。
- **AI 撰写的文档放在 `docs/ai/`**（评审报告、架构判据、决策证据、转存材料等），人工撰写的文档放在 `docs/` 的其他位置，以便区分来源（Decision-20261026-03）。根目录五件套与各目录内的 `README.md` 位置由组织规范与工具约定固定，不受此限。

## 2. 冻结区与禁止事项

- `platform/**/cubemx/**`、`*.ioc`、`.mxproject`、startup 文件与链接脚本属于 **CubeMX 生成区**：只能通过重新生成修改，**禁止手工编辑**；自定义段用 `platform/.../board/link/overlay.ld` 以第二个 `-T` 追加（Q9），不改生成 `.ld`；生成前后必须可复现（CI regenerate-diff 或生成清单校验）。
- `legacy/` 是迁移源的原样副本：只允许按 plan.md 的阶段做机械操作（搬出、改名、格式化），**禁止在 `legacy/` 里开发新功能或修 bug**；新代码从 P1 起按目标结构编写。
- **禁止手写链路协议编解码**：链路协议只允许使用 `foray_interfaces` 生成的 `generated/lower_link.hpp`（vendor 到 `src/third_party/`，带版本与 sha256）。
- 禁止引入反向/跨层依赖：`app → platform 实现`、`src → platform/app/config`、任何层 → 厂商 HAL/RTOS 头。**例外**：`src/hal` 契约头本身不得出现 HAL/RTOS 类型；`platform` 实现 `src/hal` 时可以使用厂商头。规则以评审报告 §5 与 [docs/ai/architecture.md](docs/ai/architecture.md) 为准；后者是分层判据、16 条依赖规则与例外登记表的人读入口（v1，Decision-20261026-02）。
- 禁止 `file(GLOB)` 收集业务源码；禁止全局 `include_directories()`；禁止把仓库根加入 include 路径。
- CMake 文件按层组织：一层的全部 target 写在该层的一个 `CMakeLists.txt` 里（根、`src/`、`app/`、`config/`、每块板、`platform/mock`、`test/host`），不在子目录里各放一个；需要拆分时先在 [decision.md](decision.md) 登记（Decision-20261026-07）。
- 禁止在 `app/` 与 `src/` 代码中用 `#if` 兵种/能力宏做裁剪（能力用 target/装配表达；`#if` 只允许出现在生成头与 platform 适配层）。
- 框架（`src/`、`platform/`、`cmake/`）不得引用产品（`app/`、`config/`）的路径或 target；`src/` 零 RTOS，需要临界区或时间时只经 `src/hal/os` 最小契约。
- 框架代码不直接使用 `std::atomic` 的 `fetch_*` / `exchange` / `compare_exchange_*`（Cortex-M0+ 不支持无锁实现），改用 `src/utils` 并发原语。
- 框架代码开启 `-Wdouble-promotion`；只有 IMU 姿态解算中有意使用 double 的文件可豁免，新增豁免需在 [decision.md](decision.md) 登记（Decision-20261025-03）。
- port 与 board 用 OBJECT 库，不用 STATIC（HAL 弱符号回调可能被链接器静默丢弃）。
- 破坏性接口变更、换 HAL/RTOS 版本、生成区变更，必须先在 [decision.md](decision.md) 登记，再改代码。

## 3. 工作流

- 先读后改：读相关源码、当前阶段计划与最近决策记录，再动手。
- 分支：功能分支从 `dev` 切出，PR 合到 `dev`；`main` 只放里程碑快照，阶段验收后由 `dev` 合入（组织规范 §6.1，Decision-20261026-01）。
- 小步提交：**一个 PR 只做一类变更**；"目录搬迁"与"行为修改"不得混在同一个 PR。
- 修改架构、接口、构建或目录 → 同步更新 [plan.md](plan.md)、[tree.md](tree.md)、[decision.md](decision.md) 及相关 `docs/`。
- 每个阶段必须 **可构建、可验证、可回滚**；验收标准以 [plan.md](plan.md) 为准。
- 不得在没有构建/测试证据时声称"已完成"；没有宿主测试的阶段，必须给出人工验证记录（测点、固件、日志/存档）。

## 4. 测试与基线

- 宿主测试优先（`test/`，ctest）；没有 mock 平台的模块不得依赖真实硬件验证。
- 固件改动必须对照基线检查 `.map`/`.bin`（段大小、关键符号、hash）；基线数据见 [docs/ai/decisions/2026-10-25-迁移基线与对照.md](docs/ai/decisions/2026-10-25-迁移基线与对照.md)（来源：ControllerCode `docs/固件恢复可行性评估.md`，已转存）。
- 新功能先补测试或验收脚本；历史测试从 ControllerCode 提交 `fd1edbb` 恢复（`Tests/` 26 文件 / 2,979 行；该提交共 28 文件 / 3,008 行）。其中 `Tests/ImuStream` 的被测模块已不存在，需要重写；其余测试适配后恢复。
- 结构调整阶段（P1–P2.5）的对照判据是函数与对象符号的名称、大小集合（`tool/ci/symbols.py`，readelf 归一化，不含地址），不要求 `.bin` 的 SHA 一致（链接顺序会变）；P2 起基准为 `docs/ai/decisions/2026-10-26-P1-构建骨架/symbols-*.txt`。编译选项改动要逐文件对照 `compile_commands.json`：告警选项也可能改变生成代码（GCC 15 的 `-Wall`，Decision-20261026-07）。IMU 相关改动用 `test/data/` 中的真实 IMU 数据在宿主机上对照。

## 5. 自动化与子代理

- 子代理适合做**只读**分析、检索、评审；**写操作由 Lead 统一裁决**并核对最终 diff。
- 同一文件/目录禁止并行写；共享任务必须登记写范围，避免相互覆盖。
- 生成的文件、批量脚本改名的结果必须可复现，并在 PR 描述中给出命令与对照结果。

## 6. 与组织规范的关系

- 本仓 `layer: MCU`，**不在 L1–L5 分层内**，位于 L2 下行 HAL 之下；`depends_on_layers: [X2]`，仅 vendor 消费 `foray_interfaces` 生成头文件。
- 组织合规件：`.foray-layer`、`CODEOWNERS`（`@ZQU-Foray/电控组`）、`.github/workflows/ci.yml` 已落地（CI 设计见 Decision-20261026-01）；`docs/ai/architecture.md` v1 已创建（Decision-20261026-02）。
- CI 的 `firmware` 作业用根工程构建固件，与迁移源构建逐配置比较符号集合并检查依赖图；迁移源的 Release 镜像仍须与基线逐字节一致（到 P2 搬迁为止）。`lint-arch` 与 `host-tests` 用 `tool/ci/arch_check.py` 检查依赖规则（新增一类 target 或迁移期例外，先登记再改脚本）。`lint-c` 用 `tool/ci/format.py --check` 全量检查手写 C/C++（生成区与厂商代码除外），提交前本地执行 `python tool/ci/format.py --fix`（clang-format 18.1.8）；`lint-naming` 用 `tool/ci/naming.py --check` 全量检查 C++ 命名（clang-tidy 18.1.8，编译数据库为根工程 arm-debug、arm-release、host-tests 三个 preset；Windows 上 MSYS2 的 libstdc++ 16 与 clang 18 不兼容，本地检查用 Linux/WSL）。
- 规则与组织仓库规范冲突时，以组织 `foray_docs/repository_structure.md` 为准，并在 [decision.md](decision.md) 登记。
