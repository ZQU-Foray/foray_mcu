# 目录结构说明（foray_mcu）

> 状态：**P0–P2.5 已验收（2026-10-26）；2026-10-27 起按架构 v2 重建（Decision-20261027-01），下一步 F1 框架基础件**。本文件描述"目标结构"与"当前实际状态"的差异。
> 目标结构以 [docs/ai/architecture.md](docs/ai/architecture.md) v2 为准（评审报告 v1.0 是历史记录）；"目标位置"按阶段落地，是否已落地以状态图例为准。

## 当前实际结构（2026-10-26，P2.5 完成）

```text
foray_mcu/
├── .git/                     dev（日常开发）+ main（里程碑）；tag baseline/a2cfd16；origin = GitHub ZQU-Foray/foray_mcu；legacy remote（ControllerCode 历史）
├── .gitignore / .gitattributes / .clang-format / .clang-tidy   仓库规则、格式与命名配置
├── .git-blame-ignore-revs     git blame 忽略的纯格式/纯改名提交
├── .foray-layer / CODEOWNERS  组织合规件（layer: MCU；@ZQU-Foray/电控组）
├── .github/workflows/ci.yml   CI：lint-python、lint-c、lint-naming、lint-arch、host-tests、firmware（符号对照）
├── .github/actions/setup-arm-gcc/   CI 共用：安装 xPack Arm GNU 工具链与 Ninja
├── README.md / AGENTS.md / plan.md / tree.md / decision.md   五件套
├── .clangd                    clangd 设置（P2 从迁移源的 DM_MC02 移来）
├── CMakeLists.txt / CMakePresets.json   根工程（P1）：唯一开关 -DPROFILE；preset arm-debug、arm-release、host-tests
├── cmake/
│   ├── toolchains/           arm-gcc-cortex-m7、arm-gcc-cortex-m4f、host（-m4f 尚无使用者，阶段 B 用）
│   ├── targets.cmake         严格告警、OBJECT 库目标文件传递、固件产物、烧录目标
│   └── legacy.cmake          迁移期过渡件：legacy_build_options（迁移源的两个构建期开关，F2、F3 删）
├── src/                      框架（include 根：#include "hal/can.hpp"）
│   ├── CMakeLists.txt        框架的全部 target：hal、hal_detail、hal_os、boot、utils、lib_*、driver_*
│   ├── README.md             迁移源 Libraries/README.md（正文为旧路径，F1 更新）
│   ├── hal/                  C++ 契约（can、gpio、pwm、spi、time、uart、usb_cdc）；detail/ 为迁移期 C ABI（F2 删）；os/ 为最小 OS 契约草案
│   ├── lib/                  math、control、crc、estimate、filter、fsm、kinematics、slope、component（目录名即命名空间名）
│   └── driver/               bmi088、buzzer、motor（含 dji/）、ws2812、protocol/{dji,sbus}
├── app/                      产品（include 根：#include "tasks/imu_task.hpp"）
│   ├── CMakeLists.txt        产品的全部 target：app_imu、app_chassis、app_gimbal、app_remote、app_tasks、app_common
│   ├── tasks/                imu、chassis、gimbal 的任务外壳与 IMU 数据就绪邮箱
│   ├── imu/                  姿态解算、陀螺零偏标定与观测、IMU 诊断
│   ├── chassis/              底盘控制器
│   ├── gimbal/               云台模块
│   ├── remote/               遥控接收
│   └── common/               旧任务管理（F2 归档，改为任务运行器 + assembly 的任务表）
├── config/
│   ├── CMakeLists.txt        profile_baseline、assembly_baseline；把 assembly/ 注入生成区的 include 路径（X-09）
│   ├── assembly/             entry_legacy.cpp（启动入口 Application_*）；Application/Entry.h 被生成代码按此路径 include（F2 改为 boot ABI）
│   └── profiles/             baseline/profile.cmake（a2cfd16 基线固件，DM-MC02）；profile.hpp.in 生成 profile.hpp
├── platform/
│   ├── st/stm32h7/board_dm/  board.cmake（板标识 dm_mc02）+ CMakeLists.txt（mx_generated、rtos、usb、port、board、os_freertos）
│   │   ├── cubemx/           CubeMX 生成物：Core、Drivers、Middlewares、USB_DEVICE、.ioc、启动文件、链接脚本（冻结）
│   │   └── port/             契约的手写实现（C），含 port_kit（队列/通知/临界区）与 usb_cdc；Init.h、UsbCdc.h 被生成代码按名字 include，保持原名
│   └── mock/                 platform_mock：hal_os 的宿主替身
├── legacy/                   迁移源的 README 与既有架构图（源码已在 P2、P2.5 全部搬出）
│   ├── .docs/images/         既有架构图
│   └── README.md
├── test/host/                宿主测试（由根工程 host-tests preset 引入，14 项 ctest；来源 fd1edbb，见 test/host/README.md）
├── test/data/imu/            真实 IMU 数据（原固件静态陀螺采集 ×2）
├── test/data/heater/         原固件加热冷启动曲线（L6 重做的验收下限）
├── tool/debug/               SWD 只读轮询、IMU 采集与加热日志解码（Python）
├── tool/flash/openocd/       OpenOCD 配置（DM-MC02 + CMSIS-DAP）
├── tool/ci/                  format.py（格式）、naming.py（命名）、symbols.py（符号对照）、arch_check.py（依赖图与文本扫描）
└── docs/
    ├── assets/controller_code-架构图.png                    迁移源架构图（人工绘制，PNG 内嵌 5 页）
    └── ai/                      AI 撰写的文档（Decision-20261026-03）
        ├── README.md                docs/ai 索引
        ├── architecture.md          分层判据、依赖规则、例外登记表（v1.4）
        ├── assets/controller_code-固件恢复可行性评估.md      迁移源基线证据（未入 git 的工作区文件转存）
        ├── reviews/2026-10-25-foray_mcu-架构评审报告.md      v1.0（已确认）
        ├── reviews/archive/                                  v0.1–v0.4 原文存档
        ├── decisions/2026-10-25-迁移基线与对照.md           基线数据、SHA、工具链、P0 复核记录、芯片 Flash 存档记录
        ├── decisions/2026-10-26-丢失功能与恢复依据.md       丢失功能清单（L1–L6）与行为证据
        ├── decisions/2026-10-26-P0.5-改名/                  P0.5 改名过程、改名表、手工修正脚本、改名后符号表
        ├── decisions/2026-10-26-P1-构建骨架/                P1 过程与对照、归一化符号表（P2 起的对照基准）
        ├── decisions/2026-10-26-P2-机械搬迁/                P2 移动表、搬迁与改写脚本、验证记录
        ├── decisions/2026-10-26-P2.5-去掉alg前缀/          src/lib 去掉 alg_ 前缀：改名脚本、改名表（符号对照用）
        ├── decisions/2026-10-26-P2.5-拆分搬迁/              P2.5 搬迁脚本、去向与验证记录
        └── decisions/baseline-a2cfd16/                      基线符号表与段表（nm -S / size -A）、SHA256SUMS
```

**事实**：
- CMake 文件按层组织：手写的 `CMakeLists.txt` 共 7 个（根、`src/`、`app/`、`config/`、每块板一个、`platform/mock`、`test/host`），一层的全部 target 定义在该层的文件里（Decision-20261026-07）；
- 根工程按目标结构组织 target，固件 `firmware_baseline` 的符号集合与迁移源一致（P1、P2 验收）；
- 源码已全部搬出 `legacy/`（P2、P2.5），`legacy/` 只剩迁移源 README 与架构图；迁移期过渡件只剩 `hal_detail`（F2）与 `legacy_build_options`（F2、F3）；
- `src/boot`、`src/utils`（target 已建，暂无代码）；`src/topic`、`app/topics`、按板一级的 `platform/dm_mc02` 等按架构 v2 在 F1、F2 建立。

## 目标结构（架构 v2，Decision-20261027-01）

> 框架 = `src/` + `platform/` + `cmake/`（可独立构建、可裸机构建，供其他小项目复用）；产品 = `app/` + `config/`（本仓的 RM 下位机固件）。框架不得引用产品。各层职责与机制见 [docs/ai/architecture.md](docs/ai/architecture.md) v2。

```text
foray_mcu/
├── CMakeLists.txt                构建入口（顶层只做工具链检查、选项与组合）；唯一开关 -DPROFILE
├── CMakePresets.json             arm-debug、arm-release、host-tests（阶段 B 起扩成 profile × Debug/Release 矩阵）
├── cmake/
│   ├── toolchains/               arm-gcc-cortex-m7 / arm-gcc-cortex-m4f / host；按需 cortex-r5、cortex-m0plus
│   └── targets.cmake             严格告警、OBJECT 库目标文件传递、固件产物、烧录目标
├── src/                          框架：与产品、板都无关；零 RTOS
│   ├── hal/                      硬件契约（C++ 自由函数 + 不透明句柄）：can、spi、uart、gpio、pwm、time、usb_cdc（F2）
│   │   └── os/                   OS 契约：临界区、单调时基、信号（F1）
│   ├── boot/                     启动入口 boot.h：boot_init / boot_init_rtos / boot_run（F2）
│   ├── utils/                    无领域含义的基础件：spsc_queue、latest_value、fsm、crc（F1）
│   ├── topic/                    topic 机制：最新值、单发布者、中断发布、阻塞等待（F1）
│   ├── lib/                      领域算法：math、control（pid、slope）、estimate、filter、kinematics、component
│   ├── driver/                   设备驱动与设备协议：bmi088、motor/dji、buzzer、ws2812、protocol/{dji,sbus}（F3 适配 hal v1）
│   └── third_party/              vendor 生成头：foray_interfaces/generated/lower_link.hpp + 版本/sha256（阶段 A）
├── platform/                     按板一级
│   ├── dm_mc02/                  DM-MC02（STM32H723，F2）
│   │   ├── cubemx/               生成物（冻结；USER CODE 区只调 boot_*）
│   │   ├── port/                 hal 与 hal/os 的 C++ 实现、HAL 回调（OBJECT）
│   │   ├── board/                board.hpp（物理资源句柄、能力常量）、overlay.ld（OBJECT）
│   │   ├── board.cmake           板标识、CPU、烧录配置
│   │   └── CMakeLists.txt        本板全部 target
│   ├── <F407 C 板>/              阶段 B
│   └── mock/                     hal 与 hal/os 的宿主替身（测试控制时间、注入输入、检查输出）
├── app/                          产品：RM 能力（按能力，不按兵种）
│   ├── topics/                   全部 topic 的清单（消息类型 + 实例），集中一处；以后由描述文件生成
│   ├── common/                   任务运行器（FreeRTOS 原生 API，任务表驱动）
│   └── imu/、chassis/、gimbal/、remote/、launch/、link/……   能力：init/step，零 RTOS（阶段 A）
├── config/
│   ├── profiles/<兵种>_<板>/       profile.cmake（板、能力）+ params.hpp（constexpr 参数）；框架验证用 bringup_dm_mc02
│   └── assembly/<profile>.cpp    组合根：构造对象、注入参数、任务表、实现 boot_*
├── test/
│   ├── host/                     宿主单元测试（链接 platform/mock，ctest）
│   └── data/                     录制数据（IMU、加热曲线等），回放回归
├── tool/
│   ├── flash/                    OpenOCD 配置
│   ├── debug/                    SWD 读取、数据解码
│   └── ci/                       格式、命名、依赖图与文本扫描、（F2 起）HAL 回调符号检查
├── docs/
│   ├── <人写的文档>              人工撰写的设计文档、手册、指南（F4 的使用指南放这里）
│   ├── assets/                   人工制作的图表等
│   └── ai/                       AI 撰写的文档（Decision-20261026-03）
├── legacy/                       迁移源 README 与架构图（只读）
├── .foray-layer                  layer: MCU / depends_on_layers: [X2]
├── CODEOWNERS                    @ZQU-Foray/电控组
└── .github/workflows/ci.yml
```

## 状态图例

| 标记 | 含义 |
|---|---|
| ✅ 已落地并验证 | 有代码/构建/测试证据 |
| 📄 已交付 | 文档/证据已落盘（不代表已评审通过） |
| 🟡 规划中 | 框架阶段（F1–F4）实施 |
| ⏳ 后续阶段 | 框架验收后实施（阶段 A、B、E） |
| ⛔ 冻结/禁止 | 生成区、组织契约、手写协议等 |

当前：五件套为 ✅；架构 v2 为 📄（2026-10-27，待审阅）；评审报告为历史记录；git 仓库、格式/命名配置、根工程构建骨架（P1）与源码搬迁（P2、P2.5）为 ✅；`legacy/`（迁移源 README 与架构图）与 `platform/**/cubemx/**`、`*.ioc`、`.mxproject` 为 ⛔。

## 关键位置（目标态，架构 v2）

| 要找什么 | 去哪（目标位置） | 当前状态 |
|---|---|---|
| 迁移源 README 与架构图 | `legacy/` | ⛔ 只读（源码已在 P2、P2.5 全部搬出） |
| 无领域含义的基础件 | `src/utils/` | 🟡 F1（现有 `lib/fsm`、`lib/crc` 移入；PortKit 队列改写） |
| topic 机制 / 产品的 topic 清单 | `src/topic/` / `app/topics/` | 🟡 F1 |
| OS 契约 | `src/hal/os/` | 📄 草案与宿主替身（P1）；定稿与 `Signal` 在 F1，port 实现在 F2 |
| 硬件契约 v1 | `src/hal/` | 🟡 F2（现为旧契约 + `src/hal/detail` C 接口，F2 删除） |
| 启动入口 | `src/boot/` | 🟡 F2（现为 `config/assembly` 的 `Application_*`，例外 X-09） |
| 领域算法 | `src/lib/` | ✅ 已搬入（P2）、去掉 `alg_` 前缀（P2.5）；F1 调整 slope/fsm/crc 的归属 |
| 设备驱动与协议 | `src/driver/` | ✅ 已搬入（P2）；F3 适配 hal v1 |
| 板级（生成区、port、board） | `platform/<板>/{cubemx,port,board}` | 🟡 F2 改为按板一级（现为 `platform/st/stm32h7/board_dm`，port 仍是 C） |
| 任务运行器 | `app/common/` | 🟡 F2（现为旧 TaskManager） |
| 能力 | `app/<capability>/` | ⏳ 阶段 A 重建（现有旧代码 F2 归档到 tag `archive/pre-rebuild`） |
| profile 与组合根 | `config/profiles/<兵种>_<板>/`、`config/assembly/<profile>.cpp` | 🟡 F2 建 `bringup_dm_mc02`；兵种 profile 在阶段 A |
| 宿主测试 | `test/host/` | ✅ 由根工程引入（14 项全绿）；随 F1–F3 扩充，依赖旧 app 的测试在 F2 归档 |
| 组织合规件 | `.foray-layer`、`CODEOWNERS`、`.github/workflows/ci.yml` | ✅ 已落地 |
| 链路协议生成头 | `src/third_party/foray_interfaces/generated/lower_link.hpp` | ⏳ 阶段 A（链路能力） |
| 架构判据 | `docs/ai/architecture.md` | 📄 v2（2026-10-27，待审阅） |
| 架构评审 | `docs/ai/reviews/`（v1.0；旧版在 `archive/`） | 📄 历史记录，与 v2 冲突处以 v2 为准 |
| 迁移基线证据 | `docs/ai/decisions/2026-10-25-迁移基线与对照.md`、`docs/ai/decisions/baseline-a2cfd16/`、`docs/ai/assets/controller_code-固件恢复可行性评估.md` | ✅ P0 已复核 |
| 决策 | `docs/ai/decisions/`、`decision.md` | ✅ 随阶段登记 |

## 迁移源对照（粗粒度）

> 左列来源路径都位于迁移源（`baseline/a2cfd16`）中。架构 v2 起，迁移源只作参考与零件来源：算法与驱动按新契约适配后复用，能力与启动部分按新结构重做。

| ControllerCode 现状 | 架构 v2 中的去向 |
|---|---|
| `Application/Entry.*` | `config/assembly/<profile>.cpp` 实现 `boot_*`（F2） |
| `Application/Task/TaskManager.*` | 任务表 → `config/assembly`；创建机制 → `app/common` 任务运行器（F2） |
| `Application/Task/*`、`Application/Imu/*`、`Application/Module/*` | 阶段 A 按能力重建（`app/<capability>`，init/step + topic） |
| `Libraries/Algorithm/*`、`Component/*` | `src/lib/`；fsm、crc → `src/utils/`，slope → `src/lib/control`（F1） |
| `Libraries/Device/*`、`Protocol/*` | `src/driver/`，F3 适配 hal v1 |
| `Platform/Interface/*.hpp` + `Detail/*.h` | 由 `src/hal` 契约 v1 取代（F2） |
| `Platform/PortKit/*` | 队列 → `src/utils/spsc_queue`（F1）；临界区 → `src/hal/os` 契约 + port 实现（F1、F2） |
| `Platform/STM32H7/DM_MC02/{Core,Drivers,Middlewares,USB_DEVICE,startup,.ld,.ioc}` | `platform/dm_mc02/cubemx/`（F2 起按板一级） |
| `Platform/STM32H7/DM_MC02/Port/*` | `platform/dm_mc02/port/`，F2 用 C++ 重写 |
| `Tests/`（历史，`fd1edbb`） | `test/host/` |
| `Tools/`、`scripts/`（被 ignore/丢失） | `tool/`（必须入库） |

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`.git/` 等依赖与产物目录；
- STM32 HAL 驱动的逐文件清单（属第三方，只记 `platform/**/cubemx/` 一层即可）；
- 临时文件、缓存文件、日志文件；
- 未评审通过、尚未实施的结构（必须在标题/状态中明确标注"规划中"）。
