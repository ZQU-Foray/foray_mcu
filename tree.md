# 目录结构说明（foray_mcu）

> 状态：**方案已确认，P0、P0.5 已验收（2026-10-26），下一步 P1**。本文件描述"目标结构"与"当前实际状态"的差异。
> 目标结构以 [docs/ai/reviews/2026-10-25-foray_mcu-架构评审报告.md](docs/ai/reviews/2026-10-25-foray_mcu-架构评审报告.md) 为准；"目标位置"按阶段落地，是否已落地以状态图例为准。

## 当前实际结构（2026-10-26，P0.5 完成）

```text
foray_mcu/
├── .git/                     dev（日常开发）+ main（里程碑）；tag baseline/a2cfd16；origin = GitHub ZQU-Foray/foray_mcu；legacy remote（ControllerCode 历史）
├── .gitignore / .gitattributes / .clang-format / .clang-tidy   仓库规则、格式与命名配置
├── .git-blame-ignore-revs     git blame 忽略的纯格式/纯改名提交
├── .foray-layer / CODEOWNERS  组织合规件（layer: MCU；@ZQU-Foray/电控组）
├── .github/workflows/ci.yml   CI 第一版：lint-c 增量、宿主测试、legacy 固件 Release SHA 闸门
├── README.md / AGENTS.md / plan.md / tree.md / decision.md   五件套
├── legacy/                   迁移源 ControllerCode a2cfd16 原样副本（P0–P2.5 只做机械搬出）
│   ├── Application/          业务层：任务、IMU、模块、启动入口
│   ├── Libraries/            算法、组件、设备驱动、设备协议
│   ├── Platform/             Interface（契约 + C ABI）、PortKit、STM32H7/DM_MC02（CubeMX 工程 + Port）
│   ├── .docs/images/         既有架构图
│   └── README.md
├── app/ config/              空骨架（P2 起填充）
├── platform/                 空骨架：st/board_c、st/board_dm/{bsp_impl,cubemx}、ti（空目录不入 git）
├── src/
│   ├── bsp/hardware/         7 个 0 字节占位 .hpp（未纳入 git，P1 建 src/hal 时删除）
│   └── bsp/os/、driver/、lib/、middleware/、msg/、utils/    空
├── test/host/                宿主测试（P0 独立 CMake 工程，13 项 ctest；来源 fd1edbb，见 test/host/README.md）
├── test/data/imu/            真实 IMU 数据（原固件静态陀螺采集 ×2）
├── test/data/heater/         原固件加热冷启动曲线（L6 重做的验收下限）
├── tool/debug/               SWD 只读轮询、IMU 采集与加热日志解码（Python）
├── tool/ci/format.py         C/C++ 格式检查与格式化入口（CI 与本地共用）
├── tool/ci/naming.py         C++ 命名检查与修正入口（clang-tidy，CI 与本地共用）
└── docs/
    ├── assets/controller_code-架构图.png                    迁移源架构图（人工绘制，PNG 内嵌 5 页）
    └── ai/                      AI 撰写的文档（Decision-20261026-03）
        ├── README.md                docs/ai 索引
        ├── architecture.md          分层判据、依赖规则、例外登记表（v1）
        ├── assets/controller_code-固件恢复可行性评估.md      迁移源基线证据（未入 git 的工作区文件转存）
        ├── reviews/2026-10-25-foray_mcu-架构评审报告.md      v1.0（已确认）
        ├── reviews/archive/                                  v0.1–v0.4 原文存档
        ├── decisions/2026-10-25-迁移基线与对照.md           基线数据、SHA、工具链、P0 复核记录、芯片 Flash 存档记录
        ├── decisions/2026-10-26-丢失功能与恢复依据.md       丢失功能清单（L1–L6）与行为证据
        ├── decisions/2026-10-26-P0.5-改名/                  P0.5 改名过程、改名表、手工修正脚本、改名后符号表
        └── decisions/baseline-a2cfd16/                      基线符号表与段表（nm -S / size -A）、SHA256SUMS
```

**事实**：本仓已建 git 并推送到 GitHub，迁移源在 `legacy/`，可在 `legacy/Platform/STM32H7/DM_MC02` 构建（P0 已复现，SHA-256 与基线一致）；还没有根 `CMakeLists.txt`；宿主测试已恢复到 `test/host/`（独立工程，直接编译 `legacy/` 源码）；其余目标结构目录仍是空骨架。

## 目标结构（已确认，按阶段实施）

> 框架 = `src/` + `platform/` + `cmake/`（可独立构建、可裸机构建，供其他小项目复用）；产品 = `app/` + `config/`（本仓的 RM 下位机固件）。框架不得引用产品，见评审报告 §5.9。

```text
foray_mcu/
├── CMakeLists.txt                构建入口（顶层只做 toolchain/options/组合）
├── CMakePresets.json             预设矩阵：profile × board × Debug/Release + host-tests
├── cmake/
│   ├── toolchains/               arm-gcc-cortex-m7 / arm-gcc-cortex-m4f / host；按需 arm-gcc-cortex-r5、tiarmclang-cortex-r5、arm-gcc-cortex-m0plus
│   ├── foray_targets.cmake       统一 target 属性、严格告警、公共 include 规则
│   └── checks/                   分层/include/禁止依赖检查脚本
├── app/                          业务能力（按能力/角色，不按兵种）
│   ├── common/                   任务外壳、周期循环、基础类型（唯一允许接触 OS 的 app 子目录；FreeRTOS 原生 API）
│   ├── imu/                      姿态估计、零偏标定、诊断
│   ├── chassis/                  底盘业务
│   ├── gimbal/                   云台业务
│   ├── remote/                   遥控输入
│   ├── link/                     上位机链路（消费 foray_interfaces 生成头）
│   ├── launch/                   发射机构（按兵种能力扩展）
│   ├── debug_comm/               调优调试（临时，调优后删）
│   └── tasks/                    各能力任务薄壳（线程/节拍/外设独占）
├── config/
│   ├── profiles/<profile>/       profile.cmake（纯数据：能力集合、参数、板卡）+ foray_profile.hpp.in
│   ├── assembly/                 组合根：boot ABI 实现、任务表、对象装配、参数注入
│   └── generated/                构建生成物（gitignore，不手改）
├── platform/
│   ├── st/
│   │   ├── cmake/                ST 公共：HAL/FreeRTOS/USB/Object target
│   │   ├── stm32h7/board_dm/
│   │   │   ├── cubemx/           生成物 only：Core/ Drivers/ Middlewares/ USB_DEVICE/ .ioc/ startup/ .ld
│   │   │   ├── port/             手写 C++：实现 src/hal 与 src/hal/os（原 Platform/.../Port，P3a 由 C 改写；OBJECT 库）
│   │   │   ├── board/            pinmap、时钟、能力常量、设备 ID、board bring-up、link/overlay.ld（OBJECT 库）
│   │   │   └── CMakeLists.txt    手写；add_subdirectory(cubemx)
│   │   └── stm32f4/board_c/      第二块板（P3a，验证契约 v1）
│   ├── ti/
│   │   ├── tm4c/、am26x/         Cortex-M4F / Cortex-R5F（P4 起按需）
│   │   └── mspm0/                Cortex-M0+，裸机（P5，第一个小项目启动时）
│   └── mock/                     宿主 ABI 桩 + 假 OS 契约（测试专用）
├── src/                          通用框架（平台无关、可宿主编译）
│   ├── hal/                      C++ 契约：can/gpio/pwm/spi/time/uart/usb_cdc.hpp（由 port 直接实现；不设 C ABI 子层）
│   │   └── os/                   最小 OS 契约：临界区 + 单调时基（Q10；线程、队列不进框架）
│   ├── boot/                     启动 ABI：foray_boot_init / init_rtos / run（由 Application/Entry.h 演进；生成区只认它）
│   ├── lib/                      纯算法/组件：math/control/estimate/filter/kinematics/fsm/crc/slope
│   ├── driver/                   设备与设备协议：imu/bmi088、motor/dji（含型号档案映射，Q11）、motor/damiao、misc、protocol
│   ├── msg/                      最新值邮箱/SPSC 队列（v1，与 P3b 能力拆分同步落地）；topic 在 P3b 后按需引入
│   ├── middleware/               链路适配等跨模块服务
│   ├── utils/                    并发原语（计数器、最新值邮箱、SPSC 队列；M0+ 退回临界区实现）
│   └── third_party/foray_interfaces/generated/lower_link.hpp   vendor 生成头 + 版本/sha256
├── test/
│   ├── host/{unit,integration}/  宿主单元/集成测试（链接 platform/mock，ctest）
│   ├── data/                     黄金向量（SBUS、DJI 帧、IMU 样本；P0 录制真实 IMU 数据）
│   └── hil/                      实机/验收脚本与记录
├── tool/
│   ├── flash/                    openocd 配置与烧录脚本
│   ├── debug/                    SWD 存档读取、数据解码/回放
│   └── ci/                       架构检查、生成区校验、vendor 哈希
├── docs/
│   ├── <人写的文档>              人工撰写的设计文档、手册（直接放 docs/ 下）
│   ├── assets/                   人工制作的图表等
│   └── ai/                       AI 撰写的文档（Decision-20261026-03）
│       ├── architecture.md       分层判据与依赖规则（唯一人读入口）
│       ├── reviews/              架构评审报告
│       ├── decisions/            基线记录、行为证据
│       └── assets/               AI 转存或生成的证据材料
├── .foray-layer                  layer: MCU / depends_on_layers: [X2]
├── CODEOWNERS                    @ZQU-Foray/电控组
└── .github/workflows/ci.yml
```

## 状态图例

| 标记 | 含义 |
|---|---|
| ✅ 已落地并验证 | 有代码/构建/测试证据 |
| 📄 已交付 | 文档/证据已落盘（不代表已评审通过） |
| 🟡 规划中 | 评审通过后实施 |
| ⛔ 冻结/禁止 | 生成区、组织契约、手写协议等 |

当前：五件套与评审报告为 ✅（已确认）；git 仓库、`legacy/` 布局与格式/命名配置为 ✅；其余全部 🟡；`legacy/`（只做机械搬迁）与 `platform/**/cubemx/**`、`*.ioc`、`.mxproject` 为 ⛔。
**注意**：`src/hal` 目前**不存在**（P1 建立，Q2 已确认）；当前占位是 `src/bsp/hardware` 下 7 个 0 字节 `.hpp`，未纳入 git。

## 关键位置（目标态）

| 要找什么 | 去哪（目标位置） | 当前状态 |
|---|---|---|
| 迁移源（原样） | `legacy/` | ⛔ 只做机械搬出 |
| 业务能力实现 | `app/<capability>/` | 🟡 待迁移 |
| 产品/机器人装配（能力集合、参数） | `config/profiles/<profile>/` | 🟡 待建 |
| 组合根（任务表、对象构造、启动） | `config/assembly/` | 🟡 待建 |
| 跨平台硬件契约（C++） | 目标 `src/hal/`；当前占位 `src/bsp/hardware/` | 🟡 待建（P1） |
| 启动 ABI（生成区 ↔ 装配根） | `src/boot/` | 🟡 待建 |
| 最小 OS 契约 / 并发原语 | `src/hal/os/`、`src/utils/` | 🟡 待建（P3a） |
| 设备驱动与协议 | `src/driver/` | 🟡 待迁移 |
| 算法与组件 | `src/lib/` | 🟡 待迁移 |
| 板级生成区（CubeMX） | `platform/<vendor>/<mcu>/<board>/cubemx/` | 🟡 待迁移 |
| 板级手写适配（Port） | `platform/<vendor>/<mcu>/<board>/port/` | 🟡 待迁移 |
| 宿主测试 | `test/host/`（从 ControllerCode `fd1edbb` 恢复） | ✅ 已恢复（P0 独立工程，13 项全绿；`ImuStream` 待重写） |
| 组织合规件 | `.foray-layer`、`CODEOWNERS`、`.github/workflows/ci.yml` | ✅ 已落地（P0，CI 第一版，见 Decision-20261026-01） |
| 链路协议生成头 | `src/third_party/foray_interfaces/generated/lower_link.hpp` | 🟡 待 vendor |
| 架构评审 | `docs/ai/reviews/`（v1.0；旧版在 `archive/`） | ✅ 已确认 |
| 迁移基线证据 | `docs/ai/decisions/2026-10-25-迁移基线与对照.md`、`docs/ai/decisions/baseline-a2cfd16/`、`docs/ai/assets/controller_code-固件恢复可行性评估.md` | ✅ P0 已复核（SHA 一致，符号表入档） |
| 决策与基线 | `docs/ai/decisions/`、`decision.md` | 🟡 待补 |

## 迁移源对照（粗粒度）

> 左列来源路径都位于 `legacy/` 下。

| ControllerCode 现状 | foray_mcu 目标位置 |
|---|---|
| `Application/Entry.*` | 拆：`config/assembly/` + `platform/.../board/` + `app/common/` |
| `Application/Task/TaskManager.*` | 拆：`config/assembly/task_table.cpp`（数据）+ `app/common/`（机制） |
| `Application/Task/{0_imu,1_chassis,2_gimbal}` | `app/tasks/` + 对应能力目录 |
| `Application/Task/4_communication/RemoteReceiver.*` | `app/remote/`（注意：当前**不是**独立线程） |
| `Application/Imu/*`、`Application/Module/*` | `app/imu/`、`app/gimbal/`；`ImuBodyMapping` → `src/driver/imu/` |
| `Libraries/Algorithm/*`、`Component/*` | `src/lib/` |
| `Libraries/Device/*`、`Protocol/*` | `src/driver/`（设备与设备协议） |
| `Platform/Interface/*.hpp` + `Detail/*.h` | `src/hal/*.hpp`；`Detail/*.h` 过渡期放 `src/hal/detail/`，P3a 并入 C++ 契约后删除 |
| `Platform/PortKit/*` | 拆：`src/utils/`（队列/通知 → 并发原语）+ `src/hal/os/`（临界区契约）+ `platform/.../port/`（实现） |
| `Platform/STM32H7/DM_MC02/{Core,Drivers,Middlewares,USB_DEVICE,cmake,startup,.ld,.ioc}` | `platform/st/stm32h7/board_dm/cubemx/` |
| `Platform/STM32H7/DM_MC02/Port/*` | `platform/st/stm32h7/board_dm/port/` |
| `Tests/`（历史，`fd1edbb`） | `test/host/` + `tool/` |
| `Tools/`、`scripts/`（被 ignore/丢失） | `tool/`（必须入库） |

详见评审报告附录 A。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`.git/` 等依赖与产物目录；
- STM32 HAL 驱动的逐文件清单（属第三方，只记 `platform/**/cubemx/` 一层即可）；
- 临时文件、缓存文件、日志文件；
- 未评审通过、尚未实施的结构（必须在标题/状态中明确标注"规划中"）。
