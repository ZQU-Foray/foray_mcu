# 架构判据与依赖规则（foray_mcu）

> **状态**：v1（2026-10-26，P0 创建；负责人评审即 PR 合并后生效，Decision-20261026-02）。本文件是"一段代码该放哪一层、能依赖谁"的唯一人读入口，供人工评审与 AI 代理使用。
> **权威来源**：[评审报告 v1.0](reviews/2026-10-25-foray_mcu-架构评审报告.md) §3.2–§3.3、§5，以及 [decision.md](../../decision.md) 中的 Accepted 条目。本文件与它们冲突时以后者为准，并同步修正本文件。
> **修改流程**：新增或修改规则、例外、白名单，先在 decision.md 登记，再改本文件和对应的 CI 检查（§9）。

## 0. 怎么用

- **写新代码前**：按 §2 判定位置，按 §4 核对依赖。
- **评审 PR**：对照 §4 的规则表和 §6 的例外登记。违反规则又不在例外表里的改动，不合并。
- **现阶段（P0）**：新结构代码还没有，`legacy/` 是冻结的迁移源（只允许机械操作）。规则大多从 P1 起由构建和 CI 执行，每条的生效阶段见 §4；当前 CI 已执行的检查见 §4.3。

## 1. 术语

| 术语 | 含义 |
|---|---|
| 框架 | `src/` + `platform/` + `cmake/`：与产品无关，能独立构建、能裸机构建，供其他小项目复用 |
| 产品 | `app/` + `config/`：本仓的 RM 下位机固件 |
| 能力（capability） | 一类行为，如 imu、chassis、gimbal、remote、link、launch。一个能力对应一个 target |
| profile | 一台具体机器人：用哪块板、哪些能力、什么参数，只含数据。兵种（`robot_type`）是 profile 的属性 |
| assembly（组合根） | 按 profile 构造对象、建任务表、注入参数、实现启动入口；只被最终固件链接 |
| 契约 | `src/hal`（硬件资源）、`src/hal/os`（临界区 + 时基）、`src/boot`（启动入口）中的接口头 |
| port | 某块板对契约的手写实现，可以直接使用厂商 HAL |
| board | 某块板的引脚、时钟、能力常量、设备 ID、上电初始化、链接脚本 overlay |
| 生成区 | CubeMX 产生的文件（`cubemx/`）：只能重新生成，不能手改 |

## 2. 分层判据：代码放哪里

### 2.1 按顺序问六个问题

第一个回答"是"的问题决定位置：

1. **是 CubeMX 或厂商产生的文件吗？** → `platform/<vendor>/<mcu>/<board>/cubemx/`，只能重新生成；厂商库同理，不手改。
2. **直接调用厂商 HAL、寄存器或 RTOS 实现吗？** → 实现契约的代码放 `port/`；引脚、时钟、能力常量、设备 ID、上电初始化放 `board/`。
3. **描述的是"某一台机器人"吗**（用哪块板、哪些能力、什么参数）？→ 数据放 `config/profiles/<profile>/`；对象构造、任务表、参数注入、启动入口的实现放 `config/assembly/`。
4. **是 RM 专用的行为吗**（底盘、云台、发射、遥控、上位机链路业务）？→ `app/<capability>/`；线程、节拍、外设独占这类任务外壳放 `app/tasks/` 或 `app/common/`。
5. **与板和产品都无关，并且至少两个产品真的会用吗？** → `src/` 的对应子层（§2.3）。只有一个产品用的，留在 `app/`，不为尚未出现的项目预先抽象。
6. **只为测试而存在？** → `test/`。**构建之外的工具**（烧录、调试、生成、检查）→ `tool/`。

### 2.2 一级目录

| 目录 | 回答的问题 | 可以依赖 | 不得依赖 |
|---|---|---|---|
| `app/` | 一类能力或行为怎么实现 | `src` 的契约与公共库；本层 `app/common` | `platform` 的实现与生成区；`config/assembly`；其他能力的内部实现（跨能力只经 `msg` 或契约） |
| `config/` | 一台具体机器人用哪些能力、什么参数、装在哪块板 | `app`（只有 assembly 链接）；`platform/board`（板能力） | 实现算法、协议或驱动；`profiles` 不得 include 任何代码头 |
| `platform/` | 某块板如何把契约落到 HAL 或寄存器；CubeMX 生成了什么 | `src/hal`、`src/hal/os` 契约；厂商 HAL、FreeRTOS、USB 栈、CMSIS | `app`、`config` 的任何头（boot 契约除外）；在公共头里暴露 HAL 类型 |
| `src/` | 与产品和板都无关的通用框架 | 层内白名单（§2.3）；标准库 | `platform` 实现、`app`、`config`、RTOS 与 HAL 头（`hal/os` 契约除外） |
| `test/` | 契约与行为能否脱离硬件验证 | 被测 target + `platform/mock` + ctest | 真实 port 与生成区；**不得进入固件链接图** |
| `tool/` | 烧录、调试、生成、检查怎么做 | 无构建期依赖 | 成为固件构建的必需步骤；被 `.gitignore` 忽略 |
| `docs/` | 分层判据、决策、证据链 | — | 与代码或 CI 规则不一致 |

### 2.3 子层依赖

框架（`src/`）：

| 子层 | 回答的问题 | 可以依赖 | 不得依赖 |
|---|---|---|---|
| `hal/` | 硬件资源的 C++ 契约（can、gpio、pwm、spi、time、uart、usb_cdc） | 标准库 | 其他层、OS、厂商头 |
| `hal/os/` | 最小 OS 契约：RAII 临界区守卫 + 单调时基（可选忙等延时） | 标准库 | 同上 |
| `boot/` | 生成区可以调用的启动入口（`extern "C"` 声明） | 标准库 | 同上 |
| `lib/` | 纯算法与组件（数学、控制、估计、滤波、运动学、状态机、CRC、斜坡） | 标准库、`lib` 内部 | `hal`、`driver`、`msg`、OS |
| `utils/` | 零平台工具与并发原语（计数器、最新值邮箱、SPSC 队列） | 标准库 | `hal`、`lib`、`driver`、OS |
| `driver/` | 设备驱动与设备协议 | `hal`、`lib`、驱动内协议 | `msg`、OS、`app` |
| `msg/` | 最新值邮箱 / SPSC 队列（v1）；topic 在 P3b 之后按需引入 | `utils`、`hal/os` 契约 | `driver`、`app` |
| `middleware/` | 链路适配等跨模块服务 | `msg`、`hal`、`lib`；链路适配可以依赖 `driver` | `app` |
| `third_party/` | vendor 进来的外部生成头（`foray_interfaces` 的 `lower_link.hpp`） | 标准库 | 手工修改 |

产品（`app/`）：

| 子层 | 可以依赖 | 不得依赖 |
|---|---|---|
| `app/<capability>` | `lib`、`driver`、`msg`、`middleware`、`hal` | OS、`platform` 实现、`config/assembly` |
| `app/tasks`、`app/common` | 上面全部 + `foray_os_freertos_<board>`（迁移期可用 `cmsis_os2.h`，终态用 FreeRTOS 原生 API） | `platform` 实现、`config/assembly` |

契约头（`hal`、`hal/os`、`boot`）里的类型只用 `enum class`、POD 结构与不透明 ID/句柄，不暴露厂商或 RTOS 类型。

### 2.4 容易放错的几类代码

- **型号固有参数与整车选择**（Q11）：电机、电调型号的厂商固有参数（电调档案、原生减速比）属于驱动层，`DjiMotorModelMap` 留在 `src/driver/motor/dji`；整车选哪个型号、接哪路 CAN、用什么 ID、增益与限幅，属于 `config`（profile 数据，assembly 注入）。
- **板级逻辑设备**：公共契约头不枚举"这台机器人有哪些设备"，只定义 API 和不透明 ID 类型；ID 常量由 `platform/*/board` 的头提供，驱动经 `Config` 接收，由 assembly 注入（P3a 落地）。
- **能力裁剪**：用 target 组合表达（assembly 只链接 profile 列出的能力，未链接的能力不进镜像），不用 `#if`。模块内部的差异用 `constexpr bool`、依赖注入或非虚接口表达。
- **新兵种、新行为、新板**：新兵种 = 新 profile（既有能力重组 + 参数）；新行为 = 新能力 target；新板 = 新 `platform` board。都不复制已有代码目录。
- **线程、队列、事件**：不进框架，只出现在 `app/tasks`、`app/common` 与 `config/assembly`。框架需要互斥或时间时只经 `src/hal/os`。
- **PortKit 的去向**：队列和通知 → `src/utils`；临界区契约 → `src/hal/os`；CMSIS 实现 → `platform/.../port`。
- **链路协议**：只用 vendor 的 `lower_link.hpp`，`app/` 与 `src/` 中不得手写帧或 CRC。

## 3. target 依赖图（目标态）

箭头指向被依赖方。

```text
EXECUTABLE  foray_firmware_<profile>
 ├─ STATIC       foray_assembly_<profile>        config/assembly（组合根：任务表、对象构造、boot 实现）
 │   ├─ INTERFACE foray_profile_<profile>        config/profiles（纯数据生成头，叶子，无依赖）
 │   ├─ INTERFACE foray_boot                     src/boot（C 兼容的启动契约头，extern "C"）
 │   ├─ STATIC    foray_app_<capability>         app/<capability>（集合由 profile 决定）
 │   │   ├─ INTERFACE/STATIC foray_app_common
 │   │   ├─ INTERFACE foray_hal                  src/hal（C++ 契约，由 port 直接实现）
 │   │   ├─ INTERFACE foray_hal_os               src/hal/os（临界区 + 单调时基；实现在 port）
 │   │   ├─ INTERFACE foray_utils                src/utils（零平台工具）
 │   │   ├─ STATIC    foray_lib_<pkg>            src/lib（纯算法与组件）
 │   │   ├─ STATIC    foray_driver_<device>      src/driver → foray_hal + foray_lib
 │   │   ├─ STATIC    foray_msg                  src/msg → foray_utils（+ foray_hal_os）
 │   │   └─ STATIC    foray_middleware_<svc>
 │   ├─ INTERFACE foray_os_freertos_<board>      FreeRTOS 头与宏（迁移期含 CMSIS-RTOS2；只对 app/tasks、app/common、assembly 可见）
 │   └─ OBJECT    foray_board_<board>            platform/<vendor>/<mcu>/<board>/board
 └─ OBJECT       foray_port_<board>              platform/.../port（实现 foray_hal 与 foray_hal_os；PRIVATE 依赖厂商代码）
     ├─ OBJECT   foray_mx_generated_<board>      cubemx/ 生成物（Core、USB_DEVICE、Middlewares）
     ├─ OBJECT   foray_rtos_<board>
     └─ OBJECT   foray_usb_<board>

宿主测试（不交叉编译）：
EXECUTABLE foray_test_<name>  →  被测 target + STATIC foray_platform_mock + ctest

框架独立构建 / 裸机构建（CI）：
只配置 src/ + platform/mock，不含 app/、config/，不提供任何 RTOS 头 → 全部框架 target 编译通过
```

- **只有最终固件 executable 可以链接 `foray_assembly_*`**；库和测试都不得链接。
- `foray_os_freertos_<board>` 是 INTERFACE，只暴露头和宏；FreeRTOS 的实现（OBJECT）只链入 `foray_port_<board>`。
- `foray_platform_mock` 用 C++ 实现 `foray_hal` 与 `foray_hal_os`，只供宿主测试；它是测试替身，不构成 `src` 对 `platform` 的依赖。
- **port 与 board 用 OBJECT 库，不用 STATIC**：port 中有覆盖 HAL 弱符号的回调。静态库里的目标文件只在有未解析引用时才被拉入，回调单独成文件后会被静默丢弃，中断回调退化成 HAL 的空实现。也可以用 CMake 3.24+ 的 `$<LINK_LIBRARY:WHOLE_ARCHIVE,...>`。

## 4. 依赖规则

### 4.1 执行方式

每个 target 用 PRIVATE include 之后，规则 1–8 的大部分由 CMake 本身保证：看不到的头就 include 不了。CI 只补三类检查：

- **target 图断言**：用 `cmake --graphviz` 导出依赖图，检查无环、`app` 到不了 port、`src` 到不了 `platform`、只有 executable 到得了 assembly、节点与边的集合符合 §3。
- **文本扫描**：`file(GLOB)`、全局 include、`#if FORAY_`、厂商与 RTOS 名、手写帧与 CRC、原子读改写。
- **哈希与符号检查**：生成区清单、vendor 头哈希、HAL 回调符号未被 C++ 改名。

检查脚本放在 `tool/ci/`（或 `cmake/checks/`），随规则的生效阶段逐步加入 `.github/workflows/ci.yml`。

### 4.2 规则表

规则条文与评审报告 §5.3 一致。

| # | 规则 | 主要检查方式 | 生效阶段 |
|---|---|---|---|
| 1 | `app` 不得链接或看见 `foray_port_*`、`foray_board_*`、`foray_mx_generated_*`、`foray_rtos_*`、`foray_usb_*`、`foray_assembly_*` | PRIVATE include + target 图断言 | P1（target 骨架）；`app` 代码在 P2 出现 |
| 2 | `src` 源码不得 include 或链接 `platform` 实现；`src` 的宿主测试可以链接 `platform/mock`；`src` 必须能只靠 mock 在宿主机构建 | target 图断言；框架独立构建 | P1（mock 最小实现）；框架独立构建 CI 在 P3a |
| 3 | `src/hal` 头（含 `os/`）不得出现 `stm32`、`cmsis`、`HAL_`、`FreeRTOS`、`cmsis_os2`，只允许标准库头；契约类型只用 enum class、POD 结构与不透明 ID/句柄 | 文本扫描 `src/hal/**` | P1（`src/hal` 建立）；P2 暂放的 `src/hal/detail/` 见例外 X-07 |
| 4 | `config/profiles/**` 不得 include 任何工程头；profile 是叶子 | 文本扫描；target 图断言（profile 无依赖） | P3b（profile 落地） |
| 5 | `foray_assembly_*` 是组合根：只有最终 executable 可以链接它；任何库与测试不得链接；它的头不得被其他 target include（boot 头除外） | target 图断言 | P1 |
| 6 | `test` 不得链接 `foray_port_*`、`foray_board_*`、`foray_assembly_*` 的真实实现；测试二进制不得进入固件链接图 | target 图断言 | P1 |
| 7 | `platform/**/cubemx/**` 不得 include `app/`、`config/`、`Application/` 的头；只允许 boot 与 board 契约头 | 文本扫描 | P3b（boot ABI 落地）；在此之前见例外 X-09 |
| 8 | 禁止 `file(GLOB)` 收集业务源码；禁止全局 `include_directories()`；每个 target 显式声明 PRIVATE/PUBLIC/INTERFACE | 文本扫描 CMake 文件 | P1（根 CMake）；`legacy/` 自带的构建文件见例外 X-12 |
| 9 | `app/` 与 `src/` 中禁止 `#if.*FORAY_`（能力裁剪用 target 组合）；`#if` 只允许出现在生成头与 platform 适配层 | 文本扫描 | P1 |
| 10 | 只有 `app/tasks`、`app/common` 与 `config/assembly` 可以 include RTOS 头（经 `foray_os_freertos_<board>`）；迁移期可用 `cmsis_os2.h`，终态改用 FreeRTOS 原生 API；`app/<capability>` 与整个 `src/` 零 RTOS，需要临界区或时间时只经 `foray_hal_os` | target 可见性 + 文本扫描 `src/**`、`app/<capability>/**` | P3a（OS 契约实体）；任务层改原生 API 在 P3b |
| 11 | 链路协议只允许来自 vendor 的 `lower_link.hpp`（固定版本 + sha256）；`app/` 与 `src/` 中禁止手写帧与 CRC | vendor 哈希校验 + 文本扫描 | P4 |
| 12 | `app`、`src`、`platform` 的一级目录都有白名单（§4.4）；新增需在 decision.md 登记 | 脚本比对白名单 | P1 |
| 13 | 框架代码（`src/`）不直接使用 `std::atomic` 的读改写操作（`fetch_*`、`exchange`、`compare_exchange_*`），一律用 `src/utils` 的并发原语（Cortex-M0+ 没有独占访问指令，做不到无锁） | 文本扫描 `src/**` | P3a（并发原语落地）；迁移期见例外 X-08 |
| 14 | 框架代码开启 `-Wdouble-promotion`；豁免清单只含 IMU 姿态解算中有意使用 double 的文件，新增豁免需在 decision.md 登记 | 编译选项（`cmake/foray_targets.cmake`） | P3a；豁免清单见例外 X-06 |
| 15 | 框架（`src/`、`platform/`、`cmake/`）不得引用产品（`app/`、`config/`）的任何路径或 target；框架必须能独立配置和构建 | 文本扫描 + 框架独立构建 | P1 起扫描；框架独立构建 CI 在 P3a |
| 16 | C 链接（`extern "C"`）只允许出现在 §8.3 列出的三处；覆盖 HAL 弱符号的 `.cpp` 必须 include 对应的 HAL 头，让回调继承 C 链接 | `nm` 检查 `HAL_*Callback` 符号未被改名 | P3a（port 改写为 C++）；`legacy/` 的 C 接缝见例外 X-07 |

### 4.3 当前 CI 已执行的检查（P0）

见 `.github/workflows/ci.yml` 与 Decision-20261026-01：

| 作业 | 检查内容 | 有效期 |
|---|---|---|
| `lint-c` | `tool/ci/format.py --check`：全部手写 C/C++ 文件必须符合 `.clang-format`（18.1.8）；排除生成区、厂商代码与 vendor 生成头 | 长期（P0.5 起全量，Decision-20261026-04） |
| `host-tests` | `test/host` 的 13 项 ctest | 长期 |
| `firmware-legacy` | 构建 `legacy/` 的 Debug 与 Release；**Release 镜像必须与基线 SHA-256 逐字节一致** | P0–P0.5；P1 起改为根工程构建 + 符号名与大小集合对照 |

### 4.4 一级目录白名单（规则 12）

| 目录 | 白名单 |
|---|---|
| `app/` | `common`、`imu`、`chassis`、`gimbal`、`remote`、`link`、`launch`、`debug_comm`（临时，调优后删除）、`tasks` |
| `src/` | `hal`（含 `os`）、`boot`、`lib`、`driver`、`msg`、`middleware`、`utils`、`third_party` |
| `platform/` | `st`、`ti`、`mock`；其下按 `<vendor>/<mcu>/<board>/{cubemx,port,board}` 组织（如 `st/stm32h7/board_dm`、`st/stm32f4/board_c`） |

`src/bsp/` 不在白名单内：仓库里 7 个 0 字节占位头（未入 git）在 P1 建 `src/hal` 时删除。

## 5. 生成区与 vendor 治理

- `platform/**/cubemx/**` 只放生成物（Core、Drivers、Middlewares、USB_DEVICE、`.ioc`、startup、`.ld`），手写代码 0 个；`port/`、`board/` 在物理上分开。
- 生成区只能通过重新生成修改，禁止手工编辑；生成区变更须先在 decision.md 登记，PR 中单独说明（当前只有电控组一个 team，CODEOWNERS 无法单独分派）。
- 再生成可复现：CI 默认用 `generated.manifest`（sha256 清单）校验；有装 CubeMX 的 runner 时再加 regenerate-diff。
- 现有板级 CMake 的两条再生成防护保留：USB OTG HS 内部 DMA 必须开启；UsbCdc port 必须含 PA11/PA12 复用配置。
- 链接脚本：CubeMX 生成的 `.ld` 冻结不改，自定义段放 `platform/.../board/link/overlay.ld`，作为第二个 `-T` 追加；CI 验证再生成后段仍在（Q9，P4 落地）。
- vendor 生成头：`src/third_party/foray_interfaces/` 固定版本并记录 sha256，CI 校验哈希；不手改。
- 层级声明：本仓 `.foray-layer` 为 `layer: MCU`、`depends_on_layers: [X2]`，对 X2（`foray_interfaces`）只消费 vendor 的生成头，没有构建期依赖。

## 6. 例外登记

本表是全部已批准例外的唯一清单。新增例外先在 decision.md 登记，再加入本表；到期项完成后从表中移到 decision.md 的执行记录。

| 编号 | 例外 | 涉及规则 | 依据 | 处置与到期 |
|---|---|---|---|---|
| X-01 | **UsbCdc 边界**：USB 生成代码（`usb_device.c`、`usbd_cdc_if.c`）直接调用 port 私有 ABI（`UsbCdcPort_*`），`Interface/UsbCdc.hpp` 在业务中零消费者。按 Q8 ①，UsbCdc 视为板私有能力，不纳入 `src/hal` 公共契约；中长期按 Q8 ② 改为 HAL 回调注册 | 7 | Q8 | P2.5 原样搬到 `platform/.../port/usb_cdc.c`；P4 按 Q8 重划边界 |
| X-02 | **生成区链接 seam**：`-Wl,--wrap=HAL_PCD_EP_Transmit` 保留 | 16 | 评审报告 §5.6、Q8 | P4 随 X-01 改为回调注册后删除 |
| X-03 | **链接脚本 overlay**：自定义段（如 `.dma_buffer.imu_diag`）不改生成的 `.ld`，用第二个 `-T` 追加 | 生成区冻结 | Q9 | P4 落地 |
| X-04 | **OS 抽象范围**：框架只含临界区 + 单调时基；线程、队列、事件只在产品层；迁移期任务外壳可继续用 `cmsis_os2` | 10 | Q10、Q12 | 任务层在 P3b 改用 FreeRTOS 原生 API |
| X-05 | **Dji 型号映射**：`DjiMotorModelMap`（型号 → 电调档案、原生减速比）是品牌默认值，留在驱动层 | 2.4 判据 | Q11 | 长期有效 |
| X-06 | **`-Wdouble-promotion` 豁免**：`Vqf.cpp`（现位于 `legacy/Libraries/Algorithm/alg_estimate/`）。该文件同时须有"目标无双精度 FPU 即 `#error`"的检查 | 14 | Decision-20261025-03 | 长期有效；第一个在 M4F 板上链接姿态解算的 profile 出现时重新评估 |
| X-07 | **迁移期的 C ABI 层**：`Platform/Interface/Detail/*.h` 在 P2 暂放 `src/hal/detail/`（过渡 target `foray_hal_detail`），port 与 PortKit 仍是 C | 3、16 | Q17、评审报告 §5.11 | P3a 合并进 C++ 契约后删除 |
| X-08 | **迁移期的原子读改写**：框架侧只有 `DjiMotor.cpp` 有 2 处读改写；产品侧 `ImuDataReadyMailbox.hpp`（3 处）、`ImuTask.cpp`（1 处）、`RemoteReceiver.cpp`（2 处） | 13 | 评审报告 §5.8 | P3b 迁移到 `src/utils` 并发原语 |
| X-09 | **生成区引用应用头**：`main.c`、`freertos.c` 的 USER CODE 区 include `Application/Entry.h` 并调用 `Application_*`；P2.5 起实现位于 `config/assembly/entry_legacy.*` | 7 | 评审报告 M4 | P3b 落地 boot ABI 后删除 |
| X-10 | **宿主测试的 P0 独立工程**：`test/host/CMakeLists.txt` 不链接被测 target 与 `platform/mock`，而是直接编译 `legacy/` 源码，用影子头替换 `cmsis_os2.h` 与 DJI 映射 | §2.2 `test` 行 | `test/host/README.md` | P1 改由根工程引入，改为链接被测 target |
| X-11 | **过渡 include 根**：P1 设 `foray_legacy_include`，暴露 `legacy/` 作为 include 根（零源码改动无法去掉旧路径） | 8 | 评审报告 §7.2 P1 | P2 结束时删除 |
| X-12 | **冻结的迁移源**：`legacy/` 在 P0–P2.5 期间不受目标态目录规则约束（自带 `file(GLOB)` 构建、全局 include），只允许机械操作；由 CI 的基线对照守护 | 8、12 | Decision-20261025-05 | 随 P2/P2.5 搬空后删除 |
| X-13 | ~~**C/C++ 格式增量检查**：P0.5 之前 lint-c 只查改动文件~~ | 格式规则 | Decision-20261026-01 | **已关闭**（2026-10-26，P0.5 格式化后改为全量检查，Decision-20261026-04） |

**已知结构债**（违反目标规则，不是例外，按计划偿还，见 [plan.md](../../plan.md) P3a、P3b）：

- `ImuBodyMapping` 反向依赖 Device：改为 POD 入参，或迁到 `src/driver/imu/`。
- `Application/Imu` 的时钟读取与设备类型依赖。
- 任务表外的 `defaultTask` 承载应用周期；栈巡检只遍历表内任务。
- `Entry.cpp` 检查 3 个无消费者的端点，却漏检实际使用的 Pwm。
- SPI 异步能力在契约注释里标为可选（MAY），BMI088 与 WS2812 却当作必选使用且没有回退（评审报告 §2.3 缺口 1）；P3a 契约 v1 中处理。
- 其他已知问题：迁移源架构图的 F1–F13、C1–C8 清单，见评审报告附录 B（P3b/P4 验收项）。

## 7. 框架与产品边界、平台可移植性

### 7.1 边界

- 框架 = `src/` + `platform/` + `cmake/` + 框架自己的测试；产品 = `app/` + `config/`。规则 2、15 保证框架不引用产品。
- 同仓开发，暂不拆仓。第一个小项目启动时（P5），用 `git subtree split` 把框架提成独立仓库并保留历史。
- 框架只收至少两个产品真正共用的东西；底盘、云台、发射等 RM 专用能力留在 `app/`。
- 现阶段就要守住的三条：`src/` 零 RTOS；不直接用原子读改写；全部静态分配（无堆）。

### 7.2 平台差异对框架代码的约束

| 平台 | 内核 | 硬件双精度 | 无锁原子读改写 | 周期计数器 | 中断屏蔽 | RTOS |
|---|---|---|---|---|---|---|
| STM32H723（DM-MC02） | Cortex-M7 | 有 | 有 | DWT | PRIMASK / BASEPRI | FreeRTOS |
| STM32F407（C 板） | Cortex-M4F | **无** | 有 | DWT | PRIMASK / BASEPRI | FreeRTOS |
| TM4C | Cortex-M4F | **无** | 有 | DWT（按型号确认） | PRIMASK / BASEPRI | FreeRTOS |
| AM26x | Cortex-R5F | 有（GCC 按 `vfpv3-d16`；按具体型号确认） | 有 | PMU（无 DWT） | **CPSR I/F 位（无 NVIC、PRIMASK、BASEPRI）** | FreeRTOS |
| MSPM0 | Cortex-M0+ | **无 FPU** | **无** | 无 DWT | 仅 PRIMASK | 裸机 |

由此得出的约束：

- **不假设 DWT、PRIMASK 或 CMSIS**：时间和临界区只经 `src/hal/os`，实现留在 port。
- **不用原子读改写**：规则 13。
- **double 只在豁免文件里用**：规则 14。M4F 上 double 运算会编译成软件库调用。
- **GNU 专属选项只出现在工具链或平台 CMake 中**：`--specs=nano.specs`、`-Wl,--wrap=`、GNU 链接脚本语法等。AM26x 可能使用 TI Arm Clang（Q13），代码要能同时通过 GCC 与 Clang。
- **全局对象**：MSPM0 裸机的启动文件必须调用全局构造函数（`__libc_init_array`），P5 检查。

## 8. 语言、命名与格式

### 8.1 语言

- C++17，`-fno-rtti -fno-exceptions -fno-threadsafe-statics`；无堆分配；热路径不用虚函数。
- 契约统一用 C++（`src/hal/*.hpp` 直接作为 port 要实现的接口：enum class、POD 结构、非虚自由函数），枚举只定义一份。
- 文件编码 UTF-8（无 BOM）；注释、文档、提交信息用简体中文。

### 8.2 命名（ROS 2 C++ 风格，由根目录 `.clang-tidy` 执行）

| 元素 | 规则 | 示例（现名 → 新名） |
|---|---|---|
| 文件、目录 | snake_case；C++ 头文件 `.hpp`、实现 `.cpp` | `ChassisTask.cpp` → `chassis_task.cpp` |
| 命名空间 | snake_case | `alg_math`、`device` |
| 类、结构体、枚举类型、类型别名、模板参数 | PascalCase | `DjiMotor` |
| 函数、方法 | snake_case | `SetWheelTarget` → `set_wheel_target` |
| 局部变量、参数 | snake_case | `keyLevel` → `key_level` |
| 私有、受保护成员 | snake_case + 尾下划线 | `transferActive_` → `transfer_active_` |
| 结构体公开字段 | snake_case，不加后缀 | `eulerDegrees` → `euler_degrees` |
| 全局变量（含匿名命名空间） | `g_` + snake_case | `snapshotSequence` → `g_snapshot_sequence` |
| 编译期常量、全局与静态常量 | UPPER_SNAKE_CASE | `StackHealthSamplePeriodMs` → `STACK_HEALTH_SAMPLE_PERIOD_MS` |
| 枚举值、宏 | UPPER_SNAKE_CASE | `Channel::Channel1` → `Channel::CHANNEL_1` |
| 头文件保护宏 | 路径大写 + 双下划线 | `FORAY__HAL__CAN_HPP_` |
| 本项目自有的 C 链接函数 | 模块前缀 + snake_case | `foray_boot_init` |

不改名的：HAL 回调与中断入口（`HAL_*Callback`、`*_IRQHandler`）、CubeMX 生成代码、P3a 会被合并掉的 C ABI（`CanPort_*` 等）。

### 8.3 C 链接只允许三处（规则 16）

| # | 位置 | 原因 | 写法 |
|---|---|---|---|
| 1 | 生成区调用的启动入口 `foray_boot_*` | `main.c`、`freertos.c` 是 C 文件 | `src/boot/foray_boot.h` 用 `extern "C"` 声明，实现写在 `config/assembly` 的 C++ 里 |
| 2 | 覆盖 HAL 弱符号的回调 `HAL_*Callback` | HAL 按 C 符号名调用 | 写在 port 的 `.cpp` 里，并 include 对应 HAL 头，让定义继承 C 链接；否则回调被 C++ 改名，HAL 弱实现照旧生效，编译链接都不报错 |
| 3 | 中断入口 `*_IRQHandler` | 向量表按 C 符号名引用 | 保持在生成区的 C 文件里，不动 |

### 8.4 格式（组织规则，由根目录 `.clang-format` 执行）

- 4 空格缩进、LF、UTF-8、文件末尾保留一个换行；Allman 大括号、行宽 120、指针靠右（沿用迁移源 `Libraries/Algorithm/.clang-format`）。
- clang-format 与 clang-tidy 统一用 18（CI 固定 clang-format 18.1.8）。ROS 2 中属于排版的部分（2 空格缩进、行宽 100 等）不采用。
- 生成区与厂商代码不格式化。格式化范围由 `tool/ci/format.py` 统一定义（CI 与本地共用）；不使用 `.clang-format-ignore`，因为 clang-format 18 的该文件不支持 `**`、`*` 也不跨目录（Decision-20261026-04）。
- 格式化与改名必须各自单独提交，提交号写入 `.git-blame-ignore-revs`；`.clang-format`、`.clang-tidy` 的规则调整记入 decision.md。
- 时间表：P0.5 一次性格式化手写代码并改名（含 `test/host` 中恢复的测试）；P2 搬迁时文件名改为 snake_case；P3a 改写 port 时直接按新规则命名；命名检查（clang-tidy）在 P0.5 之后加入 CI。

## 9. 修改本文件

1. 规则、白名单、例外的任何增删改，先在 [decision.md](../../decision.md) 登记（Context / Decision / Reason / Alternatives）。
2. 再修改本文件的对应小节，并同步 `.github/workflows/ci.yml` 或 `tool/ci/` 中的检查。
3. 评审报告是历史记录，不随本文件修改；两者出现差异时，以 decision.md 中较新的 Accepted 条目为准。
