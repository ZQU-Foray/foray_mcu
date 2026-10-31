# 架构判据与依赖规则（foray_mcu）

> **状态**：v2.3（2026-10-31）。v2.0 见 Decision-20261027-01；v2.1 随 F1 第 1 步更新 utils、middleware、lib 的归类：math 进 utils，fsm 进 middleware（Decision-20261027-02）；v2.2 随 F1 第 2 步加入 utils 的并发原语 `concurrency`（Decision-20261029-01），§3.4 补充 seqlock 的读写竞态（Decision-20261029-02，待定）；v2.3 随 F1 第 3 步定稿 §3.2 的 OS 契约：1 µs 时基、`Signal` 的语义、临界区只屏蔽框架中断（Decision-20261031-01～03）。v2 按负责人的骨架重写：迁移方式从"改造迁移源"改为"按新骨架重建，迁移源只作参考与零件来源"。v1.0–v1.4（P0–P2.5）见 git 历史与 decision.md。
> **权威来源**：[decision.md](../../decision.md) 中的 Accepted 条目，v2 起以 Decision-20261027-01 为准。[评审报告 v1.0](reviews/2026-10-25-foray_mcu-架构评审报告.md) 是历史记录，与本文件冲突的部分以本文件为准。
> **修改流程**：新增或修改规则、例外、白名单，先在 decision.md 登记，再改本文件和对应的 CI 检查（§10）。

## 0. 怎么用

- **写新代码前**：按 §2 判定位置，按 §3 的机制写，按 §5 核对依赖。
- **评审 PR**：对照 §5 的规则表和 §7 的例外登记。违反规则又不在例外表里的改动，不合并。
- **现阶段**：P0–P2.5 已完成（源码已搬出 `legacy/`）；下一步是 F1 框架基础件（[plan.md](../../plan.md)）。现有代码中不符合本文件的部分（`hal/detail` 的 C 接口、旧任务外壳、`Application_*` 入口等）登记在 §7，按到期阶段清理，不再为它们调整新设计。

## 1. 原则与术语

### 1.1 原则

1. **以负责人的骨架为准重建**。迁移源只作参考与零件来源：写得好的算法和驱动（PID、VQF、EKF、BMI088、DJI 电机协议等）直接搬来，按新契约适配；不为保持旧结构做妥协。
2. **先框架，后能力**。框架（`src/` + `platform/` + `cmake/`）建成并验证好用之后，再在其上重建能力；F407 C 板在 DM-MC02 重建完成后再做。
3. **按模块验证**。宿主单元测试、`test/data` 录制数据回放、实机检查清单。"与旧固件逐符号一致"的判据只用到 P2.5。
4. **框架不含产品**。`src/`、`platform/`、`cmake/` 不引用 `app/`、`config/`。框架要能给其他小项目（如 MSPM0 裸机）复用。
5. **静态与确定性**。无堆分配；C++17，`-fno-rtti -fno-exceptions -fno-threadsafe-statics`；热路径不用虚函数。

### 1.2 术语

| 术语 | 含义 |
|---|---|
| 框架 | `src/` + `platform/` + `cmake/`：与产品无关，能独立构建，能裸机构建 |
| 产品 | `app/` + `config/`：本仓的 RM 下位机固件 |
| 能力（capability） | 一类行为，如 imu、chassis、gimbal、remote、launch、link。一个能力对应一个 target |
| topic | 一种数据的发布/订阅通道：最新值语义，单发布者（§3.4） |
| profile | 兵种 × 板：用哪块板、链接哪些能力、什么参数。只含数据 |
| assembly（组合根） | 按 profile 构造对象、注入参数、定义任务表、实现启动入口；只被最终固件链接 |
| 契约 | `src/hal`（硬件资源）、`src/hal/os`（临界区、时基、信号）、`src/boot`（启动入口）中的接口头 |
| port | 某块板对契约的 C++ 实现，可以直接使用厂商 HAL |
| board | 某块板的物理资源句柄、能力常量、引脚、链接脚本 overlay |
| 生成区 | CubeMX 产生的文件（`cubemx/`）：只能重新生成；USER CODE 区只允许写启动入口调用（§6） |

## 2. 目录与分层

### 2.1 目录总览

```text
foray_mcu/
├── src/                      框架：与产品、板都无关；零 RTOS；只靠 platform/mock 就能在宿主机构建
│   ├── hal/                  硬件契约（C++ 头）：can、spi、uart、gpio、pwm、time、usb_cdc
│   │   └── os/               OS 契约：临界区、单调时基、信号（等待/唤醒）
│   ├── boot/                 启动入口 boot.h（extern "C"）：生成区只调它
│   ├── utils/                无领域含义的基础件：math、crc、concurrency（SPSC 队列、最新值邮箱）
│   ├── middleware/           给能力用的运行期机制与服务：建立在契约与基础件之上
│   │   ├── topic/            topic 机制：最新值、单发布者、可在中断里发布、可阻塞等待
│   │   └── fsm/              通用状态机
│   ├── lib/                  领域算法：control（pid、slope）、estimate、filter、kinematics、component
│   ├── driver/               设备驱动与设备协议：bmi088、motor（dji）、buzzer、ws2812、protocol/{dji,sbus}
│   └── third_party/          vendor 进来的外部生成头（foray_interfaces 的 lower_link.hpp）
├── platform/                 按板一级
│   ├── dm_mc02/              DM-MC02（STM32H723）：board.cmake + CMakeLists.txt
│   │   ├── cubemx/           CubeMX 生成物（冻结）
│   │   ├── port/             hal 与 hal/os 的 C++ 实现、HAL 回调
│   │   └── board/            board.hpp：物理资源句柄、能力常量；链接脚本 overlay
│   └── mock/                 hal 与 hal/os 的宿主替身：测试控制时间、注入外设输入、检查输出
├── app/                      产品：RM 能力
│   ├── topics/               本产品全部 topic 的清单（消息类型 + topic 实例），集中一处
│   ├── common/               任务运行器（FreeRTOS 原生 API）
│   └── <capability>/         imu、chassis、gimbal、remote、launch、link……（框架验收后重建）
├── config/
│   ├── profiles/<兵种>_<板>/   profile.cmake（选板、选能力）+ params.hpp（constexpr 参数）
│   └── assembly/             组合根：每个 profile 一个源文件
├── test/host/、test/data/    宿主测试、录制数据
├── tool/                     烧录、调试、CI 检查；将来的 topic 清单生成器
└── docs/
```

### 2.2 代码放哪里

按顺序问，第一个回答"是"的问题决定位置：

1. **是 CubeMX 或厂商产生的吗？** → `platform/<board>/cubemx/`，只能重新生成；厂商库同理，不手改。
2. **直接调用厂商 HAL、寄存器或 RTOS 实现吗？** → 实现契约的放 `platform/<board>/port/`；板的物理资源、能力常量、引脚放 `platform/<board>/board/`。
3. **描述某个兵种 × 板吗**（用哪些能力、什么参数）？→ 数据放 `config/profiles/`；对象构造、参数注入、任务表、启动入口的实现放 `config/assembly/`。
4. **是 RM 专用的吗？** → 行为放 `app/<capability>/`；产品的 topic 定义放 `app/topics/`；线程与调度机制放 `app/common/`。
5. **与产品和板都无关吗？**
   - 没有领域含义、哪一层都可能用（数学、容器、并发原语、校验）→ `src/utils/`；
   - 领域算法（控制、估计、滤波、运动学、执行器骨架）→ `src/lib/`；
   - 设备驱动与设备协议 → `src/driver/`；
   - 给能力用的运行期机制与服务（发布/订阅、状态机；将来的调试遥测、参数存储、链路传输）→ `src/middleware/<名>/`。topic 只有机制，不含任何具体 topic。

   只有一个产品用的，留在 `app/`，不为尚未出现的项目预先抽象。
6. **只为测试而存在？** → `test/`。**构建之外的工具**（烧录、调试、生成、检查）→ `tool/`。

### 2.3 框架（`src/`）子层

| 子层 | 内容 | 可以依赖 | 不得依赖 |
|---|---|---|---|
| `hal/` | 硬件资源契约（§3.1） | 标准库、`hal/os` | 厂商头、RTOS、其他层 |
| `hal/os/` | 临界区、单调时基、信号（§3.2） | 标准库 | 同上 |
| `boot/` | 启动入口声明（§3.3） | 标准库 | 同上 |
| `utils/` | 数学（向量、四元数、矩阵）、CRC、并发原语（`concurrency`：SPSC 队列、最新值邮箱 seqlock）。每个子目录一个 target `utils_<名>` | 标准库 | 其他所有层（包括迁移期的构建开关 `legacy_build_options`） |
| `middleware/` | 给能力用的运行期机制与服务：`topic`（§3.4）、`fsm`（通用状态机）；将来按需加调试遥测、参数存储、链路传输，新增先登记 | `utils`、`hal/os`、中间件内部（无环）；传输类服务另可依赖 `hal` | `driver`、`lib`、`app`、RTOS |
| `lib/` | 领域算法；层内无环（`component` → `control`）；数学基础在 `utils/math` | 标准库、`utils`、`lib` 内部 | `hal`、`driver`、`middleware` |
| `driver/` | 设备驱动与设备协议 | `hal`、`hal/os`、`lib`、`utils`、驱动内部 | `middleware`、`app`、RTOS |
| `third_party/` | vendor 的外部生成头，固定版本 + sha256 | 标准库 | 手工修改 |

- 驱动不发布 topic：驱动要能脱离本产品的 topic 清单复用，由能力把驱动的数据发布出去。
- topic 只依赖 `utils` 与 `hal/os`，裸机与宿主测试都能用。
- `middleware` 只收与产品无关、给能力用的运行期机制与服务（topic、状态机等）。硬件契约、无领域基础件、算法、设备驱动各有其层，不放这里，避免它变成杂物间。
- driver 与 lib 不依赖 middleware：状态机是给能力组织模式与流程用的；驱动内部的少量状态自己处理（Decision-20261027-02）。

### 2.4 产品（`app/`、`config/`）

| 位置 | 内容 | 可以依赖 | 不得依赖 |
|---|---|---|---|
| `app/topics/` | 全部 topic 的消息类型与实例（§3.4） | `middleware/topic`、标准库 | 能力实现、RTOS、`lib` 类型（消息只用标准标量与定长数组） |
| `app/<capability>/` | 能力实现（§3.5） | `hal`、`hal/os`、`utils`、`middleware`、`lib`、`driver`、`app/topics` | RTOS、`platform`（含 `board.hpp`）、`config`、其他能力 |
| `app/common/` | 任务运行器 | FreeRTOS 原生 API（`os_freertos_<board>`）、`hal/os` | 能力实现、`platform` 实现 |
| `config/profiles/` | `profile.cmake` 与 `params.hpp`（§3.6） | 能力的配置类型头、标准库 | 任何逻辑代码 |
| `config/assembly/` | 组合根（§3.6） | 全部能力、`app/common`、profile、`board.hpp`、`boot`、`os_freertos_<board>` | 被固件以外的任何 target 依赖 |

## 3. 关键机制

### 3.1 硬件契约 `src/hal`

- **风格**：命名空间 + 自由函数 + 不透明句柄，链接期绑定。一次构建只链接一块板的 port，没有虚函数，没有运行期分派。示意（F2 定稿）：

  ```cpp
  namespace hal::can
  {
  struct Bus
  {
      std::uint8_t index;  // 不透明句柄，值由 board.hpp 提供
  };
  enum class SendResult : std::uint8_t { QUEUED, QUEUE_FULL, NOT_READY, BUS_OFF, INVALID_ARGUMENT };
  SendResult send(Bus bus, const Frame &frame) noexcept;
  bool receive(Bus bus, Frame &frame) noexcept;  // 从 port 的接收队列取一帧
  void set_receive_notification(Bus bus, Notification notification, void *context) noexcept;
  }
  ```

- **设备不进契约**：契约头不枚举"这台车有哪些设备"。`platform/<board>/board/board.hpp` 给出物理资源句柄（如 `board::CAN_1`、`board::SPI_IMU`）。整车用哪个资源、接哪个设备，由 profile 决定，assembly 注入驱动的 `Config`。
- **能力分级**：每个外设接口分必选与可选。可选能力由 `board.hpp` 的 `constexpr` 常量声明（如 `board::SPI_ASYNC`），使用方用 `static_assert` 检查或提供回退，不留到运行期才发现。
- **类型**：只用 `enum class`、POD 结构与句柄，不出现厂商或 RTOS 类型。每个函数注明能否在中断里调用；每个回调注明在什么上下文执行。
- **初始化**：`hal::init()` 由 port 实现，assembly 在 `boot_init()` 里调用；生成区不直接调用 port。
- 现有的 `src/hal/detail`（C 接口）和 `platform::Can` 这类包装类在 F2 删除。

### 3.2 OS 契约 `src/hal/os`

契约 v1 于 F1 第 3 步定稿（Decision-20261031-01～03）。

- **时基**（`clock.hpp`）：
  - `now_us()` 返回 `Microseconds`（`std::uint32_t`），单位固定 1 µs，按 2^32 回绕（约 71.6 分钟）；任务、中断里都能调用；
  - 换算辅助 `elapsed_us`、`has_elapsed`、`to_seconds` 是 `constexpr`，两个时刻一律用无符号差比较；
  - topic 的时间戳、`Signal` 的超时都用这个单位。周期级计数器与忙等延时不进 v1。
- **框架中断与框架线**：每块板一条框架线，RTOS 板与 RTOS 允许调用中断 API 的最高优先级相同（DM-MC02 为优先级 5）。不高于这条线的中断是框架中断，可以使用框架；线以上的中断不得调用 RTOS，也不得使用框架（`hal::os`、topic），只能自己处理数据，再经框架中断或任务转交。
- **`CriticalSection`**（`critical_section.hpp`）：
  - RAII 守卫，构造时保存原状态并屏蔽全部框架中断，析构时恢复，可嵌套；任务与框架中断里都能用；
  - Cortex-M3/M4/M7 用 BASEPRI，Cortex-M0+ 用 PRIMASK，Cortex-R5 屏蔽 IRQ；
  - 临界区内不得阻塞，不得调用 RTOS API 与 `Signal`。
- **`Signal`**（`signal.hpp`）：二值信号，`notify()` 留下一个通知，`wait(timeout) -> bool` 取走它，没有时睡眠到通知或超时：
  - 通知不丢：`notify()` 早于 `wait()` 时，下一次 `wait()` 立即返回 true；多次 `notify()` 合并成一次；
  - 一次一个等待者：每次 `wait()` 时登记、返回时注销。不同任务可以先后等同一个 `Signal`，一个任务可以先后等多个；
  - true 只说明"上次取走之后有过 `notify()`"，调用方醒来后重新检查条件；
  - `notify()` 在任务与框架中断里都能调用，`wait()` 只在任务里；超时常量 `NO_WAIT`、`WAIT_FOREVER`；
  - `constexpr` 构造，可以常量初始化。
- **实现**：`platform/<board>/port`（F2）：时基用板上的 32 位定时器分频到 1 MHz（DM-MC02 为 TIM2）；`Signal` 用 FreeRTOS 的 1 号任务通知槽，裸机板用标志位加 WFI。`platform/mock`：时间由测试推进，临界区是一把进程内的锁，`Signal` 用条件变量并检查误用。
- 线程、队列、事件组不进框架。

### 3.3 启动入口 `src/boot`

```c
bool boot_init(void);       // main.c：外设初始化后、内核初始化前
bool boot_init_rtos(void);  // freertos.c：创建任务
void boot_run(void);        // 默认任务：留给 assembly 的周期工作（可为空）
```

- `src/boot/boot.h` 用 `extern "C"` 声明，实现在 `config/assembly`。
- CubeMX 生成的 `main.c`、`freertos.c` 只在 USER CODE 区调用这三个函数。CubeMX 再生成时会保留 USER CODE 区；这是生成区唯一允许的手工内容（§6）。

### 3.4 topic（`src/middleware/topic`）

**语义**：

| 项 | 规定 |
|---|---|
| 数据 | 每个 topic 一种消息类型；发布覆盖旧值，订阅者读最新一份。不排队，不保证每条都收到；丢了几次可以观测 |
| 发布者 | 每个 topic 只有一个发布者。取得发布句柄的操作每个 topic 只能成功一次，在初始化阶段完成，失败则 `boot_init()` 失败 |
| 中断 | 允许在框架中断里发布（§3.2）；中断里不订阅（理由见下方"读写竞态"） |
| 订阅 | `update(msg) -> bool`：有新数据才拷贝；`wait(msg, timeout) -> bool`：阻塞到下一次发布 |
| 消息 | 可平凡拷贝（`static_assert`），只用标准整数、浮点、`bool` 与定长数组，尽量小 |
| 观测 | 每个 topic 的发布次数、最后发布时刻；每个订阅者的丢失次数。调试通道可以列出全部 topic |
| 不做 | 队列、多发布者、运行期发现、回调式订阅（回调会在发布者的上下文里执行，中断里不可控） |

**实现要点**：
- 存储用单写者 seqlock（`utils/concurrency` 的 `LatestValue`）：只用原子读写与内存屏障，没有原子读改写，Cortex-M0+ 也能用（规则 13）。
- **读写竞态**：读端遇到写入中途会重试，直到写端写完，所以读端不能在写入中途抢占写端，否则单核上写端再也轮不到运行，读端无限重试。中断读任务发布的 topic 是一种情况；高优先级任务读低优先级任务发布的 topic 是另一种，同样会卡死。处理方式待定（Decision-20261029-02，Proposed），AI 建议发布时用 `hal::os::CriticalSection` 包住写入；临界区已定为只屏蔽框架中断（§3.2），选这个方案时线以上的中断既不能发布也不能读 topic，与 §3.2 的规则一致。决定之前：中断里不订阅，订阅者的优先级不高于发布者。
- 等待槽数量是编译期常量；订阅者用自己的 `hal::os::Signal` 注册，发布时逐个通知（在临界区外调用 `notify()`）。
- topic 对象常量初始化，不依赖全局构造顺序。

**清单**：
- 产品的全部 topic 集中定义在 `app/topics/`，按发布者所属的能力分文件、分命名空间。
- topic 的单位是"一种数据"，归发布它的能力所有。一个能力可以发布多个 topic（如 `topics::imu::attitude`、`topics::imu::raw`），也可以订阅任何 topic。
- 框架验证好用之后，清单改为从描述文件生成（生成器放 `tool/`）。

  ```cpp
  // app/topics/imu.hpp（示意）
  namespace topics::imu
  {
  struct Attitude
  {
      float quaternion[4];
      float euler_rad[3];
      hal::os::Microseconds timestamp_us;
  };
  inline topic::Topic<Attitude> attitude{"imu.attitude"};  // Topic 的构造函数是 constexpr，保证常量初始化
  }
  ```

**多个来源**：同一种数据有多个来源时（如遥控和上位机都能给云台指令），加一个仲裁能力：订阅各来源的 topic，按优先级与超时发布唯一的指令 topic。优先级规则因此显式、可测。

### 3.5 能力与任务

**能力**：
- 每个能力是一个 target `app_<capability>`。对外提供：
  - `Config`：POD，包含参数与驱动对象引用；
  - `init(const Config &) -> bool`；
  - `step(now)`。
- 输入输出只经 topic 与注入的驱动对象。能力零 RTOS，宿主测试直接调用 `step()`。
- 能力之间不 include 彼此的头，共享的只有 `app/topics`。

**任务**：
- 任务表在 assembly。每个任务包含：名称、栈大小、优先级、触发方式（周期 N 个 tick，或等待某个 topic），以及按顺序调用的 `step` 列表。
- `app/common` 的任务运行器按表创建 FreeRTOS 任务（原生 API），并巡检栈水位。
- 中断里只做"取数据 + 发布 topic"，计算放在任务里。

### 3.6 profile 与组合根

- **profile = 兵种 × 板**，目录 `config/profiles/<兵种>_<板>/`。一个兵种只有一台车，不需要单车覆盖。
  - `profile.cmake`：`PROFILE_ID`、`BOARD`、`CAPABILITIES`（链接哪些能力 target）。
  - `params.hpp`：`constexpr` 参数（PID、电机型号与 ID、机械尺寸、IMU 安装方向等）。可以 include 能力的配置类型头，只含数据。
- 唯一的构建开关是 `-DPROFILE=<名称>`。
- **assembly**：每个 profile 一个源文件 `config/assembly/<profile>.cpp`（出现第二个 profile 后再抽公共部分）。它负责：
  - 构造驱动与能力对象；
  - 把 board 句柄与 params 注入 `Config`；
  - 定义任务表；
  - 实现启动入口。
- **能力裁剪**：profile 不链接某个能力，它就不进镜像。不用 `#if`。
- **框架验证**用 `bringup_dm_mc02` profile：不含 RM 能力，只演示并检查框架（plan.md F2–F4）。

### 3.7 板级 `platform/<board>/`

- 按板一级，目录名即板标识（`dm_mc02`）。`board.cmake` 声明 CPU 与烧录配置。
- `cubemx/` 放生成物；`port/` 放 C++ 实现的 hal 与 hal/os、HAL 回调、FreeRTOS 相关实现；`board/` 放 `board.hpp`、链接脚本 overlay。
- port 与 board 用 OBJECT 库：port 里有覆盖 HAL 弱符号的回调，放进静态库会被链接器静默丢弃。
- 出现同一芯片的第二块板时，再决定共用代码放在哪里（先登记）。
- `platform/mock` 实现 hal 与 hal/os，只供宿主测试。它是测试替身，不构成 `src` 对 `platform` 的依赖。

## 4. target 依赖图

箭头指向被依赖方。

```text
EXECUTABLE  firmware_<profile>
 ├─ STATIC    assembly_<profile>        config/assembly/<profile>.cpp
 │   ├─ INTERFACE profile_<profile>     config/profiles/<profile>（params.hpp）
 │   ├─ INTERFACE boot                  src/boot
 │   ├─ STATIC    app_common            app/common → os_freertos_<board>、hal_os
 │   ├─ STATIC    app_<capability>      app/<capability>
 │   │   ├─ INTERFACE app_topics        app/topics → middleware_topic
 │   │   ├─ 库       middleware_topic   src/middleware/topic → utils、hal_os
 │   │   ├─ STATIC  driver_<device>     src/driver → hal、hal_os、lib、utils
 │   │   ├─ STATIC  lib_<pkg>           src/lib → utils
 │   │   ├─ STATIC  utils_<名>          src/utils（只依赖标准库；只有头文件的是 INTERFACE，如 utils_concurrency）
 │   │   └─ INTERFACE hal、hal_os        src/hal（实现在 port）
 │   ├─ INTERFACE os_freertos_<board>   FreeRTOS 头（只对 app_common、assembly 可见）
 │   └─ OBJECT    board_<board>         platform/<board>/board（board.hpp、链接脚本）
 └─ OBJECT    port_<board>              platform/<board>/port → mx_generated、rtos、usb（OBJECT）

宿主测试：test_<name> → 被测 target + platform_mock
框架独立构建：只配置 src/ + platform/mock，不含 app/、config/，不提供 RTOS 头
```

- 只有最终固件可以链接 `assembly_*`。
- 间接依赖的 OBJECT 库用 `cmake/targets.cmake` 的 `link_object_library()` 把目标文件传到固件。
- target 名不带项目前缀（Decision-20261026-06）。
- CMake 文件按层组织，一层一个 `CMakeLists.txt`（Decision-20261026-07）。

## 5. 依赖规则

### 5.1 执行方式

每个 target 用 PRIVATE include 之后，大部分规则由 CMake 本身保证：看不到的头就 include 不了。CI 补三类检查：
- **依赖图**：`cmake --graphviz` 导出后，由 `tool/ci/arch_check.py graph` 检查；
- **文本扫描**：由 `arch_check.py scan` 完成；
- **符号与哈希**：HAL 回调未被 C++ 改名，生成区清单与 vendor 头哈希一致。

### 5.2 规则表

| # | 规则 | 主要检查方式 | 生效 |
|---|---|---|---|
| 1 | `app` 不得链接或看见 `port_*`、`board_*`（含 `board.hpp`）、生成区、`assembly_*` | 依赖图 | 已执行；`board.hpp` 在 F2 |
| 2 | `src` 不得 include 或链接 `platform` 实现、`app`、`config`；必须能只靠 `platform/mock` 在宿主机构建 | 依赖图；框架独立构建 | 已执行；独立构建在 F1 |
| 3 | 契约头（`src/hal` 含 `os`、`src/boot`）只用标准库头；类型只用 enum class、POD、不透明句柄；不出现厂商与 RTOS 名 | 文本扫描 | 已执行；`hal/detail` 见 X-07 |
| 4 | `config/profiles` 只含数据：`params.hpp` 只能 include 能力的配置类型头与标准库头 | 文本扫描 | F2 |
| 5 | 只有最终固件可以链接 `assembly_*` | 依赖图 | 已执行 |
| 6 | 测试不得链接 port、board、生成区、assembly | 依赖图 | 已执行 |
| 7 | 生成区只在 USER CODE 区调用 `boot_*`（及 X-01 的 USB 接口）；不得 include `app`、`config` 的头 | 文本扫描 | F2；之前见 X-09 |
| 8 | 禁止 `file(GLOB)` 收集源码；禁止全局 `include_directories()`；每个 target 显式声明 PRIVATE/PUBLIC/INTERFACE | 文本扫描 | 已执行 |
| 9 | `app/`、`src/` 中不用 `#if` 做裁剪（头文件保护与 `__cplusplus` 除外） | 文本扫描 | 已执行；X-14 放行 |
| 10 | RTOS 头只对 `app/common` 与 `config/assembly` 可见，用 FreeRTOS 原生 API；能力与 `src` 零 RTOS，只经 `hal/os` | 依赖图 + 文本扫描 | F2；之前见 X-04 |
| 11 | 上位机链路协议只用 vendor 的 `lower_link.hpp`，不手写帧与校验（`utils` 的 CRC 只给设备协议用，如裁判系统） | vendor 哈希 + 文本扫描 | 链路能力重建时 |
| 12 | `app`、`src`、`platform`、`config` 的一级目录有白名单（§5.4），新增先登记 | 文本扫描 | 已执行 |
| 13 | 不用 `std::atomic` 的读改写（`fetch_*`、`exchange`、`compare_exchange_*`）；并发只用 `utils` 原语与 topic | 文本扫描 | F1；之前见 X-08 |
| 14 | 框架开启 `-Wdouble-promotion`，豁免清单见 X-06 | 编译选项 | F3 |
| 15 | 框架（`src/`、`platform/`、`cmake/`）不得引用产品的路径或 target | 文本扫描 + 框架独立构建 | 已执行 |
| 16 | C 链接只允许 §9.3 列出的几处；覆盖 HAL 弱符号的 `.cpp` 必须 include 对应 HAL 头 | `nm` 检查 `HAL_*Callback` | F2 |
| 17 | topic 只在 `app/topics` 定义；每个 topic 一个发布者；消息可平凡拷贝，只用标准标量与定长数组 | `static_assert` + 初始化检查 + 文本扫描 | F1 |
| 18 | 能力之间只经 topic 通信：`app_<capability>` 不得依赖其他 `app_<capability>` | 依赖图 | F2 |

### 5.3 CI 检查

| 作业 | 内容 | 变化 |
|---|---|---|
| `lint-python`、`lint-c`、`lint-naming` | ruff/black；clang-format 18.1.8；clang-tidy 18.1.8 命名 | 不变 |
| `lint-arch` | `arch_check.py scan` | 随 F1、F2 加入规则 4、7、13、17 |
| `host-tests` | host-tests preset 的 ctest；宿主依赖图 | F1 增加框架独立构建（`src` + `platform/mock`） |
| `firmware` | arm-debug、arm-release 构建；固件依赖图；与 P1 存档的符号对照 | 符号对照在 F2 退役（行为开始改变），改为构建 + 段大小报告；F2 增加 HAL 回调符号检查 |

### 5.4 一级目录白名单（规则 12）

| 目录 | 白名单 |
|---|---|
| `src/` | `hal`（含 `os`）、`boot`、`utils`、`middleware`、`lib`、`driver`、`third_party` |
| `app/` | `topics`、`common`、`imu`、`chassis`、`gimbal`、`remote`、`launch`、`link`（新增能力先登记） |
| `platform/` | `mock` 与各板标识（`dm_mc02`；F407 C 板在阶段 B 加入） |
| `config/` | `profiles`、`assembly` |

## 6. 生成区与 vendor 治理

- `platform/<board>/cubemx/` 只放生成物。手写内容只允许出现在 USER CODE 区，且只能是对 `boot_*` 的调用（以及 X-01 的 USB 接口调用）。这些行登记在 decision.md，由 CI 文本扫描检查。
- 生成区的其他变更只能通过重新生成，并先在 decision.md 登记。
- 再生成可复现：CI 用 sha256 清单校验；有装 CubeMX 的 runner 时再加 regenerate-diff。
- 板级 CMake 的再生成防护保留：USB OTG HS 内部 DMA 必须开启；USB port 必须含 PA11/PA12 复用配置。
- 链接脚本：生成的 `.ld` 不改，自定义段放 `platform/<board>/board/overlay.ld`，作为第二个 `-T` 追加（Q9）。
- vendor 生成头：`src/third_party/foray_interfaces/` 固定版本并记录 sha256，CI 校验，不手改。
- 层级声明：`.foray-layer` 为 `layer: MCU`、`depends_on_layers: [X2]`，对 X2（`foray_interfaces`）只消费 vendor 的生成头。

## 7. 例外登记

本表是全部生效中例外的唯一清单。新增例外先在 decision.md 登记；到期项完成后标为已关闭。已关闭的 X-11、X-12、X-13、X-15 见 decision.md。

| 编号 | 例外 | 涉及规则 | 处置与到期 |
|---|---|---|---|
| X-01 | **UsbCdc 边界**：USB 生成代码（`usb_device.c`、`usbd_cdc_if.c`）直接调用 port 的 C 接口 `UsbCdcPort_*`，`UsbCdc.h` 被生成代码按名字 include | 7、16 | F2 改写 port 时保留这组 C 接口；重建链路能力时按 Q8 改为回调注册 |
| X-02 | **生成区链接接缝**：`-Wl,--wrap=HAL_PCD_EP_Transmit` | 16 | 随 X-01 删除 |
| X-03 | **链接脚本 overlay**：自定义段用第二个 `-T` 追加，不改生成的 `.ld` | 生成区 | 第一个自定义段出现时落地 |
| X-04 | **旧任务外壳用 CMSIS-RTOS2**：`app/tasks`、`app/common` 中的旧代码 | 10 | F2 随旧能力代码移出构建 |
| X-05 | **DJI 型号映射**：`dji_motor_model_map.hpp`（型号 → 电调档案、原生减速比）是品牌默认值，留在驱动层 | §2.3 | 长期 |
| X-06 | **`-Wdouble-promotion` 豁免**：`src/lib/estimate/vqf.cpp`，须有"目标无双精度 FPU 即 `#error`"的检查 | 14 | 长期；M4F 板链接姿态解算时重新评估 |
| X-07 | **C 接口层**：`src/hal/detail/*.h`（`hal_detail`），port 仍是 C | 3、16 | F2 port 改写为 C++ 后删除 |
| X-08 | **原子读改写**：`src/driver/motor/dji/dji_motor.cpp` 2 处；旧 app 中 6 处 | 13 | 驱动在 F3 适配时改；旧 app 随移出构建消失 |
| X-09 | **生成区引用应用头**：`main.c`、`freertos.c` include `"Application/Entry.h"`，头文件位于 `config/assembly/Application/`，由 `config/CMakeLists.txt` 注入生成区的 include 路径 | 7 | F2 改为 `boot_*` 后删除 |
| X-10 | **集成测试直接编译旧 app 源码**（底盘任务、控制器） | §2.4 | F2 随旧 app 归档 |
| X-14 | **迁移期构建开关**：`legacy_build_options` 的 `BMI088_GYRO_BANDWIDTH_REGISTER`、`IMU_ENABLE_DIAGNOSTICS` | 9、15 | 诊断开关在 F2 随旧 app 删除；陀螺带宽在 F3 改为驱动 `Config` 参数 |
| X-16 | **RTOS 头的可见范围**：`os_freertos_<board>` 只能整目录暴露 CubeMX 的 `Core/Inc` | 10 | 长期（CubeMX 生成布局决定） |

**框架阶段（F2、F3）必须处理的已知问题**：
- `lib/estimate/imu_body_mapping` 依赖 BMI088 驱动类型（`lib → driver` 反向依赖），F3 改为 POD 入参；
- SPI 异步能力在契约里是可选，BMI088 与 WS2812 却当作必选（评审报告 §2.3 缺口 1），F2 契约中按 §3.1 的能力分级处理。

迁移源的其他问题（评审报告 §2、附录 B）在重建对应能力时处理。

## 8. 框架与产品边界、平台可移植性

### 8.1 边界

- 规则 2、15 保证框架不引用产品。同仓开发，暂不拆仓；第一个小项目启动时，用 `git subtree split` 把框架提成独立仓库并保留历史。
- 框架只收与产品无关、至少两个产品会用的东西。底盘、云台、发射等 RM 专用能力和产品的 topic 清单留在 `app/`。

### 8.2 平台差异对框架代码的约束

| 平台 | 内核 | 硬件双精度 | 无锁原子读改写 | 周期计数器 | 中断屏蔽 | RTOS |
|---|---|---|---|---|---|---|
| STM32H723（DM-MC02） | Cortex-M7 | 有 | 有 | DWT | PRIMASK / BASEPRI | FreeRTOS |
| STM32F407（C 板） | Cortex-M4F | **无** | 有 | DWT | PRIMASK / BASEPRI | FreeRTOS |
| TM4C | Cortex-M4F | **无** | 有 | DWT（按型号确认） | PRIMASK / BASEPRI | FreeRTOS |
| AM26x | Cortex-R5F | 有（按具体型号确认） | 有 | PMU（无 DWT） | **CPSR I/F 位（无 NVIC、PRIMASK、BASEPRI）** | FreeRTOS |
| MSPM0 | Cortex-M0+ | **无 FPU** | **无** | 无 DWT | 仅 PRIMASK | 裸机 |

由此得出的约束：
- **不假设 DWT、PRIMASK 或 CMSIS**：时间、临界区、信号只经 `src/hal/os`，实现留在 port。
- **不用原子读改写**：规则 13。`utils` 的并发原语与 topic 只用读写与内存屏障。
- **double 只在豁免文件里用**：规则 14。M4F 上 double 运算会编译成软件库调用。
- **GNU 专属选项只出现在工具链或平台 CMake 中**：代码要能同时通过 GCC 与 Clang（AM26x 可能用 TI Arm Clang，Q13）。
- **全局对象**：topic 等全局对象要求常量初始化；MSPM0 裸机的启动文件须调用全局构造（`__libc_init_array`）。

## 9. 语言、命名与格式

### 9.1 语言

- C++17，`-fno-rtti -fno-exceptions -fno-threadsafe-statics`；无堆分配；热路径不用虚函数。
- 契约与 port 都用 C++；枚举只定义一份。
- 文件编码 UTF-8（无 BOM）；注释、文档、提交信息用简体中文。

### 9.2 命名（ROS 2 C++ 风格，由根目录 `.clang-tidy` 执行）

| 元素 | 规则 | 示例 |
|---|---|---|
| 文件、目录 | snake_case；C++ 头文件 `.hpp`、实现 `.cpp` | `chassis_task.cpp` |
| 命名空间 | snake_case，与目录同名（`src/lib` 下不加 `alg_` 前缀，Decision-20261026-09） | `math`、`topic`、`hal::can` |
| 类、结构体、枚举类型、类型别名、模板参数 | PascalCase | `DjiMotor` |
| 函数、方法 | snake_case | `set_wheel_target` |
| 局部变量、参数 | snake_case | `key_level` |
| 私有、受保护成员 | snake_case + 尾下划线 | `transfer_active_` |
| 结构体公开字段 | snake_case，不加后缀 | `euler_degrees` |
| 全局变量（含匿名命名空间） | `g_` + snake_case；topic 实例按清单命名，不加 `g_` | `g_snapshot_sequence` |
| 编译期常量、全局与静态常量 | UPPER_SNAKE_CASE | `STACK_HEALTH_SAMPLE_PERIOD_MS` |
| 枚举值、宏 | UPPER_SNAKE_CASE | `Channel::CHANNEL_1` |
| 头文件保护宏 | 路径大写，单下划线分隔 | `HAL_OS_CLOCK_HPP` |
| 本项目自有的 C 链接函数 | 模块前缀 + snake_case | `boot_init` |

不改名的：HAL 回调与中断入口（`HAL_*Callback`、`*_IRQHandler`）、CubeMX 生成代码、F2 删除前的 C 接口（`CanPort_*` 等"模块_函数"），由 `.clang-tidy` 的 `FunctionIgnoredRegexp` 排除。

标识符里的数字（`.clang-tidy` 查不出，评审把关，Decision-20261026-05）："数值+单位"和序号单独成段（`ACCEL_ODR_100_HZ`、`CHANNEL_1`）；名字里固有的数字不拆（`BMI088`、`GM6020_CURRENT`、`crc8`）。

### 9.3 C 链接只允许这几处（规则 16）

| # | 位置 | 原因 | 写法 |
|---|---|---|---|
| 1 | 生成区调用的启动入口 `boot_*` | `main.c`、`freertos.c` 是 C 文件 | `src/boot/boot.h` 用 `extern "C"` 声明，实现在 `config/assembly` 的 C++ 里 |
| 2 | 覆盖 HAL 弱符号的回调 `HAL_*Callback` | HAL 按 C 符号名调用 | 写在 port 的 `.cpp` 里，并 include 对应 HAL 头，让定义继承 C 链接；否则回调被 C++ 改名，HAL 弱实现照旧生效，编译链接都不报错 |
| 3 | 中断入口 `*_IRQHandler` | 向量表按 C 符号名引用 | 保持在生成区的 C 文件里 |
| 4 | USB 生成代码调用的 `UsbCdcPort_*` | 例外 X-01 | 在 port 中以 `extern "C"` 实现，Q8 落地后删除 |

### 9.4 格式（组织规则，由根目录 `.clang-format` 执行）

- 4 空格缩进、LF、UTF-8、文件末尾保留一个换行；Allman 大括号、行宽 120、指针靠右。
- `template <...>` 总是单独一行，类、函数、成员模板、前置声明与 `template <>` 特化都适用（Decision-20261028-01）。
- clang-format 与 clang-tidy 统一用 18（CI 固定 18.1.8）。
- 生成区与厂商代码不格式化，范围由 `tool/ci/format.py` 定义（Decision-20261026-04）。
- 格式化与改名必须各自单独提交，提交号写入 `.git-blame-ignore-revs`。

## 10. 修改本文件

1. 规则、白名单、例外的任何增删改，先在 [decision.md](../../decision.md) 登记（Context / Decision / Reason / Alternatives）。
2. 再修改本文件的对应小节，并同步 `.github/workflows/ci.yml` 或 `tool/ci/` 中的检查。
3. 评审报告是历史记录，不随本文件修改。
