# 架构判据与依赖规则（foray_mcu）

> **状态**：v2.11（2026-11-01）。v2.0 见 Decision-20261027-01；v2.1 随 F1 第 1 步更新 utils、middleware、lib 的归类：math 进 utils，fsm 进 middleware（Decision-20261027-02）；v2.2 随 F1 第 2 步加入 utils 的并发原语 `concurrency`（Decision-20261029-01），§3.4 补充 seqlock 的读写竞态（Decision-20261029-02，待定）；v2.3 随 F1 第 3 步定稿 §3.2 的 OS 契约：1 µs 时基、`Signal` 的语义、临界区只屏蔽框架中断（Decision-20261031-01～03）；v2.4 随 F1 第 4 步定稿 §3.4 的 topic：发布时用临界区包住写入（Decision-20261029-02），接口、等待槽与清单约定（Decision-20261031-05）；v2.5 随 F1 的 CI 加入框架独立构建与 Cortex-M0+ 原子检查，规则 13、17 的文本扫描生效，新增例外 X-17（Decision-20261031-06）；v2.6 随 F2 第 1 步把固件的符号对照换成段大小报告，规则 16 的 HAL 回调符号检查生效（Decision-20261031-08）；v2.7 随 F2 第 4 步换上启动入口 `boot_*` 与 `hal::init()`，规则 7 的 include 检查生效，`boot` 在依赖图里单独成层，X-09 关闭（Decision-20261031-11）；v2.8 随 F2 第 4 步定下 §3.5 的任务运行器写法，加入第一个 topic 与 bringup 能力，`topics` 在依赖图里单独成层（Decision-20261031-12）；v2.9 随 F2 第 4 步归档旧能力代码（tag `archive/pre-rebuild`），X-04、X-10 关闭，规则 10、18 生效，`app/` 加入宿主构建（Decision-20261031-13）；v2.10 随 F2 第 5 步与 F3 定稿 §3.1 的 hal 契约 v1（句柄类）、`board.hpp`、C++ port 与 mock，删除 `hal/detail` 与 PortKit，X-07、X-08、X-14、X-17 关闭，规则 1（`board.hpp`）、4、14、16（C 链接位置）的检查生效；§3.5 加入 topic 触发的超时与能力 `Config` 的两部分（Decision-20261101-01～03）；v2.11 板级目录改为"厂商 / 板"两级：`platform/st/dm_mc02`（Decision-20261101-04）。v2 按负责人的骨架重写：迁移方式从"改造迁移源"改为"按新骨架重建，迁移源只作参考与零件来源"。v1.0–v1.4（P0–P2.5）见 git 历史与 decision.md。
> **权威来源**：[decision.md](../../decision.md) 中的 Accepted 条目，v2 起以 Decision-20261027-01 为准。[评审报告 v1.0](reviews/2026-10-25-foray_mcu-架构评审报告.md) 是历史记录，与本文件冲突的部分以本文件为准。
> **修改流程**：新增或修改规则、例外、白名单，先在 decision.md 登记，再改本文件和对应的 CI 检查（§10）。

## 0. 怎么用

- **写新代码前**：按 §2 判定位置，按 §3 的机制写，按 §5 核对依赖。
- **评审 PR**：对照 §5 的规则表和 §7 的例外登记。违反规则又不在例外表里的改动，不合并。
- **现阶段**：F1 各项已完成，待负责人验收；F2 第 1–5 步与 F3 的代码已完成（hal 契约 v1、C++ port、驱动改用、bringup 外设检查），2026-11-01 已上板验证（`docs/ai/decisions/2026-11-01-F2-外设上板验证/README.md`），待负责人验收（[plan.md](../../plan.md)）。仍不符合本文件的只剩 §7 的例外。

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
│   ├── hal/                  硬件契约（C++ 句柄类）：can、spi、uart、gpio、pwm、usb_cdc，callback、init
│   │   └── os/               OS 契约：临界区、单调时基、信号（等待/唤醒）
│   ├── boot/                 启动入口 boot.h（extern "C"）：生成区只调它
│   ├── utils/                无领域含义的基础件：math、crc、concurrency（SPSC 队列、最新值邮箱）
│   ├── middleware/           给能力用的运行期机制与服务：建立在契约与基础件之上
│   │   ├── topic/            topic 机制：最新值、单发布者、可在中断里发布、可阻塞等待
│   │   └── fsm/              通用状态机
│   ├── lib/                  领域算法：control（pid、slope、heater_control）、estimate、filter、kinematics、component
│   ├── driver/               设备驱动与设备协议：bmi088、motor（dji）、buzzer、ws2812、protocol/{dji,sbus}
│   └── third_party/          vendor 进来的外部生成头（foray_interfaces 的 lower_link.hpp）
├── platform/                 厂商 / 板两级
│   ├── st/                   厂商：ST
│   │   └── dm_mc02/          DM-MC02（STM32H723）：board.cmake + CMakeLists.txt
│   │       ├── cubemx/       CubeMX 生成物（冻结）
│   │       ├── port/         hal 与 hal/os 的 C++ 实现、HAL 回调
│   │       └── board/        board.hpp：物理资源句柄、外设个数、能力常量；链接脚本 overlay（以后）
│   └── mock/                 hal 与 hal/os 的宿主替身：测试控制时间、注入外设输入、检查输出（mock_hal.hpp）
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

1. **是 CubeMX 或厂商产生的吗？** → `platform/<厂商>/<board>/cubemx/`，只能重新生成；厂商库同理，不手改。
2. **直接调用厂商 HAL、寄存器或 RTOS 实现吗？** → 实现契约的放 `platform/<厂商>/<board>/port/`；板的物理资源、能力常量、引脚放 `platform/<board>/board/`。
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

契约 v1 于 F2 第 5 步定稿（Decision-20261101-01）。

- **写法：句柄类**。每类外设一个句柄类，成员函数由 port 实现（链接期绑定，没有虚函数，没有运行期分派），宿主测试由 `platform/mock` 实现：

  ```cpp
  // src/hal/can.hpp（节选）
  namespace hal
  {
  class Can final
  {
    public:
      static constexpr std::size_t MAX_DATA_LENGTH{8U};
      enum class SendResult : std::uint8_t { QUEUED, QUEUE_FULL, NOT_READY, BUS_OFF, INVALID_ARGUMENT, ERROR };
      struct Frame { /* … */ };

      constexpr explicit Can(std::uint8_t index) noexcept : index_{index} {}
      [[nodiscard]] bool claim_receiver() const noexcept; // 每条总线一个接收方
      [[nodiscard]] SendResult send(const Frame &frame) const noexcept;
      [[nodiscard]] ReceiveResult receive(Frame &frame) const noexcept;
      [[nodiscard]] bool set_receive_notification(Callback callback, void *context) const noexcept;
      [[nodiscard]] Statistics statistics() const noexcept;

    private:
      std::uint8_t index_; // 只有 port 与 mock 能读
  };
  }

  // platform/st/dm_mc02/board/board.hpp
  inline constexpr hal::Can CAN_1{0U};

  // 组合根把句柄注入驱动；驱动按值保存，调用时 bus_.send(frame)
  device::DjiMotor g_chassis_motors{board::CAN_1, device::DjiMotor::Profile::c620()};
  ```

- **外设**：`hal::Gpio`、`hal::Pwm`、`hal::Can`、`hal::Spi`（设备 = 总线 + 片选）、`hal::Uart`、`hal::UsbCdc`。`Uart` 与 `UsbCdc` 的读写接口同形（`read`、`write`、`WriteResult`），处理字节流的代码可以对两者写成同一个模板。
- **单例用自由函数**：全板只有一份、不需要句柄的用自由函数：`hal::init()`、`hal::os::now_us()`。与 hal/os 的规律相同：多实例的用类（`Signal`），单例的用自由函数。
- **句柄**：只含一个私有编号（`std::uint8_t`），`constexpr explicit` 构造，可以拷贝，驱动按值存进自己的成员或 `Config`。编号由 `board.hpp` 给出，是 port 外设表的下标；驱动与能力读不到也改不了。
- **设备不进契约**：契约头不枚举"这台车有哪些设备"。`board.hpp` 给出物理资源句柄（`board::CAN_1`、`board::SPI_IMU_GYRO`）；整车用哪个资源、接哪个设备，由组合根注入驱动。
- **能力分级**：每个成员函数注明必选还是可选。可选能力不支持时返回 `UNSUPPORTED`（或 `false`），板级用 `board.hpp` 的 `constexpr` 常量声明（如 `board::SPI_ASYNC`）。要求可选能力的驱动给出编译期常量（`device::Bmi088::REQUIRES_SPI_ASYNC`），组合根 `static_assert`；能回退的驱动在运行期回退（WS2812 遇到 `UNSUPPORTED` 改用阻塞发送）。
- **类型**：只用 `enum class`、POD 结构与句柄，收在类里（`hal::Can::Frame`），枚举只定义一份；不出现厂商或 RTOS 类型。时间参数用 `hal::os::Microseconds`。
- **回调**：`hal::Callback`（`void (*)(void *context) noexcept`，`src/hal/callback.hpp`），在框架中断里执行，只做取数据、发布 topic、`Signal::notify()`。
- **文档**：每个成员函数注明在任务还是中断里调用、是否阻塞。
- **初始化**：`hal::init()` 由 port 实现，组合根在 `boot_init()` 里调用；生成区不直接调用 port（USB 的 X-01 例外）。

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
- **实现**：`platform/<厂商>/<board>/port/os`：时基用板上的 32 位定时器分频到 1 MHz（DM-MC02 为 TIM2，调试暂停时冻结）；`Signal` 用 FreeRTOS 的 1 号任务通知槽（槽数由板级 CMake 的编译宏设为 2，Decision-20261031-10），裸机板用标志位加 WFI。DM-MC02 的实现在 F2 第 3 步完成。`platform/mock`：时间由测试推进，临界区是一把进程内的锁，`Signal` 用条件变量并检查误用。
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
| 发布者 | 每个 topic 只有一个发布者。`claim_publisher()` 每个 topic 只能成功一次，在初始化阶段调用，返回无效句柄则 `boot_init()` 失败；句柄可移动、不可拷贝 |
| 中断 | 框架中断里可以发布、可以 `update()`，不能 `wait()`；框架线以上的中断不得使用 topic（§3.2，见下方"读写竞态"） |
| 订阅 | `Subscriber` 记住自己读到的版本，一个上下文一个。`update(msg) -> bool`：有没读过的新数据才拷贝；`wait(msg, timeout) -> bool`：有没读过的就立即返回，否则睡到下一次发布或超时，只在任务里用；用 `wait()` 的先在初始化阶段 `enable_wait()` 占一个等待槽 |
| 消息 | 可平凡拷贝，不超过 `topic::MAX_MESSAGE_SIZE`（暂定 64 字节），都由 `static_assert` 检查；只用标准整数、浮点、`bool` 与定长数组 |
| 观测 | `TopicBase` 的 `name()`、`publish_count()`、`last_publish_us()`；订阅者的 `lost_count()`。调试通道经清单里的 `const TopicBase *` 数组列出全部 topic |
| 不做 | 队列、多发布者、运行期发现、回调式订阅（回调会在发布者的上下文里执行，中断里不可控） |

**实现要点**：
- 存储用单写者 seqlock（`utils/concurrency` 的 `LatestValue`）：只用原子读写与内存屏障，没有原子读改写，Cortex-M0+ 也能用（规则 13）。
- **读写竞态**（Decision-20261029-02）：读端遇到写入中途会重试，直到写端写完；如果读端在写入中途抢占了写端，单核上写端再也轮不到运行，读端无限重试。所以发布时用 `hal::os::CriticalSection` 包住写入：写入期间框架中断与任务切换都被屏蔽，读端抢占不了写端。任务之间因此没有优先级限制，框架中断里也能读。代价是每次发布关一次框架中断，时长约等于一次消息拷贝，所以消息有大小上限。临界区只屏蔽框架中断（§3.2），线以上的中断既不能发布也不能读 topic。
- 等待槽：每个 topic 固定 `WAIT_SLOTS`（4）个，用 `TopicBase::add_waiter(signal)` 登记：订阅者经 `enable_wait()` 登记自己的 `Signal`，任务运行器按 topic 触发任务时直接登记（Decision-20261031-12）。发布时在临界区外逐个 `notify()`。没有登记就 `wait()` 的，发布叫不醒它，只能睡到超时。
- 只用原子读写与内存屏障：每个计数都只有一个写者，用"读一次、写一次"更新（规则 13）。
- `Topic`、`Subscriber`、`Publisher` 的构造函数都是 `constexpr`，作全局对象时常量初始化，不依赖全局构造顺序。
- 只有头文件：`src/middleware/topic/topic.hpp`，target `middleware_topic`（Decision-20261031-05）。

**清单**：
- 产品的全部 topic 集中定义在 `app/topics/`，按发布者所属的能力分文件、分命名空间；约定与示例见 [`app/topics/README.md`](../../app/topics/README.md)。`app/topics/all.hpp` 列出全部 topic，给调试通道用。
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

**任务**（Decision-20261031-12）：
- 任务表在 assembly：`constexpr std::array<common::TaskSpec, N> TASKS`。每行包含：
  - 名称；
  - 栈（字）；
  - FreeRTOS 优先级；
  - 触发方式：`common::every_ms(周期)`；`common::on_topic(topic)`（topic 每次发布后运行一轮）；`common::on_topic(topic, 超时 ms)`（另外在超过这么久没有发布时也运行一轮，Decision-20261101-03）；
  - 按顺序调用的 step 列表，最多 8 个。每个 step 是 `void (*)(hal::os::Microseconds now)`，组合根用普通函数调能力的 `step(now)`；同一轮的各 step 拿到同一个 `now`。

  ```cpp
  constexpr std::array<common::TaskSpec, 2> TASKS{{
      {"heartbeat", 256U, 25U, common::every_ms(1U), {heartbeat_step}},
      {"monitor", 256U, 26U, common::on_topic(topics::bringup::heartbeat), {monitor_step}},
  }};
  common::TaskRunner<TASKS> g_runner;
  ```
- `app/common` 的任务运行器 `common::TaskRunner<TASKS>`：
  - 用 FreeRTOS 原生 API，任务表在编译期检查；
  - 栈与任务控制块静态分配，在 `.bss`；
  - `init()`、`start()`、`sample_stack_health()` 分别在 `boot_init`、`boot_init_rtos`、`boot_run` 里调用；
  - 记录每个任务的运行轮数、单轮最长耗时、错过周期次数、剩余栈。
- 中断里只做"取数据 + 发布 topic"，计算放在任务里。示范见 `app/bringup/imu_check.cpp`：两个数据就绪中断与 SPI 完成中断在临界区内更新计数、取快照，在临界区外发布 `bringup.imu_event`；IMU 任务按它触发（Decision-20261101-03）。
- **能力的 `Config` 分两部分**：组合根注入的依赖（驱动对象的指针、引脚等句柄）与 profile 给的 `Parameters`（只含数据，写在 `params.hpp`）。`params.hpp` 看不到 `board.hpp` 与驱动对象（规则 1、4），由组合根组装：

  ```cpp
  // config/assembly/bringup_dm_mc02.cpp
  g_imu_check.init(bringup::ImuCheck::Config{&g_imu, &g_heater, board::IMU_ACCEL_INTERRUPT,
                                             board::IMU_GYRO_INTERRUPT, params::IMU});
  ```

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
- **框架验证**用 `bringup_dm_mc02` profile：不含 RM 能力，链接 `app_bringup`（心跳与监视；IMU、CAN、SBUS、USB、指示的外设检查），只演示并检查框架（plan.md F2–F4，Decision-20261031-12、Decision-20261101-03）。`profile.cmake` 的 `CAPABILITIES` 决定组合根链接哪些 `app_<能力>`。

### 3.7 板级 `platform/<厂商>/<board>/`

- "厂商 / 板"两级（Decision-20261101-04）：`platform/st/dm_mc02`。板目录名即板标识（`BOARD_ID`，target 名的后缀）；profile 的 `BOARD` 写 `platform/` 下的路径（`st/dm_mc02`）。`board.cmake` 声明 CPU 与烧录配置。
- `cubemx/` 放生成物；`port/` 放 C++ 实现的 hal 与 hal/os（每个外设一个 `.cpp`）、HAL 回调、port 私有的 `port.hpp`（外设初始化、中断通知 `Notifier`）；`board/` 放 `board.hpp`、链接脚本 overlay。
- `board.hpp`（命名空间 `board`）只 include `hal/*.hpp`：物理资源句柄、每类外设的个数、能力常量。只有组合根与本板的 port 能看到它（`board_<板>` 的 include 路径只开到 `board/`，规则 1）。port 的外设表按初始化列表推导长度，`static_assert` 与 `board.hpp` 的个数核对。
- port 里与中断共享的状态用 `hal::os::CriticalSection` 保护（DM-MC02 的外设中断都在优先级 5，即框架线上）；接收队列用 `concurrency::SpscQueue`。
- port 与 board 用 OBJECT 库：port 里有覆盖 HAL 弱符号的回调，放进静态库会被链接器静默丢弃。
- 同一厂商的板之间要共用代码时，再决定放在 `platform/<厂商>/` 下的哪里（先登记）。
- `platform/mock` 实现 hal 与 hal/os，只供宿主测试。它是测试替身，不构成 `src` 对 `platform` 的依赖。测试控制接口在 `mock_hal.hpp`（`mock::can::inject`、`mock::spi::set_responder`、`mock::gpio::trigger_interrupt` 等）：按编号注入输入、检查输出；SPI 由测试登记设备应答函数，异步事务由测试决定何时完成。

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
 │   └─ OBJECT    board_<board>         platform/<厂商>/<board>/board（board.hpp、链接脚本）→ hal
 └─ OBJECT    port_<board>              platform/<厂商>/<board>/port → board、hal、hal_os、utils、mx_generated、rtos、usb（OBJECT）

宿主测试：test_<name> → 被测 target + platform_mock
框架独立构建（preset framework、framework-m0plus，-DFRAMEWORK_ONLY=ON）：
  只配置 src/（宿主上另加 platform/mock）与 check_templates（test/framework，实例化只有头文件的模板），
  不含 app/、config/、test/host 与板卡，不提供 RTOS 头
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
- **框架独立构建**：在只含框架文件的副本里构建（宿主与 Cortex-M0+），M0+ 的静态库由 `arch_check.py atomics` 检查原子库调用；
- **符号与哈希**：HAL 回调未被 C++ 改名，生成区清单与 vendor 头哈希一致。

### 5.2 规则表

| # | 规则 | 主要检查方式 | 生效 |
|---|---|---|---|
| 1 | `app` 不得链接或看见 `port_*`、`board_*`（含 `board.hpp`）、生成区、`assembly_*` | 依赖图 + 文本扫描（只有组合根与本板的 port 可以 include `board.hpp`） | 已执行（`board.hpp` 自 F2 第 5 步） |
| 2 | `src` 不得 include 或链接 `platform` 实现、`app`、`config`；必须能只靠 `platform/mock` 在宿主机构建 | 依赖图；框架独立构建 | 已执行（框架独立构建自 F1） |
| 3 | 契约头（`src/hal` 含 `os`、`src/boot`）只用标准库头；类型只用 enum class、POD、不透明句柄；不出现厂商与 RTOS 名 | 文本扫描 | 已执行 |
| 4 | `config/profiles` 只含数据：`params.hpp` 只能 include 能力的配置类型头与标准库头 | 文本扫描（include 只能指向 `app/` 下能力目录的头与标准库头） | 已执行（F2，Decision-20261101-03） |
| 5 | 只有最终固件可以链接 `assembly_*` | 依赖图 | 已执行 |
| 6 | 测试不得链接 port、board、生成区、assembly | 依赖图 | 已执行 |
| 7 | 生成区只在 USER CODE 区调用 `boot_*`（及 X-01 的 USB 接口）；不得 include `app`、`config` 的头 | 文本扫描（include 只能指向生成区自己的头、厂商 HAL 头、`boot/boot.h` 与 X-01 的 `UsbCdc.h`）；USER CODE 的改动行登记在 decision.md | 已执行（F2 第 4 步，Decision-20261031-11） |
| 8 | 禁止 `file(GLOB)` 收集源码；禁止全局 `include_directories()`；每个 target 显式声明 PRIVATE/PUBLIC/INTERFACE | 文本扫描 | 已执行 |
| 9 | `app/`、`src/` 中不用 `#if` 做裁剪（头文件保护与 `__cplusplus` 除外） | 文本扫描 | 已执行（X-14 在 F2 第 5 步关闭，没有放行） |
| 10 | RTOS 头只对 `app/common` 与 `config/assembly` 可见，用 FreeRTOS 原生 API；能力与 `src` 零 RTOS，只经 `hal/os` | 依赖图（能力不得依赖 `app_common`）+ 文本扫描（`src/`、`app/` 只有 `app/common` 可以 include RTOS 头） | 已执行（F2 第 4 步，Decision-20261031-13） |
| 11 | 上位机链路协议只用 vendor 的 `lower_link.hpp`，不手写帧与校验（`utils` 的 CRC 只给设备协议用，如裁判系统） | vendor 哈希 + 文本扫描 | 链路能力重建时 |
| 12 | `app`、`src`、`platform`、`config` 的一级目录有白名单（§5.4），新增先登记 | 文本扫描 | 已执行 |
| 13 | 不用 `std::atomic` 的读改写（`fetch_*`、`exchange`、`compare_exchange_*`）；并发只用 `utils` 原语与 topic | 文本扫描 + Cortex-M0+ 库调用检查（整个框架） | 已执行（F1；X-08、X-17 在 F2、F3 关闭，没有放行） |
| 14 | 框架与能力开启 `-Wdouble-promotion`，豁免清单见 X-06 | 编译选项（`framework_library`、`utils_library`、`app_library`） | 已执行（F3，Decision-20261101-02） |
| 15 | 框架（`src/`、`platform/`、`cmake/`）不得引用产品的路径或 target | 文本扫描 + 框架独立构建 | 已执行 |
| 16 | C 链接只允许 §9.3 列出的几处；覆盖 HAL 弱符号的 `.cpp` 必须 include 对应 HAL 头 | `nm` 检查 `HAL_*Callback`：port 定义的回调都是强符号，没有被 C++ 改名的回调；文本扫描：`extern "C"` 只出现在 §9.3 的位置 | 已执行（回调自 F2 第 1 步，C 链接位置自 F2 第 5 步） |
| 17 | topic 只在 `app/topics` 定义；每个 topic 一个发布者；消息可平凡拷贝，只用标准标量与定长数组 | `static_assert` + 初始化检查 + 文本扫描（定义位置、发布句柄只在一处取得、清单头的 include） | 已执行（F1） |
| 18 | 能力之间只经 topic 通信：`app_<capability>` 不得依赖其他 `app_<capability>` | 依赖图（能力层只能依赖 `app_topics`，不能依赖能力层自己） | 已执行（F2 第 4 步，旧 app 归档后不再有放行） |

### 5.3 CI 检查

| 作业 | 内容 | 变化 |
|---|---|---|
| `lint-python`、`lint-c`、`lint-naming` | ruff/black；clang-format 18.1.8；clang-tidy 18.1.8 命名 | F1 起命名检查加上 framework 的编译数据库 |
| `lint-arch` | `arch_check.py scan` | F1 已加入规则 13、17；F2 加入规则 1（`board.hpp`）、4、7、10、16（C 链接位置） |
| `host-tests` | host-tests preset 的 ctest；宿主依赖图 | 不变 |
| `framework` | 只取出框架文件（`git archive`），在副本里：framework preset 构建 + `graph --mode framework`；framework-m0plus 构建 + `atomics` | F1 新增（Decision-20261031-06）；X-17 关闭后 M0+ 编译全部框架（Decision-20261101-02） |
| `firmware` | arm-debug、arm-release 构建；固件依赖图；HAL 回调符号检查（`arch_check.py callbacks`）；段大小报告（`symbols.py report`，写入作业摘要） | F2 第 1 步：与 P1 存档的符号对照退役，加入回调检查与段大小报告（Decision-20261031-08） |

### 5.4 一级目录白名单（规则 12）

| 目录 | 白名单 |
|---|---|
| `src/` | `hal`（含 `os`）、`boot`、`utils`、`middleware`、`lib`、`driver`、`third_party` |
| `app/` | `topics`、`common`、`bringup`（框架验证，Decision-20261031-12）、`imu`、`chassis`、`gimbal`、`remote`、`launch`、`link`（新增能力先登记） |
| `platform/` | 一级：`mock` 与厂商（`st`）；二级：每个厂商下的板标识（`st/dm_mc02`；F407 C 板在阶段 B 加入）（Decision-20261101-04） |
| `config/` | `profiles`、`assembly` |

## 6. 生成区与 vendor 治理

- `platform/<厂商>/<board>/cubemx/` 只放生成物。手写内容只允许出现在 USER CODE 区，且只能是对 `boot_*` 的调用（以及 X-01 的 USB 接口调用）。这些行登记在 decision.md，由 CI 文本扫描检查。
- 生成区的其他变更只能通过重新生成，并先在 decision.md 登记。
- 再生成可复现：CI 用 sha256 清单校验；有装 CubeMX 的 runner 时再加 regenerate-diff。
- 板级 CMake 的再生成防护保留：USB OTG HS 内部 DMA 必须开启；USB port 必须含 PA11/PA12 复用配置。
- 链接脚本：生成的 `.ld` 不改，自定义段放 `platform/<厂商>/<board>/board/overlay.ld`，作为第二个 `-T` 追加（Q9）。
- vendor 生成头：`src/third_party/foray_interfaces/` 固定版本并记录 sha256，CI 校验，不手改。
- 层级声明：`.foray-layer` 为 `layer: MCU`、`depends_on_layers: [X2]`，对 X2（`foray_interfaces`）只消费 vendor 的生成头。

## 7. 例外登记

本表是全部生效中例外的唯一清单。新增例外先在 decision.md 登记；到期项完成后标为已关闭。已关闭的 X-04、X-10（F2 第 4 步，Decision-20261031-13）、X-09（Decision-20261031-11）、X-07、X-08、X-14（F2 第 5 步，Decision-20261101-01）、X-17（F3，Decision-20261101-02）、X-11、X-12、X-13、X-15 见 decision.md。

| 编号 | 例外 | 涉及规则 | 处置与到期 |
|---|---|---|---|
| X-01 | **UsbCdc 边界**：USB 生成代码（`usb_device.c`、`usbd_cdc_if.c`）直接调用 port 的 C 接口 `UsbCdcPort_*`（`port/usb_cdc.cpp` 以 `extern "C"` 实现），`UsbCdc.h` 被生成代码按名字 include | 7、16 | 重建链路能力时按 Q8 改为回调注册 |
| X-02 | **生成区链接接缝**：`-Wl,--wrap=HAL_PCD_EP_Transmit` | 16 | 随 X-01 删除 |
| X-03 | **链接脚本 overlay**：自定义段用第二个 `-T` 追加，不改生成的 `.ld` | 生成区 | 第一个自定义段出现时落地 |
| X-05 | **DJI 型号映射**：`dji_motor_model_map.hpp`（型号 → 电调档案、原生减速比）是品牌默认值，留在驱动层 | §2.3 | 长期 |
| X-06 | **`-Wdouble-promotion` 豁免**：`src/lib/estimate/vqf.cpp`（按文件关掉，有意用 double）；"目标没有双精度 FPU 就报错"的检查推迟到阶段 B（Decision-20261101-02） | 14 | 长期；F407 等 M4F 板链接姿态解算时由板级 CMake 声明 FPU 能力、组合根检查 |
| X-16 | **RTOS 头的可见范围**：`os_freertos_<board>` 只能整目录暴露 CubeMX 的 `Core/Inc` | 10 | 长期（CubeMX 生成布局决定） |

**框架阶段（F2、F3）的已知问题**都已处理：`lib/estimate/imu_body_mapping` 不再依赖 BMI088 驱动（Decision-20261101-02）；SPI 异步按 §3.1 的能力分级使用，BMI088 编译期要求、WS2812 运行期回退（Decision-20261101-01）。

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

不改名的：HAL 回调与中断入口（`HAL_*Callback`、`*_IRQHandler`）、CubeMX 生成代码、USB 生成区接缝的 C 接口（`UsbCdcPort_*`，X-01），由 `.clang-tidy` 的 `FunctionIgnoredRegexp` 排除；链接器约定的名字（`__wrap_HAL_PCD_EP_Transmit`、`__usb_dma_buffer_start__`）与生成区的全局变量（`hUsbDeviceHS`）在声明处用 `// NOLINT(readability-identifier-naming)` 注明原因。

标识符里的数字（`.clang-tidy` 查不出，评审把关，Decision-20261026-05）："数值+单位"和序号单独成段（`ACCEL_ODR_100_HZ`、`CHANNEL_1`）；名字里固有的数字不拆（`BMI088`、`GM6020_CURRENT`、`crc8`）。

### 9.3 C 链接只允许这几处（规则 16）

| # | 位置 | 原因 | 写法 |
|---|---|---|---|
| 1 | 生成区调用的启动入口 `boot_*` | `main.c`、`freertos.c` 是 C 文件 | `src/boot/boot.h` 用 `extern "C"` 声明，实现在 `config/assembly` 的 C++ 里 |
| 2 | 覆盖 HAL 弱符号的回调 `HAL_*Callback` | HAL 按 C 符号名调用 | 写在 port 的 `.cpp` 里，并 include 对应 HAL 头，让定义继承 C 链接；否则回调被 C++ 改名，HAL 弱实现照旧生效，编译链接都不报错 |
| 3 | 中断入口 `*_IRQHandler` | 向量表按 C 符号名引用 | 保持在生成区的 C 文件里 |
| 4 | USB 生成代码调用的 `UsbCdcPort_*`；`--wrap` 的 `__wrap_HAL_PCD_EP_Transmit` | 例外 X-01、X-02 | 在 `port/usb_cdc.cpp` 以 `extern "C"` 实现（声明在 `UsbCdc.h`），Q8 落地后删除 |

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
