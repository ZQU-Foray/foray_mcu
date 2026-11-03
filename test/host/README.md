# 宿主测试（test/host）

在宿主机上编译被测代码，用 ctest 运行，不需要开发板。由根工程在宿主构建中引入（`host-tests` preset）。

- 单元测试直接链接被测 target：框架的 `lib_*`、`driver_*`、`middleware_*`，能力的 `app_*`（能力零 RTOS，`app/` 也加入宿主构建，任务运行器 `app_common` 除外），以及 `platform_mock`；
- 外设与时间都经 `platform_mock` 的替身：测试用 `mock_hal.hpp`（`mock::can::inject`、`mock::spi::set_responder`、`mock::gpio::trigger_interrupt` 等）注入输入、检查输出，用 `mock_os.hpp` 控制时间（Decision-20261101-01）。迁移期的 C ABI 桩已删除。被测代码的头文件随链接的 target 而来（`"driver/bmi088/bmi088.hpp"`、`"bringup/monitor.hpp"`）。
- 旧 app 的底盘集成测试已随旧 app 归档到 tag `archive/pre-rebuild`（Decision-20261031-13，例外 X-10 关闭）。

## 运行

需要 CMake ≥ 3.22、Ninja，以及 GCC 或 Clang（不支持 MSVC）。在仓库根执行：

```sh
cmake --preset host-tests
cmake --build --preset host-tests
ctest --preset host-tests
```

Windows 上用 MSYS2 UCRT64 的 gcc/g++：在 `CMakeUserPresets.json`（不入库）里继承 `host-tests` 并指定 `CMAKE_C_COMPILER`、`CMAKE_CXX_COMPILER`，或配置时加 `-DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe`，并保证 `C:\msys64\ucrt64\bin` 在 `PATH` 中（运行测试需要它的运行库）。

CI 的 `host-tests` 作业（`.github/workflows/ci.yml`）在 GitHub Actions 的 Ubuntu 上运行同样的三条命令，并检查宿主构建的依赖图。

已验证环境：Ubuntu 24.04 + g++ 13.3.0 + CMake 3.28.3（2026-11-02，31/31 通过）；WSL Ubuntu 22.04 + g++ 11.4.0 + CMake 3.31.6（2026-10-29，17/17 通过）；Windows + MSYS2 UCRT64 g++ 16.2.0 + CMake 4.4.4（2026-10-26，14/14 通过；F1 新增的 `spsc_queue`、`latest_value` 于 2026-10-29 用同一编译器直接编译运行通过）。

## 测试清单

可执行文件名为 `test_<ctest 名称>`。

| ctest 名称 | 测试源码 | 被测 target 或源码 |
|---|---|---|
| `dji_protocol` | `unit/protocol/dji_esc_test.cpp` | `driver_protocol_dji`（含只管字节布局的 `decode_feedback_data()`、`encode_frame()`，Decision-20261102-03） |
| `dm_protocol` | `unit/protocol/dm_esc_test.cpp` | `driver_protocol_dm`（达妙协议：四种模式的控制帧与帧 ID、系统命令、寄存器读写与存储、应答与反馈的解码；样例按说明书手算，寄存器应答取自 DM-J4310 实测帧） |
| `dji_motor` | `unit/motor/dji_motor_test.cpp` | `driver_motor_dji` + `platform_mock`（经 CanBus 收帧、经关节控制，Decision-20261102-03：保活、反馈换算、多圈、力矩换算与饱和、第二组、反向机构、C610/C620/GM6020 混接、绝对位置、离线、ID 冲突、同一总线两个实例被拒、负载计算） |
| `angle_wrap` | `unit/motor/angle_wrap_test.cpp` | `driver_motor_dji` + `platform_mock`（转子多圈累计按不超过半圈解释） |
| `can_bus` | `unit/can_bus/can_bus_test.cpp` | `driver_can_bus` + `platform_mock`（按帧 ID 分发、区间重叠、兜底、没人认领、每周期回调顺序、总线占用、负载估算） |
| `joint` | `unit/motor/joint_test.cpp` | `driver_motor` + `platform_mock`（关节：绑定检查、机构换算、出力门控、指令超时、时间戳晚于 now、安全门禁止、全组使能，Decision-20261102-03） |
| `dm_motor` | `unit/motor/dm_motor_test.cpp` | `driver_motor_dm` + `platform_mock`（经 CanBus 收帧、经关节控制：启动核对的帧顺序、重发、各项不符、配置检查、清错与使能确认、指令超时、时间戳晚于本轮 now、使能失败、运行中故障与复位、离线、多圈位置与机构换算、阻抗的位置回绕、共用 MST_ID、同一 CanBus 上两组）。电机由 `unit/motor/dm_fake_motor.hpp` 按说明书的帧格式扮演 |
| `bmi088_acquisition` | `unit/imu_acquisition/bmi088_acquisition_test.cpp` | `driver_bmi088` + `platform_mock`（SPI 应答函数扮演 BMI088：初始化、进行中的数据就绪、共享 SPI、合并、有界队列、提前完成、时基回绕、DMA 错误） |
| `hal_gpio_pwm`、`hal_can`、`hal_spi`、`hal_uart`、`hal_usb_cdc` | `unit/hal/*_test.cpp` | `platform_mock`：hal 契约 v1 在替身上的语义（参数检查、结果优先级、通知时机、能力分级、Uart 与 UsbCdc 同形）。替身是驱动测试的依据 |
| `buzzer` | `unit/buzzer/buzzer_test.cpp` | `driver_buzzer` + `platform_mock` |
| `ws2812` | `unit/ws2812/ws2812_test.cpp` | `driver_ws2812` + `platform_mock`（波形编码、亮度、出错重发、不支持异步时的阻塞回退） |
| `heater_control` | `unit/control/heater_control_test.cpp` | `lib_control`（加热恒温控制接 DM-MC02 热模型仿真：加热器强度 ±25%、起点 25/37 ℃、4 种传感器相位下不过冲 0.25 ℃ 以上、16 s 内稳定；断电恢复；参数校验，Decision-20261101-05） |
| `gyro_bias_calibration` | `unit/estimate/gyro_bias_calibration_test.cpp` | `lib_estimate`（上电零偏标定，Decision-20261103-01：温度门控与稳定门控、静止窗口的结果、被碰、缓慢转动、加速度模长、超时预算跨窗口累计、温度出带暂停预算、时间戳回绕、终态、参数检查） |
| `imu_ready` / `imu_degraded` / `imu_reject` | `unit/imu/imu_test.cpp` | `app_imu` + `platform_mock`（IMU 能力，BMI088 由 SPI 替身扮演、中断按 2 kHz 触发；每个 topic 只能有一个发布者，按参数分进程：静止时 SETTLING → READY，零偏交给 VQF、角速度扣零偏、yaw 归零后不漂、转动时 yaw 增量；一直晃时超时进入 DEGRADED，安装方向换轴；参数检查） |
| `axis_mapping` | `unit/estimate/axis_mapping_test.cpp` | `lib_estimate`（传感器 → 机体的轴映射） |
| `hal_os_mock` | `unit/hal_os/mock_os_test.cpp` | `platform_mock`（hal_os 的宿主替身：时基由测试控制、换算辅助跨回绕、临界区嵌套与两线程互斥） |
| `signal` | `unit/hal_os/signal_test.cpp` | `platform_mock`（`Signal`：通知不丢、多次合并、超时按测试推进的时间且跨回绕、跨线程唤醒、一个线程先后等多个、多个线程轮流等、先检查后等待的写法在 20 万次通知下不丢唤醒） |
| `signal_misuse_notify_in_critical` / `signal_misuse_wait_in_critical` / `signal_misuse_two_waiters` | `unit/hal_os/signal_misuse_test.cpp` | `platform_mock` 的误用检查：每个场景都应打印原因并 `abort()`，按输出判定 |
| `crc16` | `unit/crc/crc16_test.cpp` | `utils_crc`（CRC-16/MCRF4XX：标准校验值、与逐位实现对照、裁判系统帧尾的小端约定） |
| `robolink_wire` | `unit/robolink/wire_test.cpp` | `middleware_robolink`（RoboLink v0.2 帧层与槽位层，Decision-20261103-01：对照 `test/data/robolink/golden_v0_2.txt` 逐字节组帧、逐条核对拒收原因；字节流解析的分块喂入、假同步、最长帧、末尾 0xD3、CRC 错整帧丢弃、缓冲区满） |
| `robolink_session` | `unit/robolink/session_test.cpp` | `middleware_robolink`（MCU 端连接会话，测试扮演 PC：握手应答对照标准帧、忙时重发、完整状态、200 Hz 状态与姿态同帧、诊断尾部顺延姿态、保活、判活边界、重新握手序号归零、拒绝的 HELLO、控制快照、断开重连、每轮最多 8 帧、开机时长跨回绕、时刻稍早于上次发出） |
| `robolink_messages` | `unit/robolink/messages_test.cpp` | `middleware_robolink`（8 条消息的载荷：解码标准帧逐字段核对，协议规定的拒收与改写，末尾追加字段按长度截断，编码长度） |
| `spsc_queue` | `unit/concurrency/spsc_queue_test.cpp` | `utils_concurrency`（SPSC 队列：空与满、下标绕回、批量截断与绕回、`clear`、容量 1、计数器跨过 2^32；生产者、消费者、旁观者三线程压力；`-O2`） |
| `latest_value` | `unit/concurrency/latest_value_test.cpp` | `utils_concurrency`（最新值邮箱：初值、覆盖、大小不是 4 的倍数、版本号按 2^31 回绕；一写三读的一致性压力；`-O2`） |
| `topic` | `unit/topic/topic_test.cpp` | `middleware_topic` + `platform_mock`（常量初始化与清单写法、单发布者、新数据判断与丢失计数、观测计数、发布只进一次临界区、`wait()` 立即返回、跨线程唤醒、超时跨回绕、旧通知不提前结束等待、等待槽上限、直接登记的 `Signal`、一次发布叫醒全部等待者；一个发布线程对阻塞等待、轮询、模拟中断三个订阅者的压力测试；`-O2`） |
| `bringup` | `unit/bringup/bringup_test.cpp` | `app_bringup` + `platform_mock`（Heartbeat 单发布者与序号；Monitor 的收到、丢失、延迟超限、延迟跨回绕；`topics::ALL`） |
| `bringup_imu` | `unit/bringup/imu_check_test.cpp` | `app_bringup` + `platform_mock`（ImuCheck：中断里发布 `bringup.imu_event`、驱动初始化、样本换算与 VQF、限频发布、加热、延迟超限） |
| `bringup_io` | `unit/bringup/io_checks_test.cpp` | `app_bringup` + `platform_mock`（CanCheck 只收、SbusCheck 跨段拼帧与失控保护、UsbReport 状态行与回显、Indicator 两声与灯的状态、TextLine 定点格式化） |
| `bringup_dm` | `unit/bringup/dm_check_test.cpp` | `app_bringup` + `platform_mock`（达妙电机检查，Decision-20261102-02：整个流程、跳过转动、没有电机、核对不符、使能失败；模拟电机含通讯超时与按指令的转速） |
| `safety_gate` | `unit/actuator/safety_gate_test.cpp` | `app_actuator` + `platform_mock`（安全门：上电锁住、确认后放开、急停锁住、解除后要再确认、发布者超时锁住，Decision-20261102-03） |

并发测试（`spsc_queue`、`latest_value`、`hal_os_mock`、`signal`、`topic`）可以加 `-fsanitize=thread` 在本地用 ThreadSanitizer 运行，CI 不跑。GCC 12 起会对 `LatestValue` 里的 `atomic_thread_fence` 报 `-Wtsan`，与 `-Werror` 一起用时加 `-Wno-tsan`。`spsc_queue`、`latest_value` 的回绕部分分别要读写 2^32 个字节、写 2^31 次，用 ThreadSanitizer 跑时先把轮数改小。ThreadSanitizer 程序启动即报 "unexpected memory mapping" 时（WSL 与 Ubuntu 24.04 都遇到过），用 `setarch $(uname -m) -R ./test_xxx` 运行。

配置阶段另有两类断言，失败会直接中止配置：

- **边界检查**：通用电机头（`joint.hpp`）不得引用具体电机协议或驱动：能力只用关节（Decision-20261102-03）。
- **topic 的消息约束**：正好 `MAX_MESSAGE_SIZE` 字节的消息能编译（对照）；大 1 字节的、不可平凡拷贝的都必须被拒绝。

## 目录

```text
test/host/
├── CMakeLists.txt
└── unit/{protocol,motor,imu_acquisition,hal,hal_os,crc,concurrency,topic,robolink,estimate,imu,buzzer,ws2812,bringup}/ 单元测试
```

## 来源与适配记录

> F2–F4（2026-11-01）：hal 契约 v1（Decision-20261101-01）——电机与 BMI088 的测试改用 `platform_mock`，删除 `support/dji_platform_stub.hpp`；新增 `hal_*`、`buzzer`、`ws2812`；`axis_mapping`（Decision-20261101-02）；`bringup_imu`、`bringup_io`（Decision-20261101-03）。
> F2（2026-10-31）：归档旧 app（Decision-20261031-13）——删除底盘集成测试 `chassis_controller`、`controller_equivalence`、`mock_*`（6 项）及其支持文件；`bmi088_acquisition` 去掉测旧 app 的两段；`bringup` 改为链接 `app_bringup`；`host_test()` 去掉 `MOCK_BACKEND`。
> F2（2026-10-31）：新增 `bringup`（测 `app/bringup` 的 Heartbeat、Monitor，测试里直接编译其源码，旧 app 归档后改为链接 `app_bringup`）；`topic` 补直接登记 `Signal` 的用例（Decision-20261031-12）。
> F1（2026-10-31）：新增 `topic` 与消息约束的配置期检查，测 F1 第 4 步的 `middleware_topic`，不来自 `fd1edbb`。
> F1（2026-10-31）：`hal_os_mock` 随 hal/os 契约 v1 改写（时基改为 1 µs，补多线程互斥），新增 `signal`、`signal_misuse`，不来自 `fd1edbb`。
> F1（2026-10-28、10-29）：新增 `crc16`、`spsc_queue`、`latest_value`，测 F1 新写的 utils 代码，不来自 `fd1edbb`。
> P2.5（2026-10-26）：`Application/Imu` 搬到 `app/imu/` 后，测试改为 `#include "imu/imu_diagnostics.cpp"`，不再链接 `legacy_include`（Decision-20261026-10）。
> P2（2026-10-26）：测试与支持文件名改为 snake_case，影子头随被替换的头文件改到 `shadow/driver/...`；include 改为相对层根目录的写法（Decision-20261026-08）。
> P1（2026-10-26）：宿主测试改由根工程引入，`foray_host_test()` 改名为 `host_test()`（Decision-20261026-06），框架单元测试改为链接被测 target，新增 `hal_os_mock`。
> P0.5（2026-10-26）把测试与被测代码中的 C++ 标识符按 ROS 2 命名统一改名（如 `SetEnabled` → `set_enabled`，`canStub` → `g_can_stub`）。下表是 P0 时的适配记录，保留当时的名字；新旧名字对照见 [P0.5 改名表](../../docs/ai/decisions/2026-10-26-P0.5-改名/改名表.tsv)，文件名对照见 [P2 移动表](../../docs/ai/decisions/2026-10-26-P2-机械搬迁/移动表.tsv)。

全部测试来自 ControllerCode 提交 `fd1edbb` 的 `Tests/`。提交 `21f78d6` 原样放入（blob 哈希与 `fd1edbb` 逐一一致），随后的提交只做以下适配，**未改动 `legacy/` 中任何文件**：

| 文件 | 改动 | 原因 |
|---|---|---|
| `CMakeLists.txt` | 重写：三个旧工程合为一个；被测源码与边界清单显式列出；include 路径与编译选项按 target 设置；编译期检查每次配置都重新运行，并加两项对照检查 | 组织规则禁止 `file(GLOB)` 收集源码与全局 `include_directories()`；旧工程按旧目录写死相对路径 |
| `unit/motor/{DjiMotorAdapter,AngleWrap,MotorGroup}Test.cpp`、`support/shadow/.../DjiMotorModelMap.hpp`、`integration/chassis/chassis_mock_test.cpp` | include 路径从 `Tests/Motor/Support/...` 改为 `support/...` | 目录布局变化 |
| `unit/motor/dji_motor_adapter_test.cpp` | 离线判定时刻由写死的 110 改为 `100U + DjiMotor::LinkTimeoutMs` | `a2cfd16` 把离线超时从 10 ms 调为 50 ms；仍验证"恰好到超时即离线" |
| `support/MockMotor.{hpp,cpp}` | 新增 `MaximumTorqueNewtonMeter()`，默认 1.8 N·m，未就绪返回 0 | `a2cfd16` 的 `MotorGroup` 要求适配器提供该接口（与 `DjiMotorAdapter` 同语义） |
| `integration/chassis/chassis_mock_test.cpp` | 启动任务前调用 `ChassisTask::SetEnabled(true)` | `a2cfd16` 起底盘上电默认失能，`Run` 不再自行使能 |

## 未恢复与已知缺口

- **`Tests/ImuStream`（待重写）**：被测模块 `Application/Imu/ImuStream.hpp`、`Libraries/Protocol/imu/ImuFrame.hpp` 在 `a2cfd16` 中已不存在。原文件可用 `git show fd1edbb:Tests/ImuStream/<文件>` 取回，供重写时参考。
- **底盘力矩能力检查未覆盖**：`ChassisTask::Init` 在 `a2cfd16` 新增"控制器限幅不得超过电机能力"的检查，现有场景没有覆盖能力不足时的失败路径。
- **发现（不在 P0 修复）**：`a2cfd16` 中没有任何代码调用 `ChassisTask::SetEnabled(true)`，该版本固件的底盘不会输出力矩。丢失的遥控任务（见 `docs/ai/decisions/2026-10-25-迁移基线与对照.md` §8）可能承担使能职责，P3b 重做时处理。
- `unit/motor/VALIDATION.md` 与 `unit/imu_acquisition/README.md` 是迁移源时期的历史记录，其中的路径、命令和资源数据对应旧工程。
