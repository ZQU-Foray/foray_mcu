# 宿主测试（test/host）

在宿主机上编译被测代码，用 ctest 运行，不需要开发板。由根工程在宿主构建中引入（`host-tests` preset）。

- 测框架代码的单元测试直接链接框架 target（`lib_*`、`driver_*`）与 `platform_mock`；
- 测产品代码的底盘集成测试要用影子头替换 DJI 映射与 `cmsis_os2.h`，仍在测试里直接编译 app 源码（例外 X-10，P3b 改为链接）；
- 测试以 `test/host` 为根引用支持文件（`"support/mock_motor.hpp"`），以 `src/`、`app/` 为根引用被测代码（`"driver/bmi088/bmi088.hpp"`、`"tasks/chassis_task.hpp"`、`"imu/imu_diagnostics.hpp"`）。

## 运行

需要 CMake ≥ 3.22、Ninja，以及 GCC 或 Clang（不支持 MSVC）。在仓库根执行：

```sh
cmake --preset host-tests
cmake --build --preset host-tests
ctest --preset host-tests
```

Windows 上用 MSYS2 UCRT64 的 gcc/g++：在 `CMakeUserPresets.json`（不入库）里继承 `host-tests` 并指定 `CMAKE_C_COMPILER`、`CMAKE_CXX_COMPILER`，或配置时加 `-DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe`，并保证 `C:\msys64\ucrt64\bin` 在 `PATH` 中（运行测试需要它的运行库）。

CI 的 `host-tests` 作业（`.github/workflows/ci.yml`）在 GitHub Actions 的 Ubuntu 上运行同样的三条命令，并检查宿主构建的依赖图。

已验证环境（2026-10-26，14/14 通过）：WSL Ubuntu 22.04 + g++ 11.4.0 + CMake 3.31.6；Windows + MSYS2 UCRT64 g++ 16.2.0 + CMake 4.4.4。

## 测试清单

可执行文件名为 `test_<ctest 名称>`。

| ctest 名称 | 测试源码 | 被测 target 或源码 |
|---|---|---|
| `dji_protocol` | `unit/protocol/dji_esc_test.cpp` | `driver_protocol_dji` |
| `dji_motor` | `unit/motor/dji_motor_test.cpp` | `driver_motor_dji`（含协议） |
| `dji_adapter` | `unit/motor/dji_motor_adapter_test.cpp` | `driver_motor_dji` |
| `angle_wrap` | `unit/motor/angle_wrap_test.cpp` | `driver_motor_dji` |
| `motor_group` | `unit/motor/motor_group_test.cpp` | `driver_motor_dji` |
| `brand_extension` | `unit/motor/brand_extension_test.cpp` | `driver_motor`（只用 `motor_group.hpp`，非 DJI 最小适配器） |
| `bmi088_acquisition` | `unit/imu_acquisition/bmi088_acquisition_test.cpp` | `driver_bmi088`；测试源码直接 `#include` 了 `imu/imu_diagnostics.cpp`（`app/imu/`） |
| `hal_os_mock` | `unit/hal_os/mock_os_test.cpp` | `platform_mock`（hal_os 的宿主替身：时基、临界区嵌套） |
| `crc16` | `unit/crc/crc16_test.cpp` | `utils_crc`（CRC-16/MCRF4XX：标准校验值、与逐位实现对照、裁判系统帧尾的小端约定） |
| `chassis_controller` | `integration/chassis/chassis_controller_test.cpp` | `lib_control`；直接编译 `app/chassis/chassis_controller.cpp`、`app/tasks/chassis_task.cpp`（模拟电机） |
| `controller_equivalence` | `integration/chassis/controller_equivalence_test.cpp` | 同上 |
| `mock_normal` / `mock_read_failure` / `mock_command_failure` / `mock_init_failure` | `integration/chassis/chassis_mock_test.cpp` | 同上，按场景参数运行 |

配置阶段另有两类断言，失败会直接中止配置：

- **边界检查**：底盘任务与通用电机头（`motor.hpp`、`motor_group.hpp`）不得引用具体电机协议、适配器或驱动。
- **编译期检查**：合法电机组能编译；数量为 0、超过型号能力、未映射型号、未包含品牌映射这几种情况都必须被拒绝。对照检查（头文件本身可编译、每个被拒绝的型号在合法数量下能编译）保证"被拒绝"不是无关错误（如型号名写错）造成的。

## 目录

```text
test/host/
├── CMakeLists.txt
├── unit/{protocol,motor,imu_acquisition,hal_os,crc}/ 单元测试
├── integration/chassis/                     底盘任务 + 控制器 + 模拟电机
└── support/                                 测试替身
    ├── dji_platform_stub.hpp                CAN/时基 C ABI 桩
    ├── mock_motor.{hpp,cpp}                 模拟电机适配器
    ├── task_runtime_stub.cpp                RTOS/时基桩（纯控制测试）
    └── shadow/                              影子头：只对 MOCK_BACKEND 目标放在 include 最前
        ├── cmsis_os2.h                      替代 CMSIS-RTOS2 头
        └── driver/motor/dji/dji_motor_model_map.hpp   把 DJI 映射重定向到模拟适配器
```

## 来源与适配记录

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
