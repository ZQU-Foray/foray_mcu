# 宿主测试（test/host）

在宿主机上编译 `legacy/` 中的被测源码，用 ctest 运行，不需要开发板。

> 当前是 P0 的独立 CMake 工程：还没有根 CMakeLists，被测源码直接引用 `legacy/` 路径。
> P1 建根工程后改由根工程引入（`host-tests` 预设），P2 搬迁源码时同步更新 `CMakeLists.txt` 中的路径与边界检查清单。

## 运行

需要 CMake ≥ 3.22、Ninja，以及 GCC 或 Clang（不支持 MSVC）。

```sh
cmake -S test/host -B build/host-tests -G Ninja
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

Windows 上用 MSYS2 UCRT64 的 g++：配置时加 `-DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe`，并保证 `C:\msys64\ucrt64\bin` 在 `PATH` 中（运行测试需要它的运行库）。

已验证环境（2026-10-26，13/13 通过）：WSL Ubuntu 22.04 + g++ 11.4.0 + CMake 3.31.6；Windows + MSYS2 UCRT64 g++ 16.2.0 + CMake 4.4.4。

## 测试清单

| ctest 名称 | 测试源码 | 被测源码（`legacy/` 下） |
|---|---|---|
| `dji_protocol` | `unit/protocol/DjiEscTest.cpp` | `Libraries/Protocol/dji/DjiEsc.cpp` |
| `dji_motor` | `unit/motor/DjiMotorTest.cpp` | `DjiMotor.cpp` + 协议 |
| `dji_adapter` | `unit/motor/DjiMotorAdapterTest.cpp` | `DjiMotorAdapter.cpp` + 驱动 + 协议 |
| `angle_wrap` | `unit/motor/AngleWrapTest.cpp` | 驱动 + 协议 |
| `motor_group` | `unit/motor/MotorGroupTest.cpp` | 适配器 + 驱动 + 协议 |
| `brand_extension` | `unit/motor/BrandExtensionTest.cpp` | 只用 `MotorGroup.hpp`（非 DJI 最小适配器） |
| `bmi088_acquisition` | `unit/imu_acquisition/bmi088_acquisition_test.cpp` | `Libraries/Device/bmi088/*.cpp`、`Application/Imu/ImuDiagnostics.cpp` |
| `chassis_controller` | `integration/chassis/ChassisControllerTest.cpp` | `ChassisController.cpp`、`Pid.cpp`、`ChassisTask.cpp`（模拟电机） |
| `controller_equivalence` | `integration/chassis/ControllerEquivalenceTest.cpp` | 同上 |
| `mock_normal` / `mock_read_failure` / `mock_command_failure` / `mock_init_failure` | `integration/chassis/ChassisMockTest.cpp` | 同上，按场景参数运行 |

配置阶段另有两类断言，失败会直接中止配置：

- **边界检查**：底盘任务与通用电机头（`Motor.hpp`、`MotorGroup.hpp`）不得引用具体电机协议、适配器或驱动。
- **编译期检查**：合法电机组能编译；数量为 0、超过型号能力、未映射型号、未包含品牌映射这几种情况都必须被拒绝。两项对照检查（头文件本身可编译）保证"被拒绝"不是无关错误造成的。

## 目录

```text
test/host/
├── CMakeLists.txt
├── unit/{protocol,motor,imu_acquisition}/   单元测试
├── integration/chassis/                     底盘任务 + 控制器 + 模拟电机
└── support/                                 测试替身
    ├── DjiPlatformStub.hpp                  CAN/时基 C ABI 桩
    ├── MockMotor.{hpp,cpp}                  模拟电机适配器
    ├── TaskRuntimeStub.cpp                  RTOS/时基桩（纯控制测试）
    └── shadow/                              影子头：只对 MOCK_BACKEND 目标放在 include 最前
        ├── cmsis_os2.h                      替代 CMSIS-RTOS2 头
        └── Libraries/Device/motor/dji/DjiMotorModelMap.hpp   把 DJI 映射重定向到模拟适配器
```

## 来源与适配记录

全部测试来自 ControllerCode 提交 `fd1edbb` 的 `Tests/`。提交 `21f78d6` 原样放入（blob 哈希与 `fd1edbb` 逐一一致），随后的提交只做以下适配，**未改动 `legacy/` 中任何文件**：

| 文件 | 改动 | 原因 |
|---|---|---|
| `CMakeLists.txt` | 重写：三个旧工程合为一个；被测源码与边界清单显式列出；include 路径与编译选项按 target 设置；编译期检查每次配置都重新运行，并加两项对照检查 | 组织规则禁止 `file(GLOB)` 收集源码与全局 `include_directories()`；旧工程按旧目录写死相对路径 |
| `unit/motor/{DjiMotorAdapter,AngleWrap,MotorGroup}Test.cpp`、`support/shadow/.../DjiMotorModelMap.hpp`、`integration/chassis/ChassisMockTest.cpp` | include 路径从 `Tests/Motor/Support/...` 改为 `support/...` | 目录布局变化 |
| `unit/motor/DjiMotorAdapterTest.cpp` | 离线判定时刻由写死的 110 改为 `100U + DjiMotor::LinkTimeoutMs` | `a2cfd16` 把离线超时从 10 ms 调为 50 ms；仍验证"恰好到超时即离线" |
| `support/MockMotor.{hpp,cpp}` | 新增 `MaximumTorqueNewtonMeter()`，默认 1.8 N·m，未就绪返回 0 | `a2cfd16` 的 `MotorGroup` 要求适配器提供该接口（与 `DjiMotorAdapter` 同语义） |
| `integration/chassis/ChassisMockTest.cpp` | 启动任务前调用 `ChassisTask::SetEnabled(true)` | `a2cfd16` 起底盘上电默认失能，`Run` 不再自行使能 |

## 未恢复与已知缺口

- **`Tests/ImuStream`（待重写）**：被测模块 `Application/Imu/ImuStream.hpp`、`Libraries/Protocol/imu/ImuFrame.hpp` 在 `a2cfd16` 中已不存在。原文件可用 `git show fd1edbb:Tests/ImuStream/<文件>` 取回，供重写时参考。
- **底盘力矩能力检查未覆盖**：`ChassisTask::Init` 在 `a2cfd16` 新增"控制器限幅不得超过电机能力"的检查，现有场景没有覆盖能力不足时的失败路径。
- **发现（不在 P0 修复）**：`a2cfd16` 中没有任何代码调用 `ChassisTask::SetEnabled(true)`，该版本固件的底盘不会输出力矩。丢失的遥控任务（见 `docs/decisions/2026-10-25-迁移基线与对照.md` §8）可能承担使能职责，P3b 重做时处理。
- `unit/motor/VALIDATION.md` 与 `unit/imu_acquisition/README.md` 是迁移源时期的历史记录，其中的路径、命令和资源数据对应旧工程。
