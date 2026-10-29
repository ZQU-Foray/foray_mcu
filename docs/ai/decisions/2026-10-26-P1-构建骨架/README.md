# P1 构建骨架：过程、对照与发现（2026-10-26）

根工程按目标结构（评审报告 §5.2）建立 target，源码仍在 `legacy/` 原地编译，**业务源码零改动**。决策见 [decision.md](../../../../decision.md) Decision-20261026-06（target 名去掉项目前缀）、Decision-20261026-07（P1 的设计决定）。

## 构建方式

```sh
# 固件（Arm GNU 工具链取自环境变量 ARM_GCC_ROOT 或 PATH；本机路径可写在 CMakeUserPresets.json）
cmake --preset arm-debug   && cmake --build --preset arm-debug     # → build/arm-debug/firmware_baseline.{elf,bin,hex,map}
cmake --preset arm-release && cmake --build --preset arm-release
cmake --build --preset arm-debug --target flash                     # 烧录（OpenOCD + CMSIS-DAP），会覆盖芯片固件

# 宿主测试（GCC 或 Clang）
cmake --preset host-tests && cmake --build --preset host-tests && ctest --preset host-tests
```

唯一的开关是 `-DPROFILE`（preset 里固定为 `baseline`）。profile 的 `profile.cmake` 声明板卡目录，板卡的 `board.cmake` 声明板标识与 CPU，配置时与工具链的 CPU 核对，不一致直接报错。

## target 与源码归属

每个 legacy 文件都明确归属于一个 target（头文件也列入），P2 按这份归属搬迁。CMake 文件按层组织：一层的全部 target 写在该层的一个 `CMakeLists.txt` 里，手写的共 7 个（根、`src/`、`app/`、`config/`、`platform/st/stm32h7/board_dm/`、`platform/mock/`、`test/host/`）。P1 初版曾按子目录各放一个（共 14 个），按负责人意见合并；合并前后依赖图（节点、边与可见性）与 Debug、Release、宿主三套编译数据库的每条编译命令逐字相同，符号对照与 14 项测试照常通过。

| 层 | target | 迁移期来源（`legacy/` 下） | P2/P2.5 目标位置 |
|---|---|---|---|
| 契约 | `hal`（INTERFACE） | `Platform/Interface/*.hpp` | `src/hal/` |
| | `hal_detail`（INTERFACE，过渡） | `Platform/Interface/Detail/*.h` | `src/hal/detail/`（P3a 删除） |
| | `hal_os`（INTERFACE） | 新写：`src/hal/os/{clock,critical_section}.hpp` | — |
| | `boot`、`utils`（INTERFACE） | 暂无内容（P3b、P3a） | — |
| lib | `lib_math`、`lib_control`、`lib_crc`、`lib_filter`、`lib_estimate`、`lib_fsm`、`lib_kinematics`、`lib_slope`（STATIC），`lib_component`（INTERFACE） | `Libraries/Algorithm/alg_*`、`Libraries/Component` | `src/lib/` |
| driver | `driver_protocol_dji`、`driver_protocol_sbus`、`driver_motor_dji`、`driver_bmi088`、`driver_buzzer`、`driver_ws2812`（STATIC），`driver_motor`（INTERFACE） | `Libraries/Protocol`、`Libraries/Device` | `src/driver/`、`src/driver/protocol/` |
| app | `app_imu`、`app_chassis`、`app_gimbal`、`app_remote`、`app_tasks`、`app_common`（STATIC） | `Application/Imu`、`Module`、`Task/*` | `app/` 各能力目录、`app/tasks`、`app/common` |
| config | `profile_baseline`（INTERFACE，生成头）、`assembly_baseline`（STATIC） | `Application/Entry.*` | `config/assembly/`（P2.5） |
| platform | `mx_generated_dm_mc02`、`rtos_dm_mc02`、`usb_dm_mc02`、`port_dm_mc02`、`board_dm_mc02`、`portkit`（OBJECT），`os_freertos_dm_mc02`（INTERFACE） | `Platform/STM32H7/DM_MC02/{Core,USB_DEVICE,Drivers,Middlewares,Port,startup,.ld}`、`Platform/PortKit` | `platform/st/stm32h7/board_dm/{cubemx,port,board}/` |
| mock | `platform_mock`（STATIC） | 新写：`platform/mock/mock_os.*` | — |
| 过渡 | `legacy_include`、`legacy_build_options`（INTERFACE） | `cmake/legacy.cmake` | P2、P3b 删除 |
| 固件 | `firmware_baseline`（EXECUTABLE） | 链接 `assembly_baseline` 与 `port_dm_mc02` | — |

`driver_buzzer`、`driver_ws2812`、`lib_crc`、`lib_fsm`、`lib_kinematics`、`lib_slope` 没有被任何能力使用，照常编译，但不进入固件。这和迁移源一致：迁移源里它们的代码在链接时被 `--gc-sections` 全部回收。

## 验证

### 符号集合（主判据，评审报告 §7.3）

`tool/ci/symbols.py compare <迁移源 ELF> <根工程 ELF>`：比较函数与对象符号的名称、大小、类型、绑定和输出段，不比较地址。

| 配置 | Windows，xPack GCC 15.2.1 | Linux，Arm GNU Toolchain 15.2.Rel1 |
|---|---|---|
| Debug | 一致，1426 个 | 一致，1426 个 |
| Release | 一致，1038 个 | 一致，1038 个 |

完整符号表（含没有大小的符号）只有一处不同：根工程里没有 `QuaternionEkf.cpp` 的文件名符号。原因如下：
- 迁移源把全部目标文件直接链接，这个文件的代码虽然全部被 `--gc-sections` 回收，文件名符号仍留在符号表里；
- 根工程里它在静态库 `lib_estimate` 中，没有被任何代码引用，所以不会从库里取出。

### 镜像与段（辅助参考）

| 项 | 迁移源 | 根工程 | 说明 |
|---|---:|---:|---|
| Debug `.bin` | 238,320 B | 238,304 B | 链接顺序改变，对齐填充不同 |
| Debug `.text` | 200,200 B | 200,184 B | 同上 |
| Debug `.bss` | 90,112 B | 90,108 B | 同上 |
| Release `.bin` | 100,664 B | 100,660 B | 同上 |
| Release `.data` | 4,432 B | 4,428 B | 同上 |
| Release `.bss` | 90,052 B | 90,044 B | 同上 |

SHA 如预期不同，P1–P2.5 不以 SHA 为判据。迁移源自身的构建没有变化，它的 Release 镜像仍与基线逐字节一致（CI 继续校验到 P2）。

### 编译选项

逐文件比较两边 `compile_commands.json`（Debug，112 个文件）：两边编译的文件集合相同；告警、优化、调试、语言标准、浮点等选项全部一致。差异只有宏，而且都是对应文件不使用的：

| 文件 | 迁移源多出 | 根工程多出 |
|---|---|---|
| Application、Libraries（36） | CubeMX 宏、`ARM_MATH_CM7` | — |
| Core/Src 生成代码、USB_DEVICE、启动文件（22） | `BMI088_GYRO_BANDWIDTH_REGISTER`、`IMU_ENABLE_DIAGNOSTICS` | — |
| Port（8） | `ARM_MATH_CM7` 与上述两个开关 | — |
| HAL 驱动、`system_stm32h7xx.c`（30） | — | `ARM_MATH_CM7`（与生成代码同在 `mx_generated`） |
| FreeRTOS、USB 库、PortKit（16） | — | — |

include 路径按 target 收窄：业务代码不再看得到 CubeMX 与 HAL 的头文件。

### 其他

- **宿主测试**：`ctest --preset host-tests` 共 14 项（原 13 项加 `hal_os_mock`）全部通过。验证环境为 Windows MSYS2 g++ 16.2 与 WSL g++ 11.4。所有框架库都在宿主机上带 `-Werror` 编译通过。
- **依赖图**：`tool/ci/arch_check.py graph` 检查固件图（39 个 target、103 条边）与宿主图（35 个 target、72 条边），都通过。迁移期放行的边有 5 条，见 Decision-20261026-07。反向验证也做了：分别人为加入 `app_imu → port_dm_mc02`、`lib_math → driver_bmi088`、`driver_bmi088 → app_imu`、`app_tasks → assembly_baseline` 和一条成环的边，都被拦下，报出的规则编号正确。
- **文本扫描**：`tool/ci/arch_check.py scan`（规则 3、8、9、12、15）通过。
- **命名与格式**：`naming.py --check`（根工程三个 preset，50 个 C++ 源文件加 2 个孤立头）0 处违规；`format.py --check` 共 131 个文件通过。

## 发现

1. **`-Wall` 会改变 -O0 下生成的代码**（GCC 15.2.1 实测）：`while (1) {}` 这样的空死循环，带 `-Wall` 时循环里多一条 `nop`，函数大 2 字节。最初只给手写代码加 `-Wall` 时，有 34 个函数各差 2 字节（中断处理函数、`main`、FreeRTOS 中含 `configASSERT` 的函数）。改为和迁移源一样给全部代码加 `-Wall` 后一致。所以对照编译选项时，告警选项也要算进去。
2. **编译选项以 `compile_commands.json` 为准**：我起初从 `build.ninja` 摘选项时把相邻条目对错了，误以为厂商 C 文件没有 `-std=gnu11`。逐文件比较编译数据库后确认，全部 C 文件都是 gnu11。
3. **OBJECT 库的目标文件不会传递**：CMake 只把直接链接的 OBJECT 库的目标文件放进最终链接。port 下的生成区、assembly 下的 board 都要用 `link_object_library()`，把 `$<TARGET_OBJECTS:…>` 放进链接接口，目标文件才能进入固件。这样依赖图也和评审报告 §5.2 保持一致：graphviz 不把这个生成器表达式显示成节点，只显示为一条自环，检查脚本会忽略它。
4. **头文件保护宏不能用双下划线**：`architecture.md` 原来的示例 `FORAY__HAL__CAN_HPP_` 通不过 `.clang-tidy` 的宏命名检查，而且双下划线在 C++ 里是保留标识符。改为与迁移源一致的单下划线写法（如 `HAL_OS_CLOCK_HPP`）。
5. **P0.5 遗留的空过检查**：`test/host/CMakeLists.txt` 的"8 个 GM6020 必须被拒绝"仍用改名前的 `GM6020Current`。旧名不存在，编译照样失败，检查看起来通过，实际已不再检验数量上限。已在本 PR 的第一个提交修复，并补了正向对照。

## 存档

- [symbols-Debug.txt](symbols-Debug.txt)、[symbols-Release.txt](symbols-Release.txt)：迁移源构建的归一化符号表（`symbols.py dump`，xPack GCC 15.2.1）。它与 Linux 上 Arm GNU Toolchain 15.2.Rel1 的构建结果也逐项一致，可以跨平台使用。P2 搬迁后迁移源自身的构建失效，CI 的符号对照改用这两份作基准。

## 迁移期过渡件与到期

| 过渡件 | 作用 | 到期 |
|---|---|---|
| `legacy_include`（X-11） | 暴露 `legacy/` 作为 include 根 | P2 结束时删除 |
| `legacy_build_options`（X-14） | 两个构建期开关加给全部业务 target | P3b 收进 profile 后删除 |
| `hal_detail`（X-07） | C ABI 层 | P3a 删除 |
| `portkit`（X-15） | PortKit 原样编译 | P2.5 拆分后删除 |
| `hal` 暴露 `legacy/Platform/Interface` | 迁移源按裸头名 include（`"Time.hpp"`） | P2 去掉裸头名 include 后删除 |
| 底盘集成测试直接编译 app 源码（X-10） | 影子头替换 DJI 映射与 `cmsis_os2.h` | P3b |
| CI 的迁移源构建与 SHA 校验 | 证明 `legacy/` 没有被改动 | P2 搬迁时删除 |
