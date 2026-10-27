# 框架（评审报告 §5.9）：平台无关、可宿主编译；不得引用 app/、config/ 的路径或 target（规则 15）。
# 固件构建与宿主构建都会加入本文件。CMake 文件按层组织：src/ 的全部 target 定义在这里，
# 不在子目录里各放一个（Decision-20261026-07）。
#
# 迁移期（P1–P2.5）源码仍在 legacy/ 原地，各 target 按目标位置组织、引用 legacy/ 的路径；
# P2 把文件搬到 src/ 对应子目录后，这里只改路径（docs/ai/decisions/2026-10-26-P1-构建骨架/）。
# 新代码以 src/ 为 include 根（#include "hal/os/clock.hpp"）。

# ==== 契约：hal、hal_detail、hal_os、boot ====

# hal：硬件资源的 C++ 契约（can、gpio、pwm、spi、time、uart、usb_cdc），由 platform 的 port 实现。
# 迁移期为 legacy/Platform/Interface/*.hpp，P2 搬到 src/hal/，P3a 定稿契约 v1。
# 迁移期还要暴露 legacy/Platform/Interface 本身：迁移源有按裸头名 include 的写法（如 "Time.hpp"），P2 去掉。
set(interface_dir "${LEGACY_DIR}/Platform/Interface")

add_library(hal INTERFACE
    "${interface_dir}/Can.hpp"
    "${interface_dir}/Gpio.hpp"
    "${interface_dir}/Pwm.hpp"
    "${interface_dir}/Spi.hpp"
    "${interface_dir}/Time.hpp"
    "${interface_dir}/Uart.hpp"
    "${interface_dir}/UsbCdc.hpp")
target_include_directories(hal INTERFACE "${interface_dir}")
target_link_libraries(hal INTERFACE hal_detail legacy_include)
target_compile_features(hal INTERFACE cxx_std_17)

# hal_detail：迁移期的 C ABI 层（Detail/*.h），port 的 C 实现与 C++ 契约之间的过渡；
# P2 暂放 src/hal/detail/，P3a 并入 C++ 契约后删除（例外 X-07）。
add_library(hal_detail INTERFACE
    "${interface_dir}/Detail/Can.h"
    "${interface_dir}/Detail/Gpio.h"
    "${interface_dir}/Detail/Pwm.h"
    "${interface_dir}/Detail/Spi.h"
    "${interface_dir}/Detail/Time.h"
    "${interface_dir}/Detail/Uart.h"
    "${interface_dir}/Detail/UsbCdc.h")
target_include_directories(hal_detail INTERFACE "${interface_dir}")

# hal_os：最小 OS 契约，只有 RAII 临界区与单调时基（评审报告 §5.7、Q10）。
# 线程、队列、事件不进框架。实现：固件在 port（P3a），宿主测试在 platform/mock。
# P1 是草案，P3a 冻结契约 v1 时可以调整。
add_library(hal_os INTERFACE
    hal/os/clock.hpp
    hal/os/critical_section.hpp)
target_include_directories(hal_os INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(hal_os INTERFACE cxx_std_17)

# boot：生成区可以调用的启动入口（extern "C"，评审报告 §5.6）。
# P1 只建立 target；迁移期的启动入口仍是 legacy/Application/Entry.h 的 Application_*（例外 X-09），
# P3b 落地 boot_init / boot_init_rtos / boot_run（src/boot/boot.h）后，生成区只认它。
add_library(boot INTERFACE)
target_include_directories(boot INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

# ==== utils：零平台工具与并发原语 ====
# 计数器、最新值邮箱、SPSC 队列（M0+ 上退回临界区实现）。P1 只建立 target；
# 内容在 P3a 由 PortKit 的队列/通知与现有序号快照写法通用化而来（评审报告 §5.7、§5.8）。
add_library(utils INTERFACE)
target_include_directories(utils INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(utils INTERFACE cxx_std_17)

# ==== lib：纯算法与组件，零平台（评审报告 §5.7）。只依赖标准库与 lib 内部 ====
# 迁移期源码在 legacy/Libraries/{Algorithm,Component}，P2 按子目录原名搬到 src/lib/，P3b 统一为小写分类。
set(algorithm_dir "${LEGACY_DIR}/Libraries/Algorithm")

legacy_library(lib_math
    "${algorithm_dir}/alg_math/BasicMath.cpp"
    "${algorithm_dir}/alg_math/BasicMath.h"
    "${algorithm_dir}/alg_math/Matrix.h"
    "${algorithm_dir}/alg_math/Matrix3.cpp"
    "${algorithm_dir}/alg_math/Matrix3.h"
    "${algorithm_dir}/alg_math/Quaternion.cpp"
    "${algorithm_dir}/alg_math/Quaternion.h"
    "${algorithm_dir}/alg_math/Vector3.h")

legacy_library(lib_control
    "${algorithm_dir}/alg_controller/Pid.cpp"
    "${algorithm_dir}/alg_controller/Pid.h")
target_link_libraries(lib_control PUBLIC lib_math)

legacy_library(lib_crc
    "${algorithm_dir}/alg_crc/Crc32.cpp"
    "${algorithm_dir}/alg_crc/Crc32.h"
    "${algorithm_dir}/alg_crc/Crc8.cpp"
    "${algorithm_dir}/alg_crc/Crc8.h")

legacy_library(lib_filter
    "${algorithm_dir}/alg_filter/ButterworthLowPass.hpp"
    "${algorithm_dir}/alg_filter/Filter.cpp"
    "${algorithm_dir}/alg_filter/Filter.h")

# 已知结构债：ImuBodyMapping 依赖 BMI088 驱动（lib → driver 反向依赖），P3b 改为 POD 入参或迁到 driver/imu
# （docs/ai/architecture.md "已知结构债"）。依赖图检查对这条边单独放行。
legacy_library(lib_estimate
    "${algorithm_dir}/alg_estimate/ExtendedStateObserver.hpp"
    "${algorithm_dir}/alg_estimate/ImuBodyMapping.cpp"
    "${algorithm_dir}/alg_estimate/ImuBodyMapping.hpp"
    "${algorithm_dir}/alg_estimate/KalmanFilter.hpp"
    "${algorithm_dir}/alg_estimate/QuaternionEkf.cpp"
    "${algorithm_dir}/alg_estimate/QuaternionEkf.hpp"
    "${algorithm_dir}/alg_estimate/Vqf.cpp"
    "${algorithm_dir}/alg_estimate/Vqf.hpp")
target_link_libraries(lib_estimate PUBLIC lib_math lib_filter driver_bmi088)

legacy_library(lib_fsm
    "${algorithm_dir}/alg_fsm/Fsm.cpp"
    "${algorithm_dir}/alg_fsm/Fsm.h")

legacy_library(lib_kinematics
    "${algorithm_dir}/alg_kinematics/Kinematics.cpp"
    "${algorithm_dir}/alg_kinematics/Kinematics.h")

legacy_library(lib_slope
    "${algorithm_dir}/alg_slope/Slope.cpp"
    "${algorithm_dir}/alg_slope/Slope.h")

legacy_interface(lib_component
    "${LEGACY_DIR}/Libraries/Component/AxisController.hpp")
target_link_libraries(lib_component INTERFACE lib_control lib_math)

# ==== driver：设备驱动与设备协议（评审报告 §5.7） ====
# 可以依赖 hal、lib 与驱动内的协议；不得依赖 msg、OS、app。
# 迁移期源码在 legacy/Libraries/{Device,Protocol}，P2 分别搬到 src/driver/ 与 src/driver/protocol/。
set(device_dir "${LEGACY_DIR}/Libraries/Device")
set(protocol_dir "${LEGACY_DIR}/Libraries/Protocol")

legacy_library(driver_protocol_dji
    "${protocol_dir}/dji/DjiEsc.cpp"
    "${protocol_dir}/dji/DjiEsc.hpp")

legacy_library(driver_protocol_sbus
    "${protocol_dir}/sbus/Sbus.cpp"
    "${protocol_dir}/sbus/Sbus.hpp")

# 电机的通用接口与编译期装配（不含任何具体品牌）。
legacy_interface(driver_motor
    "${device_dir}/motor/Motor.hpp"
    "${device_dir}/motor/MotorGroup.hpp")
target_link_libraries(driver_motor INTERFACE hal)

# DJI 电机：驱动、适配器与型号映射（型号固有参数属驱动层，Q11、例外 X-05）。
legacy_library(driver_motor_dji
    "${device_dir}/motor/dji/DjiMotor.cpp"
    "${device_dir}/motor/dji/DjiMotor.hpp"
    "${device_dir}/motor/dji/DjiMotorAdapter.cpp"
    "${device_dir}/motor/dji/DjiMotorAdapter.hpp"
    "${device_dir}/motor/dji/DjiMotorModelMap.hpp")
target_link_libraries(driver_motor_dji PUBLIC driver_motor driver_protocol_dji hal)

legacy_library(driver_bmi088
    "${device_dir}/bmi088/Bmi088.cpp"
    "${device_dir}/bmi088/Bmi088.hpp"
    "${device_dir}/bmi088/Bmi088Accel.cpp"
    "${device_dir}/bmi088/Bmi088Accel.hpp"
    "${device_dir}/bmi088/Bmi088Gyro.cpp"
    "${device_dir}/bmi088/Bmi088Gyro.hpp"
    "${device_dir}/bmi088/Bmi088Heater.cpp"
    "${device_dir}/bmi088/Bmi088Heater.hpp"
    "${device_dir}/bmi088/Bmi088SampleQueue.hpp"
    "${device_dir}/bmi088/Bmi088Temperature.cpp"
    "${device_dir}/bmi088/Bmi088Temperature.hpp"
    "${device_dir}/bmi088/Bmi088Transfer.hpp")
target_link_libraries(driver_bmi088 PUBLIC hal lib_control)

legacy_library(driver_buzzer
    "${device_dir}/buzzer/Buzzer.cpp"
    "${device_dir}/buzzer/Buzzer.hpp")
target_link_libraries(driver_buzzer PUBLIC hal)

legacy_library(driver_ws2812
    "${device_dir}/ws2812/Ws2812.cpp"
    "${device_dir}/ws2812/Ws2812.hpp")
target_link_libraries(driver_ws2812 PUBLIC hal)
