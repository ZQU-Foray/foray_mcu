# 框架（评审报告 §5.9）：平台无关、可宿主编译；不得引用 app/、config/ 的路径或 target（规则 15）。
# 固件构建与宿主构建都会加入本文件。CMake 文件按层组织：src/ 的全部 target 定义在这里，
# 不在子目录里各放一个（Decision-20261026-07）。
# 以 src/ 为 include 根：#include "hal/can.hpp"、"driver/bmi088/bmi088.hpp"。

# framework_library(<名称> <源文件与头文件>...)：框架的 STATIC 库，带严格告警。
#   头文件也列出来，表明它属于哪个 target。迁移期的构建开关见例外 X-14（F2、F3 去掉）。
function(framework_library name)
    add_library(${name} STATIC ${ARGN})
    target_include_directories(${name} PUBLIC "${PROJECT_SOURCE_DIR}/src")
    target_link_libraries(${name} PRIVATE legacy_build_options)
    enable_strict_warnings(${name})
endfunction()

# utils_library(<名称> <源文件与头文件>...)：utils 的 STATIC 库，带严格告警。
#   utils 只依赖标准库，不链接迁移期的构建开关（例外 X-14 不适用于 utils）。
function(utils_library name)
    add_library(${name} STATIC ${ARGN})
    target_include_directories(${name} PUBLIC "${PROJECT_SOURCE_DIR}/src")
    enable_strict_warnings(${name})
endfunction()

# framework_interface(<名称> <头文件>...)：只有头文件的框架库。
function(framework_interface name)
    add_library(${name} INTERFACE ${ARGN})
    target_include_directories(${name} INTERFACE "${PROJECT_SOURCE_DIR}/src")
    target_compile_features(${name} INTERFACE cxx_std_17)
endfunction()

# ==== 契约：hal、hal_detail、hal_os、boot ====

# hal：硬件资源的 C++ 契约（can、gpio、pwm、spi、time、uart、usb_cdc），由 platform 的 port 实现；P3a 定稿契约 v1。
framework_interface(hal
    hal/can.hpp
    hal/gpio.hpp
    hal/pwm.hpp
    hal/spi.hpp
    hal/time.hpp
    hal/uart.hpp
    hal/usb_cdc.hpp)
target_link_libraries(hal INTERFACE hal_detail)

# hal_detail：迁移期的 C ABI 层，port 的 C 实现与 C++ 契约之间的过渡；P3a 并入 C++ 契约后删除（例外 X-07）。
# 只能经 src/ 根以 "hal/detail/xxx.h" 引用：其中的 time.h、spi.h 等与标准库、CubeMX 的头文件同名，
# 不得把 src/hal/detail 本身加入 include 路径。
framework_interface(hal_detail
    hal/detail/can.h
    hal/detail/gpio.h
    hal/detail/pwm.h
    hal/detail/spi.h
    hal/detail/time.h
    hal/detail/uart.h
    hal/detail/usb_cdc.h)

# hal_os：最小 OS 契约，只有 RAII 临界区与单调时基（评审报告 §5.7、Q10）。
# 线程、队列、事件不进框架。实现：固件在 port（P3a），宿主测试在 platform/mock。
# P1 是草案，P3a 冻结契约 v1 时可以调整。
framework_interface(hal_os
    hal/os/clock.hpp
    hal/os/critical_section.hpp)

# boot：生成区可以调用的启动入口（extern "C"，评审报告 §5.6）。
# 迁移期的启动入口仍是 legacy/Application/Entry.h 的 Application_*（例外 X-09），
# P3b 落地 boot_init / boot_init_rtos / boot_run（src/boot/boot.h）后，生成区只认它。
framework_interface(boot)

# ==== utils：没有领域含义、哪一层都可能用的基础件（docs/ai/architecture.md §2.3） ====
# 只依赖标准库，用 utils_library()。每个子目录一个 target：utils/<名> → utils_<名>，命名空间与子目录同名（math::、crc::）。

utils_library(utils_math
    utils/math/basic_math.cpp
    utils/math/basic_math.hpp
    utils/math/matrix.hpp
    utils/math/matrix3.cpp
    utils/math/matrix3.hpp
    utils/math/quaternion.cpp
    utils/math/quaternion.hpp
    utils/math/vector3.hpp)

utils_library(utils_crc
    utils/crc/crc32.cpp
    utils/crc/crc32.hpp
    utils/crc/crc8.cpp
    utils/crc/crc8.hpp)

# ==== middleware：给能力用的运行期机制与服务（docs/ai/architecture.md §2.3） ====
# 可以依赖 utils 与 hal/os 契约（传输类服务另可依赖 hal）；driver、lib 不得依赖 middleware。
# 每个子目录一个 target：middleware/<名> → middleware_<名>。

framework_library(middleware_fsm
    middleware/fsm/fsm.cpp
    middleware/fsm/fsm.hpp)

# ==== lib：领域算法（docs/ai/architecture.md §2.3）。只依赖标准库、utils 与 lib 内部 ====
# 子目录名即命名空间名、即 target 名的后缀（lib/control → control:: → lib_control）。

framework_library(lib_control
    lib/control/pid.cpp
    lib/control/pid.hpp
    lib/control/slope.cpp
    lib/control/slope.hpp)
target_link_libraries(lib_control PUBLIC utils_math)

framework_library(lib_filter
    lib/filter/butterworth_low_pass.hpp
    lib/filter/filter.cpp
    lib/filter/filter.hpp)

# 已知结构债：imu_body_mapping 依赖 BMI088 驱动（lib → driver 反向依赖），F3 改为 POD 入参
# （docs/ai/architecture.md "已知结构债"）。依赖图检查对这条边单独放行。
framework_library(lib_estimate
    lib/estimate/extended_state_observer.hpp
    lib/estimate/imu_body_mapping.cpp
    lib/estimate/imu_body_mapping.hpp
    lib/estimate/kalman_filter.hpp
    lib/estimate/quaternion_ekf.cpp
    lib/estimate/quaternion_ekf.hpp
    lib/estimate/vqf.cpp
    lib/estimate/vqf.hpp)
target_link_libraries(lib_estimate PUBLIC utils_math lib_filter driver_bmi088)

framework_library(lib_kinematics
    lib/kinematics/kinematics.cpp
    lib/kinematics/kinematics.hpp)

framework_interface(lib_component
    lib/component/axis_controller.hpp)
target_link_libraries(lib_component INTERFACE lib_control utils_math)

# ==== driver：设备驱动与设备协议（评审报告 §5.7） ====
# 可以依赖 hal、lib、utils 与驱动内的协议；不得依赖 middleware、OS、app。

framework_library(driver_protocol_dji
    driver/protocol/dji/dji_esc.cpp
    driver/protocol/dji/dji_esc.hpp)

framework_library(driver_protocol_sbus
    driver/protocol/sbus/sbus.cpp
    driver/protocol/sbus/sbus.hpp)

# 电机的通用接口与编译期装配（不含任何具体品牌）。
framework_interface(driver_motor
    driver/motor/motor.hpp
    driver/motor/motor_group.hpp)
target_link_libraries(driver_motor INTERFACE hal)

# DJI 电机：驱动、适配器与型号映射（型号固有参数属驱动层，Q11、例外 X-05）。
framework_library(driver_motor_dji
    driver/motor/dji/dji_motor.cpp
    driver/motor/dji/dji_motor.hpp
    driver/motor/dji/dji_motor_adapter.cpp
    driver/motor/dji/dji_motor_adapter.hpp
    driver/motor/dji/dji_motor_model_map.hpp)
target_link_libraries(driver_motor_dji PUBLIC driver_motor driver_protocol_dji hal)

framework_library(driver_bmi088
    driver/bmi088/bmi088.cpp
    driver/bmi088/bmi088.hpp
    driver/bmi088/bmi088_accel.cpp
    driver/bmi088/bmi088_accel.hpp
    driver/bmi088/bmi088_gyro.cpp
    driver/bmi088/bmi088_gyro.hpp
    driver/bmi088/bmi088_heater.cpp
    driver/bmi088/bmi088_heater.hpp
    driver/bmi088/bmi088_sample_queue.hpp
    driver/bmi088/bmi088_temperature.cpp
    driver/bmi088/bmi088_temperature.hpp
    driver/bmi088/bmi088_transfer.hpp)
target_link_libraries(driver_bmi088 PUBLIC hal lib_control)

framework_library(driver_buzzer
    driver/buzzer/buzzer.cpp
    driver/buzzer/buzzer.hpp)
target_link_libraries(driver_buzzer PUBLIC hal)

framework_library(driver_ws2812
    driver/ws2812/ws2812.cpp
    driver/ws2812/ws2812.hpp)
target_link_libraries(driver_ws2812 PUBLIC hal)
