# 框架（评审报告 §5.9）：平台无关、可宿主编译；不得引用 app/、config/ 的路径或 target（规则 15）。
# 固件构建与宿主构建都会加入本文件。CMake 文件按层组织：src/ 的全部 target 定义在这里，
# 不在子目录里各放一个（Decision-20261026-07）。
# 以 src/ 为 include 根：#include "hal/can.hpp"、"driver/bmi088/bmi088.hpp"。

# framework_library(<名称> <源文件与头文件>...)：框架的 STATIC 库，带严格告警与 -Wdouble-promotion
#   （规则 14：float 被隐式提升为 double 时报错，Cortex-M4F 上 double 是软件库调用；豁免见例外 X-06）。
#   头文件也列出来，表明它属于哪个 target。
function(framework_library name)
    add_library(${name} STATIC ${ARGN})
    target_include_directories(${name} PUBLIC "${PROJECT_SOURCE_DIR}/src")
    enable_strict_warnings(${name})
    target_compile_options(${name} PRIVATE $<$<COMPILE_LANGUAGE:C,CXX>:-Wdouble-promotion>)
endfunction()

# utils_library(<名称> <源文件与头文件>...)：utils 的 STATIC 库，带严格告警。与 framework_library 相同，
#   单独起名是为了在依赖图里一眼看出 utils 只依赖标准库。
function(utils_library name)
    add_library(${name} STATIC ${ARGN})
    target_include_directories(${name} PUBLIC "${PROJECT_SOURCE_DIR}/src")
    enable_strict_warnings(${name})
    target_compile_options(${name} PRIVATE $<$<COMPILE_LANGUAGE:C,CXX>:-Wdouble-promotion>)
endfunction()

# framework_interface(<名称> <头文件>...)：只有头文件的框架库。
function(framework_interface name)
    add_library(${name} INTERFACE ${ARGN})
    target_include_directories(${name} INTERFACE "${PROJECT_SOURCE_DIR}/src")
    target_compile_features(${name} INTERFACE cxx_std_17)
endfunction()

# ==== 契约：hal、hal_os、boot ====

# hal：硬件资源的 C++ 契约 v1（Decision-20261101-01）：每类外设一个句柄类（can、gpio、pwm、spi、uart、usb_cdc），
# 成员函数由 platform 的 port 实现、宿主测试由 platform/mock 实现；hal::init() 是自由函数。
# 时间参数用 hal_os 的 Microseconds；中断回调统一为 hal::Callback。
framework_interface(hal
    hal/callback.hpp
    hal/can.hpp
    hal/gpio.hpp
    hal/init.hpp
    hal/pwm.hpp
    hal/spi.hpp
    hal/uart.hpp
    hal/usb_cdc.hpp)
target_link_libraries(hal INTERFACE hal_os)

# hal_os：最小 OS 契约 v1——1 µs 单调时基、RAII 临界区、二值信号（Decision-20261031-01～03）。
# 线程、队列、事件组不进框架。实现：固件在 port（F2），宿主测试在 platform/mock。
framework_interface(hal_os
    hal/os/clock.hpp
    hal/os/critical_section.hpp
    hal/os/signal.hpp)

# boot：生成区唯一可以调用的启动入口 boot_init / boot_init_rtos / boot_run（C 头，extern "C"，
# 架构 v2 §3.3，Decision-20261031-11）。实现在组合根，只有生成区与 assembly 依赖它。
framework_interface(boot
    boot/boot.h)

# ==== utils：没有领域含义、哪一层都可能用的基础件（docs/ai/architecture.md §2.3） ====
# 只依赖标准库，用 utils_library()；只有头文件的用 framework_interface()。
# 每个子目录一个 target：utils/<名> → utils_<名>，命名空间与子目录同名（math::、crc::、concurrency::）。

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
    utils/crc/crc16.cpp
    utils/crc/crc16.hpp
    utils/crc/crc32.cpp
    utils/crc/crc32.hpp
    utils/crc/crc8.cpp
    utils/crc/crc8.hpp)

# 并发原语：单生产者单消费者队列、单写者最新值邮箱（seqlock）。只用原子读写与内存屏障，
# 不用原子读改写（规则 13），Cortex-M0+ 也能用（Decision-20261029-01）。
framework_interface(utils_concurrency
    utils/concurrency/latest_value.hpp
    utils/concurrency/spsc_queue.hpp)

# ==== middleware：给能力用的运行期机制与服务（docs/ai/architecture.md §2.3） ====
# 可以依赖 utils 与 hal/os 契约（传输类服务另可依赖 hal）；driver、lib 不得依赖 middleware。
# 每个子目录一个 target：middleware/<名> → middleware_<名>。

framework_library(middleware_fsm
    middleware/fsm/fsm.cpp
    middleware/fsm/fsm.hpp)

# topic：最新值、单发布者、中断里可发布、阻塞等待、观测计数（架构 v2 §3.4，Decision-20261031-05）。
# 只有头文件，只依赖 utils_concurrency 与 hal_os 契约；发布时用临界区包住写入（Decision-20261029-02）。
framework_interface(middleware_topic
    middleware/topic/topic.hpp)
target_link_libraries(middleware_topic INTERFACE hal_os utils_concurrency)

# robolink：上位机链路协议 RoboLink v0.2 的帧层、槽位层、消息编解码与 MCU 端连接会话（Decision-20261103-01）。本仓唯一组 RoboLink 帧、
# 算链路 CRC 的地方（规则 11），由 tool/robolink/make_golden.py 独立生成的标准帧逐字节核对。只处理内存里的字节，
# 不依赖 hal 与 hal/os。
framework_library(middleware_robolink
    middleware/robolink/bytes.hpp
    middleware/robolink/messages.cpp
    middleware/robolink/messages.hpp
    middleware/robolink/session.cpp
    middleware/robolink/session.hpp
    middleware/robolink/wire.cpp
    middleware/robolink/wire.hpp)
target_link_libraries(middleware_robolink PRIVATE utils_crc)

# ==== lib：领域算法（docs/ai/architecture.md §2.3）。只依赖标准库、utils 与 lib 内部 ====
# 子目录名即命名空间名、即 target 名的后缀（lib/control → control:: → lib_control）。

framework_library(lib_control
    lib/control/heater_control.cpp
    lib/control/heater_control.hpp
    lib/control/pid.cpp
    lib/control/pid.hpp
    lib/control/slope.cpp
    lib/control/slope.hpp)
target_link_libraries(lib_control PUBLIC utils_math)

framework_library(lib_filter
    lib/filter/butterworth_low_pass.hpp
    lib/filter/filter.cpp
    lib/filter/filter.hpp)

# imu_body_mapping 只处理三维向量（轴映射），不再依赖 BMI088 驱动（F3，Decision-20261101-02）。
# gyro_bias_calibration：上电静止零偏标定，由迁移源的 GyroBiasCalibration 改写（Decision-20261103-01）。
framework_library(lib_estimate
    lib/estimate/extended_state_observer.hpp
    lib/estimate/gyro_bias_calibration.cpp
    lib/estimate/gyro_bias_calibration.hpp
    lib/estimate/imu_body_mapping.cpp
    lib/estimate/imu_body_mapping.hpp
    lib/estimate/kalman_filter.hpp
    lib/estimate/quaternion_ekf.cpp
    lib/estimate/quaternion_ekf.hpp
    lib/estimate/vqf.cpp
    lib/estimate/vqf.hpp)
target_link_libraries(lib_estimate PUBLIC utils_math lib_filter)
# 规则 14 的豁免（例外 X-06）：VQF 有意用 double 保精度，只在这个文件关掉 -Wdouble-promotion。
set_source_files_properties(lib/estimate/vqf.cpp PROPERTIES COMPILE_OPTIONS "-Wno-double-promotion")

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

framework_library(driver_protocol_dm
    driver/protocol/dm/dm_esc.cpp
    driver/protocol/dm/dm_esc.hpp)

framework_library(driver_protocol_sbus
    driver/protocol/sbus/sbus.cpp
    driver/protocol/sbus/sbus.hpp)

# CAN 总线的帧分发：一条总线唯一的接收方，按帧 ID 分给登记的驱动（Decision-20261102-03 D9）。
framework_library(driver_can_bus
    driver/can_bus/can_bus.cpp
    driver/can_bus/can_bus.hpp)
target_link_libraries(driver_can_bus PUBLIC hal hal_os)

# 电机的通用接口（不含任何具体品牌）：关节 Joint（Decision-20261102-03）。品牌驱动在 dji/、dm/，型号常量随品牌。
framework_library(driver_motor
    driver/motor/joint.cpp
    driver/motor/joint.hpp)
target_link_libraries(driver_motor PUBLIC hal hal_os)

# DJI 电机：驱动登记到 CanBus、经关节 Joint 控制，可混接 C610/C620/GM6020（Decision-20261102-03）；型号常量（型号固有参数属驱动层，Q11、例外 X-05）。
framework_library(driver_motor_dji
    driver/motor/dji/dji_models.hpp
    driver/motor/dji/dji_motor.cpp
    driver/motor/dji/dji_motor.hpp)
target_link_libraries(driver_motor_dji PUBLIC driver_can_bus driver_motor driver_protocol_dji hal)

# 达妙电机：驱动（启动回读核对、使能，Decision-20261102-01）登记到 CanBus、经关节 Joint 控制，型号常量（Decision-20261102-03）。
framework_library(driver_motor_dm
    driver/motor/dm/dm_models.hpp
    driver/motor/dm/dm_motor.cpp
    driver/motor/dm/dm_motor.hpp)
target_link_libraries(driver_motor_dm PUBLIC driver_can_bus driver_motor driver_protocol_dm hal)

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
target_link_libraries(driver_bmi088 PUBLIC hal lib_control utils_math)

framework_library(driver_buzzer
    driver/buzzer/buzzer.cpp
    driver/buzzer/buzzer.hpp)
target_link_libraries(driver_buzzer PUBLIC hal)

framework_library(driver_ws2812
    driver/ws2812/ws2812.cpp
    driver/ws2812/ws2812.hpp)
target_link_libraries(driver_ws2812 PUBLIC hal)
