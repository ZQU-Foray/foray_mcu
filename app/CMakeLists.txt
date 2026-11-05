# app：RM 业务能力（产品）。CMake 文件按层组织：app/ 的全部 target 定义在这里，不在能力子目录里各放一个。
# 以 app/ 为 include 根：#include "topics/bringup.hpp"、"bringup/monitor.hpp"。
#
# - app_topics：本产品的全部 topic（规则 17），能力之间只经它通信（规则 18）；
# - app_<能力>：一个能力一个 target，只提供 init/step，零 RTOS，不得依赖其他能力、任务运行器与 platform；
# - app_common：任务运行器（FreeRTOS 原生 API），只有它与组合根能看到 RTOS 头（规则 10），只在固件构建中定义。
# 固件构建与宿主构建都加入本文件；宿主测试直接链接能力 target。

# app_library(<名称> <源文件与头文件>...)：能力的 STATIC 库，带严格告警与 -Wdouble-promotion。
function(app_library name)
    add_library(${name} STATIC ${ARGN})
    target_include_directories(${name} PUBLIC "${PROJECT_SOURCE_DIR}/app")
    enable_strict_warnings(${name})
    # 与框架相同（规则 14）：能力也要在没有双精度 FPU 的板上跑
    target_compile_options(${name} PRIVATE $<$<COMPILE_LANGUAGE:C,CXX>:-Wdouble-promotion>)
endfunction()

# app_topics：本产品的全部 topic（消息类型 + 实例，规则 17），只依赖 topic 机制与时基。
add_library(app_topics INTERFACE
    topics/all.hpp
    topics/bringup.hpp
    topics/command.hpp
    topics/gimbal.hpp
    topics/imu.hpp
    topics/operator_input.hpp
    topics/pc_link.hpp
    topics/remote.hpp
    topics/safety.hpp)
target_include_directories(app_topics INTERFACE "${PROJECT_SOURCE_DIR}/app")
target_link_libraries(app_topics INTERFACE middleware_topic hal_os)

# app_actuator：安全门，按 safety.stop 集中决定一组关节能不能出力。
app_library(app_actuator
    actuator/safety_gate.cpp
    actuator/safety_gate.hpp
    actuator/power_outputs.cpp
    actuator/power_outputs.hpp)
target_link_libraries(app_actuator PUBLIC app_topics middleware_topic driver_motor hal_os)

# app_imu：板载 BMI088 → 云台姿态 imu.attitude；上电零偏标定（SETTLING/READY/DEGRADED）与 yaw 归零。
# 由 bringup::ImuCheck 改写。
app_library(app_imu
    imu/imu.cpp
    imu/imu.hpp)
target_link_libraries(app_imu PUBLIC
    app_topics
    middleware_topic
    hal
    hal_os
    driver_bmi088
    lib_estimate
    utils_math)

# app_pc_link：上位机链路 RoboLink v0.2 经 USB 虚拟串口：imu.attitude → GIMBAL_ATTITUDE，
# 收到的控制快照发布到 pc_link.control（第一版只记账）。不叫 link：与 POSIX 的 link() 重名。
app_library(app_pc_link
    pc_link/pc_link.cpp
    pc_link/pc_link.hpp)
target_link_libraries(app_pc_link PUBLIC
    app_topics
    middleware_topic
    middleware_robolink
    hal
    hal_os)

# app_command：仲裁。把操作输入（现在只有 SBUS 遥控）翻译成功能，发布 safety.stop 与 command.gimbal。
# 绑定与功能映射在 Parameters 里，能力本身不认具体的拨杆。
app_library(app_command
    command/arbiter.cpp
    command/arbiter.hpp)
target_link_libraries(app_command PUBLIC
    app_topics
    middleware_topic
    hal_os
    utils_math)

# app_remote：遥控的最小版——只把 SBUS 解析成 safety.stop（急停与人工确认），不控制云台与底盘。
# 仲裁能力做好后改发布 remote.input，safety.stop 由仲裁发布。
app_library(app_remote
    remote/remote.cpp
    remote/remote.hpp)
target_link_libraries(app_remote PUBLIC
    app_topics
    middleware_topic
    hal
    hal_os
    driver_protocol_sbus)

# app_bringup：框架验证用的能力（心跳与监视），不含 RM 业务，零 RTOS。
# 外设检查：IMU（BMI088 + 中断里发布 + VQF + 加热）、CAN（只收）、SBUS、USB 报告、指示；
# 上电打开板上电源输出；达妙电机检查。
app_library(app_bringup
    bringup/can_check.cpp
    bringup/can_check.hpp
    bringup/dm_check.cpp
    bringup/dm_check.hpp
    bringup/heartbeat.cpp
    bringup/heartbeat.hpp
    bringup/imu_check.cpp
    bringup/imu_check.hpp
    bringup/indicator.cpp
    bringup/indicator.hpp
    bringup/monitor.cpp
    bringup/monitor.hpp
    bringup/sbus_check.cpp
    bringup/sbus_check.hpp
    bringup/text_line.cpp
    bringup/text_line.hpp
    bringup/usb_report.cpp
    bringup/usb_report.hpp)
target_link_libraries(app_bringup PUBLIC
    app_topics
    middleware_topic
    hal
    hal_os
    driver_bmi088
    driver_buzzer
    driver_motor_dm
    driver_ws2812
    driver_protocol_sbus
    lib_estimate
    utils_math)

# app_gimbal：云台能力。按 imu.attitude 稳住指向、跟踪 PC 目标；只用关节 Joint，不认识电机品牌。
# axis_loop 是单轴串级的控制律（纯计算），由 gimbal.cpp 使用，也单独做宿主测试。
app_library(app_gimbal
    gimbal/axis_loop.cpp
    gimbal/axis_loop.hpp
    gimbal/gimbal.cpp
    gimbal/gimbal.hpp)
target_link_libraries(app_gimbal PUBLIC
    app_topics
    middleware_topic
    driver_motor
    hal_os
    utils_math
    lib_control)

if(CMAKE_CROSSCOMPILING)
    # app_common：任务运行器（task_runner.hpp，只有头文件）。头文件用到 FreeRTOS 类型，
    # RTOS 头随之给组合根（规则 10：只有 app_common 与 assembly 可见）。
    add_library(app_common INTERFACE
        common/task_runner.hpp)
    target_include_directories(app_common INTERFACE "${PROJECT_SOURCE_DIR}/app")
    target_link_libraries(app_common INTERFACE hal_os middleware_topic os_freertos_${BOARD_ID})
endif()
