# app：RM 业务能力（产品）。CMake 文件按层组织：app/ 的全部 target 定义在这里，不在能力子目录里各放一个
# （Decision-20261026-07）。以 app/ 为 include 根：#include "topics/bringup.hpp"、"bringup/monitor.hpp"。
#
# - app_topics：本产品的全部 topic（规则 17），能力之间只经它通信（规则 18）；
# - app_<能力>：一个能力一个 target，只提供 init/step，零 RTOS，不得依赖其他能力、任务运行器与 platform；
# - app_common：任务运行器（FreeRTOS 原生 API），只有它与组合根能看到 RTOS 头（规则 10），只在固件构建中定义。
# 固件构建与宿主构建都加入本文件；宿主测试直接链接能力 target（Decision-20261031-13）。

# app_library(<名称> <源文件与头文件>...)：能力的 STATIC 库，带严格告警与 -Wdouble-promotion。
function(app_library name)
    add_library(${name} STATIC ${ARGN})
    target_include_directories(${name} PUBLIC "${PROJECT_SOURCE_DIR}/app")
    enable_strict_warnings(${name})
    # 与框架相同（规则 14）：能力也要在没有双精度 FPU 的板上跑
    target_compile_options(${name} PRIVATE $<$<COMPILE_LANGUAGE:C,CXX>:-Wdouble-promotion>)
endfunction()

# app_topics：本产品的全部 topic（消息类型 + 实例，规则 17），只依赖 topic 机制与时基（Decision-20261031-12）。
add_library(app_topics INTERFACE
    topics/all.hpp
    topics/bringup.hpp)
target_include_directories(app_topics INTERFACE "${PROJECT_SOURCE_DIR}/app")
target_link_libraries(app_topics INTERFACE middleware_topic hal_os)

# app_bringup：框架验证用的能力（心跳与监视），不含 RM 业务，零 RTOS（Decision-20261031-12）。
# 外设检查（Decision-20261101-03）：IMU（BMI088 + 中断里发布 + VQF + 加热）、CAN（只收）、SBUS、USB 报告、指示；
# 上电打开板上电源输出（Decision-20261101-06）；达妙电机检查（Decision-20261102-02）。
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
    bringup/power_outputs.cpp
    bringup/power_outputs.hpp
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

if(CMAKE_CROSSCOMPILING)
    # app_common：任务运行器（task_runner.hpp，只有头文件，Decision-20261031-12）。头文件用到 FreeRTOS 类型，
    # RTOS 头随之给组合根（规则 10：只有 app_common 与 assembly 可见）。
    add_library(app_common INTERFACE
        common/task_runner.hpp)
    target_include_directories(app_common INTERFACE "${PROJECT_SOURCE_DIR}/app")
    target_link_libraries(app_common INTERFACE hal_os middleware_topic os_freertos_${BOARD_ID})
endif()
