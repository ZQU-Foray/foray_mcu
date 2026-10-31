# app：RM 业务能力（产品，评审报告 §5.9）。只在固件构建中加入。
# CMake 文件按层组织：app/ 的全部 target 定义在这里，不在能力子目录里各放一个（Decision-20261026-07）。
# 能力（imu、chassis、gimbal、remote）各是一个 target，不得依赖 OS 与 platform 实现；
# 线程、节拍等任务外壳在 app_tasks，任务表与创建机制在 app_common，只有这两者与 assembly 能看到 RTOS 头（规则 10）。
# 以 app/ 为 include 根：#include "tasks/imu_task.hpp"、"chassis/chassis_controller.hpp"。
#
# 过渡事实：TaskManager（任务表 + 创建机制）在 app_common 并依赖 app_tasks，P3b 把任务表拆到 config/assembly 后消除。
set(os_freertos os_freertos_${BOARD_ID})

# app_library(<名称> <源文件与头文件>...)：产品的 STATIC 库，带严格告警。
#   迁移期的构建开关见例外 X-14（P3b 收进 profile 后去掉）。
function(app_library name)
    add_library(${name} STATIC ${ARGN})
    target_include_directories(${name} PUBLIC "${PROJECT_SOURCE_DIR}/app")
    target_link_libraries(${name} PRIVATE legacy_build_options)
    enable_strict_warnings(${name})
endfunction()

# app_topics：本产品的全部 topic（消息类型 + 实例，规则 17），只依赖 topic 机制与时基（Decision-20261031-12）。
# 能力、任务运行器与组合根都可以依赖它；能力之间只经它通信（规则 18）。
add_library(app_topics INTERFACE
    topics/all.hpp
    topics/bringup.hpp)
target_include_directories(app_topics INTERFACE "${PROJECT_SOURCE_DIR}/app")
target_link_libraries(app_topics INTERFACE middleware_topic hal_os)

# app_bringup：框架验证用的能力（心跳与监视），不含 RM 业务，零 RTOS（Decision-20261031-12）。
app_library(app_bringup
    bringup/heartbeat.cpp
    bringup/heartbeat.hpp
    bringup/monitor.cpp
    bringup/monitor.hpp)
target_link_libraries(app_bringup PUBLIC app_topics middleware_topic hal_os)

app_library(app_imu
    imu/attitude_estimator.cpp
    imu/attitude_estimator.hpp
    imu/gyro_bias_calibration.cpp
    imu/gyro_bias_calibration.hpp
    imu/imu_diagnostics.cpp
    imu/imu_diagnostics.hpp
    imu/zero_rate_bias_observer.cpp
    imu/zero_rate_bias_observer.hpp)
target_link_libraries(app_imu PUBLIC lib_estimate utils_math driver_bmi088 hal)

app_library(app_chassis
    chassis/chassis_controller.cpp
    chassis/chassis_controller.hpp)
target_link_libraries(app_chassis PUBLIC lib_control utils_math driver_motor hal)

app_library(app_gimbal
    gimbal/gimbal_module.cpp
    gimbal/gimbal_module.hpp)
target_link_libraries(app_gimbal PUBLIC lib_component lib_control utils_math driver_motor hal)

app_library(app_remote
    remote/remote_receiver.cpp
    remote/remote_receiver.hpp)
target_link_libraries(app_remote PUBLIC driver_protocol_sbus hal)

app_library(app_tasks
    tasks/imu_data_ready_mailbox.hpp
    tasks/imu_task.cpp
    tasks/imu_task.hpp
    tasks/chassis_task.cpp
    tasks/chassis_task.hpp
    tasks/gimbal_task.cpp
    tasks/gimbal_task.hpp)
target_link_libraries(app_tasks
    PUBLIC app_imu app_chassis app_gimbal driver_bmi088 driver_motor driver_motor_dji hal
    PRIVATE ${os_freertos})

# app_common：任务运行器（task_runner.hpp，FreeRTOS 原生 API，Decision-20261031-12）。
# 迁移期还含旧的 TaskManager（task_manager.*，依赖 app_tasks），F2 第 4 步随旧 app 归档。
# 运行器的头文件用到 FreeRTOS 类型，RTOS 头随之 PUBLIC 给组合根（规则 10：只有 app_common 与 assembly 可见）。
app_library(app_common
    common/task_manager.cpp
    common/task_manager.hpp
    common/task_runner.hpp)
target_link_libraries(app_common
    PUBLIC app_tasks hal hal_os middleware_topic ${os_freertos})
