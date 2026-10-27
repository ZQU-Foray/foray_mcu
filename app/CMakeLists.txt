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

app_library(app_imu
    imu/attitude_estimator.cpp
    imu/attitude_estimator.hpp
    imu/gyro_bias_calibration.cpp
    imu/gyro_bias_calibration.hpp
    imu/imu_diagnostics.cpp
    imu/imu_diagnostics.hpp
    imu/zero_rate_bias_observer.cpp
    imu/zero_rate_bias_observer.hpp)
target_link_libraries(app_imu PUBLIC lib_estimate lib_math driver_bmi088 hal)

app_library(app_chassis
    chassis/chassis_controller.cpp
    chassis/chassis_controller.hpp)
target_link_libraries(app_chassis PUBLIC lib_control lib_math driver_motor hal)

app_library(app_gimbal
    gimbal/gimbal_module.cpp
    gimbal/gimbal_module.hpp)
target_link_libraries(app_gimbal PUBLIC lib_component lib_control lib_math driver_motor hal)

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

app_library(app_common
    common/task_manager.cpp
    common/task_manager.hpp)
target_link_libraries(app_common
    PUBLIC app_tasks hal
    PRIVATE ${os_freertos})
