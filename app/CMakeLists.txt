# app：RM 业务能力（产品，评审报告 §5.9）。只在固件构建中加入。
# CMake 文件按层组织：app/ 的全部 target 定义在这里，不在能力子目录里各放一个（Decision-20261026-07）。
# 能力（imu、chassis、gimbal、remote）各是一个 target，不得依赖 OS 与 platform 实现；
# 线程、节拍等任务外壳在 app_tasks，任务表与创建机制在 app_common，只有这两者与 assembly 能看到 RTOS 头（规则 10）。
#
# 迁移期源码在 legacy/Application，P2/P2.5 按评审报告附录 A 搬到对应目录。
# 过渡事实：TaskManager（任务表 + 创建机制）在 app_common 并依赖 app_tasks，P3b 把任务表拆到 config/assembly 后消除。
set(application_dir "${LEGACY_DIR}/Application")
set(task_dir "${application_dir}/Task")
set(os_freertos os_freertos_${BOARD_ID})

legacy_library(app_imu
    "${application_dir}/Imu/AttitudeEstimator.cpp"
    "${application_dir}/Imu/AttitudeEstimator.hpp"
    "${application_dir}/Imu/GyroBiasCalibration.cpp"
    "${application_dir}/Imu/GyroBiasCalibration.hpp"
    "${application_dir}/Imu/ImuDiagnostics.cpp"
    "${application_dir}/Imu/ImuDiagnostics.hpp"
    "${application_dir}/Imu/ZeroRateBiasObserver.cpp"
    "${application_dir}/Imu/ZeroRateBiasObserver.hpp")
target_link_libraries(app_imu PUBLIC lib_estimate lib_math driver_bmi088 hal)

legacy_library(app_chassis
    "${task_dir}/1_chassis/ChassisController.cpp"
    "${task_dir}/1_chassis/ChassisController.hpp")
target_link_libraries(app_chassis PUBLIC lib_control lib_math driver_motor hal)

legacy_library(app_gimbal
    "${application_dir}/Module/GimbalModule.cpp"
    "${application_dir}/Module/GimbalModule.hpp")
target_link_libraries(app_gimbal PUBLIC lib_component lib_control lib_math driver_motor hal)

legacy_library(app_remote
    "${task_dir}/4_communication/RemoteReceiver.cpp"
    "${task_dir}/4_communication/RemoteReceiver.hpp")
target_link_libraries(app_remote PUBLIC driver_protocol_sbus hal)

legacy_library(app_tasks
    "${task_dir}/0_imu/ImuDataReadyMailbox.hpp"
    "${task_dir}/0_imu/ImuTask.cpp"
    "${task_dir}/0_imu/ImuTask.hpp"
    "${task_dir}/1_chassis/ChassisTask.cpp"
    "${task_dir}/1_chassis/ChassisTask.hpp"
    "${task_dir}/2_gimbal/GimbalTask.cpp"
    "${task_dir}/2_gimbal/GimbalTask.hpp")
target_link_libraries(app_tasks
    PUBLIC app_imu app_chassis app_gimbal driver_bmi088 driver_motor driver_motor_dji hal
    PRIVATE ${os_freertos})

legacy_library(app_common
    "${task_dir}/TaskManager.cpp"
    "${task_dir}/TaskManager.hpp")
target_link_libraries(app_common
    PUBLIC app_tasks hal
    PRIVATE ${os_freertos})
