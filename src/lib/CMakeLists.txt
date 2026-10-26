# lib：纯算法与组件，零平台（评审报告 §5.7）。只依赖标准库与 lib 内部。
# 迁移期源码在 legacy/Libraries/{Algorithm,Component}，P2 按子目录原名搬到本目录，P3b 统一为小写分类。
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
