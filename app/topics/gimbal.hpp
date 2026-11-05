#ifndef TOPICS_GIMBAL_HPP
#define TOPICS_GIMBAL_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/command.hpp"

// 云台能力 gimbal::Gimbal 发布的 topic。

namespace topics::gimbal
{

/**
 * @brief 云台当前的状态：模式、实际用的目标、测量的姿态、yaw 关节角。
 *
 * 仲裁用它判断自瞄目标离当前瞄准方向多远；底盘以后跟随云台也用这一份，不必自己从 imu.attitude 算。
 */
struct State
{
    topics::command::GimbalMode mode;
    float yaw_rad;        // 测量值，回绕在 (−π, π]
    float pitch_rad;      // 测量值
    float yaw_rate;       // 测量值，rad/s
    float pitch_rate;     // 测量值，rad/s
    float yaw_target_rad; // 实际用的目标（有限位时已钳位）
    float pitch_target_rad;
    float yaw_joint_rad; // yaw 关节相对底座的角度
    hal::os::Microseconds timestamp_us;
};

inline topic::Topic<State> state{"gimbal.state"};

} // namespace topics::gimbal

#endif
