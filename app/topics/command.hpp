#ifndef TOPICS_COMMAND_HPP
#define TOPICS_COMMAND_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"

#include <cstdint>

// 仲裁能力 command::Arbiter 发布的 topic。

namespace topics::command
{

/** 云台怎么动。取值按顺序比较，新增的加在最后。 */
enum class GimbalMode : std::uint8_t
{
    HOLD = 0U,    // 目标不变
    RATE = 1U,    // 手动：yaw_rate、pitch_rate 是角速度指令，云台自己积分
    ABSOLUTE = 2U // 自瞄、全自动：跟踪世界系目标，角速度是前馈
};

/**
 * 云台指令。
 * - RATE：yaw_rate、pitch_rate 有效，yaw_rad、pitch_rad 忽略；
 * - ABSOLUTE：yaw_rad、pitch_rad 是世界系目标（左转、低头为正），yaw_rate、pitch_rate 是前馈；
 * - HOLD：其余字段忽略，云台保持当前目标。
 */
struct Gimbal
{
    GimbalMode mode;
    float yaw_rad;
    float pitch_rad;
    float yaw_rate;
    float pitch_rate;
    hal::os::Microseconds timestamp_us;
};

inline topic::Topic<Gimbal> gimbal{"command.gimbal"};

} // namespace topics::command

#endif
