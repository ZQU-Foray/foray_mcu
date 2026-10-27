#ifndef DRIVER_MOTOR_MOTOR_HPP
#define DRIVER_MOTOR_MOTOR_HPP

#include "hal/time.hpp"
#include <cstddef>
#include <cstdint>

namespace device
{

// 物理量均指配置的输出轴与正方向。
// 位置以首次反馈归零；离线期间的运动不做推断。
struct MotorState final
{
    float angle_degrees{0.0F};
    float speed_rpm{0.0F};
    bool feedback_valid{false};
    bool online{false};
    platform::Time::Tick last_update_tick{0U};
};

// 型号选择只用于装配；控制器不依赖本枚举。
enum class MotorModel : std::uint8_t
{
    M2006,
    M3508,
    GM6020_CURRENT
};
struct MotorConnection final
{
    std::uint8_t device_id{0U};
    int direction{1};
};

} // namespace device
#endif
