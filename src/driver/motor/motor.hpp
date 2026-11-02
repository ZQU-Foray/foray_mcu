#ifndef DRIVER_MOTOR_MOTOR_HPP
#define DRIVER_MOTOR_MOTOR_HPP

#include "hal/os/clock.hpp"
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
    hal::os::Microseconds last_update_us{0U}; // 最后一次收到反馈的时刻
    // 正在出力：已 enable()，且电机确认已使能。DJI 电调没有使能命令，enable() 之后即为 true。
    bool enabled{false};
    // 电机的设置与型号档案不符、电机报故障或使能失败（Decision-20261102-01）。
    bool fault{false};
};

// 型号选择只用于装配；控制器不依赖本枚举。
enum class MotorModel : std::uint8_t
{
    M2006,
    M3508,
    GM6020_CURRENT,
    DM_J4310
};

struct MotorConnection final
{
    std::uint8_t device_id{0U}; // DJI：电调编号；达妙：ESC_ID
    int direction{1};
    std::uint16_t master_id{0U}; // 达妙：反馈与应答的帧 ID（MST_ID）；DJI 不用
};

} // namespace device
#endif
