#ifndef PARAMS_HPP
#define PARAMS_HPP

#include "actuator/power_outputs.hpp"
#include "actuator/safety_gate.hpp"
#include "command/arbiter.hpp"
#include "gimbal/gimbal.hpp"
#include "imu/imu.hpp"
#include "pc_link/pc_link.hpp"
#include "remote/remote.hpp"

#include <cstdint>

namespace params
{
inline constexpr device::Bmi088Heater::Config HEATER{control::HeaterControl::Model{0.967F, 15.5F, 0.95F}};

inline constexpr imu::Imu::Parameters IMU{
    device::Bmi088::Config{device::Bmi088Gyro::Bandwidth::ODR_2000_HZ_532_HZ},
    estimate::IDENTITY_AXIS_MAPPING,
    1000U,
    50.0F,
    estimate::GyroBiasCalibration::Config{},
    0.0F,
    0.35F,
    0.05F,
};

// 上位机链路（RoboLink v0.2）：固件版本 0.1.0；心跳 20 ms、状态 200 Hz、诊断尾部 1 s、判活 500 ms、
// 每轮最多处理 8 帧；每轮最多读 8 次 USB（512 B）
inline constexpr pc_link::PcLink::Parameters PC_LINK{
    robolink::Session::Config{{0U, 1U, 0U}, 0U, 0U, 20U, 5000U, 1000000U, 500000U, 8U},
    8U,
};

inline constexpr actuator::PowerOutputs::Parameters POWER_OUTPUTS{true};

// 云台电机：CAN1 上两台 DM-J4310（MIT 模式，PMAX 12.5、VMAX 30、TMAX 10）。机构：输出轴直连云台（传动比 1），
// 方向让关节与 IMU 同号（yaw 左转为正、pitch 低头为正），零点在云台水平朝前。
inline constexpr std::uint16_t YAW_ESC_ID{1U};
inline constexpr std::uint16_t YAW_MASTER_ID{0x11U};
inline constexpr device::JointMechanics YAW_MECHANICS{1, 1.0F, 0.0F, 3000U};
inline constexpr std::uint16_t PITCH_ESC_ID{2U};
inline constexpr std::uint16_t PITCH_MASTER_ID{0x12U};
inline constexpr device::JointMechanics PITCH_MECHANICS{1, 1.0F, 0.0F, 3000U};

// 云台（角速度环 → 角度环，力矩模式）。下面是调参的起步值：先 pitch、先角速度环，边调边改这里。
// - yaw 先不出力：两环增益都给 0，用手扶着；调好 pitch 以后再照同样的顺序调 yaw；
// - pitch 角速度环 kp 从 0.05 起，每步翻倍到出现嗡嗡声或抖动再退一半；ki 从 kp 的 5–10 倍起，消掉重力下垂；
// - pitch 角度环 kp 从 5 起往上加，到出现超调再退一点；角速度上限先给 6 rad/s；
// - 总限幅第一次出力只给 1 N·m（4310 峰值 12.5 N·m，夹到手指会很疼），调好后再放宽；
// - 重力前馈先给 0：水平静止时读 Report 里 pitch 的 compensation_newton_meter，那个稳态值就是 τ_g。
inline constexpr gimbal::Gimbal::Parameters GIMBAL{
    // yaw：小 yaw 按有限位处理，偏移 = IMU 的 yaw − 小 yaw 关节角。范围是占位值，按线缆实测收紧
    gimbal::Gimbal::AxisParameters{
        gimbal::AxisLoop::Parameters{
            gimbal::AxisLoop::Gains{0.0F, 0.0F, 0.0F, 6.0F},
            gimbal::AxisLoop::Gains{0.0F, 0.0F, 0.0F, 1.0F},
        },
        true,
        -3.14159265F,
        3.14159265F,
    },
    // pitch：机械限位按关节角给（水平为 0、低头为正）。范围是占位值，上板实测后改
    gimbal::Gimbal::AxisParameters{
        gimbal::AxisLoop::Parameters{
            gimbal::AxisLoop::Gains{5.0F, 0.0F, 0.0F, 6.0F},
            gimbal::AxisLoop::Gains{0.05F, 0.3F, 1.0F, 1.0F},
        },
        true,
        -0.6F,
        1.0F,
    },
    0.0F,    // pitch_gravity_newton_meter：低头方向为正，枪管一侧偏重时是负数
    1.0F,    // torque_limit_newton_meter
    10000U,  // attitude_timeout_us：10 ms 没有新姿态就停
    200000U, // enable_timeout_us：200 ms 没确认就回 DISABLED
    100000U, // target_timeout_us：PC 目标默认 100 ms 有效
};

// 安全门：仲裁每轮（1 kHz）发布一次 safety.stop，50 ms 收不到就当发布者挂了
inline constexpr actuator::SafetyGate::Parameters SAFETY{50000U};

// 仲裁：拨杆 → 功能。哪个拨杆的哪一档是什么功能是占位值，上板核对后改。
// 每个功能最多两条绑定（以后一条给遥控、一条给键鼠），任一条成立功能就成立。
inline constexpr command::Parameters COMMAND{
    std::array<command::Bindings, command::FUNCTION_COUNT>{
        // RUN（放开安全门）：拨杆 0 在上档
        command::Bindings{command::on_switch(command::Source::REMOTE, 0U, topics::SwitchPosition::UP),
                          command::unbound()},
        // AUTO_AIM（自瞄开关）：拨杆 1 在上档
        command::Bindings{command::on_switch(command::Source::REMOTE, 1U, topics::SwitchPosition::UP),
                          command::unbound()},
        // FIRE（开火）：拨杆 2 在下档，最好是带回弹的开关
        command::Bindings{command::on_switch(command::Source::REMOTE, 2U, topics::SwitchPosition::DOWN),
                          command::unbound()},
    },
    // 手动瞄准：右摇杆 x 管 yaw、右摇杆 y 管 pitch；满量程 ±3 rad/s，上限 6 rad/s
    command::ManualAim{topics::STICK_RIGHT_X, topics::STICK_RIGHT_Y, 3.0F, 3.0F, 6.0F},
    0.2F,   // engage_range_rad：PC 目标离瞄准方向超过 0.2 rad 就不接管（0 表示不限制）
    50000U, // input_timeout_us：50 ms 收不到操作输入就当遥控断了
    20000U, // target_timeout_us：PC 目标的默认有效期
};

// 遥控：摇杆与拨杆的标定。通道、量程、门限都是占位值（SBUS 典型原始值 172/992/1811、三档 1000/1500/2000），
// 上板先用 bringup 固件读 bringup.sbus 的通道、把摇杆推到底、把拨杆拨一遍，记下实际值再改这里。
// 摇杆顺序：右 x、右 y、左 x、左 y（topics::STICK_RIGHT_X 等）。
inline constexpr remote::Remote::Parameters REMOTE{
    {
        remote::Remote::Stick{0U, 172U, 992U, 1811U, false, 0.05F},
        remote::Remote::Stick{1U, 172U, 992U, 1811U, false, 0.05F},
        remote::Remote::Stick{2U, 172U, 992U, 1811U, false, 0.05F},
        remote::Remote::Stick{3U, 172U, 992U, 1811U, false, 0.05F},
    },
    {
        remote::Remote::Switch{4U, 1200U, 1600U, topics::SwitchPosition::DOWN, topics::SwitchPosition::UP},
        remote::Remote::Switch{5U, 1200U, 1600U, topics::SwitchPosition::DOWN, topics::SwitchPosition::UP},
        remote::Remote::Switch{6U, 1200U, 1600U, topics::SwitchPosition::DOWN, topics::SwitchPosition::UP},
    },
    100000U, // frame_timeout_us：100 ms 没有正常帧就当链路断了
};
} // namespace params

#endif
