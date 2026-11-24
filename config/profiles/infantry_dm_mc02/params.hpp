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

// IMU 的安装方向：板子跟着 pitch 转，但水平转了 90°，BMI088 的 x 指向云台左、y 指向后、z 朝上
// （2026-11-06 实测：低头时不改映射的话 IMU 报的是 roll）。云台系 x（枪管向前）= −y，y（向左）= x，z = z。
inline constexpr imu::Imu::Parameters IMU{
    device::Bmi088::Config{device::Bmi088Gyro::Bandwidth::ODR_2000_HZ_532_HZ},
    estimate::AxisMapping{{1U, 0U, 2U}, {-1, 1, 1}},
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

// 云台电机：CAN1 上两台 DM-J4310（MIT 模式，PMAX 12.5、VMAX 30、TMAX 10）。
// 机构：pitch 输出轴直连云台；yaw 经同步带传动，约 1:1（实测关节角与 IMU 的比约 1.02，按 1 处理）。
// 方向让关节与 IMU 同号（yaw 左转为正、pitch 低头为正），零点在云台水平朝前。
// 2026-11-06 实测：两台的方向都是 +1；零点已在电机里设成云台水平朝前（断电不丢），所以 zero_offset_rad 为 0。
// yaw 的同步带松了会跳齿：力矩打满来回猛抖时跳过约 66°，零点和软限位跟着偏。跳过以后要拉紧、在正前方
// 重新设零点；平时看 IMU yaw 与关节角之差（底座朝向）是否不变。
inline constexpr std::uint16_t YAW_ESC_ID{1U};
inline constexpr std::uint16_t YAW_MASTER_ID{0x11U};
inline constexpr device::JointMechanics YAW_MECHANICS{1, 1.0F, 0.0F, 3000U};
inline constexpr std::uint16_t PITCH_ESC_ID{2U};
inline constexpr std::uint16_t PITCH_MASTER_ID{0x12U};
inline constexpr device::JointMechanics PITCH_MECHANICS{1, 1.0F, 0.0F, 3000U};

// 云台（角速度环 → 角度环，力矩模式）。2026-11-06 上板调出的 PID（点亮用，之后换 LQR + ESO）：
// - 角速度环 kp：只开 P、逐档翻倍。pitch 在 1.6 振荡（约 49 Hz），yaw 在 3.2 振荡（约 24 Hz）；
//   陀螺静止噪声约 0.06 rad/s，kp 再大主要是放大噪声，所以离振荡点留 2.5 倍以上：pitch 0.5、yaw 0.6；
// - 角速度环积分取 kp 的 5 倍，积分上限 0.3 N·m：上限给 1 N·m 时按住再松手会攒满积分、超调约 90%；
//   托住云台只要零点几 N·m（重力矩约 0.05，齿轮箱静摩擦约 0.2）；
// - pitch 角度环 kp 20：压下 3–5° 松手约 0.15 s 回位、超调约 10%；
// - yaw 角度环 kp 8、角速度上限 3 rad/s：yaw 惯量是 pitch 的 3–5 倍，总限幅 1 N·m 下 kp 20 会打满力矩、
//   来回猛抖（约 5 Hz）；kp 8 时拨开 2–4° 约 0.5 s 回位、无超调；
// - 总限幅 1 N·m（4310 峰值 12.5 N·m，夹到手指会很疼），放宽前先重新验 yaw；
// - 重力前馈：点亮时读不出 τ_g（重力矩比摩擦小，稳态补偿力矩随摩擦在 −0.1 ~ +0.05 N·m 之间漂）；
//   2026-11-06 辨识的 G 项（四段扫描合并）是 −0.046 N·m，按它填；
// - 积分碰上库仑摩擦会慢慢"爬行"：静止时每十来秒跳一次、幅度约 0.2°。
inline constexpr gimbal::Gimbal::Parameters GIMBAL{
    // yaw：小 yaw 没有滑环，按有限位处理，偏移 = IMU 的 yaw − 小 yaw 关节角。
    // 2026-11-06 实测两端在 +1.574（左）与 −1.573（右），每边往里收 0.07 rad
    gimbal::Gimbal::AxisParameters{
        gimbal::AxisLoop::Parameters{
            gimbal::AxisLoop::Gains{8.0F, 0.0F, 0.0F, 3.0F},
            gimbal::AxisLoop::Gains{0.6F, 3.0F, 0.3F, 1.0F},
        },
        true,
        -1.50F,
        1.50F,
        6.0F, // maximum_target_rate：目标转动速度上限 rad/s（自瞄吸过去的速度、手动角速度的钳位）
    },
    // pitch：机械限位按关节角给（水平为 0、低头为正）。
    // 2026-11-06 实测挡块在 −0.581（抬头）与 +0.378（低头），每边往里收 0.06 rad
    gimbal::Gimbal::AxisParameters{
        gimbal::AxisLoop::Parameters{
            gimbal::AxisLoop::Gains{20.0F, 0.0F, 0.0F, 6.0F},
            gimbal::AxisLoop::Gains{0.5F, 2.5F, 0.3F, 1.0F},
        },
        true,
        -0.52F,
        0.32F,
        6.0F, // maximum_target_rate
    },
    -0.046F, // pitch_gravity_newton_meter：低头方向为正，枪管一侧偏重时是负数；辨识值
    1.0F,    // torque_limit_newton_meter
    10000U,  // attitude_timeout_us：10 ms 没有新姿态就停
    200000U, // enable_timeout_us：200 ms 没确认就回 DISABLED
    20000U, // command_timeout_us：20 ms 收不到 command.gimbal 就按 HOLD（仲裁同一个任务里每轮都发）
};

// 安全门：仲裁每轮（1 kHz）发布一次 safety.stop，50 ms 收不到就当发布者挂了
inline constexpr actuator::SafetyGate::Parameters SAFETY{50000U};

// 仲裁：拨杆 → 功能。每个功能最多两条绑定（以后一条给遥控、一条给键鼠），任一条成立功能就成立。
// 自瞄开着并按住开火、PC 目标有效时，云台由 PC 接管。
inline constexpr command::Parameters COMMAND{
    std::array<command::Bindings, command::FUNCTION_COUNT>{
        // RUN（放开安全门）：拨杆 0（总开关）在上档
        command::Bindings{command::on_switch(command::Source::REMOTE, 0U, topics::SwitchPosition::UP),
                          command::unbound()},
        // AUTO_AIM（自瞄开关）：拨杆 1（三档）在上档；中档、下档是关
        command::Bindings{command::on_switch(command::Source::REMOTE, 1U, topics::SwitchPosition::UP),
                          command::unbound()},
        // FIRE（开火）：拨杆 2（弹性开关）按住
        command::Bindings{command::on_switch(command::Source::REMOTE, 2U, topics::SwitchPosition::DOWN),
                          command::unbound()},
    },
    // 手动瞄准：右摇杆 x 管 yaw、右摇杆 y 管 pitch。摇杆向右、向上为正，而 yaw 左转、pitch 低头为正，
    // 所以两个满量程都是负的（往右推 → yaw 往右转，往上推 → 抬头）；上限 6 rad/s
    command::ManualAim{topics::STICK_RIGHT_X, topics::STICK_RIGHT_Y, -3.0F, -3.0F, 6.0F},
    0.2F,    // engage_range_rad：PC 目标离瞄准方向超过 0.2 rad 就不接管（0 表示不限制）
    50000U,  // input_timeout_us：50 ms 收不到操作输入就当遥控断了
    100000U, // target_timeout_us：PC 目标的默认有效期。PC 按 100 Hz 发（10 ms 一帧），取 10 倍；
    // 太短的话 USB 抖一下就会"接管 → 放开 → 再接"来回闪（上限见仲裁里的 200 ms）
};

// 遥控：摇杆与拨杆的标定，2026-11-06 上板实测（负责人在遥控器上设好通道后，经 SWD 读 SBUS 原始帧）。
// 通道号从 0 起（遥控器上的第 n 条是 n − 1 号）：0 总开关（两档），1 左摇杆上下，2 左摇杆左右，
// 3 右摇杆上下，4 右摇杆左右，11 三档开关，12 弹性开关；5–10 号没有设。
// 四个摇杆方向都是 321 / 992 / 1663（两端 / 回中）；总开关关 321、开 1663；三档开关上 1663、中 992、下 321；
// 弹性开关松开 321、按住 1663。
// 约定摇杆向右、向上为正。右 y、左 x、左 y 往正方向推时原始值变小，所以设 inverted。
// 摇杆顺序：右 x、右 y、左 x、左 y（topics::STICK_RIGHT_X 等）。
inline constexpr remote::Remote::Parameters REMOTE{
    {
        remote::Remote::Stick{4U, 321U, 992U, 1663U, false, 0.05F}, // 右 x：往右 1663
        remote::Remote::Stick{3U, 321U, 992U, 1663U, true, 0.05F},  // 右 y：往上 321
        remote::Remote::Stick{2U, 321U, 992U, 1663U, true, 0.05F},  // 左 x：往右 321
        remote::Remote::Stick{1U, 321U, 992U, 1663U, true, 0.05F},  // 左 y：往上 321
    },
    {
        // 拨杆 0：总开关，两档（两个门限相同），开 = UP
        remote::Remote::Switch{0U, 992U, 992U, topics::SwitchPosition::DOWN, topics::SwitchPosition::UP},
        // 拨杆 1：三档开关，门限取相邻两档的中点
        remote::Remote::Switch{11U, 656U, 1328U, topics::SwitchPosition::DOWN, topics::SwitchPosition::UP},
        // 拨杆 2：弹性开关，两档；按住记作 DOWN、松开记作 UP
        remote::Remote::Switch{12U, 992U, 992U, topics::SwitchPosition::UP, topics::SwitchPosition::DOWN},
    },
    100000U, // frame_timeout_us：100 ms 没有正常帧就当链路断了
};
} // namespace params

#endif
