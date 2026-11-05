#ifndef PARAMS_HPP
#define PARAMS_HPP

#include "actuator/power_outputs.hpp"
#include "actuator/safety_gate.hpp"
#include "gimbal/gimbal.hpp"
#include "imu/imu.hpp"
#include "pc_link/pc_link.hpp"
#include "remote/remote.hpp"

#include <cstdint>

namespace params
{
// BMI088 加热：DM-MC02 的热模型（与 bringup 相同）。加热要接 24 V（XT30）
inline constexpr device::Bmi088Heater::Config HEATER{control::HeaterControl::Model{0.967F, 15.5F, 0.95F}};

// IMU：
// - 陀螺 2000 Hz / 532 Hz；imu.attitude 最多 1 kHz；加热到 50 ℃，零偏标定在 50 ℃ 上做；
// - 安装方向：板载 BMI088 的坐标系即云台系（x 沿枪管 / 相机光轴向前、y 向左、z 向上），负责人 2026-11-03 确认；
//   以后装法不同时改这里（estimate::AxisMapping：云台系每个轴取传感器的哪个轴、什么符号）；
// - 零偏标定：恒温稳定 20 s、静止窗口 5 s、累计 20 s 没有合格窗口就放弃（旧板实测参数）；
// - 标定结果交给 VQF 时**不重置协方差**（0）：保留 VQF 的静止零偏更新，让它继续跟踪残余/温度漂移；
// - VQF 静止门限按 2026-11-03 本板实测：60 s 静止三轴偏差 norm RMS 6.2 °/s、最大 19.3 °/s。
//   默认 2 °/s 时有 95.4% 的样本超门限，rest 永不触发；0.35 rad/s（20.1 °/s）覆盖实测最大值，
//   bias_clip 0.05 rad/s（2.9 °/s）大于本板各轴零偏。
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

// 云台电机：CAN1 上两台 DM-J4310，设置见指引 6.1。机构：输出轴直连云台（传动比 1），
// 方向让关节与 IMU 同号（yaw 左转为正、pitch 低头为正），零点在云台水平朝前。
inline constexpr std::uint16_t YAW_ESC_ID{1U};
inline constexpr std::uint16_t YAW_MASTER_ID{0x11U};
inline constexpr device::JointMechanics YAW_MECHANICS{1, 1.0F, 0.0F, 3000U};
inline constexpr std::uint16_t PITCH_ESC_ID{2U};
inline constexpr std::uint16_t PITCH_MASTER_ID{0x12U};
inline constexpr device::JointMechanics PITCH_MECHANICS{1, 1.0F, 0.0F, 3000U};

// 云台（角速度环 → 角度环，力矩模式）。第 6 步的起步值：先 pitch、先角速度环，边调边改这里。
// - yaw 先不出力：两环增益都给 0，用手扶着；调好 pitch 以后再照同样的顺序调 yaw；
// - pitch 角速度环 kp 从 0.05 起，每步翻倍到出现嗡嗡声或抖动再退一半；ki 从 kp 的 5–10 倍起，消掉重力下垂；
// - pitch 角度环 kp 从 5 起往上加，到出现超调再退一点；角速度上限先给 6 rad/s；
// - 总限幅第一次出力只给 1 N·m（4310 峰值 12.5 N·m，夹到手指会很疼），调好后再放宽；
// - 重力前馈先给 0：水平静止时读 Report 里 pitch 的 compensation_newton_meter，那个稳态值就是 τ_g。
inline constexpr gimbal::Gimbal::Parameters GIMBAL{
    // yaw：小 yaw 按有限位处理，偏移 = IMU 的 yaw − 小 yaw 关节角。范围是占位值，按线缆实测收紧
    gimbal::Gimbal::AxisParameters{
        gimbal::Gimbal::LoopGains{0.0F, 0.0F, 0.0F, 6.0F},
        gimbal::Gimbal::LoopGains{0.0F, 0.0F, 0.0F, 1.0F},
        true,
        -3.14159265F,
        3.14159265F,
    },
    // pitch：机械限位按关节角给（水平为 0、低头为正）。范围是占位值，上板实测后改
    gimbal::Gimbal::AxisParameters{
        gimbal::Gimbal::LoopGains{5.0F, 0.0F, 0.0F, 6.0F},
        gimbal::Gimbal::LoopGains{0.05F, 0.3F, 1.0F, 1.0F},
        true,
        -0.6F,
        1.0F,
    },
    0.0F,    // pitch_gravity_newton_meter
    1.0F,    // torque_limit_newton_meter
    10000U,  // attitude_timeout_us：10 ms 没有新姿态就停
    200000U, // enable_timeout_us：200 ms 没确认就回 DISABLED
    100000U, // target_timeout_us：PC 目标默认 100 ms 有效
};

// 安全门：遥控每 5 ms 发布一次 safety.stop，50 ms 收不到就当发布者挂了
inline constexpr actuator::SafetyGate::Parameters SAFETY{50000U};

// 遥控最小版：拨杆通道与运行档的原始值。上板先用 bringup 固件读 bringup.sbus 的通道、拨一拨记下三档的值
inline constexpr remote::Remote::Parameters REMOTE{4U, 1500U, 1900U, 100000U};
} // namespace params

#endif
