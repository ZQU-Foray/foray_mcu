#ifndef PARAMS_HPP
#define PARAMS_HPP

// link_dm_mc02 的参数（只含数据，规则 4）。

#include "imu/imu.hpp"
#include "pc_link/pc_link.hpp"

namespace params
{

// BMI088 加热：DM-MC02 的热模型（Decision-20261101-05，与 bringup 相同）。加热要接 24 V（XT30）
inline constexpr device::Bmi088Heater::Config HEATER{control::HeaterControl::Model{0.967F, 15.5F, 0.95F}};

// IMU（Decision-20261103-01 第 9 点）：
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

} // namespace params

#endif // PARAMS_HPP
