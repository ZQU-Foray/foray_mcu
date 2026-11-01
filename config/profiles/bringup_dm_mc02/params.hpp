#ifndef PARAMS_HPP
#define PARAMS_HPP

// bringup_dm_mc02 的参数（只含数据，规则 4）。

#include "bringup/imu_check.hpp"
#include "bringup/indicator.hpp"
#include "bringup/monitor.hpp"
#include "bringup/power_outputs.hpp"
#include "bringup/usb_report.hpp"

namespace params
{

// 心跳从发布到被监视任务读到的延迟上限：监视任务的优先级更高，正常应在几十 µs 内
inline constexpr bringup::Monitor::Config MONITOR{100U};

// IMU：陀螺 2000 Hz / 532 Hz；板载 BMI088 的坐标系与 DM-MC02 板面一致（安装方向按整车再定）；
// bringup.imu 最多 1 kHz；中断发布到 IMU 任务读到的延迟上限 200 µs；加热到 50 ℃（与原固件相同）
inline constexpr bringup::ImuCheck::Parameters IMU{
    device::Bmi088::Config{device::Bmi088Gyro::Bandwidth::ODR_2000_HZ_532_HZ},
    estimate::IDENTITY_AXIS_MAPPING,
    1000U,
    200U,
    50.0F,
};

// BMI088 加热：DM-MC02 的热模型，2026-11-01 接 24 V 实测辨识（Decision-20261101-05）——
// 每 1‰ 占空比稳态高出环境 0.967 ℃；被加热区时间常数 15.5 s；传感器滞后 0.95 s。控制参数用默认值。
inline constexpr device::Bmi088Heater::Config HEATER{control::HeaterControl::Model{0.967F, 15.5F, 0.95F}};

// 上电打开板上两路 24 V 输出：接在上面的电机（GM6020、DM-J4310）随之上电（Decision-20261101-06）
inline constexpr bringup::PowerOutputs::Parameters POWER_OUTPUTS{true};

// USB 状态行 10 Hz
inline constexpr bringup::UsbReport::Parameters USB_REPORT{100000U};

// 指示：200 ms 没有新数据算停；灯亮度 32/255；上电响两声
inline constexpr bringup::Indicator::Parameters INDICATOR{200000U, 32U, true};

} // namespace params

#endif // PARAMS_HPP
