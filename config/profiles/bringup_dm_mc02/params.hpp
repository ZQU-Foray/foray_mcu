#ifndef PARAMS_HPP
#define PARAMS_HPP

// bringup_dm_mc02 的参数（只含数据，规则 4）。

#include "bringup/dm_check.hpp"
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

// BMI088 加热：DM-MC02 的热模型，2026-11-01 接 24 V 实测辨识——
// 每 1‰ 占空比稳态高出环境 0.967 ℃；被加热区时间常数 15.5 s；传感器滞后 0.95 s。控制参数用默认值。
inline constexpr device::Bmi088Heater::Config HEATER{control::HeaterControl::Model{0.967F, 15.5F, 0.95F}};

// 上电打开板上两路 24 V 输出：接在上面的电机（GM6020、DM-J4310）随之上电
inline constexpr bringup::PowerOutputs::Parameters POWER_OUTPUTS{true};

// USB 状态行 10 Hz
inline constexpr bringup::UsbReport::Parameters USB_REPORT{100000U};

// 指示：200 ms 没有新数据算停；灯亮度 32/255；上电响两声
inline constexpr bringup::Indicator::Parameters INDICATOR{200000U, 32U, true};

// 达妙电机检查：CAN1 上 ESC_ID 1、MST_ID 0x11 的 DM-J4310，上电后跑一遍。
// 转动：力矩 1.0 N·m、Kd 1.0，稳态不超过 1 rad/s，2 s 最多 2 rad。2026-11-02 上板用 0.3 N·m 时，
// 减速箱的静摩擦（约 0.1–0.3 N·m）让电机时转时不转，所以加大力矩，同时按比例加大 Kd，限速不变。
// 电机里的通讯超时是 50 ms，停发 300 ms。
// 时限：核对 5 s（电机上电自检约 1 s）；失能保活 0.5 s；使能 0.2 s；零力矩 1 s；转动 2 s；刹车 0.5 s；恢复 3 s。
// 之后是 1 kHz 控制环基准：使能 10 s，PD 保持位置（Kp 2 N·m/rad、Kd 0.05，限 0.5 N·m）。
// 装到整车上之前把 spin 与 benchmark 改为 false，或者去掉这个检查
inline constexpr bringup::DmCheck::Parameters DM_CHECK{
    1U,
    0x11U,
    true,
    1.0F,
    1.0F,
    5000000U,
    500000U,
    200000U,
    1000000U,
    2000000U,
    500000U,
    300000U,
    3000000U,
    true,
    10000000U,
    2.0F,
    0.05F,
    0.5F,
};

} // namespace params

#endif // PARAMS_HPP
