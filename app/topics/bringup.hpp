#ifndef TOPICS_BRINGUP_HPP
#define TOPICS_BRINGUP_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"

#include <array>
#include <cstdint>

// bringup 能力发布的 topic：框架验证用，不含 RM 业务。

namespace topics::bringup
{

/** 心跳：bringup::Heartbeat 每轮发布一次。 */
struct Heartbeat
{
    std::uint32_t sequence;             // 从 1 开始，每次发布加 1
    hal::os::Microseconds timestamp_us; // 发布时刻（发布任务本轮开始的时刻）
};

inline topic::Topic<Heartbeat> heartbeat{"bringup.heartbeat"};

/**
 * IMU 事件：bringup::ImuCheck 在中断里发布——BMI088 两个数据就绪中断与 SPI 事务完成中断（F2 验收的"中断里发布"）。
 * 按它触发的任务读出累计次数与时刻，交给 BMI088 驱动安排读取。
 */
struct ImuEvent
{
    std::uint32_t accel_sequence;       // 加速度计数据就绪中断的累计次数
    hal::os::Microseconds accel_us;     // 最近一次加速度计数据就绪的时刻
    std::uint32_t gyro_sequence;        // 陀螺仪数据就绪中断的累计次数
    hal::os::Microseconds gyro_us;      // 最近一次陀螺仪数据就绪的时刻
    std::uint32_t transfer_count;       // SPI 事务完成的累计次数
    hal::os::Microseconds published_us; // 发布这一份的时刻（与上面的时刻是同一次读取）
};

inline topic::Topic<ImuEvent> imu_event{"bringup.imu_event"};

/** IMU 读数与姿态（机体坐标系）：bringup::ImuCheck 在任务里发布。 */
struct Imu
{
    std::array<float, 3> gyro_rad_per_sec;
    std::array<float, 3> accel_m_per_s2;
    std::array<float, 4> quaternion; // w、x、y、z，机体 → 地面（VQF，重力沿地面 +z）
    float temperature_celsius;
    std::uint32_t gyro_sample_count;    // 累计处理的陀螺样本数
    hal::os::Microseconds timestamp_us; // 最新陀螺样本的数据就绪时刻
};

inline topic::Topic<Imu> imu{"bringup.imu"};

/** 遥控接收机的 SBUS 帧：bringup::SbusCheck 发布。 */
struct Sbus
{
    std::array<std::uint16_t, 16> channels; // 原始值，约 172–1811
    std::uint32_t frame_count;              // 累计收到的有效帧
    hal::os::Microseconds timestamp_us;     // 收到这一帧的时刻
    bool frame_lost;                        // 接收机报告丢帧
    bool failsafe;                          // 接收机进入失控保护
};

inline topic::Topic<Sbus> sbus{"bringup.sbus"};

} // namespace topics::bringup

#endif // TOPICS_BRINGUP_HPP
