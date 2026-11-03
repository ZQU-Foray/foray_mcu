#ifndef TOPICS_IMU_HPP
#define TOPICS_IMU_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"

#include <array>
#include <cstdint>

// IMU 能力 imu::Imu 发布的 topic。

namespace topics::imu
{

/**
 * IMU 事件：imu::Imu 在框架中断里发布——BMI088 两个数据就绪中断与 SPI 事务完成中断。
 * IMU 任务按它触发，读出累计次数与时刻，交给 BMI088 驱动安排读取。
 */
struct Event
{
    std::uint32_t accel_sequence;       // 加速度计数据就绪中断的累计次数
    hal::os::Microseconds accel_us;     // 最近一次加速度计数据就绪的时刻
    std::uint32_t gyro_sequence;        // 陀螺仪数据就绪中断的累计次数
    hal::os::Microseconds gyro_us;      // 最近一次陀螺仪数据就绪的时刻
    std::uint32_t transfer_count;       // SPI 事务完成的累计次数
    hal::os::Microseconds published_us; // 发布这一份的时刻
};

inline topic::Topic<Event> event{"imu.event"};

/** 零偏标定的状态，取值与 RoboLink 的 imu_state 相同。 */
enum class State : std::uint8_t
{
    SETTLING = 0U, // 恒温或零偏标定中：roll、pitch 可用，yaw 零偏未定
    READY = 1U,    // 零偏已标定，yaw 零点已定
    DEGRADED = 2U  // 标定超时（标定期间一直在动），按未标定运行
};

/**
 * 云台姿态。坐标系：
 * - 云台系：x 沿枪管向前、y 向左、z 向上（安装方向在 profile 里）；
 * - 世界系：z 竖直向上；yaw 零点在 READY（或 DEGRADED）之前是上电时的朝向，进入时重新取当时的朝向。
 * yaw 左转为正、pitch 低头为正（四元数的 ZYX 分解）。
 */
struct Attitude
{
    std::array<float, 4> quaternion;       // w、x、y、z：云台系 → 世界系
    std::array<float, 3> gyro_rad_per_sec; // 云台系角速度，已扣除零偏
    hal::os::Microseconds timestamp_us;    // 最新陀螺样本的数据就绪时刻
    State state;
    bool valid; // 姿态解算有效：已收到陀螺与加速度计样本
};

inline topic::Topic<Attitude> attitude{"imu.attitude"};

} // namespace topics::imu

#endif // TOPICS_IMU_HPP
