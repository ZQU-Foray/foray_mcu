#ifndef IMU_IMU_HPP
#define IMU_IMU_HPP

#include "driver/bmi088/bmi088.hpp"
#include "driver/bmi088/bmi088_heater.hpp"
#include "hal/gpio.hpp"
#include "hal/os/clock.hpp"
#include "lib/estimate/gyro_bias_calibration.hpp"
#include "lib/estimate/imu_body_mapping.hpp"
#include "lib/estimate/vqf.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/imu.hpp"
#include "utils/math/quaternion.hpp"

#include <cstdint>

namespace imu
{

/**
 * @brief IMU 能力（Decision-20261103-01 第 9 点）：板载 BMI088 → 云台姿态 topic imu.attitude。
 *
 * - 中断（框架中断）：两个数据就绪中断与 SPI 事务完成中断各自更新累计次数与时刻，发布 imu.event；
 * - 任务（按 imu.event 触发，带约 1 ms 的超时让驱动初始化按时间推进）：
 *   1. 把事件交给驱动，收割样本，换到云台坐标系（安装方向在 profile）；
 *   2. VQF 解算姿态，加热器恒温；
 *   3. 零偏标定：恒温到位并稳定后取静止窗口（estimate::GyroBiasCalibration）。完成时把零偏交给 VQF，
 *      并把世界系的 yaw 零点重新取为当时的朝向（进入 READY）；超时进入 DEGRADED，同样重取 yaw 零点；
 *   4. 限频发布 imu.attitude：四元数（云台系 → 世界系）、扣除零偏的角速度、采样时刻、状态。
 * 由 bringup::ImuCheck 改写。能力零 RTOS；驱动对象与引脚句柄由组合根注入。
 */
class Imu final
{
  public:
    /** 来自 profile 的参数（params.hpp）。 */
    struct Parameters
    {
        device::Bmi088::Config driver;           // 陀螺输出速率与带宽
        estimate::AxisMapping mounting;          // 传感器 → 云台系（x 沿枪管向前、y 左、z 上）
        hal::os::Microseconds publish_period_us; // imu.attitude 的发布间隔（按陀螺采样时刻，允许提前 1/4）
        float heater_target_celsius;             // 加热目标，也是零偏标定的温度门控目标
        estimate::GyroBiasCalibration::Config calibration; // target_celsius 由 heater_target_celsius 覆盖
        // > 0：把标定不确定度写进 VQF 零偏协方差；0：保留 VQF 现有协方差（旧 AttitudeEstimator 的行为）
        float calibrated_bias_uncertainty_rad_per_sec;
        // VQF 静止零偏更新（2026-11-03 本板实测）：静止三轴偏差 norm RMS 6.2 °/s、最大 19.3 °/s；
        // 默认 2 °/s 会让 rest 永不触发。0.35 rad/s = 20.1 °/s 覆盖实测最大值。
        float rest_threshold_gyr_rad_per_sec{0.35F};
        float bias_clip_rad_per_sec{0.05F};
    };

    /** 组合根注入的依赖与参数。 */
    struct Config
    {
        device::Bmi088 *imu;
        device::Bmi088Heater *heater; // 零偏标定要恒温，不能为空
        hal::Gpio accel_interrupt;    // 加速度计数据就绪（外部中断）
        hal::Gpio gyro_interrupt;     // 陀螺仪数据就绪（外部中断）
        Parameters parameters;
    };

    /** 统计与诊断，调试器可读（只有运行 step() 的任务写）。 */
    struct Report
    {
        std::uint32_t events;        // 读到的 imu.event 次数
        std::uint32_t gyro_samples;  // 处理的陀螺样本
        std::uint32_t accel_samples; // 处理的加速度计样本
        std::uint32_t published;     // 发布 imu.attitude 的次数
        float temperature_celsius;
        topics::imu::State state;
        estimate::GyroBiasCalibration::State calibration;
        std::uint32_t calibration_restarts;            // 标定窗口因运动、温度重启的次数
        hal::os::Microseconds calibration_sampling_us; // 累计处于采样状态的时间
        math::Vector3 calibrated_bias_rad_per_sec;     // 标定结果（DEGRADED 时为 0）
        math::Vector3 bias_rad_per_sec;   // VQF 当前的零偏估计（发布的角速度扣的就是它）
        bool rest_detected;               // VQF 当前是否判为静止（静止零偏更新是否在跑）
        float rest_gyro_ratio;            // 陀螺侧偏离 / 阈值，1.0 为边界
        float rest_accel_ratio;           // 加计侧偏离 / 阈值，1.0 为边界
        float yaw_zero_rad;               // 进入 READY/DEGRADED 时扣掉的 yaw
        hal::os::Microseconds settled_us; // 进入 READY/DEGRADED 的时刻（0：还在 SETTLING）
        bool sensor_ready;                // BMI088 两个传感器都已配置好
        bool sensor_error;                // BMI088 进入错误状态
    };

    [[nodiscard]] bool init(const Config &config) noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    // 框架中断里调用
    static void on_accel_ready(void *context) noexcept;
    static void on_gyro_ready(void *context) noexcept;
    static void on_transfer_complete(void *context) noexcept;
    void publish_event(bool accel, bool gyro, bool transfer) noexcept;

    void handle_sample(const device::Bmi088SampleRecord &record, bool gyroscope) noexcept;
    void settle(topics::imu::State state, hal::os::Microseconds now) noexcept;
    [[nodiscard]] topics::imu::Attitude make_attitude(const estimate::Vqf::State &estimate) const noexcept;

    Config config_{nullptr, nullptr, hal::Gpio{0U}, hal::Gpio{0U}, Parameters{}};
    topic::Publisher<topics::imu::Event> event_publisher_{};
    topic::Subscriber<topics::imu::Event> event_{topics::imu::event};
    topic::Publisher<topics::imu::Attitude> attitude_publisher_{};
    estimate::Vqf vqf_{};
    estimate::GyroBiasCalibration calibration_{};

    // 只在中断里读写：三个回调都在框架中断里，更新与取快照在临界区内
    topics::imu::Event interrupt_state_{};

    math::Vector3 latest_accel_{};
    math::Vector3 latest_gyro_{};
    hal::os::Microseconds latest_gyro_us_{0U};
    float temperature_celsius_{0.0F};
    bool has_temperature_{false};
    math::Quaternion yaw_zero_{1.0F, 0.0F, 0.0F, 0.0F}; // 世界系相对 VQF 地球系的 yaw 旋转
    bool has_new_gyro_{false};
    hal::os::Microseconds last_publish_us_{0U}; // 上次发布的那个陀螺样本的采样时刻
    bool has_published_{false};
    Report report_{};
};

} // namespace imu

#endif // IMU_IMU_HPP
