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
 * @brief 上电快速零偏的默认参数：不等恒温，0.5 s 静止窗口，1 s 拿不到就放弃。
 *
 * 精确标定（Parameters::calibration）要等温度稳定，冷启动会晚几十秒；云台在 READY 之前不出力，
 * 所以上电先用这个短窗口：停在当前温度上的 0.5 s 均值误差约 0.1 °/s 量级，够云台先动起来，
 * 精确结果随后在后台补上。
 */
constexpr estimate::GyroBiasCalibration::Config power_on_calibration() noexcept
{
    estimate::GyroBiasCalibration::Config config{};
    config.temperature_gated = false;
    config.temperature_settle_us = 0U;
    config.window_us = 500000U;
    config.timeout_us = 1000000U;
    return config;
}

/**
 * @brief IMU 能力：板载 BMI088 → 云台姿态 topic imu.attitude。
 *
 * - 中断（框架中断）：两个数据就绪中断与 SPI 事务完成中断各自更新累计次数与时刻，发布 imu.event；
 * - 任务（按 imu.event 触发，带约 1 ms 的超时让驱动初始化按时间推进）：
 *   1. 把事件交给驱动，收割样本，换到云台坐标系（安装方向在 profile）；
 *   2. VQF 解算姿态，加热器恒温；
 *   3. 上电零偏（power_on_calibration）：不等温度，0.5 s 静止窗口，拿到就把零偏交给 VQF 并进入 READY；
 *      1 s 还拿不到（上电时在动）就按未标定进入 DEGRADED。两种都在这一刻取世界系 yaw 零点，之后不再重取；
 *   4. 后台精确标定（calibration）：恒温到位并稳定后取 5 s 静止窗口，完成时只把零偏交给 VQF（改进之后的
 *      角速度与积分），不改状态、不重取 yaw 零点——正在保持的云台和 PC 的世界系目标不会跳。加热器坏或没接
 *      24 V 时它停在等温度，不影响 READY；
 *   5. 限频发布 imu.attitude：四元数（云台系 → 世界系）、扣除零偏的角速度、采样时刻、状态。
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
        float heater_target_celsius;             // 加热目标，也是精确标定的温度门控目标
        // 后台精确标定；target_celsius 由 heater_target_celsius 覆盖
        estimate::GyroBiasCalibration::Config calibration;
        // 上电快速零偏（不等温度）；默认见 power_on_calibration()
        estimate::GyroBiasCalibration::Config quick_calibration{power_on_calibration()};
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
        device::Bmi088Heater *heater; // 精确标定与恒温要用；快窗口不依赖它，但恒温是长期精度的前提
        hal::Gpio accel_interrupt; // 加速度计数据就绪（外部中断）
        hal::Gpio gyro_interrupt;  // 陀螺仪数据就绪（外部中断）
        Parameters parameters;
    };

    /** 上电零偏的来源（READY 之后不再变化）。 */
    enum class BiasSource : std::uint8_t
    {
        NONE = 0U,   // 上电快窗口没拿到（DEGRADED），零偏从 0 起
        QUICK = 1U,  // 上电快窗口给出的零偏（READY）
        PRECISE = 2U // 精确标定比快窗口先出结果（参数极端时），由它定 READY
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
        estimate::GyroBiasCalibration::State calibration;       // 后台精确标定
        estimate::GyroBiasCalibration::State quick_calibration; // 上电快速零偏
        BiasSource bias_source;
        bool precise_applied;                          // 后台精确标定已完成并把零偏交给了 VQF
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
    void apply_bias(const math::Vector3 &bias_rad_per_sec) noexcept;
    void settle(topics::imu::State state, hal::os::Microseconds now) noexcept;
    [[nodiscard]] topics::imu::Attitude make_attitude(const estimate::Vqf::State &estimate) const noexcept;

    Config config_{nullptr, nullptr, hal::Gpio{0U}, hal::Gpio{0U}, Parameters{}};
    topic::Publisher<topics::imu::Event> event_publisher_{};
    topic::Subscriber<topics::imu::Event> event_{topics::imu::event};
    topic::Publisher<topics::imu::Attitude> attitude_publisher_{};
    estimate::Vqf vqf_{};
    estimate::GyroBiasCalibration calibration_{};
    estimate::GyroBiasCalibration quick_calibration_{};
    bool quick_applied_{false};   // 上电零偏已经定过（READY 或 DEGRADED）
    bool precise_applied_{false}; // 后台精确标定的结果已交给 VQF

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
