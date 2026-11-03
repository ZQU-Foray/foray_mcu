#include "imu/imu.hpp"

#include "hal/os/critical_section.hpp"
#include "utils/math/vector3.hpp"

#include <cmath>
#include <limits>

namespace imu
{

namespace
{

constexpr float ACCEL_OUTPUT_RATE_HZ{1600.0F}; // Bmi088Accel 固定配置为 1600 Hz

} // namespace

bool Imu::init(const Config &config) noexcept
{
    if (config.imu == nullptr || config.heater == nullptr ||
        !estimate::is_valid_axis_mapping(config.parameters.mounting) || config.parameters.publish_period_us == 0U ||
        !std::isfinite(config.parameters.heater_target_celsius))
    {
        return false;
    }
    estimate::GyroBiasCalibration::Config calibration = config.parameters.calibration;
    calibration.target_celsius = config.parameters.heater_target_celsius; // 标定在恒温目标上做
    if (!calibration_.init(calibration))
    {
        return false;
    }
    config_ = config;
    event_publisher_ = topics::imu::event.claim_publisher();
    attitude_publisher_ = topics::imu::attitude.claim_publisher();
    if (!event_publisher_ || !attitude_publisher_)
    {
        return false;
    }

    const float gyro_rate_hz =
        static_cast<float>(device::Bmi088Gyro::output_rate_hz(config.parameters.driver.gyro_bandwidth));
    estimate::Vqf::Config vqf_config{};
    vqf_config.rest_threshold_gyr_rad_per_sec = config.parameters.rest_threshold_gyr_rad_per_sec;
    vqf_config.bias_clip_rad_per_sec = config.parameters.bias_clip_rad_per_sec;
    if (!vqf_.init(vqf_config, 1.0F / gyro_rate_hz, 1.0F / ACCEL_OUTPUT_RATE_HZ))
    {
        return false;
    }

    config.imu->init(config.parameters.driver);
    config.imu->set_completion_notification(on_transfer_complete, this);
    // 中断最后打开：之前的状态都已就绪
    if (!config.accel_interrupt.set_interrupt(on_accel_ready, this) ||
        !config.gyro_interrupt.set_interrupt(on_gyro_ready, this))
    {
        return false;
    }
    if (!config.heater->init() || !config.heater->set_target_celsius(config.parameters.heater_target_celsius))
    {
        return false;
    }
    config.heater->set_enabled(true);

    report_ = Report{};
    report_.state = topics::imu::State::SETTLING;
    yaw_zero_ = math::Quaternion{1.0F, 0.0F, 0.0F, 0.0F};
    has_temperature_ = false;
    has_new_gyro_ = false;
    has_published_ = false;
    return true;
}

void Imu::step(hal::os::Microseconds now) noexcept
{
    device::Bmi088 &imu = *config_.imu;
    topics::imu::Event event{};
    if (event_.update(event))
    {
        ++report_.events;
        imu.notify_data_ready(device::Bmi088DataReady{event.accel_sequence, event.accel_us},
                              device::Bmi088DataReady{event.gyro_sequence, event.gyro_us});
    }

    imu.process();
    device::Bmi088Temperature::Sample temperature{};
    if (imu.get_temperature().try_get_sample(temperature))
    {
        temperature_celsius_ = device::Bmi088Temperature::to_celsius(temperature.raw);
        has_temperature_ = true;
        report_.temperature_celsius = temperature_celsius_;
    }
    device::Bmi088SampleRecord record{};
    bool gyroscope = false;
    while (imu.pop_next_sample(record, gyroscope))
    {
        handle_sample(record, gyroscope);
    }
    config_.heater->process(imu.get_temperature());
    report_.sensor_ready = imu.is_ready();
    report_.sensor_error = imu.get_state() == device::Bmi088::State::ERROR;

    // 零偏标定的结果
    const estimate::GyroBiasCalibration::Snapshot calibration = calibration_.snapshot();
    report_.calibration = calibration.state;
    report_.calibration_restarts = calibration.restarts;
    report_.calibration_sampling_us = calibration.sampling_us;
    if (report_.state == topics::imu::State::SETTLING)
    {
        if (calibration.state == estimate::GyroBiasCalibration::State::COMPLETED)
        {
            // 旧 AttitudeEstimator 只写零偏、保留 VQF 协方差，让静止零偏更新继续跟踪；
            // 传正的不确定度会重置协方差，实测会让静止更新变得很弱。
            if (config_.parameters.calibrated_bias_uncertainty_rad_per_sec > 0.0F)
            {
                vqf_.set_bias_estimate(calibration.bias_rad_per_sec,
                                       config_.parameters.calibrated_bias_uncertainty_rad_per_sec);
            }
            else
            {
                vqf_.set_bias_estimate(calibration.bias_rad_per_sec);
            }
            report_.calibrated_bias_rad_per_sec = calibration.bias_rad_per_sec;
            settle(topics::imu::State::READY, now);
        }
        else if (calibration.state == estimate::GyroBiasCalibration::State::TIMED_OUT)
        {
            settle(topics::imu::State::DEGRADED, now);
        }
    }

    // 限频按陀螺的采样时刻、允许提前 1/4 周期：任务醒来的时刻有抖动，按 now 判"满 1 ms"会时常差一点、
    // 多等一个样本，2 kHz 陀螺下平均只有约 860 Hz（2026-11-03 上板）。按采样时刻判正好每隔一个样本发一次
    const hal::os::Microseconds period = config_.parameters.publish_period_us;
    if (has_new_gyro_ &&
        (!has_published_ || hal::os::elapsed_us(last_publish_us_, latest_gyro_us_) >= period - period / 4U))
    {
        const estimate::Vqf::State estimate = vqf_.get_state();
        const estimate::Vqf::RestObservables rest = vqf_.get_rest_observables();
        report_.bias_rad_per_sec = estimate.gyro_bias_rad_per_sec;
        report_.rest_detected = estimate.rest_detected;
        report_.rest_gyro_ratio = rest.gyroscope_deviation_ratio;
        report_.rest_accel_ratio = rest.accelerometer_deviation_ratio;
        attitude_publisher_.publish(make_attitude(estimate));
        has_new_gyro_ = false;
        last_publish_us_ = latest_gyro_us_;
        has_published_ = true;
        ++report_.published;
    }
}

void Imu::handle_sample(const device::Bmi088SampleRecord &record, bool gyroscope) noexcept
{
    float sensor[3]{};
    float gimbal[3]{};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        sensor[axis] = gyroscope ? device::Bmi088Gyro::to_radians_per_second(record.xyz[axis])
                                 : device::Bmi088Accel::to_mps2(record.xyz[axis]);
    }
    estimate::map_sensor_vector_to_body(config_.parameters.mounting, sensor, gimbal);
    const math::Vector3 vector{gimbal[0], gimbal[1], gimbal[2]};
    if (gyroscope)
    {
        calibration_.process(vector,
                             latest_accel_,
                             has_temperature_ ? temperature_celsius_ : std::numeric_limits<float>::quiet_NaN(),
                             record.drdy_us);
        vqf_.update_gyroscope(vector);
        latest_gyro_ = vector;
        latest_gyro_us_ = record.drdy_us;
        ++report_.gyro_samples;
        has_new_gyro_ = true;
    }
    else
    {
        vqf_.update_accelerometer(vector);
        latest_accel_ = vector;
        ++report_.accel_samples;
    }
}

void Imu::settle(topics::imu::State state, hal::os::Microseconds now) noexcept
{
    // 世界系的 yaw 零点取这一刻的朝向：扣掉标定期间零偏造成的 yaw 漂移（云台静止时就是上电时的朝向）
    const float yaw = math::to_euler_zyx_radians(vqf_.get_quaternion()).x;
    yaw_zero_ = math::from_euler_zyx_radians(math::Vector3{-yaw, 0.0F, 0.0F});
    report_.yaw_zero_rad = yaw;
    report_.settled_us = now == 0U ? 1U : now;
    report_.state = state;
}

topics::imu::Attitude Imu::make_attitude(const estimate::Vqf::State &estimate) const noexcept
{
    math::Quaternion attitude = math::multiply(yaw_zero_, estimate.quaternion);
    const bool normalized = math::normalize(attitude);
    const math::Vector3 &bias = estimate.gyro_bias_rad_per_sec;

    topics::imu::Attitude message{};
    message.quaternion = {attitude.w, attitude.x, attitude.y, attitude.z};
    message.gyro_rad_per_sec = {latest_gyro_.x - bias.x, latest_gyro_.y - bias.y, latest_gyro_.z - bias.z};
    message.timestamp_us = latest_gyro_us_;
    message.state = report_.state;
    message.valid = normalized && estimate.valid && report_.accel_samples > 0U;
    return message;
}

// 框架中断

void Imu::on_accel_ready(void *context) noexcept
{
    static_cast<Imu *>(context)->publish_event(true, false, false);
}

void Imu::on_gyro_ready(void *context) noexcept
{
    static_cast<Imu *>(context)->publish_event(false, true, false);
}

void Imu::on_transfer_complete(void *context) noexcept
{
    static_cast<Imu *>(context)->publish_event(false, false, true);
}

void Imu::publish_event(bool accel, bool gyro, bool transfer) noexcept
{
    topics::imu::Event snapshot{};
    {
        // 三个回调可能在不同优先级的框架中断里：更新与取快照一起做，发布放在临界区外（发布会叫醒等待者）
        const hal::os::CriticalSection guard;
        const hal::os::Microseconds now = hal::os::now_us();
        if (accel)
        {
            ++interrupt_state_.accel_sequence;
            interrupt_state_.accel_us = now;
        }
        if (gyro)
        {
            ++interrupt_state_.gyro_sequence;
            interrupt_state_.gyro_us = now;
        }
        if (transfer)
        {
            ++interrupt_state_.transfer_count;
        }
        interrupt_state_.published_us = now;
        snapshot = interrupt_state_;
    }
    event_publisher_.publish(snapshot);
}

} // namespace imu
