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
// 快窗口出结果时温度寄存器可能还没更新（1.28 s 才更新一次）：等这么久还没有温度就丢掉这条记录
constexpr hal::os::Microseconds PENDING_RECORD_TIMEOUT_US{3000000U};

// estimate::BiasStore 的存储后端接到 hal::Nvs 上（记录层只管算法，不依赖 hal）
std::size_t nvs_size(void *context) noexcept
{
    return static_cast<const hal::Nvs *>(context)->size();
}

std::size_t nvs_granularity(void *context) noexcept
{
    return static_cast<const hal::Nvs *>(context)->write_granularity();
}

bool nvs_read(void *context, std::size_t offset, std::uint8_t *data, std::size_t length) noexcept
{
    return static_cast<const hal::Nvs *>(context)->read(offset, data, length) == hal::Nvs::Result::OK;
}

bool nvs_write(void *context, std::size_t offset, const std::uint8_t *data, std::size_t length) noexcept
{
    return static_cast<const hal::Nvs *>(context)->write(offset, data, length) == hal::Nvs::Result::OK;
}

bool nvs_erase(void *context) noexcept
{
    return static_cast<const hal::Nvs *>(context)->erase() == hal::Nvs::Result::OK;
}

} // namespace

bool Imu::init(const Config &config) noexcept
{
    if (config.imu == nullptr || config.heater == nullptr ||
        !estimate::is_valid_axis_mapping(config.parameters.mounting) || config.parameters.publish_period_us == 0U ||
        !std::isfinite(config.parameters.heater_target_celsius) ||
        !std::isfinite(config.parameters.bias_update_temperature_celsius) ||
        config.parameters.bias_update_temperature_celsius < 0.0F)
    {
        return false;
    }
    config_ = config;
    estimate::GyroBiasCalibration::Config calibration = config.parameters.calibration;
    calibration.target_celsius = config.parameters.heater_target_celsius; // 标定在恒温目标上做
    estimate::GyroBiasCalibration::Config quick = config.parameters.quick_calibration;
    quick.target_celsius = config.parameters.heater_target_celsius; // 快速零偏不看温度，目标只保持一致
    if (!calibration_.init(calibration) || !quick_calibration_.init(quick))
    {
        return false;
    }
    // 零偏-温度记录：区满或中间有坏记录时在这里压缩（整区擦除），必须在只有启动代码跑的时候
    const estimate::BiasStore::Storage storage{&config_.nvs, nvs_size, nvs_granularity, nvs_read, nvs_write, nvs_erase};
    if (!store_.init(storage, config.parameters.bias_store))
    {
        return false;
    }
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
    quick_applied_ = false;
    precise_applied_ = false;
    pending_record_ = PendingRecord{};
    report_.store_available = store_.available();
    report_.store_has_bias = store_.has_bias();
    report_.store_records = store_.snapshot().records;
    report_.store_fit_is_line = store_.snapshot().fit_is_line;
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

    // 1) Flash 里有零偏-温度记录：不等静止窗口，第一个样本后就能 READY（复位时云台可能在动）
    if (report_.state == topics::imu::State::SETTLING && report_.accel_samples > 0U && store_.has_bias())
    {
        apply_bias_at_temperature(has_temperature_ ? temperature_celsius_ : store_.newest_temperature_celsius());
        report_.bias_source = BiasSource::STORED;
        settle(topics::imu::State::READY, now);
    }

    // 2) 温度变了就按 b(T) 重算零偏：升温、降温的几十秒里零偏能差 0.1–0.2 °/s，等 VQF 的静止更新追太慢。
    //    只在变化超过阈值时动，稳态下温度不变就不会去碰 VQF；姿态不跳，只改积分用的角速度
    if (config_.parameters.bias_update_temperature_celsius > 0.0F && store_.has_bias() && has_temperature_ &&
        std::fabs(temperature_celsius_ - report_.bias_temperature_celsius) >=
            config_.parameters.bias_update_temperature_celsius)
    {
        apply_bias_at_temperature(temperature_celsius_);
        ++report_.bias_updates;
    }

    // 3) 上电快窗口：0.5 s 静止窗口，不等温度。它决定什么时候进 READY（云台从这一刻起才出力）
    const estimate::GyroBiasCalibration::Snapshot quick = quick_calibration_.snapshot();
    report_.quick_calibration = quick.state;
    if (!quick_applied_ && report_.accel_samples > 0U)
    {
        if (quick.state == estimate::GyroBiasCalibration::State::COMPLETED)
        {
            apply_bias(quick.bias_rad_per_sec);
            report_.calibrated_bias_rad_per_sec = quick.bias_rad_per_sec;
            report_.bias_temperature_celsius = has_temperature_ ? temperature_celsius_ : 0.0F;
            store_result(has_temperature_ ? temperature_celsius_ : 0.0F,
                         quick.bias_rad_per_sec,
                         config_.parameters.quick_calibration.window_us);
            quick_applied_ = true;
            if (report_.state == topics::imu::State::SETTLING)
            {
                report_.bias_source = BiasSource::QUICK;
                settle(topics::imu::State::READY, now);
            }
        }
        else if (quick.state == estimate::GyroBiasCalibration::State::TIMED_OUT)
        {
            // 上电时在动：按零偏 0 出力，VQF 的静止更新之后会自己收敛
            quick_applied_ = true;
            if (report_.state == topics::imu::State::SETTLING)
            {
                settle(topics::imu::State::DEGRADED, now);
            }
        }
    }

    // 4) 后台精确标定：只改进 VQF 的零偏，不改状态、不重取 yaw 零点（正在保持的云台和 PC 的目标不会跳）
    const estimate::GyroBiasCalibration::Snapshot calibration = calibration_.snapshot();
    report_.calibration = calibration.state;
    report_.calibration_restarts = calibration.restarts;
    report_.calibration_sampling_us = calibration.sampling_us;
    if (calibration.state == estimate::GyroBiasCalibration::State::COMPLETED && !precise_applied_)
    {
        apply_bias(calibration.bias_rad_per_sec);
        report_.calibrated_bias_rad_per_sec = calibration.bias_rad_per_sec;
        report_.bias_temperature_celsius = has_temperature_ ? temperature_celsius_ : 0.0F;
        precise_applied_ = true;
        report_.precise_applied = true;
        store_result(has_temperature_ ? temperature_celsius_ : 0.0F,
                     calibration.bias_rad_per_sec,
                     config_.parameters.calibration.window_us);
        if (report_.state == topics::imu::State::SETTLING)
        {
            // 精确标定比快速窗口还先出结果（参数极端时）：它来定 READY
            quick_applied_ = true;
            report_.bias_source = BiasSource::PRECISE;
            settle(topics::imu::State::READY, now);
        }
    }

    // 5) 待写的记录：温度到了就写，太久没有温度就丢掉
    if (pending_record_.active)
    {
        if (has_temperature_)
        {
            (void)store_.record(temperature_celsius_, pending_record_.bias_rad_per_sec, pending_record_.window_us);
            pending_record_.active = false;
        }
        else if (hal::os::has_elapsed(pending_record_.queued_us, PENDING_RECORD_TIMEOUT_US, now))
        {
            pending_record_.active = false;
        }
    }
    report_.store_written = store_.snapshot().written;
    report_.store_records = store_.snapshot().records;
    report_.store_fit_is_line = store_.snapshot().fit_is_line;

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
        const float temperature = has_temperature_ ? temperature_celsius_ : std::numeric_limits<float>::quiet_NaN();
        calibration_.process(vector, latest_accel_, temperature, record.drdy_us);
        quick_calibration_.process(vector, latest_accel_, temperature, record.drdy_us);
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

void Imu::apply_bias(const math::Vector3 &bias_rad_per_sec) noexcept
{
    // 旧 AttitudeEstimator 只写零偏、保留 VQF 协方差，让静止零偏更新继续跟踪；
    // 传正的不确定度会重置协方差，实测会让静止更新变得很弱。
    if (config_.parameters.calibrated_bias_uncertainty_rad_per_sec > 0.0F)
    {
        vqf_.set_bias_estimate(bias_rad_per_sec, config_.parameters.calibrated_bias_uncertainty_rad_per_sec);
    }
    else
    {
        vqf_.set_bias_estimate(bias_rad_per_sec);
    }
}

void Imu::apply_bias_at_temperature(float temperature_celsius) noexcept
{
    const math::Vector3 bias = store_.bias_at(temperature_celsius);
    apply_bias(bias);
    report_.calibrated_bias_rad_per_sec = bias;
    report_.bias_temperature_celsius = temperature_celsius;
}

void Imu::store_result(float temperature_celsius,
                       const math::Vector3 &bias_rad_per_sec,
                       std::uint32_t window_us) noexcept
{
    if (!store_.available())
    {
        return;
    }
    if (has_temperature_ && std::isfinite(temperature_celsius))
    {
        (void)store_.record(temperature_celsius, bias_rad_per_sec, window_us);
        return;
    }
    pending_record_.active = true;
    pending_record_.bias_rad_per_sec = bias_rad_per_sec;
    pending_record_.window_us = window_us;
    pending_record_.queued_us = hal::os::now_us();
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
