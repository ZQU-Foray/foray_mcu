#include "bringup/imu_check.hpp"

#include "hal/os/critical_section.hpp"
#include "utils/math/vector3.hpp"

namespace bringup
{

namespace
{

constexpr float ACCEL_OUTPUT_RATE_HZ{1600.0F}; // Bmi088Accel 固定配置为 1600 Hz

} // namespace

bool ImuCheck::init(const Config &config) noexcept
{
    if (config.imu == nullptr || !estimate::is_valid_axis_mapping(config.parameters.mounting))
    {
        return false;
    }
    config_ = config;
    event_publisher_ = topics::bringup::imu_event.claim_publisher();
    imu_publisher_ = topics::bringup::imu.claim_publisher();
    if (!event_publisher_ || !imu_publisher_)
    {
        return false;
    }

    const float gyro_rate_hz =
        static_cast<float>(device::Bmi088Gyro::output_rate_hz(config.parameters.driver.gyro_bandwidth));
    if (!vqf_.init(estimate::Vqf::Config{}, 1.0F / gyro_rate_hz, 1.0F / ACCEL_OUTPUT_RATE_HZ))
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

    if (config.heater != nullptr)
    {
        if (!config.heater->init() || !config.heater->set_target_celsius(config.parameters.heater_target_celsius))
        {
            return false;
        }
        config.heater->set_enabled(true);
    }
    report_ = Report{};
    latest_ = topics::bringup::Imu{};
    latest_.quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
    return true;
}

void ImuCheck::step(hal::os::Microseconds now) noexcept
{
    device::Bmi088 &imu = *config_.imu;
    topics::bringup::ImuEvent event{};
    if (event_.update(event))
    {
        // 延迟：中断里发布到本轮开始
        const hal::os::Microseconds latency = hal::os::elapsed_us(topics::bringup::imu_event.last_publish_us(), now);
        ++report_.events;
        report_.lost_events = event_.lost_count();
        report_.last_latency_us = latency;
        if (latency > report_.max_latency_us)
        {
            report_.max_latency_us = latency;
        }
        if (latency > config_.parameters.latency_limit_us)
        {
            ++report_.late;
        }
        imu.notify_data_ready(device::Bmi088DataReady{event.accel_sequence, event.accel_us},
                              device::Bmi088DataReady{event.gyro_sequence, event.gyro_us});
    }

    imu.process();
    device::Bmi088SampleRecord record{};
    bool gyroscope = false;
    while (imu.pop_next_sample(record, gyroscope))
    {
        handle_sample(record, gyroscope);
    }

    device::Bmi088Temperature::Sample temperature{};
    if (imu.get_temperature().try_get_sample(temperature))
    {
        latest_.temperature_celsius = device::Bmi088Temperature::to_celsius(temperature.raw);
        report_.temperature_celsius = latest_.temperature_celsius;
    }
    if (config_.heater != nullptr)
    {
        config_.heater->process(imu.get_temperature());
    }
    report_.ready = imu.is_ready();
    report_.error = imu.get_state() == device::Bmi088::State::ERROR;

    if (has_new_gyro_ && hal::os::has_elapsed(last_publish_us_, config_.parameters.publish_period_us, now))
    {
        const math::Quaternion attitude = vqf_.get_quaternion();
        latest_.quaternion = {attitude.w, attitude.x, attitude.y, attitude.z};
        imu_publisher_.publish(latest_);
        has_new_gyro_ = false;
        last_publish_us_ = now;
        ++report_.published;
    }
}

void ImuCheck::handle_sample(const device::Bmi088SampleRecord &record, bool gyroscope) noexcept
{
    float sensor[3]{};
    float body[3]{};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        sensor[axis] = gyroscope ? device::Bmi088Gyro::to_radians_per_second(record.xyz[axis])
                                 : device::Bmi088Accel::to_mps2(record.xyz[axis]);
    }
    estimate::map_sensor_vector_to_body(config_.parameters.mounting, sensor, body);
    const math::Vector3 vector{body[0], body[1], body[2]};
    if (gyroscope)
    {
        vqf_.update_gyroscope(vector);
        latest_.gyro_rad_per_sec = {body[0], body[1], body[2]};
        latest_.timestamp_us = record.drdy_us;
        latest_.gyro_sample_count = ++report_.gyro_samples;
        has_new_gyro_ = true;
    }
    else
    {
        vqf_.update_accelerometer(vector);
        latest_.accel_m_per_s2 = {body[0], body[1], body[2]};
        ++report_.accel_samples;
    }
}

// ---- 框架中断 ----

void ImuCheck::on_accel_ready(void *context) noexcept
{
    static_cast<ImuCheck *>(context)->publish_event(true, false, false);
}

void ImuCheck::on_gyro_ready(void *context) noexcept
{
    static_cast<ImuCheck *>(context)->publish_event(false, true, false);
}

void ImuCheck::on_transfer_complete(void *context) noexcept
{
    static_cast<ImuCheck *>(context)->publish_event(false, false, true);
}

void ImuCheck::publish_event(bool accel, bool gyro, bool transfer) noexcept
{
    topics::bringup::ImuEvent snapshot{};
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
        snapshot = interrupt_state_;
    }
    event_publisher_.publish(snapshot);
}

} // namespace bringup
