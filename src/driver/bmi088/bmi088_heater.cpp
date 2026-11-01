#include "driver/bmi088/bmi088_heater.hpp"

#include <cmath>

namespace device
{

namespace
{

constexpr float MAX_DUTY_PERMILLE{1000.0F};

} // namespace

bool Bmi088Heater::init() noexcept
{
    const control::HeaterControl::Config control_config{
        config_.model,
        control::HeaterControl::Sensor{
            TEMPERATURE_UPDATE_PERIOD_S, Bmi088Temperature::CELSIUS_PER_LSB, TEMPERATURE_READ_LAG_S},
        config_.tuning,
    };
    initialized_ = control_.init(control_config) && pwm_.is_ready();
    enabled_ = false;
    sample_seen_ = false;
    state_ = initialized_ ? State::DISABLED : State::FAULT;
    target_celsius_ = DEFAULT_TARGET_CELSIUS;
    temperature_celsius_ = 0.0F;
    duty_permille_ = 0U;
    last_temperature_read_count_ = 0U;
    last_sample_us_ = hal::os::now_us();
    last_change_us_ = last_sample_us_;
    last_control_us_ = last_sample_us_;
    last_pwm_result_ = pwm_.off();
    if (last_pwm_result_ != hal::Pwm::Result::COMPLETED)
    {
        initialized_ = false;
        state_ = State::FAULT;
    }
    return initialized_;
}

void Bmi088Heater::set_enabled(bool enabled) noexcept
{
    if (!initialized_)
    {
        disable_output(State::FAULT);
        return;
    }

    enabled_ = enabled;
    control_.reset();
    last_control_us_ = hal::os::now_us();
    if (!enabled)
    {
        disable_output(State::DISABLED);
        return;
    }

    disable_output(State::WAITING_FOR_TEMPERATURE);
}

bool Bmi088Heater::set_target_celsius(float target_celsius) noexcept
{
    if (!std::isfinite(target_celsius) || target_celsius < MINIMUM_TARGET_CELSIUS ||
        target_celsius > MAXIMUM_TARGET_CELSIUS)
    {
        return false;
    }

    target_celsius_ = target_celsius;
    return true;
}

void Bmi088Heater::process(const Bmi088Temperature &temperature) noexcept
{
    if (!initialized_ || !enabled_)
    {
        return;
    }

    if (temperature.get_state() == Bmi088Temperature::State::ERROR)
    {
        disable_output(State::FAULT);
        control_.reset();
        return;
    }

    Bmi088Temperature::Sample sample{};
    const std::uint32_t read_count = temperature.get_read_count();
    if (read_count != last_temperature_read_count_ && temperature.try_get_sample(sample))
    {
        last_temperature_read_count_ = read_count;
        temperature_celsius_ = Bmi088Temperature::to_celsius(sample.raw);
        last_sample_us_ = hal::os::now_us();
        if (!sample_seen_ || sample.raw != last_raw_)
        {
            // 读数变化：传感器刚更新过，控制器据此锁定更新相位
            last_raw_ = sample.raw;
            last_change_us_ = temperature.get_completion_us();
        }
        sample_seen_ = true;
    }

    if (!sample_seen_)
    {
        disable_output(State::WAITING_FOR_TEMPERATURE);
        return;
    }

    const hal::os::Microseconds now = hal::os::now_us();
    if (!std::isfinite(temperature_celsius_) || temperature_celsius_ > MAXIMUM_SAFE_CELSIUS ||
        hal::os::has_elapsed(last_sample_us_, SAMPLE_TIMEOUT_US, now))
    {
        disable_output(State::FAULT);
        control_.reset();
        return;
    }

    if (!hal::os::has_elapsed(last_control_us_, CONTROL_PERIOD_US, now))
    {
        return;
    }

    const float delta_seconds = hal::os::to_seconds(hal::os::elapsed_us(last_control_us_, now));
    last_control_us_ = now;
    const float reading_age_seconds = hal::os::to_seconds(hal::os::elapsed_us(last_change_us_, now));
    const float duty = control_.update(target_celsius_, temperature_celsius_, reading_age_seconds, delta_seconds);
    const float rounded = std::round(duty < 0.0F ? 0.0F : (duty > MAX_DUTY_PERMILLE ? MAX_DUTY_PERMILLE : duty));
    if (set_duty(static_cast<std::uint16_t>(rounded)))
    {
        state_ = control_.phase() == control::HeaterControl::Phase::REGULATE ? State::REGULATING : State::PREHEATING;
    }
}

void Bmi088Heater::disable_output(State next_state) noexcept
{
    last_pwm_result_ = pwm_.off();
    duty_permille_ = 0U;
    state_ = last_pwm_result_ == hal::Pwm::Result::COMPLETED ? next_state : State::FAULT;
}

bool Bmi088Heater::set_duty(std::uint16_t duty_permille) noexcept
{
    last_pwm_result_ = pwm_.set(PWM_FREQUENCY_HZ, duty_permille);
    if (last_pwm_result_ != hal::Pwm::Result::COMPLETED)
    {
        duty_permille_ = 0U;
        state_ = State::FAULT;
        (void)pwm_.off();
        return false;
    }

    duty_permille_ = duty_permille;
    return true;
}

} // namespace device
