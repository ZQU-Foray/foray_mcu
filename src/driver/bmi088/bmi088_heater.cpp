#include "driver/bmi088/bmi088_heater.hpp"

#include <cmath>

namespace device
{

namespace
{

constexpr float PID_KP{10.0F};
constexpr float PID_KI{1.0F};
constexpr float PID_KD{0.0F};
// 输出上限与积分上限按 PWM 千分比给出。实测原值 30‰（3%）不足以维持恒温服务点，
// 因此放大到 30% 并保持原有增益不变：比例项偏弱、无微分项，响应慢但过阻尼，
// 不会在预热门槛附近产生极限环。65 ℃ 硬保护与 500 ms 样本超时保持不变。
constexpr float PID_MAXIMUM_OUTPUT{300.0F};
constexpr float PID_INTEGRAL_LIMIT{300.0F};
constexpr float DEFAULT_CONTROL_PERIOD_SECONDS{0.128F};
constexpr float preheat_threshold_for(float target_celsius)
{
    return target_celsius - 2.0F < Bmi088Heater::PREHEAT_THRESHOLD_CELSIUS ? target_celsius - 2.0F
                                                                           : Bmi088Heater::PREHEAT_THRESHOLD_CELSIUS;
}
// 默认服务点的预热门槛受 PREHEAT_THRESHOLD_CELSIUS 钳位（45 ℃）。
static_assert(preheat_threshold_for(Bmi088Heater::DEFAULT_TARGET_CELSIUS) == Bmi088Heater::PREHEAT_THRESHOLD_CELSIUS);
static_assert(preheat_threshold_for(32.0F) == 30.0F);

} // namespace

bool Bmi088Heater::init() noexcept
{
    alg_controller::PID::Config config{};
    config.mode = alg_controller::PIDMode::POSITION;
    config.kp = PID_KP;
    config.ki = PID_KI;
    config.kd = PID_KD;
    config.default_dt = DEFAULT_CONTROL_PERIOD_SECONDS;
    config.maxout = PID_MAXIMUM_OUTPUT;
    config.integral_limit = PID_INTEGRAL_LIMIT;

    initialized_ = pid_.init(config) && platform::Pwm::is_ready(platform::Pwm::Channel::IMU_HEATER);
    enabled_ = false;
    sample_seen_ = false;
    state_ = initialized_ ? State::DISABLED : State::FAULT;
    target_celsius_ = DEFAULT_TARGET_CELSIUS;
    temperature_celsius_ = 0.0F;
    duty_permille_ = 0U;
    last_temperature_read_count_ = 0U;
    last_sample_tick_ = platform::Time::now_ticks();
    last_control_tick_ = last_sample_tick_;
    last_pwm_result_ = platform::Pwm::silence(platform::Pwm::Channel::IMU_HEATER);
    if (last_pwm_result_ != platform::Pwm::Result::COMPLETED)
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
    pid_.reset();
    last_control_tick_ = platform::Time::now_ticks();
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
        pid_.reset();
        return;
    }

    Bmi088Temperature::Sample sample{};
    const std::uint32_t read_count = temperature.get_read_count();
    if (read_count != last_temperature_read_count_ && temperature.try_get_sample(sample))
    {
        last_temperature_read_count_ = read_count;
        temperature_celsius_ = Bmi088Temperature::to_celsius(sample.raw);
        last_sample_tick_ = platform::Time::now_ticks();
        sample_seen_ = true;
    }

    if (!sample_seen_)
    {
        disable_output(State::WAITING_FOR_TEMPERATURE);
        return;
    }

    if (!std::isfinite(temperature_celsius_) || temperature_celsius_ > MAXIMUM_SAFE_CELSIUS ||
        platform::Time::has_elapsed_ms(last_sample_tick_, SAMPLE_TIMEOUT_MS))
    {
        disable_output(State::FAULT);
        pid_.reset();
        return;
    }

    if (!platform::Time::has_elapsed_ms(last_control_tick_, CONTROL_PERIOD_MS))
    {
        return;
    }

    const platform::Time::Tick now = platform::Time::now_ticks();
    const float delta_seconds = platform::Time::ticks_to_seconds(now - last_control_tick_);
    last_control_tick_ = now;

    // 默认 50 C 目标沿用原有 45 C 快速预热边界。
    // 更低的诊断设定值不得被强行推过其目标。
    const float preheat_threshold = preheat_threshold_for(target_celsius_);
    if (temperature_celsius_ < preheat_threshold)
    {
        pid_.reset();
        if (set_duty(PREHEAT_DUTY_PERMILLE))
        {
            state_ = State::PREHEATING;
        }
        return;
    }

    if (!pid_.calculate_loop(temperature_celsius_, target_celsius_, delta_seconds))
    {
        disable_output(State::FAULT);
        pid_.reset();
        return;
    }

    // 单向积分钳位：加热器只有加热权限，负输出恒被钳为 0，负积分没有物理意义，
    // 只会在超温段持续累积（实测可达 −300‰ 饱和），温度跌回带内后压住比例项、
    // 使加热器长时间无输出（实测 48 ℃ 以下仍为 0），显著拉长回摆时间。
    if (pid_.get_integral_error() < 0.0F)
    {
        (void)pid_.set_integral_error(0.0F);
    }

    float output = pid_.get_out();
    if (!std::isfinite(output) || output < 0.0F)
    {
        output = 0.0F;
    }
    if (output > PID_MAXIMUM_OUTPUT)
    {
        output = PID_MAXIMUM_OUTPUT;
    }

    if (set_duty(static_cast<std::uint16_t>(output + 0.5F)))
    {
        state_ = State::REGULATING;
    }
}

void Bmi088Heater::disable_output(State next_state) noexcept
{
    last_pwm_result_ = platform::Pwm::silence(platform::Pwm::Channel::IMU_HEATER);
    duty_permille_ = 0U;
    state_ = last_pwm_result_ == platform::Pwm::Result::COMPLETED ? next_state : State::FAULT;
}

bool Bmi088Heater::set_duty(std::uint16_t duty_permille) noexcept
{
    last_pwm_result_ = platform::Pwm::set(platform::Pwm::Channel::IMU_HEATER, PWM_FREQUENCY_HZ, duty_permille);
    if (last_pwm_result_ != platform::Pwm::Result::COMPLETED)
    {
        duty_permille_ = 0U;
        state_ = State::FAULT;
        (void)platform::Pwm::silence(platform::Pwm::Channel::IMU_HEATER);
        return false;
    }

    duty_permille_ = duty_permille;
    return true;
}

} // namespace device
