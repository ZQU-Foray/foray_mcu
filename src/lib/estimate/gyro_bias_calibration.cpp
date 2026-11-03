#include "lib/estimate/gyro_bias_calibration.hpp"

#include <algorithm>
#include <cmath>

namespace estimate
{

namespace
{

constexpr float REFERENCE_GRAVITY_MPS2{9.80665F};

math::Vector3 component_min(const math::Vector3 &first, const math::Vector3 &second) noexcept
{
    return math::Vector3{std::min(first.x, second.x), std::min(first.y, second.y), std::min(first.z, second.z)};
}

math::Vector3 component_max(const math::Vector3 &first, const math::Vector3 &second) noexcept
{
    return math::Vector3{std::max(first.x, second.x), std::max(first.y, second.y), std::max(first.z, second.z)};
}

bool any_above(const math::Vector3 &value, float limit) noexcept
{
    return value.x > limit || value.y > limit || value.z > limit;
}

} // namespace

bool GyroBiasCalibration::init(const Config &config) noexcept
{
    const bool finite =
        std::isfinite(config.target_celsius) && std::isfinite(config.temperature_tolerance_celsius) &&
        std::isfinite(config.temperature_stability_celsius) && std::isfinite(config.maximum_mean_rate_rad_per_sec) &&
        std::isfinite(config.maximum_spread_rad_per_sec) && std::isfinite(config.maximum_accel_norm_error_mps2);
    if (!finite || config.temperature_tolerance_celsius < 0.0F || config.temperature_stability_celsius < 0.0F ||
        config.maximum_mean_rate_rad_per_sec <= 0.0F || config.maximum_spread_rad_per_sec <= 0.0F ||
        config.maximum_accel_norm_error_mps2 <= 0.0F || config.window_us == 0U || config.timeout_us == 0U)
    {
        return false;
    }
    config_ = config;
    reset();
    return true;
}

void GyroBiasCalibration::reset() noexcept
{
    state_ = State::WAITING_FOR_TEMPERATURE;
    bias_ = math::Vector3{};
    spread_ = math::Vector3{};
    restart_window();
    restarts_ = 0U; // 在 restart_window() 之后清零，否则复位本身会被计成一次重启
    sampling_us_ = 0U;
    hold_us_ = 0U;
    hold_minimum_ = 0.0F;
    hold_maximum_ = 0.0F;
    hold_active_ = false;
    timing_anchored_ = false;
    last_timestamp_us_ = 0U;
    temperature_celsius_ = 0.0F;
    bias_valid_ = false;
}

void GyroBiasCalibration::restart_window() noexcept
{
    if (count_ != 0U)
    {
        ++restarts_; // 只统计"已经开始积累后被中断"
    }
    sum_ = math::Vector3{};
    minimum_ = math::Vector3{};
    maximum_ = math::Vector3{};
    count_ = 0U;
    window_us_ = 0U;
}

void GyroBiasCalibration::process(const math::Vector3 &gyro_rad_per_sec,
                                  const math::Vector3 &accel_meters_per_sec2,
                                  float temperature_celsius,
                                  std::uint32_t timestamp_us) noexcept
{
    if (state_ == State::COMPLETED || state_ == State::TIMED_OUT)
    {
        return; // 终态：标定只做一次
    }

    // 时基：第一个样本只记起点。时间戳倒退（不该发生）按 0 计
    std::uint32_t delta_us = 0U;
    if (timing_anchored_)
    {
        const auto elapsed = static_cast<std::int32_t>(timestamp_us - last_timestamp_us_);
        delta_us = elapsed > 0 ? static_cast<std::uint32_t>(elapsed) : 0U;
    }
    timing_anchored_ = true;
    last_timestamp_us_ = timestamp_us;
    temperature_celsius_ = temperature_celsius;

    // 温度门控：离开容差就清空窗口与稳定保持
    const bool temperature_ready =
        std::isfinite(temperature_celsius) &&
        std::fabs(temperature_celsius - config_.target_celsius) <= config_.temperature_tolerance_celsius;
    if (!temperature_ready)
    {
        if (state_ != State::WAITING_FOR_TEMPERATURE || count_ != 0U)
        {
            restart_window();
        }
        state_ = State::WAITING_FOR_TEMPERATURE;
        hold_active_ = false;
        hold_us_ = 0U;
        return;
    }

    // 温度稳定门控：带内极差超限就从当前温度重新计时
    if (!hold_active_)
    {
        hold_active_ = true;
        hold_us_ = 0U;
        hold_minimum_ = temperature_celsius;
        hold_maximum_ = temperature_celsius;
    }
    else
    {
        hold_minimum_ = std::min(hold_minimum_, temperature_celsius);
        hold_maximum_ = std::max(hold_maximum_, temperature_celsius);
    }
    hold_us_ += delta_us;
    if (hold_maximum_ - hold_minimum_ > config_.temperature_stability_celsius)
    {
        hold_minimum_ = temperature_celsius;
        hold_maximum_ = temperature_celsius;
        hold_us_ = 0U;
    }

    if (state_ == State::WAITING_FOR_TEMPERATURE)
    {
        if (hold_us_ < config_.temperature_settle_us)
        {
            return; // 温度还没稳定够久
        }
        state_ = State::SAMPLING;
        restart_window();
    }

    // 采样预算跨窗口累计：窗口因运动重启不重置；温度出带后暂停增长但保留已用额度
    sampling_us_ += delta_us;
    if (sampling_us_ >= config_.timeout_us)
    {
        state_ = State::TIMED_OUT;
        bias_valid_ = false;
        restart_window();
        return;
    }

    // 静止门控：加速度模长接近重力
    if (!math::is_finite(accel_meters_per_sec2) ||
        std::fabs(math::norm(accel_meters_per_sec2) - REFERENCE_GRAVITY_MPS2) > config_.maximum_accel_norm_error_mps2)
    {
        restart_window();
        return;
    }

    accumulate(gyro_rad_per_sec, delta_us);
}

void GyroBiasCalibration::accumulate(const math::Vector3 &gyro_rad_per_sec, std::uint32_t delta_us) noexcept
{
    if (!math::is_finite(gyro_rad_per_sec))
    {
        restart_window();
        return;
    }
    if (count_ == 0U)
    {
        minimum_ = gyro_rad_per_sec;
        maximum_ = gyro_rad_per_sec;
    }
    else
    {
        minimum_ = component_min(minimum_, gyro_rad_per_sec);
        maximum_ = component_max(maximum_, gyro_rad_per_sec);
    }
    sum_ = math::Vector3{sum_.x + gyro_rad_per_sec.x, sum_.y + gyro_rad_per_sec.y, sum_.z + gyro_rad_per_sec.z};
    ++count_;
    window_us_ += delta_us;

    // 每个样本都查极差：被碰、振动在下一个样本就重启窗口
    const math::Vector3 spread{maximum_.x - minimum_.x, maximum_.y - minimum_.y, maximum_.z - minimum_.z};
    if (any_above(spread, config_.maximum_spread_rad_per_sec))
    {
        restart_window();
        return;
    }

    // 周期性查部分均值：抓缓慢转动
    if ((count_ % PARTIAL_CHECK_INTERVAL) == 0U)
    {
        const float inverse = 1.0F / static_cast<float>(count_);
        if (!mean_within_limit(math::Vector3{sum_.x * inverse, sum_.y * inverse, sum_.z * inverse}))
        {
            restart_window();
            return;
        }
    }

    if (window_us_ >= config_.window_us)
    {
        finalize();
    }
}

bool GyroBiasCalibration::mean_within_limit(const math::Vector3 &mean) const noexcept
{
    const float limit = config_.maximum_mean_rate_rad_per_sec;
    return math::is_finite(mean) && std::fabs(mean.x) <= limit && std::fabs(mean.y) <= limit &&
           std::fabs(mean.z) <= limit;
}

void GyroBiasCalibration::finalize() noexcept
{
    if (count_ == 0U)
    {
        restart_window();
        return;
    }
    const float inverse = 1.0F / static_cast<float>(count_);
    const math::Vector3 mean{sum_.x * inverse, sum_.y * inverse, sum_.z * inverse};
    const math::Vector3 spread{maximum_.x - minimum_.x, maximum_.y - minimum_.y, maximum_.z - minimum_.z};
    // 均值超限说明窗口内有慢速转动，不能当零偏
    if (!mean_within_limit(mean) || any_above(spread, config_.maximum_spread_rad_per_sec))
    {
        restart_window();
        return;
    }
    bias_ = mean;
    spread_ = spread;
    bias_valid_ = true;
    state_ = State::COMPLETED;
}

GyroBiasCalibration::Snapshot GyroBiasCalibration::snapshot() const noexcept
{
    Snapshot snapshot{};
    snapshot.state = state_;
    snapshot.bias_rad_per_sec = bias_;
    snapshot.spread_rad_per_sec = spread_;
    snapshot.samples = count_;
    snapshot.restarts = restarts_;
    snapshot.sampling_us = static_cast<std::uint32_t>(std::min<std::uint64_t>(sampling_us_, 0xFFFFFFFFU));
    snapshot.temperature_hold_us = static_cast<std::uint32_t>(std::min<std::uint64_t>(hold_us_, 0xFFFFFFFFU));
    snapshot.temperature_celsius = temperature_celsius_;
    snapshot.bias_valid = bias_valid_;
    return snapshot;
}

} // namespace estimate
