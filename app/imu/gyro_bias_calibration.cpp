#include "Application/Imu/GyroBiasCalibration.hpp"
#include <cmath>

namespace application
{

namespace
{
constexpr float REFERENCE_GRAVITY_MPS2{9.80665F};
constexpr std::uint64_t MILLISECONDS_PER_SECOND{1000U};
} // namespace

void GyroBiasCalibration::init(const Config &config) noexcept
{
    config_ = config;
    reset();
}

void GyroBiasCalibration::reset() noexcept
{
    state_ = State::WAITING_FOR_TEMPERATURE;
    bias_ = math::Vector3{};
    spread_ = math::Vector3{};
    restart_window();
    restarts_ = 0U; // 必须在 restart_window 之后清零，否则复位本身会被计成一次重启
    sampling_ticks_ = 0U;
    hold_ticks_ = 0U;
    hold_minimum_ = 0.0F;
    hold_maximum_ = 0.0F;
    hold_active_ = false;
    timing_anchored_ = false;
    last_tick_ = 0U;
    temperature_celsius_ = 0.0F;
    bias_valid_ = false;
}

void GyroBiasCalibration::restart_window() noexcept
{
    if (count_ != 0U)
    {
        ++restarts_; // 只统计"已经开始积累后被中断"的情况
    }
    sum_x_ = 0.0;
    sum_y_ = 0.0;
    sum_z_ = 0.0;
    minimum_ = math::Vector3{};
    maximum_ = math::Vector3{};
    count_ = 0U;
    window_ticks_ = 0U;
}

std::uint64_t GyroBiasCalibration::ticks_from_ms(std::uint32_t milliseconds) const noexcept
{
    // 不做 32 位饱和：480 MHz 下 20 s 是 9.6e9 tick，远大于 2^32，
    // 饱和成 0xFFFFFFFF 会让"超过时限"的比较永远不成立。
    return static_cast<std::uint64_t>(milliseconds) * static_cast<std::uint64_t>(config_.tick_frequency_hz) /
           MILLISECONDS_PER_SECOND;
}

void GyroBiasCalibration::process(const math::Vector3 &gyro_rad_per_sec,
                                  const math::Vector3 &accel_meters_per_sec2,
                                  float temperature_celsius,
                                  std::uint32_t tick) noexcept
{
    if (state_ == State::COMPLETED || state_ == State::TIMED_OUT)
    {
        return; // 终态不再改变：标定只做一次
    }
    if (config_.tick_frequency_hz == 0U)
    {
        return; // 没有可信时基：既不积累也不判定
    }

    // 建立时基：首个样本只记录起点，不产生时间增量。
    const std::uint32_t delta_ticks = timing_anchored_ ? (tick - last_tick_) : 0U;
    timing_anchored_ = true;
    last_tick_ = tick;
    temperature_celsius_ = temperature_celsius;

    // 温度门控：离开容差就清空窗口与稳定保持，并重新开始计时。
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
        hold_ticks_ = 0U;
        return;
    }

    // 温度稳定门控：进带后还要保持足够久且极差足够小才允许开窗，
    // 带内极差超限就从当前温度重新开始保持计时。
    if (!hold_active_)
    {
        hold_active_ = true;
        hold_ticks_ = 0U;
        hold_minimum_ = temperature_celsius;
        hold_maximum_ = temperature_celsius;
    }
    else
    {
        if (temperature_celsius < hold_minimum_)
        {
            hold_minimum_ = temperature_celsius;
        }
        if (temperature_celsius > hold_maximum_)
        {
            hold_maximum_ = temperature_celsius;
        }
    }
    hold_ticks_ += delta_ticks;
    if (hold_maximum_ - hold_minimum_ > config_.temperature_stability_celsius)
    {
        hold_minimum_ = temperature_celsius;
        hold_maximum_ = temperature_celsius;
        hold_ticks_ = 0U;
    }

    if (state_ == State::WAITING_FOR_TEMPERATURE)
    {
        if (hold_ticks_ < ticks_from_ms(config_.temperature_settle_ms))
        {
            return; // 温度还没稳定够久：不开窗、不积累样本
        }
        state_ = State::SAMPLING;
        restart_window();
    }

    // 采样预算跨窗口累计：窗口因运动重启不重置预算；温度出带后暂停增长
    // 但保留已用额度，避免"开机后被搬动几秒"就花掉整轮标定机会。
    sampling_ticks_ += delta_ticks;
    if (sampling_ticks_ >= ticks_from_ms(config_.timeout_ms))
    {
        state_ = State::TIMED_OUT;
        bias_valid_ = false;
        restart_window();
        return;
    }

    // 静止门控：加速度范数必须接近重力。
    const float accel_norm = math::norm(accel_meters_per_sec2);
    if (!math::is_finite(accel_meters_per_sec2) ||
        std::fabs(accel_norm - REFERENCE_GRAVITY_MPS2) > config_.maximum_accel_norm_error_mps2)
    {
        restart_window();
        return;
    }

    accumulate(gyro_rad_per_sec, delta_ticks);
}

void GyroBiasCalibration::accumulate(const math::Vector3 &gyro_rad_per_sec, std::uint32_t delta_ticks) noexcept
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
        if (gyro_rad_per_sec.x < minimum_.x)
        {
            minimum_.x = gyro_rad_per_sec.x;
        }
        if (gyro_rad_per_sec.y < minimum_.y)
        {
            minimum_.y = gyro_rad_per_sec.y;
        }
        if (gyro_rad_per_sec.z < minimum_.z)
        {
            minimum_.z = gyro_rad_per_sec.z;
        }
        if (gyro_rad_per_sec.x > maximum_.x)
        {
            maximum_.x = gyro_rad_per_sec.x;
        }
        if (gyro_rad_per_sec.y > maximum_.y)
        {
            maximum_.y = gyro_rad_per_sec.y;
        }
        if (gyro_rad_per_sec.z > maximum_.z)
        {
            maximum_.z = gyro_rad_per_sec.z;
        }
    }

    sum_x_ += static_cast<double>(gyro_rad_per_sec.x);
    sum_y_ += static_cast<double>(gyro_rad_per_sec.y);
    sum_z_ += static_cast<double>(gyro_rad_per_sec.z);
    ++count_;
    window_ticks_ += delta_ticks;

    // 每个样本都检查极差：被碰或振动必须在下一个样本就重启窗口，
    // 否则短促扰动会被记进窗口，只能等到周期检查或收尾时才被剔除。
    const math::Vector3 spread{maximum_.x - minimum_.x, maximum_.y - minimum_.y, maximum_.z - minimum_.z};
    if (spread.x > config_.maximum_spread_rad_per_sec || spread.y > config_.maximum_spread_rad_per_sec ||
        spread.z > config_.maximum_spread_rad_per_sec)
    {
        restart_window();
        return;
    }

    // 周期性检查部分均值：抓缓慢转动，代价是与样本数无关的常数开销。
    if ((count_ % PARTIAL_CHECK_INTERVAL) == 0U)
    {
        const double inverse = 1.0 / static_cast<double>(count_);
        const math::Vector3 mean{static_cast<float>(sum_x_ * inverse),
                                 static_cast<float>(sum_y_ * inverse),
                                 static_cast<float>(sum_z_ * inverse)};
        if (std::fabs(mean.x) > config_.maximum_mean_rate_rad_per_sec ||
            std::fabs(mean.y) > config_.maximum_mean_rate_rad_per_sec ||
            std::fabs(mean.z) > config_.maximum_mean_rate_rad_per_sec)
        {
            restart_window();
            return;
        }
    }

    if (window_ticks_ >= ticks_from_ms(config_.window_ms))
    {
        finalize();
    }
}

void GyroBiasCalibration::finalize() noexcept
{
    if (count_ == 0U)
    {
        restart_window();
        return;
    }

    const double inverse = 1.0 / static_cast<double>(count_);
    const math::Vector3 mean{static_cast<float>(sum_x_ * inverse),
                             static_cast<float>(sum_y_ * inverse),
                             static_cast<float>(sum_z_ * inverse)};
    const math::Vector3 spread{maximum_.x - minimum_.x, maximum_.y - minimum_.y, maximum_.z - minimum_.z};

    // 均值超限说明窗口内存在慢速转动，不能当作零偏，重新开始。
    if (!math::is_finite(mean) || std::fabs(mean.x) > config_.maximum_mean_rate_rad_per_sec ||
        std::fabs(mean.y) > config_.maximum_mean_rate_rad_per_sec ||
        std::fabs(mean.z) > config_.maximum_mean_rate_rad_per_sec || spread.x > config_.maximum_spread_rad_per_sec ||
        spread.y > config_.maximum_spread_rad_per_sec || spread.z > config_.maximum_spread_rad_per_sec)
    {
        restart_window();
        return;
    }

    bias_ = mean;
    spread_ = spread;
    bias_valid_ = true;
    state_ = State::COMPLETED;
}

GyroBiasCalibration::Snapshot GyroBiasCalibration::get_snapshot() const noexcept
{
    Snapshot snapshot{};
    snapshot.state = state_;
    snapshot.bias_rad_per_sec = bias_;
    snapshot.spread_rad_per_sec = spread_;
    snapshot.samples = count_;
    snapshot.restarts = restarts_;
    // 时基不可用时不做除零，时长一律报 0。
    const std::uint32_t frequency_hz = config_.tick_frequency_hz;
    if (frequency_hz != 0U)
    {
        snapshot.elapsed_ms = static_cast<std::uint32_t>(sampling_ticks_ * MILLISECONDS_PER_SECOND / frequency_hz);
        snapshot.temperature_hold_ms = static_cast<std::uint32_t>(hold_ticks_ * MILLISECONDS_PER_SECOND / frequency_hz);
    }
    snapshot.temperature_celsius = temperature_celsius_;
    snapshot.bias_valid = bias_valid_;
    return snapshot;
}

} // namespace application
