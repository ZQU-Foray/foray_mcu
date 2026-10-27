#include "imu/zero_rate_bias_observer.hpp"
#include <cmath>

namespace application
{

namespace
{
constexpr float REFERENCE_GRAVITY_MPS2{9.80665F};
constexpr double MILLISECONDS_PER_SECOND{1000.0};
} // namespace

void ZeroRateBiasObserver::init(const Config &config) noexcept
{
    config_ = config;
    reset();
}

void ZeroRateBiasObserver::reset() noexcept
{
    bias_rad_per_sec_ = 0.0F;
    variance_ = config_.initial_variance_rad_per_sec2;
    process_noise_per_tick_ = config_.tick_frequency_hz == 0U ? 0.0F
                                                              : config_.process_noise_rad_per_sec2_per_sec /
                                                                    static_cast<float>(config_.tick_frequency_hz);
    for (int axis = 0; axis < 3; ++axis)
    {
        sum_[axis] = 0.0;
        sum_squares_[axis] = 0.0;
        block_sum_[axis] = 0.0F;
        block_sum_squares_[axis] = 0.0F;
        window_mean_[axis] = 0.0F;
        window_std_[axis] = 0.0F;
    }
    accel_norm_sum_ = 0.0;
    accel_norm_sum_squares_ = 0.0;
    block_accel_sum_ = 0.0F;
    block_accel_sum_squares_ = 0.0F;
    accel_norm_mean_ = 0.0F;
    accel_norm_std_ = 0.0F;
    count_ = 0U;
    last_tick_ = 0U;
    qualified_windows_ = 0U;
    observation_count_ = 0U;
    rejected_windows_ = 0U;
    window_ticks_ = 0U;
    static_ticks_ = 0U;
    window_valid_ = true;
    static_qualified_ = false;
    timing_anchored_ = false;
}

std::uint64_t ZeroRateBiasObserver::ticks_from_ms(std::uint32_t ms) const noexcept
{
    // 不做 32 位饱和：480 MHz 下超过 8.9 s 的时长无法用 32 位差值表示。
    return static_cast<std::uint64_t>(ms) * static_cast<std::uint64_t>(config_.tick_frequency_hz) / 1000U;
}

void ZeroRateBiasObserver::fold_block() noexcept
{
    for (int axis = 0; axis < 3; ++axis)
    {
        sum_[axis] += static_cast<double>(block_sum_[axis]);
        sum_squares_[axis] += static_cast<double>(block_sum_squares_[axis]);
        block_sum_[axis] = 0.0F;
        block_sum_squares_[axis] = 0.0F;
    }
    accel_norm_sum_ += static_cast<double>(block_accel_sum_);
    accel_norm_sum_squares_ += static_cast<double>(block_accel_sum_squares_);
    block_accel_sum_ = 0.0F;
    block_accel_sum_squares_ = 0.0F;
}

void ZeroRateBiasObserver::process(const math::Vector3 &gyro_rad_per_sec,
                                   const math::Vector3 &accel_meters_per_sec2,
                                   std::uint32_t tick) noexcept
{
    if (config_.window_samples == 0U || config_.tick_frequency_hz == 0U)
    {
        return; // 参数或时基不可用：既不统计也不判定
    }

    // 建立时基：首个样本只记录起点，不产生时间增量。
    const std::uint32_t delta_ticks = timing_anchored_ ? (tick - last_tick_) : 0U;
    timing_anchored_ = true;
    last_tick_ = tick;
    window_ticks_ += delta_ticks;
    // 过程噪声（预测步）：零偏随机游走，与是否静止无关；
    // 运动期间没有观测，协方差就只长不消，回到静止后增益自然变大、重新收敛。
    variance_ += process_noise_per_tick_ * static_cast<float>(delta_ticks);
    if (variance_ > config_.maximum_variance_rad_per_sec2)
    {
        variance_ = config_.maximum_variance_rad_per_sec2;
    }

    if (math::is_finite(gyro_rad_per_sec) && math::is_finite(accel_meters_per_sec2))
    {
        // 热路径只用单精度累加当前子块；双精度总量在子块满时才更新。
        const float axes[3]{gyro_rad_per_sec.x, gyro_rad_per_sec.y, gyro_rad_per_sec.z};
        for (int axis = 0; axis < 3; ++axis)
        {
            block_sum_[axis] += axes[axis];
            block_sum_squares_[axis] += axes[axis] * axes[axis];
            // 撞击/异常样本让整窗作废：均值门挡不住它，方差门又可能被单点带偏。
            if (std::fabs(axes[axis]) > config_.maximum_instantaneous_rate_rad_per_sec)
            {
                window_valid_ = false;
            }
        }
        const float accel_norm = math::norm(accel_meters_per_sec2);
        block_accel_sum_ += accel_norm;
        block_accel_sum_squares_ += accel_norm * accel_norm;
    }
    else
    {
        window_valid_ = false;
    }

    ++count_;
    const std::uint32_t block_samples = config_.sub_block_samples == 0U ? 1U : config_.sub_block_samples;
    if ((count_ % block_samples) == 0U)
    {
        fold_block();
    }
    if (count_ >= config_.window_samples)
    {
        finish_window();
    }
}

void ZeroRateBiasObserver::finish_window() noexcept
{
    // 收尾：把可能不满一个子块的尾料并入总量，保证统计量覆盖全部 count_ 个样本。
    fold_block();
    const double inverse = count_ == 0U ? 0.0 : 1.0 / static_cast<double>(count_);

    for (int axis = 0; axis < 3; ++axis)
    {
        const double mean = sum_[axis] * inverse;
        const double variance = sum_squares_[axis] * inverse - mean * mean;
        window_mean_[axis] = static_cast<float>(mean);
        window_std_[axis] = static_cast<float>(std::sqrt(variance > 0.0 ? variance : 0.0));
    }
    accel_norm_mean_ = static_cast<float>(accel_norm_sum_ * inverse);
    const double accel_variance = accel_norm_sum_squares_ * inverse -
                                  static_cast<double>(accel_norm_mean_) * static_cast<double>(accel_norm_mean_);
    accel_norm_std_ = static_cast<float>(std::sqrt(accel_variance > 0.0 ? accel_variance : 0.0));

    bool qualified = window_valid_ && count_ > 0U;
    // 严格均值门只作用于被观测的 z 轴：x/y 的零偏可以远大于该门槛（板上实测
    // y 轴 0.52 °/s），把它们一起判会把所有窗口都拒掉；而慢速倾斜并不影响
    // z 轴零偏的可观测性，只需一个宽松上限排除大幅翻滚。
    if (std::fabs(window_mean_[2]) > config_.maximum_mean_rate_rad_per_sec)
    {
        qualified = false;
    }
    if (std::fabs(window_mean_[0]) > config_.maximum_cross_axis_mean_rate_rad_per_sec ||
        std::fabs(window_mean_[1]) > config_.maximum_cross_axis_mean_rate_rad_per_sec)
    {
        qualified = false;
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        if (window_std_[axis] > config_.maximum_standard_deviation_rad_per_sec)
        {
            qualified = false;
        }
    }
    if (std::fabs(accel_norm_mean_ - REFERENCE_GRAVITY_MPS2) > config_.maximum_accel_norm_error_mps2)
    {
        qualified = false;
    }
    if (accel_norm_std_ > config_.maximum_accel_standard_deviation_mps2)
    {
        qualified = false;
    }
    static_qualified_ = qualified;

    if (qualified)
    {
        ++qualified_windows_;
        static_ticks_ += window_ticks_;
        // 连续静止够久才允许修正；此前只统计，避免被瞬态骗到。
        if (static_ticks_ >= ticks_from_ms(config_.confirm_ms) && config_.apply_correction)
        {
            apply_zero_rate_observation();
        }
    }
    else
    {
        ++rejected_windows_;
        static_ticks_ = 0U;
    }

    for (int axis = 0; axis < 3; ++axis)
    {
        sum_[axis] = 0.0;
        sum_squares_[axis] = 0.0;
    }
    accel_norm_sum_ = 0.0;
    accel_norm_sum_squares_ = 0.0;
    count_ = 0U;
    window_ticks_ = 0U;
    window_valid_ = true;
}
void ZeroRateBiasObserver::apply_zero_rate_observation() noexcept
{
    // 量测：静止时窗口内的 z 均值应等于零偏本身。
    // 量测方差取"窗口均值"的方差，而不是单样本方差。
    float measure_variance = (window_std_[2] * window_std_[2]) / static_cast<float>(config_.window_samples);
    if (measure_variance < config_.minimum_measure_variance_rad_per_sec2)
    {
        measure_variance = config_.minimum_measure_variance_rad_per_sec2;
    }

    const float gain = variance_ / (variance_ + measure_variance);
    float correction = gain * (window_mean_[2] - bias_rad_per_sec_);
    // 限幅：即使某个窗口被误判为静止，也只能把 trim 拉动有限一步。
    if (correction > config_.maximum_correction_step_rad_per_sec)
    {
        correction = config_.maximum_correction_step_rad_per_sec;
    }
    else if (correction < -config_.maximum_correction_step_rad_per_sec)
    {
        correction = -config_.maximum_correction_step_rad_per_sec;
    }
    bias_rad_per_sec_ += correction;
    variance_ = (1.0F - gain) * variance_;

    ++observation_count_;
}

ZeroRateBiasObserver::Snapshot ZeroRateBiasObserver::get_snapshot() const noexcept
{
    Snapshot snapshot{};
    snapshot.static_qualified = static_qualified_;
    snapshot.correction_applied = is_correction_applied();
    snapshot.bias_rad_per_sec = bias_rad_per_sec_;
    snapshot.variance_rad_per_sec2 = variance_;
    for (int axis = 0; axis < 3; ++axis)
    {
        snapshot.window_mean_rad_per_sec[axis] = window_mean_[axis];
        snapshot.window_std_rad_per_sec[axis] = window_std_[axis];
    }
    snapshot.accel_norm_mean_mps2 = accel_norm_mean_;
    snapshot.accel_norm_std_mps2 = accel_norm_std_;
    if (config_.tick_frequency_hz != 0U)
    {
        snapshot.static_ms = static_cast<std::uint32_t>(static_ticks_ * 1000U / config_.tick_frequency_hz);
    }
    snapshot.qualified_windows = qualified_windows_;
    snapshot.observation_count = observation_count_;
    snapshot.rejected_windows = rejected_windows_;
    return snapshot;
}

} // namespace application
