#include "lib/estimate/eso.hpp"

#include <cmath>

namespace estimate
{

namespace
{

// 设计周期的上限：比这还大就不是"每个控制周期算一次"的观测器了，一般是单位写错（ms 当 s）。
constexpr float MAXIMUM_SAMPLE_TIME_SECOND{0.1F};
// ω₀·h 的上限：超过它观测器几乎不滤波，以后一拍就跳，一般是 ω₀ 填成了 Hz。
constexpr float MAXIMUM_BANDWIDTH_TIME_PRODUCT{1.0F};
// dt 超过设计周期的这么多倍就当作断档，不跨着预测（一拍的计时要走 1 ms，10 倍已经丢了 9 份以上）。
constexpr float GAP_SAMPLE_TIME_RATIO{10.0F};

bool parameters_valid(const float bandwidth, const float input_gain, const float sample_time) noexcept
{
    return std::isfinite(bandwidth) && std::isfinite(input_gain) && std::isfinite(sample_time) && bandwidth > 0.0F &&
           input_gain > 0.0F && sample_time > 0.0F && sample_time <= MAXIMUM_SAMPLE_TIME_SECOND &&
           bandwidth * sample_time <= MAXIMUM_BANDWIDTH_TIME_PRODUCT;
}

} // namespace

template <std::size_t Order>
bool Eso<Order>::init(const Parameters &parameters) noexcept
{
    const float bandwidth = parameters.bandwidth_rad_per_second;
    const float input_gain = parameters.input_gain;
    const float sample_time = parameters.sample_time_second;
    if (!parameters_valid(bandwidth, input_gain, sample_time))
    {
        return false;
    }
    // 误差的 Order + 1 个极点全放在 β = e^(−ω₀·h)；增益有闭式解（当前估计形式，见类注释）。
    const float beta = std::exp(-bandwidth * sample_time);
    const float one_minus_beta = 1.0F - beta;
    std::array<float, STATE_COUNT> gains{};
    if constexpr (Order == 1U)
    {
        gains[0] = 1.0F - beta * beta;
        gains[1] = one_minus_beta * one_minus_beta / sample_time;
    }
    else
    {
        gains[0] = 1.0F - beta * beta * beta;
        gains[1] = 3.0F * one_minus_beta * one_minus_beta * (1.0F + beta) / (2.0F * sample_time);
        gains[2] = one_minus_beta * one_minus_beta * one_minus_beta / (sample_time * sample_time);
    }
    for (const float gain : gains)
    {
        if (!std::isfinite(gain) || gain <= 0.0F)
        {
            return false;
        }
    }
    gains_ = gains;
    input_gain_ = input_gain;
    sample_time_ = sample_time;
    configured_ = true;
    return true;
}

template <std::size_t Order>
void Eso<Order>::reset() noexcept
{
    states_ = {};
    initialized_ = false;
}

template <std::size_t Order>
bool Eso<Order>::update(const float measurement, const float applied_input, const float dt) noexcept
{
    if (!configured_ || !std::isfinite(measurement) || !std::isfinite(applied_input) || !std::isfinite(dt) ||
        dt <= 0.0F)
    {
        return false;
    }
    if (!initialized_)
    {
        // reset() 之后的第一拍：用测量初始化对象状态，扰动从 0 开始（几十毫秒内收敛）。
        states_ = {};
        states_[0] = measurement;
        initialized_ = true;
        return true;
    }
    if (dt > GAP_SAMPLE_TIME_RATIO * sample_time_)
    {
        // 断档：跨着预测会把扰动估计也带偏，云台一沉；对象状态用测量重新初始化，扰动估计保留。
        const float disturbance = states_[Order];
        states_ = {};
        states_[0] = measurement;
        states_[Order] = disturbance;
        return true;
    }

    // 预测：Φ(dt)·z + Γ(dt)·u，扩张状态（f）在周期内当常数，不变。
    const float input = input_gain_ * applied_input;
    std::array<float, STATE_COUNT> predicted = states_;
    if constexpr (Order == 1U)
    {
        predicted[0] = states_[0] + dt * (states_[1] + input);
    }
    else
    {
        const float acceleration = states_[2] + input;
        predicted[0] = states_[0] + dt * states_[1] + 0.5F * dt * dt * acceleration;
        predicted[1] = states_[1] + dt * acceleration;
    }

    // 修正：z = z⁻ + L·(y − z⁻₁)。先算出整拍再一起写回，要么整拍成功、要么不变。
    const float error = measurement - predicted[0];
    std::array<float, STATE_COUNT> next{};
    for (std::size_t index = 0U; index < STATE_COUNT; ++index)
    {
        next[index] = predicted[index] + gains_[index] * error;
        if (!std::isfinite(next[index]))
        {
            return false;
        }
    }
    states_ = next;
    return true;
}

template <std::size_t Order>
float Eso<Order>::state(const std::size_t index) const noexcept
{
    return index < STATE_COUNT ? states_[index] : 0.0F;
}

template <std::size_t Order>
float Eso<Order>::disturbance() const noexcept
{
    return states_[Order];
}

template <std::size_t Order>
const std::array<float, Eso<Order>::STATE_COUNT> &Eso<Order>::gains() const noexcept
{
    return gains_;
}

// 用户只会用这两个阶数，其他阶数编译期就被 static_assert 挡住。
template class Eso<1U>;
template class Eso<2U>;

} // namespace estimate
