#include "gimbal/axis_loop.hpp"

#include "utils/math/basic_math.hpp"

#include <cmath>

namespace gimbal
{

namespace
{

constexpr float DEFAULT_DT_SECONDS{0.001F};
constexpr float MAXIMUM_DT_SECONDS{1.0F}; // control::PID 只接受 (0, 1) 的 dt
// 判断"实际下发的力矩和控制器算的一样不一样"用的容差。调用方会先加自己的前馈、再减掉，浮点上不保证
// 逐位相等；1e-4 N·m 远小于达妙 12 位量化的一格（约 0.005 N·m），不会把真正的削减漏掉。
constexpr float TORQUE_COMPARE_TOLERANCE_NEWTON_METER{1.0e-4F};

bool gains_valid(const AxisLoop::Gains &gains) noexcept
{
    return std::isfinite(gains.kp) && std::isfinite(gains.ki) && std::isfinite(gains.integral_limit) &&
           std::isfinite(gains.output_limit) && gains.kp >= 0.0F && gains.ki >= 0.0F && gains.integral_limit >= 0.0F &&
           gains.output_limit >= 0.0F;
}

control::PID::Config pid_config(const AxisLoop::Gains &gains) noexcept
{
    control::PID::Config config{};
    config.mode = control::PIDMode::POSITION;
    config.improve.integral_limit = 1U;
    config.kp = gains.kp;
    config.ki = gains.ki;
    config.kd = 0.0F; // 角速度环不加微分：陀螺本身就是角度的微分
    config.kf = 0.0F; // 前馈由 update() 加在角速度目标上，不用 PID 的参考前馈
    config.default_dt = DEFAULT_DT_SECONDS;
    config.maxout = gains.output_limit;
    config.feed_forward_limit = 0.0F;
    config.integral_limit = gains.integral_limit;
    return config;
}

} // namespace

bool AxisLoop::init(const Gains &angle, const Gains &rate) noexcept
{
    initialized_ = false;
    if (!gains_valid(angle) || !gains_valid(rate))
    {
        return false;
    }
    if (!angle_pid_.init(pid_config(angle)) || !rate_pid_.init(pid_config(rate)))
    {
        return false;
    }
    integral_before_ = 0.0F;
    torque_ = 0.0F;
    initialized_ = true;
    return true;
}

void AxisLoop::reset() noexcept
{
    angle_pid_.reset();
    rate_pid_.reset();
    integral_before_ = 0.0F;
    torque_ = 0.0F;
}

AxisLoop::Output
AxisLoop::update(float angle, float angle_target, float rate, float rate_feedforward, float dt) noexcept
{
    Output output{};
    if (!initialized_ || !std::isfinite(angle) || !std::isfinite(angle_target) || !std::isfinite(rate) ||
        !std::isfinite(rate_feedforward) || !std::isfinite(dt))
    {
        torque_ = 0.0F; // 非有限输入不往下传，环的内部状态不动（PID 失败时自己回退）
        return output;
    }
    if (dt <= 0.0F || dt >= MAXIMUM_DT_SECONDS)
    {
        dt = DEFAULT_DT_SECONDS;
    }
    integral_before_ = rate_pid_.get_integral_error();
    if (!angle_pid_.calculate_loop(angle, angle_target, dt))
    {
        torque_ = 0.0F;
        return output;
    }
    const auto &angle_gains = angle_pid_.get_config();
    const float rate_target =
        math::limit(angle_pid_.get_out() + rate_feedforward, -angle_gains.maxout, angle_gains.maxout);
    if (!rate_pid_.calculate_loop(rate, rate_target, dt))
    {
        torque_ = 0.0F;
        return output;
    }
    torque_ = rate_pid_.get_out();
    output.torque = torque_;
    output.diagnostics.rate_target = rate_target;
    output.diagnostics.compensation_newton_meter = rate_pid_.get_integral_error();
    return output;
}

void AxisLoop::apply(float torque) noexcept
{
    if (!initialized_ || !std::isfinite(torque))
    {
        return;
    }
    if (std::fabs(torque - torque_) <= TORQUE_COMPARE_TOLERANCE_NEWTON_METER)
    {
        return; // 没被总限幅削掉
    }
    const float increment = rate_pid_.get_integral_error() - integral_before_;
    if (increment * torque_ > 0.0F)
    {
        (void)rate_pid_.set_integral_error(integral_before_);
    }
}

} // namespace gimbal
