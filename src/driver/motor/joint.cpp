#include "driver/motor/joint.hpp"

#include <cmath>

namespace device
{

namespace
{

bool finite(float value)
{
    return std::isfinite(value);
}

} // namespace

bool Joint::enable() noexcept
{
    if (!available())
    {
        return false;
    }
    if (!enable_requested_)
    {
        enable_requested_ = true;
        clear_command();
    }
    return true;
}

void Joint::disable() noexcept
{
    enable_requested_ = false;
    clear_command();
}

bool Joint::set_torque(float newton_meter) noexcept
{
    if (!finite(newton_meter))
    {
        clear_command();
        return false;
    }
    if (!enable_requested_ || inhibited_)
    {
        return false;
    }
    command_ = JointCommand{};
    command_.torque_newton_meter = newton_meter;
    command_stamp_us_ = hal::os::now_us();
    has_command_ = true;
    timed_out_ = false;
    return true;
}

bool Joint::set_impedance(float position_rad,
                          float velocity_rad_per_second,
                          float stiffness,
                          float damping,
                          float torque_newton_meter) noexcept
{
    if (!finite(position_rad) || !finite(velocity_rad_per_second) || !finite(stiffness) || !finite(damping) ||
        !finite(torque_newton_meter) || stiffness < 0.0F || damping < 0.0F)
    {
        clear_command();
        return false;
    }
    if (!supports(JointMode::IMPEDANCE) || !enable_requested_ || inhibited_)
    {
        return false;
    }
    command_ = JointCommand{
        JointMode::IMPEDANCE, torque_newton_meter, position_rad, velocity_rad_per_second, stiffness, damping};
    command_stamp_us_ = hal::os::now_us();
    has_command_ = true;
    timed_out_ = false;
    return true;
}

void Joint::set_inhibited(bool inhibited) noexcept
{
    inhibited_ = inhibited;
    if (inhibited)
    {
        enable_requested_ = false;
        clear_command();
    }
}

void Joint::clear_command() noexcept
{
    command_ = JointCommand{};
    has_command_ = false;
    timed_out_ = false;
}

bool JointLink::bind(Joint &joint, const Binding &binding) noexcept
{
    const JointMechanics &mechanics = binding.mechanics;
    if (joint.bound_ || (mechanics.direction != 1 && mechanics.direction != -1) || !finite(mechanics.gear_ratio) ||
        mechanics.gear_ratio <= 0.0F || !finite(mechanics.zero_offset_rad) ||
        (binding.modes & Joint::mode_bit(JointMode::TORQUE)) == 0U ||
        !finite(binding.motor_maximum_torque_newton_meter) || binding.motor_maximum_torque_newton_meter < 0.0F)
    {
        return false;
    }
    joint.mechanics_ = mechanics;
    joint.modes_ = binding.modes;
    joint.maximum_torque_newton_meter_ = binding.motor_maximum_torque_newton_meter * mechanics.gear_ratio;
    joint.state_ = JointState{};
    joint.state_.position_absolute = binding.position_absolute;
    joint.state_.has_temperature = binding.has_temperature;
    joint.bound_ = true;
    return true;
}

void JointLink::set_available(Joint &joint, bool available) noexcept
{
    joint.available_ = available;
    if (!available)
    {
        drop_enable(joint);
    }
}

void JointLink::publish(Joint &joint, const MotorFeedback &feedback) noexcept
{
    const JointMechanics &mechanics = joint.mechanics_;
    const auto direction = static_cast<float>(mechanics.direction);
    JointState &state = joint.state_;
    state.position_rad = direction * (feedback.position_rad - mechanics.zero_offset_rad) / mechanics.gear_ratio;
    state.velocity_rad_per_second = direction * feedback.velocity_rad_per_second / mechanics.gear_ratio;
    state.torque_newton_meter = direction * feedback.torque_newton_meter * mechanics.gear_ratio;
    state.temperature_celsius = feedback.temperature_celsius;
    state.stamp_us = feedback.stamp_us;
    state.feedback_valid = true;
}

void JointLink::set_status(Joint &joint, bool online, bool enabled, bool fault, std::uint8_t fault_code) noexcept
{
    joint.state_.online = online;
    joint.state_.enabled = enabled;
    joint.state_.fault = fault;
    joint.state_.fault_code = fault_code;
}

void JointLink::drop_enable(Joint &joint) noexcept
{
    joint.enable_requested_ = false;
    joint.clear_command();
}

JointCommand JointLink::command(Joint &joint, hal::os::Microseconds now) noexcept
{
    if (!enable_requested(joint) || !joint.has_command_)
    {
        return JointCommand{};
    }
    if (hal::os::has_elapsed(joint.command_stamp_us_, joint.mechanics_.command_timeout_us, now))
    {
        if (!joint.timed_out_)
        {
            joint.timed_out_ = true;
            ++joint.command_timeout_count_;
        }
        return JointCommand{};
    }

    // 关节单位 → 电机输出轴单位：p = zero + d·g·q，v = d·g·q̇，τ = d·τj / g，刚度与阻尼除以 g²
    const JointMechanics &mechanics = joint.mechanics_;
    const auto direction = static_cast<float>(mechanics.direction);
    const float gear = mechanics.gear_ratio;
    const JointCommand &input = joint.command_;
    JointCommand output{};
    output.mode = input.mode;
    output.torque_newton_meter = direction * input.torque_newton_meter / gear;
    if (input.mode == JointMode::IMPEDANCE)
    {
        output.position_rad = mechanics.zero_offset_rad + direction * gear * input.position_rad;
        output.velocity_rad_per_second = direction * gear * input.velocity_rad_per_second;
        output.stiffness = input.stiffness / (gear * gear);
        output.damping = input.damping / (gear * gear);
    }
    return output;
}

bool enable_all(Joint *const *joints, std::size_t count) noexcept
{
    if (joints == nullptr)
    {
        return false;
    }
    for (std::size_t index = 0U; index < count; ++index)
    {
        if (joints[index] == nullptr || !joints[index]->available())
        {
            return false;
        }
    }
    bool enabled = true;
    for (std::size_t index = 0U; index < count; ++index)
    {
        enabled = joints[index]->enable() && enabled;
    }
    return enabled;
}

void disable_all(Joint *const *joints, std::size_t count) noexcept
{
    if (joints == nullptr)
    {
        return;
    }
    for (std::size_t index = 0U; index < count; ++index)
    {
        if (joints[index] != nullptr)
        {
            joints[index]->disable();
        }
    }
}

} // namespace device
