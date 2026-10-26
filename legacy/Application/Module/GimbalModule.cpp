#include "Application/Module/GimbalModule.hpp"
#include <cmath>

namespace application::module
{

bool GimbalModule::init(const Config &config, float device_maximum_torque_newton_meter) noexcept
{
    axes_.reset();
    init_failure_ = InitFailure::NONE;

    if (!std::isfinite(device_maximum_torque_newton_meter) || device_maximum_torque_newton_meter <= 0.0F)
    {
        // 能力不可查询时按"无法校验"处理：宁可拒绝启动，也不让限幅成为假的约束。
        init_failure_ = InitFailure::TORQUE_LIMIT_EXCEEDS_DEVICE;
        return false;
    }
    device_maximum_torque_newton_meter_ = device_maximum_torque_newton_meter;

    if (!std::isfinite(config.max_torque_newton_meter) || config.max_torque_newton_meter <= 0.0F)
    {
        init_failure_ = InitFailure::TORQUE_LIMIT_INVALID;
        return false;
    }
    if (!std::isfinite(config.max_gimbal_speed_rpm) || config.max_gimbal_speed_rpm <= 0.0F)
    {
        init_failure_ = InitFailure::SPEED_LIMIT_INVALID;
        return false;
    }
    // D4b：策略总限幅不得超过执行器能力（否则"总限幅"形同虚设）。
    if (config.max_torque_newton_meter > device_maximum_torque_newton_meter_)
    {
        init_failure_ = InitFailure::TORQUE_LIMIT_EXCEEDS_DEVICE;
        return false;
    }
    for (std::size_t gimbal = 0U; gimbal < MOTOR_COUNT; ++gimbal)
    {
        const AxisConfig &axis = config.axis[gimbal];
        // 角度范围必须显式给出，否则忘配置会把所有目标静默钳到零。
        if (!std::isfinite(axis.min_angle_deg) || !std::isfinite(axis.max_angle_deg) ||
            axis.min_angle_deg >= axis.max_angle_deg)
        {
            init_failure_ = InitFailure::ANGLE_RANGE_INVALID;
            return false;
        }
        // 电调/热限幅同样不得超过执行器能力（两级限幅都是对执行器的约束）。
        if (std::isfinite(axis.speed_loop.maxout) && axis.speed_loop.maxout > device_maximum_torque_newton_meter_)
        {
            init_failure_ = InitFailure::SPEED_LIMIT_EXCEEDS_DEVICE;
            return false;
        }
    }

    // 映射到骨架配置：产品决策在此显式落地（D1 失败策略、D2 初始模式、逐轴目标钳位）。
    component::AxisController<MOTOR_COUNT>::Config mapped{};
    mapped.max_torque_newton_meter = config.max_torque_newton_meter;
    mapped.max_speed_rpm = config.max_gimbal_speed_rpm;
    mapped.initial_mode = GimbalControlMode::ANGLE;
    mapped.failure_policy = component::FailurePolicy::HOLD_TARGETS;
    for (std::size_t gimbal = 0U; gimbal < MOTOR_COUNT; ++gimbal)
    {
        mapped.axis[gimbal].speed_loop = config.axis[gimbal].speed_loop;
        mapped.axis[gimbal].angle_loop = config.axis[gimbal].angle_loop;
        mapped.axis[gimbal].clamp = component::TargetClamp::ABSOLUTE_RANGE;
        mapped.axis[gimbal].min_target = config.axis[gimbal].min_angle_deg;
        mapped.axis[gimbal].max_target = config.axis[gimbal].max_angle_deg;
    }
    if (!axes_.init(mapped))
    {
        init_failure_ = InitFailure::CONTROLLER_REJECTED;
        return false;
    }

    command_failure_count_ = 0U;
    return true;
}

void GimbalModule::reset() noexcept
{
    axes_.reset();
}

void GimbalModule::set_enabled(bool enabled) noexcept
{
    axes_.set_enabled(enabled);
}

void GimbalModule::set_mode(GimbalControlMode mode) noexcept
{
    axes_.set_mode(mode);
}

void GimbalModule::set_target(std::size_t gimbal, float value) noexcept
{
    axes_.set_target(gimbal, value);
}

void GimbalModule::on_command_failure() noexcept
{
    ++command_failure_count_;
    axes_.on_command_failure();
}

bool GimbalModule::is_ready() const noexcept
{
    return axes_.is_ready();
}

bool GimbalModule::is_enabled() const noexcept
{
    return axes_.is_enabled();
}

GimbalModule::GimbalControlMode GimbalModule::mode() const noexcept
{
    return axes_.get_mode();
}

float GimbalModule::target(std::size_t gimbal) const noexcept
{
    return axes_.get_target(gimbal);
}

GimbalModule::InitFailure GimbalModule::get_init_failure() const noexcept
{
    return init_failure_;
}

component::ControllerStatus GimbalModule::get_status() const noexcept
{
    return axes_.get_status();
}

bool GimbalModule::get_axis_status(std::size_t gimbal, component::AxisStatus &status) const noexcept
{
    return axes_.get_axis_status(gimbal, status);
}

std::uint32_t GimbalModule::get_loop_fault_count() const noexcept
{
    return axes_.get_loop_fault_count();
}

std::uint32_t GimbalModule::get_command_failure_count() const noexcept
{
    return command_failure_count_;
}

void GimbalModule::update(const component::AxisMeasurement (&measurements)[MOTOR_COUNT],
                          float dt,
                          float (&out_torque_newton_meter)[MOTOR_COUNT]) noexcept
{
    axes_.update(measurements, dt, out_torque_newton_meter);
}

} // namespace application::module
