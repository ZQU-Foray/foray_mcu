#include "Application/Task/1_chassis/ChassisController.hpp"

namespace application::chassis
{

bool ChassisController::init(const Config &config) noexcept
{
    component::AxisController<MOTOR_COUNT>::Config mapped{};
    mapped.max_torque_newton_meter = config.max_torque_newton_meter;
    mapped.max_speed_rpm = config.max_wheel_speed_rpm;
    // D2：底盘默认转速模式（目标 0 即停转，比角度闭环更保守）。
    mapped.initial_mode = WheelControlMode::SPEED;
    // D1：要求底盘运动目标立即归零，失败后必须由上层重新下发。
    mapped.failure_policy = component::FailurePolicy::CLEAR_TARGETS;
    for (std::size_t wheel = 0U; wheel < MOTOR_COUNT; ++wheel)
    {
        mapped.axis[wheel].speed_loop = config.speed_loop;
        mapped.axis[wheel].angle_loop = config.angle_loop;
        mapped.axis[wheel].clamp = component::TargetClamp::NONE;
    }
    return axes_.init(mapped);
}

void ChassisController::reset() noexcept
{
    axes_.reset();
}

void ChassisController::set_enabled(bool enabled) noexcept
{
    axes_.set_enabled(enabled);
}

void ChassisController::set_mode(WheelControlMode mode) noexcept
{
    axes_.set_mode(mode);
}

void ChassisController::set_wheel_target(std::size_t wheel, float value) noexcept
{
    axes_.set_target(wheel, value);
}

void ChassisController::on_command_failure() noexcept
{
    axes_.on_command_failure();
}

bool ChassisController::is_ready() const noexcept
{
    return axes_.is_ready();
}

bool ChassisController::is_enabled() const noexcept
{
    return axes_.is_enabled();
}

ChassisController::WheelControlMode ChassisController::mode() const noexcept
{
    return axes_.get_mode();
}

float ChassisController::wheel_target(std::size_t wheel) const noexcept
{
    return axes_.get_target(wheel);
}

component::ControllerStatus ChassisController::get_status() const noexcept
{
    return axes_.get_status();
}

bool ChassisController::get_axis_status(std::size_t wheel, component::AxisStatus &status) const noexcept
{
    return axes_.get_axis_status(wheel, status);
}

std::uint32_t ChassisController::get_loop_fault_count() const noexcept
{
    return axes_.get_loop_fault_count();
}

void ChassisController::update(const device::MotorState (&snapshots)[MOTOR_COUNT],
                               float dt,
                               float (&out_torque_newton_meter)[MOTOR_COUNT]) noexcept
{
    // 设备快照 → 骨架量测：一次 4 字段拷贝，换取算法层不依赖设备层。
    component::AxisMeasurement measurements[MOTOR_COUNT]{};
    for (std::size_t wheel = 0U; wheel < MOTOR_COUNT; ++wheel)
    {
        measurements[wheel].angle_degrees = snapshots[wheel].angle_degrees;
        measurements[wheel].speed_rpm = snapshots[wheel].speed_rpm;
        measurements[wheel].feedback_valid = snapshots[wheel].feedback_valid;
        measurements[wheel].online = snapshots[wheel].online;
    }
    axes_.update(measurements, dt, out_torque_newton_meter);
}

} // namespace application::chassis
