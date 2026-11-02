#include "driver/motor/dm/dm_motor_adapter.hpp"

#include <cmath>

namespace device
{

namespace
{

constexpr float PI{3.14159265358979F};
constexpr float DEGREES_PER_RADIAN{180.0F / PI};
constexpr float RPM_PER_RADIAN_PER_SECOND{60.0F / (2.0F * PI)};

} // namespace

DmMotorAdapter::DmMotorAdapter(const Config &config) noexcept : config_{config}, motors_{motor_config(config)} {}

DmMotor::Config DmMotorAdapter::motor_config(const Config &config) noexcept
{
    DmMotor::Config result{config.bus, config.profile, {}, config.count};
    for (std::size_t index = 0U; index < config.count && index < result.motors.size(); ++index)
    {
        result.motors[index] = {config.mappings[index].esc_id, config.mappings[index].master_id};
    }
    return result;
}

bool DmMotorAdapter::init() noexcept
{
    if (ready_)
    {
        return true;
    }
    if (config_.count == 0U || config_.count > config_.mappings.size())
    {
        return false;
    }
    for (std::size_t id = 0U; id < config_.count; ++id)
    {
        const int direction = config_.mappings[id].direction;
        if (direction != 1 && direction != -1)
        {
            return false;
        }
    }
    // ESC_ID、MST_ID 与档案由驱动检查
    if (!motors_.init())
    {
        return false;
    }
    ready_ = true;
    return true;
}

void DmMotorAdapter::process() noexcept
{
    if (ready_)
    {
        motors_.process();
    }
}

bool DmMotorAdapter::enable() noexcept
{
    if (!ready_)
    {
        return false;
    }
    for (std::size_t id = 0U; id < config_.count; ++id)
    {
        if (!motors_.can_enable(id))
        {
            return false;
        }
    }
    bool enabled = true;
    for (std::size_t id = 0U; id < config_.count; ++id)
    {
        enabled = motors_.enable(id) && enabled;
    }
    return enabled;
}

void DmMotorAdapter::disable() noexcept
{
    for (std::size_t id = 0U; id < config_.count; ++id)
    {
        (void)motors_.disable(id);
    }
}

bool DmMotorAdapter::read_state(std::size_t id, MotorState &state) const noexcept
{
    state = {};
    DmMotor::Snapshot snapshot{};
    if (!ready_ || id >= config_.count || !motors_.get_snapshot(id, snapshot))
    {
        return false;
    }
    const auto direction = static_cast<float>(config_.mappings[id].direction);
    state.angle_degrees = snapshot.position_from_first_feedback_rad * DEGREES_PER_RADIAN * direction;
    state.speed_rpm = snapshot.velocity_rad_per_second * RPM_PER_RADIAN_PER_SECOND * direction;
    state.feedback_valid = snapshot.has_feedback;
    state.online = snapshot.online;
    state.last_update_us = snapshot.last_update_us;
    state.enabled = snapshot.output_active;
    state.fault =
        snapshot.phase == DmMotor::Phase::MISMATCH || snapshot.enable_failed || DmMotor::is_fault(snapshot.state);
    return true;
}

bool DmMotorAdapter::set_torque(std::size_t id, float value) noexcept
{
    if (!ready_ || id >= config_.count)
    {
        return false;
    }
    if (!std::isfinite(value))
    {
        (void)motors_.set_torque(id, 0.0F);
        return false;
    }
    return motors_.set_torque(id, value * static_cast<float>(config_.mappings[id].direction));
}

void DmMotorAdapter::clear_commands() noexcept
{
    motors_.clear_all_commands();
}

float DmMotorAdapter::maximum_torque_newton_meter() const noexcept
{
    return ready_ ? motors_.maximum_torque_newton_meter() : 0.0F;
}

} // namespace device
