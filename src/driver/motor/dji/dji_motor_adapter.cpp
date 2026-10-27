#include "driver/motor/dji/dji_motor_adapter.hpp"
#include <cmath>

namespace device
{
DjiMotorAdapter::DjiMotorAdapter(const Config &config) noexcept : config_{config}, bus_{config.channel, config.profile}
{
}

bool DjiMotorAdapter::init() noexcept
{
    if (ready_)
    {
        return true;
    }
    const auto &profile = config_.profile;
    const auto &current = profile.dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::CURRENT)];
    if (static_cast<unsigned>(config_.channel) > static_cast<unsigned>(platform::Can::Channel::CHANNEL_3) ||
        config_.count == 0U || config_.count > config_.mappings.size() || current.group_one_control_identifier == 0U ||
        current.control_full_scale_counts == 0U || current.control_full_scale_counts > 32767U ||
        profile.dialect.maximum_device_count > DjiMotor::MAXIMUM_MOTORS ||
        !std::isfinite(profile.current_full_scale_ampere) || profile.current_full_scale_ampere <= 0.0F ||
        !std::isfinite(profile.torque_constant_newton_meter_per_ampere) ||
        profile.torque_constant_newton_meter_per_ampere <= 0.0F)
    {
        return false;
    }
    for (std::size_t id = 0U; id < config_.count; ++id)
    {
        const auto &mapping = config_.mappings[id];
        if (mapping.device_id == 0U || mapping.device_id > profile.dialect.maximum_device_count ||
            !std::isfinite(mapping.gear_ratio) || mapping.gear_ratio < 1.0F ||
            (mapping.direction != 1 && mapping.direction != -1) ||
            (mapping.device_id > protocol::DjiEsc::DEVICES_PER_FRAME && current.group_two_control_identifier == 0U))
        {
            return false;
        }
        for (std::size_t previous = 0U; previous < id; ++previous)
        {
            if (config_.mappings[previous].device_id == mapping.device_id)
            {
                return false;
            }
        }
    }
    if (!bus_.init())
    {
        return false;
    }
    for (std::size_t id = 0U; id < config_.count; ++id)
    {
        if (!bus_.enable_motor(config_.mappings[id].device_id))
        {
            bus_.clear_all_commands();
            return false;
        }
    }
    ready_ = true;
    return true;
}

void DjiMotorAdapter::process() noexcept
{
    if (ready_)
    {
        bus_.process();
    }
}

bool DjiMotorAdapter::read_state(std::size_t id, MotorState &state) const noexcept
{
    state = {};
    if (!ready_ || id >= config_.count)
    {
        return false;
    }
    DjiMotor::Snapshot snapshot{};
    const auto &mapping = config_.mappings[id];
    if (!bus_.get_snapshot(mapping.device_id, snapshot))
    {
        return false;
    }
    const float factor = static_cast<float>(mapping.direction) / mapping.gear_ratio;
    state.angle_degrees = snapshot.total_angle_degrees * factor;
    state.speed_rpm = static_cast<float>(snapshot.rotor_speed_rpm) * factor;
    state.feedback_valid = snapshot.ever_received;
    state.online = snapshot.online;
    state.last_update_tick = snapshot.last_update_tick;
    return true;
}

bool DjiMotorAdapter::set_torque(std::size_t id, float value) noexcept
{
    if (!ready_ || id >= config_.count)
    {
        return false;
    }
    const auto &mapping = config_.mappings[id];
    if (!std::isfinite(value))
    {
        const bool cleared = bus_.set_raw_command(mapping.device_id, 0);
        if (!cleared)
        {
            bus_.clear_all_commands();
        }
        return false;
    }
    // 档案 Kt 已是输出轴力矩口径，切勿再乘减速比。
    return bus_.set_torque(mapping.device_id, value * mapping.direction);
}

void DjiMotorAdapter::clear_commands() noexcept
{
    bus_.clear_all_commands();
}
} // namespace device
