#include "driver/motor/dji/dji_motor.hpp"

namespace device
{

DjiMotor::DjiMotor(hal::Can bus, const Profile &profile) noexcept
    : bus_{bus}, esc_{profile.dialect}, profile_{profile},
      command_kind_{profile.dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::CURRENT)]
                                .group_one_control_identifier != 0U
                        ? protocol::DjiEsc::CommandKind::CURRENT
                        : protocol::DjiEsc::CommandKind::VOLTAGE}
{
}

bool DjiMotor::init() noexcept
{
    motors_ = {};
    tx_result_counts_ = {};
    bus_statistics_ = {};
    feedback_ = {};

    const protocol::DjiEsc::CommandSpec &current =
        profile_.dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::CURRENT)];
    const protocol::DjiEsc::CommandSpec &voltage =
        profile_.dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::VOLTAGE)];
    if (current.group_one_control_identifier == 0U && voltage.group_one_control_identifier == 0U)
    {
        ready_ = false;
        return false;
    }

    // 这条总线的接收队列只能有一个消费者。最后才占用：失败的实例不占住总线；重复 init() 时不再占用
    if (!bus_.is_ready() || (!receiver_claimed_ && !bus_.claim_receiver()))
    {
        ready_ = false;
        return false;
    }
    receiver_claimed_ = true;

    ready_ = true;
    return true;
}

void DjiMotor::process() noexcept
{
    if (!ready_)
    {
        return;
    }

    receive_frames();
    update_link_aging();
    transmit_commands();
    bus_statistics_ = bus_.statistics();
}

bool DjiMotor::enable_motor(std::uint8_t device_id) noexcept
{
    if (!ready_ || !device_id_valid(device_id))
    {
        return false;
    }
    motors_[device_id - 1U].enabled = true;
    return true;
}

bool DjiMotor::set_raw_command(std::uint8_t device_id, std::int16_t raw_command) noexcept
{
    if (!device_id_valid(device_id))
    {
        return false;
    }
    motors_[device_id - 1U].command = esc_.saturate(raw_command, command_kind_);
    return true;
}

bool DjiMotor::set_current_ampere(std::uint8_t device_id, float ampere) noexcept
{
    if (!device_id_valid(device_id) || profile_.current_full_scale_ampere <= 0.0F)
    {
        return false;
    }

    float ratio = ampere / profile_.current_full_scale_ampere;
    if (ratio > 1.0F)
    {
        ratio = 1.0F;
    }
    else if (ratio < -1.0F)
    {
        ratio = -1.0F;
    }
    const std::int16_t full_scale_counts =
        static_cast<std::int16_t>(esc_.get_command_spec(command_kind_).control_full_scale_counts);
    return set_raw_command(device_id, static_cast<std::int16_t>(ratio * full_scale_counts));
}

bool DjiMotor::set_torque(std::uint8_t device_id, float newton_meter) noexcept
{
    if (profile_.torque_constant_newton_meter_per_ampere <= 0.0F)
    {
        return false;
    }
    return set_current_ampere(device_id, newton_meter / profile_.torque_constant_newton_meter_per_ampere);
}

bool DjiMotor::set_torque_ratio(std::uint8_t device_id, float ratio) noexcept
{
    if (!device_id_valid(device_id))
    {
        return false;
    }
    if (ratio > 1.0F)
    {
        ratio = 1.0F;
    }
    else if (ratio < -1.0F)
    {
        ratio = -1.0F;
    }
    const std::int16_t full_scale_counts =
        static_cast<std::int16_t>(esc_.get_command_spec(command_kind_).control_full_scale_counts);
    return set_raw_command(device_id, static_cast<std::int16_t>(ratio * full_scale_counts));
}

void DjiMotor::clear_all_commands() noexcept
{
    for (MotorState &motor : motors_)
    {
        motor.command = 0;
    }
}

bool DjiMotor::get_snapshot(std::uint8_t device_id, Snapshot &snapshot) const noexcept
{
    if (device_id == 0U || device_id > MAXIMUM_MOTORS)
    {
        return false;
    }
    fill_snapshot(device_id - 1U, snapshot);
    return true;
}

bool DjiMotor::get_statistics(Statistics &statistics) const noexcept
{
    const protocol::DjiEsc::Statistics &esc_statistics = esc_.get_statistics();
    statistics.tx_result_counts = tx_result_counts_;
    statistics.accepted_frame_count = esc_statistics.accepted_frame_count;
    statistics.rejected_frame_count = esc_statistics.rejected_frame_count;
    statistics.unknown_identifier_count = esc_statistics.unknown_identifier_count;
    statistics.rx_dropped_count = bus_statistics_.rx_dropped_count;
    statistics.rx_hardware_loss_event_count = bus_statistics_.rx_hardware_loss_event_count;
    statistics.bus_off_count = bus_statistics_.bus_off_count;
    return true;
}

void DjiMotor::receive_frames() noexcept
{
    hal::Can::Frame frame{};
    while (bus_.receive(frame) == hal::Can::ReceiveResult::RECEIVED)
    {
        if (esc_.decode_feedback(frame.identifier,
                                 frame.identifier_type == hal::Can::IdentifierType::EXTENDED,
                                 frame.length,
                                 frame.data.data(),
                                 feedback_) != protocol::DjiEsc::DecodeResult::ACCEPTED)
        {
            continue;
        }

        const std::size_t index = feedback_.device_id - 1U;
        if (index >= motors_.size())
        {
            continue;
        }

        MotorState &motor = motors_[index];
        if (motor.has_angle)
        {
            std::int32_t delta =
                static_cast<std::int32_t>(feedback_.rotor_angle_raw) - static_cast<std::int32_t>(motor.last_angle_raw);
            constexpr std::int32_t COUNTS = protocol::DjiEsc::ANGLE_COUNTS_PER_REVOLUTION;
            // 恰好半圈时保留符号；该速率下方向本身有歧义。
            if (delta > COUNTS / 2)
            {
                delta -= COUNTS;
            }
            else if (delta < -COUNTS / 2)
            {
                delta += COUNTS;
            }
            motor.total_angle_counts += delta;
        }
        else
        {
            motor.has_angle = true;
        }
        motor.last_angle_raw = feedback_.rotor_angle_raw;
        motor.feedback = feedback_;
        motor.last_update_us = frame.received_us; // 帧进入接收队列的时刻，不是取出的时刻（Decision-20261102-04）
        motor.ever_received = true;
        motor.stale = false;
        ++motor.accepted_frame_count;
    }
}

void DjiMotor::update_link_aging() noexcept
{
    const hal::os::Microseconds now = hal::os::now_us();
    for (MotorState &motor : motors_)
    {
        if (motor.ever_received && !motor.stale && hal::os::has_elapsed(motor.last_update_us, LINK_TIMEOUT_US, now))
        {
            motor.stale = true;
        }
    }
}

void DjiMotor::transmit_commands() noexcept
{
    const protocol::DjiEsc::Group groups[]{protocol::DjiEsc::Group::FIRST, protocol::DjiEsc::Group::SECOND};
    for (const protocol::DjiEsc::Group group : groups)
    {
        if (!group_enabled(group))
        {
            continue;
        }

        std::array<std::int16_t, protocol::DjiEsc::DEVICES_PER_FRAME> targets{};
        for (std::size_t slot = 0U; slot < targets.size(); ++slot)
        {
            const std::size_t index =
                group == protocol::DjiEsc::Group::FIRST ? slot : slot + protocol::DjiEsc::DEVICES_PER_FRAME;
            targets[slot] = motors_[index].command;
        }

        protocol::DjiEsc::ControlFrame command{};
        if (!esc_.encode(command_kind_, group, targets, command))
        {
            continue;
        }

        hal::Can::Frame frame{};
        frame.identifier = command.identifier;
        frame.identifier_type = hal::Can::IdentifierType::STANDARD;
        frame.length = static_cast<std::uint8_t>(command.data.size());
        frame.data = command.data;
        const std::size_t result = static_cast<std::size_t>(bus_.send(frame));
        ++tx_result_counts_[result < tx_result_counts_.size() ? result : tx_result_counts_.size() - 1U];
    }
}

void DjiMotor::fill_snapshot(std::size_t index, Snapshot &snapshot) const noexcept
{
    const MotorState &motor = motors_[index];

    snapshot = Snapshot{};
    snapshot.device_id = static_cast<std::uint8_t>(index + 1U);
    snapshot.enabled = motor.enabled;
    snapshot.ever_received = motor.ever_received;
    snapshot.online = motor.ever_received && !motor.stale;
    snapshot.last_update_us = motor.last_update_us;
    snapshot.accepted_frame_count = motor.accepted_frame_count;
    snapshot.rotor_angle_raw = motor.feedback.rotor_angle_raw;
    snapshot.rotor_angle_degrees = protocol::DjiEsc::rotor_angle_degrees(motor.feedback.rotor_angle_raw);
    snapshot.total_angle_counts = motor.total_angle_counts;
    snapshot.total_turns =
        motor.total_angle_counts / static_cast<std::int32_t>(protocol::DjiEsc::ANGLE_COUNTS_PER_REVOLUTION);
    snapshot.total_angle_degrees = static_cast<float>(motor.total_angle_counts) * 360.0F /
                                   static_cast<float>(protocol::DjiEsc::ANGLE_COUNTS_PER_REVOLUTION);
    snapshot.rotor_speed_rpm = motor.feedback.rotor_speed_raw;
    snapshot.torque_current_raw = motor.feedback.torque_current_raw;
    snapshot.torque_current_ratio =
        protocol::DjiEsc::control_ratio(motor.feedback.torque_current_raw, esc_.get_command_spec(command_kind_));
    snapshot.motor_temperature_celsius =
        profile_.has_temperature_feedback ? motor.feedback.motor_temperature_celsius : 0U;
}

bool DjiMotor::device_id_valid(std::uint8_t device_id) const noexcept
{
    return device_id >= 1U && device_id <= maximum_device_count();
}

bool DjiMotor::group_enabled(protocol::DjiEsc::Group group) const noexcept
{
    const std::size_t first = group == protocol::DjiEsc::Group::FIRST ? 0U : protocol::DjiEsc::DEVICES_PER_FRAME;
    for (std::size_t index = first; index < first + protocol::DjiEsc::DEVICES_PER_FRAME && index < motors_.size();
         ++index)
    {
        if (motors_[index].enabled)
        {
            return true;
        }
    }
    return false;
}

} // namespace device
