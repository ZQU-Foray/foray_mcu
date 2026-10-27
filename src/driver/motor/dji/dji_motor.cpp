#include "Libraries/Device/motor/dji/DjiMotor.hpp"

namespace device
{

namespace
{

constexpr std::size_t SEND_RESULT_COUNT{6U};

} // namespace

DjiMotor::DjiMotor(platform::Can::Channel channel, const Profile &profile) noexcept
    : channel_{channel}, esc_{profile.dialect}, profile_{profile},
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
    port_statistics_ = {};
    feedback_ = {};
    published_snapshots_ = {};
    published_statistics_ = {};
    snapshot_sequence_.store(0U, std::memory_order_relaxed);
    dirty_ = false;

    const protocol::DjiEsc::CommandSpec &current =
        profile_.dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::CURRENT)];
    const protocol::DjiEsc::CommandSpec &voltage =
        profile_.dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::VOLTAGE)];
    if (current.group_one_control_identifier == 0U && voltage.group_one_control_identifier == 0U)
    {
        ready_ = false;
        return false;
    }

    if (!platform::Can::is_ready(channel_) || !platform::Time::is_ready())
    {
        ready_ = false;
        return false;
    }

    ready_ = true;
    dirty_ = true;
    publish();
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
    (void)platform::Can::get_statistics(channel_, port_statistics_);
    if (dirty_)
    {
        publish();
    }
}

bool DjiMotor::enable_motor(std::uint8_t device_id) noexcept
{
    if (!ready_ || !device_id_valid(device_id))
    {
        return false;
    }
    motors_[device_id - 1U].enabled = true;
    dirty_ = true;
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

    Snapshot staged{};
    std::uint32_t sequence_before = 0U;
    std::uint32_t sequence_after = 0U;
    do
    {
        sequence_before = snapshot_sequence_.load(std::memory_order_acquire);
        if ((sequence_before & 1U) != 0U)
        {
            continue;
        }
        staged = published_snapshots_[device_id - 1U];
        sequence_after = snapshot_sequence_.load(std::memory_order_acquire);
    } while (sequence_before != sequence_after || (sequence_after & 1U) != 0U);

    snapshot = staged;
    return true;
}

bool DjiMotor::get_statistics(Statistics &statistics) const noexcept
{
    Statistics staged{};
    std::uint32_t sequence_before = 0U;
    std::uint32_t sequence_after = 0U;
    do
    {
        sequence_before = snapshot_sequence_.load(std::memory_order_acquire);
        if ((sequence_before & 1U) != 0U)
        {
            continue;
        }
        staged = published_statistics_;
        sequence_after = snapshot_sequence_.load(std::memory_order_acquire);
    } while (sequence_before != sequence_after || (sequence_after & 1U) != 0U);

    statistics = staged;
    return true;
}

void DjiMotor::receive_frames() noexcept
{
    platform::Can::Frame frame{};
    for (;;)
    {
        const platform::Can::ReceiveResult result = platform::Can::try_receive(channel_, frame);
        if (result != platform::Can::ReceiveResult::RECEIVED)
        {
            return;
        }

        if (esc_.decode_feedback(frame.identifier,
                                 frame.identifier_type == platform::Can::IdentifierType::EXTENDED,
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
        motor.last_update_tick = platform::Time::now_ticks();
        motor.ever_received = true;
        motor.stale = false;
        ++motor.accepted_frame_count;
        dirty_ = true;
    }
}

void DjiMotor::update_link_aging() noexcept
{
    for (MotorState &motor : motors_)
    {
        if (!motor.ever_received || motor.stale)
        {
            continue;
        }
        if (platform::Time::has_elapsed_ms(motor.last_update_tick, LINK_TIMEOUT_MS))
        {
            motor.stale = true;
            dirty_ = true;
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

        platform::Can::Frame frame{};
        frame.identifier = command.identifier;
        frame.identifier_type = platform::Can::IdentifierType::STANDARD;
        frame.length = static_cast<std::uint8_t>(command.data.size());
        frame.data = command.data;
        const platform::Can::SendResult result = platform::Can::try_send(channel_, frame);
        const std::size_t index = static_cast<std::size_t>(result) < SEND_RESULT_COUNT
                                      ? static_cast<std::size_t>(result)
                                      : SEND_RESULT_COUNT - 1U;
        ++tx_result_counts_[index];
        dirty_ = true;
    }
}

void DjiMotor::publish() noexcept
{
    snapshot_sequence_.fetch_add(1U, std::memory_order_acq_rel);

    for (std::size_t index = 0U; index < motors_.size(); ++index)
    {
        fill_snapshot(index, published_snapshots_[index]);
    }

    const protocol::DjiEsc::Statistics &esc_statistics = esc_.get_statistics();
    published_statistics_.tx_result_counts = tx_result_counts_;
    published_statistics_.accepted_frame_count = esc_statistics.accepted_frame_count;
    published_statistics_.rejected_frame_count = esc_statistics.rejected_frame_count;
    published_statistics_.unknown_identifier_count = esc_statistics.unknown_identifier_count;
    published_statistics_.rx_dropped_count = port_statistics_.rx_dropped_count;
    published_statistics_.rx_hardware_loss_event_count = port_statistics_.rx_hardware_loss_event_count;
    published_statistics_.bus_off_count = port_statistics_.bus_off_count;

    snapshot_sequence_.fetch_add(1U, std::memory_order_release);
    dirty_ = false;
}

void DjiMotor::fill_snapshot(std::size_t index, Snapshot &snapshot) const noexcept
{
    const MotorState &motor = motors_[index];

    snapshot = Snapshot{};
    snapshot.device_id = static_cast<std::uint8_t>(index + 1U);
    snapshot.enabled = motor.enabled;
    snapshot.ever_received = motor.ever_received;
    snapshot.online = motor.ever_received && !motor.stale;
    snapshot.last_update_tick = motor.last_update_tick;
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
