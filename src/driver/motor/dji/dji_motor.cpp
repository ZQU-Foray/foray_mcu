#include "driver/motor/dji/dji_motor.hpp"

#include <cmath>

namespace device
{

namespace
{

using Dji = protocol::DjiEsc;

constexpr float TWO_PI{6.28318530717959F};
constexpr std::int32_t COUNTS_PER_REVOLUTION{Dji::ANGLE_COUNTS_PER_REVOLUTION};
constexpr float SECONDS_PER_MINUTE{60.0F};

/**
 * 从 stamp 起是否已过 duration。stamp 可能比 now 晚一点（帧在这一轮开始之后才收到），无符号相减会回绕成很大的数、
 * 误判为离线，所以用有符号差，这时按"刚刚"算（2026-11-02 上板在达妙驱动上发现的同类问题）。
 */
bool expired(hal::os::Microseconds stamp, hal::os::Microseconds duration, hal::os::Microseconds now)
{
    return static_cast<std::int32_t>(now - stamp) >= static_cast<std::int32_t>(duration);
}

const Dji::CommandSpec &current_spec(const DjiMotor::Model &model)
{
    return model.profile.dialect.commands[static_cast<std::size_t>(Dji::CommandKind::CURRENT)];
}

bool model_valid(const DjiMotor::Model &model)
{
    const DjiMotor::Profile &profile = model.profile;
    const Dji::CommandSpec &current = current_spec(model);
    return current.group_one_control_identifier != 0U && current.control_full_scale_counts != 0U &&
           current.control_full_scale_counts <= 32767U && std::isfinite(profile.current_full_scale_ampere) &&
           profile.current_full_scale_ampere > 0.0F && std::isfinite(profile.torque_constant_newton_meter_per_ampere) &&
           profile.torque_constant_newton_meter_per_ampere > 0.0F && std::isfinite(model.gear_ratio) &&
           model.gear_ratio > 0.0F;
}

bool mechanics_valid(const JointMechanics &mechanics)
{
    return (mechanics.direction == 1 || mechanics.direction == -1) && std::isfinite(mechanics.gear_ratio) &&
           mechanics.gear_ratio > 0.0F && std::isfinite(mechanics.zero_offset_rad);
}

} // namespace

bool DjiMotor::init() noexcept
{
    ready_ = false;
    if (!config_valid() || !plan() || !attach())
    {
        return false;
    }
    statistics_ = {};
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        const Slot planned = slots_[index];
        slots_[index] = Slot{};
        slots_[index].feedback_identifier = planned.feedback_identifier;
        slots_[index].frame = planned.frame;
        slots_[index].slot = planned.slot;
    }
    ready_ = true;
    return true;
}

bool DjiMotor::config_valid() const noexcept
{
    if (config_.count == 0U || config_.count > MAXIMUM_MOTORS)
    {
        return false;
    }
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        const Motor &motor = config_.motors[index];
        std::uint32_t identifier = 0U;
        std::size_t slot = 0U;
        if (motor.joint == nullptr || (!attached_ && motor.joint->supports(JointMode::TORQUE)) ||
            !model_valid(motor.model) || !mechanics_valid(motor.mechanics) || !control_slot(motor, identifier, slot))
        {
            return false;
        }
        for (std::size_t other = 0U; other < index; ++other)
        {
            if (config_.motors[other].joint == motor.joint)
            {
                return false;
            }
        }
    }
    return true;
}

bool DjiMotor::plan() noexcept
{
    // 每台电机的反馈帧 ID、控制帧与格；反馈 ID 相同或占了同一帧的同一格都是冲突
    std::array<ControlFrame, MAXIMUM_CONTROL_FRAMES> frames{};
    std::size_t frame_count = 0U;
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        const Motor &motor = config_.motors[index];
        const std::uint32_t feedback_identifier =
            motor.model.profile.dialect.feedback_identifier_base + motor.device_id;
        for (std::size_t other = 0U; other < index; ++other)
        {
            if (slots_[other].feedback_identifier == feedback_identifier)
            {
                return false;
            }
        }
        std::uint32_t identifier = 0U;
        std::size_t slot = 0U;
        (void)control_slot(motor, identifier, slot); // config_valid() 已经检查过
        std::size_t frame = 0U;
        while (frame < frame_count && frames[frame].identifier != identifier)
        {
            ++frame;
        }
        if (frame == frame_count)
        {
            if (frame_count == frames.size())
            {
                return false;
            }
            frames[frame_count++].identifier = identifier;
        }
        if (frames[frame].motor[slot] >= 0)
        {
            return false;
        }
        frames[frame].motor[slot] = static_cast<std::int8_t>(index);
        slots_[index].feedback_identifier = feedback_identifier;
        slots_[index].frame = static_cast<std::uint8_t>(frame);
        slots_[index].slot = static_cast<std::uint8_t>(slot);
    }
    frames_ = frames;
    frame_count_ = frame_count;
    return true;
}

bool DjiMotor::attach() noexcept
{
    if (attached_)
    {
        return true;
    }
    // 反馈帧 ID 与本组用到的控制帧 ID 都登记：后者让同一条总线上的第二个 DJI 实例登记失败（两份同 ID 的控制帧会互相覆盖）
    std::array<CanBus::Filter, MAXIMUM_MOTORS + MAXIMUM_CONTROL_FRAMES> filters{};
    std::size_t filter_count = 0U;
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        const std::uint32_t identifier = slots_[index].feedback_identifier;
        filters[filter_count++] = CanBus::Filter{identifier, identifier, hal::Can::IdentifierType::STANDARD};
    }
    for (std::size_t frame = 0U; frame < frame_count_; ++frame)
    {
        const std::uint32_t identifier = frames_[frame].identifier;
        filters[filter_count++] = CanBus::Filter{identifier, identifier, hal::Can::IdentifierType::STANDARD};
    }
    if (!bus_.attach(CanBus::Client{this, on_frame, on_transmit}, filters.data(), filter_count))
    {
        return false;
    }
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        const Motor &motor = config_.motors[index];
        JointLink::Binding binding{};
        binding.modes = JointLink::TORQUE_ONLY;
        binding.motor_maximum_torque_newton_meter = motor.model.maximum_torque_newton_meter();
        binding.position_absolute = motor.model.position_absolute;
        binding.has_temperature = motor.model.profile.has_temperature_feedback;
        binding.mechanics = motor.mechanics;
        (void)JointLink::bind(*motor.joint, binding); // config_valid() 已经检查过
        JointLink::set_available(*motor.joint, true); // DJI 电调没有使能命令：接上即可出力
    }
    attached_ = true;
    return true;
}

void DjiMotor::on_frame(void *context, const hal::Can::Frame &frame) noexcept
{
    static_cast<DjiMotor *>(context)->receive(frame);
}

void DjiMotor::on_transmit(void *context, hal::os::Microseconds now) noexcept
{
    static_cast<DjiMotor *>(context)->transmit(now);
}

void DjiMotor::receive(const hal::Can::Frame &frame) noexcept
{
    if (!ready_)
    {
        return;
    }
    for (std::size_t control = 0U; control < frame_count_; ++control)
    {
        if (frames_[control].identifier == frame.identifier)
        {
            ++statistics_.foreign_control_frame_count;
            return;
        }
    }
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        if (slots_[index].feedback_identifier != frame.identifier)
        {
            continue;
        }
        Dji::Feedback feedback{};
        if (frame.identifier_type != hal::Can::IdentifierType::STANDARD ||
            !Dji::decode_feedback_data(frame.length, frame.data.data(), feedback))
        {
            ++statistics_.rejected_frame_count;
            return;
        }
        feedback.device_id = config_.motors[index].device_id;
        accept_feedback(index, feedback, frame.received_us);
        return;
    }
}

void DjiMotor::accept_feedback(std::size_t index,
                               const protocol::DjiEsc::Feedback &feedback,
                               hal::os::Microseconds received_us) noexcept
{
    Slot &slot = slots_[index];
    const Model &model = config_.motors[index].model;
    if (slot.ever_received)
    {
        std::int32_t delta = static_cast<std::int32_t>(feedback.rotor_angle_raw) -
                             static_cast<std::int32_t>(slot.feedback.rotor_angle_raw);
        // 恰好半圈时保留符号；该速率下方向本身有歧义
        if (delta > COUNTS_PER_REVOLUTION / 2)
        {
            delta -= COUNTS_PER_REVOLUTION;
        }
        else if (delta < -COUNTS_PER_REVOLUTION / 2)
        {
            delta += COUNTS_PER_REVOLUTION;
        }
        slot.total_counts += delta;
    }
    else
    {
        // 绝对型号（GM6020）从上电时的角度起算，其余从 0 起算
        slot.total_counts = model.position_absolute ? static_cast<std::int32_t>(feedback.rotor_angle_raw) : 0;
        slot.ever_received = true;
    }
    slot.feedback = feedback;
    slot.last_update_us = received_us;
    ++slot.feedback_count;
    ++statistics_.feedback_count;

    const Profile &profile = model.profile;
    const float full_scale_counts = static_cast<float>(current_spec(model).control_full_scale_counts);
    MotorFeedback motor_feedback{};
    motor_feedback.position_rad =
        static_cast<float>(slot.total_counts) * TWO_PI / static_cast<float>(COUNTS_PER_REVOLUTION) / model.gear_ratio;
    motor_feedback.velocity_rad_per_second =
        static_cast<float>(feedback.rotor_speed_raw) * TWO_PI / SECONDS_PER_MINUTE / model.gear_ratio;
    motor_feedback.torque_newton_meter = static_cast<float>(feedback.torque_current_raw) / full_scale_counts *
                                         profile.current_full_scale_ampere *
                                         profile.torque_constant_newton_meter_per_ampere;
    motor_feedback.temperature_celsius =
        profile.has_temperature_feedback ? static_cast<float>(feedback.motor_temperature_celsius) : 0.0F;
    motor_feedback.stamp_us = received_us;
    JointLink::publish(*config_.motors[index].joint, motor_feedback);
}

std::int16_t DjiMotor::command_counts(std::size_t index, hal::os::Microseconds now) noexcept
{
    const Model &model = config_.motors[index].model;
    const Profile &profile = model.profile;
    // 关节给的电机输出轴力矩；没有请求出力、没有指令、指令超时时 JointLink 给 0
    const JointCommand command = JointLink::command(*config_.motors[index].joint, now);
    const float ampere = command.torque_newton_meter / profile.torque_constant_newton_meter_per_ampere;
    const float full_scale = static_cast<float>(current_spec(model).control_full_scale_counts);
    float counts = ampere / profile.current_full_scale_ampere * full_scale;
    if (!std::isfinite(counts))
    {
        counts = 0.0F;
    }
    counts = counts > full_scale ? full_scale : (counts < -full_scale ? -full_scale : counts);
    return static_cast<std::int16_t>(counts);
}

void DjiMotor::transmit(hal::os::Microseconds now) noexcept
{
    if (!ready_)
    {
        return;
    }
    std::array<std::array<std::int16_t, Dji::DEVICES_PER_FRAME>, MAXIMUM_CONTROL_FRAMES> counts{};
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        Slot &slot = slots_[index];
        Joint &joint = *config_.motors[index].joint;
        slot.command = command_counts(index, now);
        counts[slot.frame][slot.slot] = slot.command;
        const bool online = slot.ever_received && !expired(slot.last_update_us, LINK_TIMEOUT_US, now);
        JointLink::set_status(joint, online, JointLink::enable_requested(joint), false, 0U);
    }
    // 用到的帧全部发出：C610/C620 要先收到控制帧才回反馈（保活）
    for (std::size_t frame = 0U; frame < frame_count_; ++frame)
    {
        const Dji::ControlFrame encoded = Dji::encode_frame(frames_[frame].identifier, counts[frame]);
        hal::Can::Frame out{};
        out.identifier = encoded.identifier;
        out.identifier_type = hal::Can::IdentifierType::STANDARD;
        out.length = static_cast<std::uint8_t>(encoded.data.size());
        out.data = encoded.data;
        const std::size_t result = static_cast<std::size_t>(bus_.bus().send(out));
        std::array<std::uint32_t, hal::Can::SEND_RESULT_COUNT> &results = statistics_.tx_result_counts;
        ++results[result < results.size() ? result : results.size() - 1U];
    }
}

bool DjiMotor::get_snapshot(std::size_t index, Snapshot &snapshot) const noexcept
{
    if (index >= config_.count)
    {
        return false;
    }
    const Slot &slot = slots_[index];
    Snapshot result{};
    result.device_id = config_.motors[index].device_id;
    result.feedback_identifier = slot.feedback_identifier;
    result.control_identifier = frames_[slot.frame].identifier;
    result.ever_received = slot.ever_received;
    result.online = ready_ && slot.ever_received && !expired(slot.last_update_us, LINK_TIMEOUT_US, hal::os::now_us());
    result.last_update_us = slot.last_update_us;
    result.feedback_count = slot.feedback_count;
    result.rotor_angle_raw = slot.feedback.rotor_angle_raw;
    result.total_angle_counts = slot.total_counts;
    result.rotor_speed_rpm = slot.feedback.rotor_speed_raw;
    result.torque_current_raw = slot.feedback.torque_current_raw;
    result.motor_temperature_celsius =
        config_.motors[index].model.profile.has_temperature_feedback ? slot.feedback.motor_temperature_celsius : 0U;
    result.command_raw = slot.command;
    snapshot = result;
    return true;
}

bool DjiMotor::get_statistics(Statistics &statistics) const noexcept
{
    statistics = statistics_;
    return true;
}

} // namespace device
