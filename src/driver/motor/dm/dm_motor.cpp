#include "driver/motor/dm/dm_motor.hpp"

#include <cmath>

namespace device
{

namespace
{

using Dm = protocol::DmEsc;

constexpr std::uint8_t POSITION_BITS{16U};
constexpr std::uint8_t TWELVE_BITS{12U};
constexpr std::int32_t POSITION_COUNTS{65536};    // 16 位位置一圈回绕的计数
constexpr std::uint16_t ESC_ID_MASK{0x0FU};       // 反馈 D0 低 4 位
constexpr std::uint16_t FRAME_ID_LOW_MASK{0xFFU}; // 电机按帧 ID 低 8 位认帧
constexpr float LIMIT_TOLERANCE{1e-5F};           // 量程核对的相对误差

/** 核对时读写的寄存器：用来认出不在等待中的重复应答。 */
bool is_verification_register(std::uint8_t address)
{
    switch (static_cast<Dm::Register>(address))
    {
    case Dm::Register::CONTROL_MODE:
    case Dm::Register::POSITION_MAX:
    case Dm::Register::VELOCITY_MAX:
    case Dm::Register::TORQUE_MAX:
    case Dm::Register::TIMEOUT:
        return true;
    default:
        return false;
    }
}

bool same_limit(float actual, float expected)
{
    return std::isfinite(actual) && std::fabs(actual - expected) <= LIMIT_TOLERANCE * std::fabs(expected);
}

bool finite(const Dm::MitCommand &command)
{
    return std::isfinite(command.position_rad) && std::isfinite(command.velocity_rad_per_second) &&
           std::isfinite(command.kp) && std::isfinite(command.kd) && std::isfinite(command.torque_newton_meter);
}

} // namespace

bool DmMotor::init() noexcept
{
    ready_ = false;
    slots_ = {};
    statistics_ = {};
    if (!config_valid())
    {
        return false;
    }

    // 这条总线的接收队列只能有一个消费者。最后才占用：失败的实例不占住总线；重复 init() 时不再占用
    if (!config_.bus.is_ready() || (!receiver_claimed_ && !config_.bus.claim_receiver()))
    {
        return false;
    }
    receiver_claimed_ = true;

    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        restart_verification(slots_[index]);
    }
    ready_ = true;
    return true;
}

void DmMotor::process() noexcept
{
    if (!ready_)
    {
        return;
    }

    const hal::os::Microseconds now = hal::os::now_us();
    receive_frames(now);
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        switch (slots_[index].phase)
        {
        case Phase::VERIFYING:
            service_verification(index, now);
            break;
        case Phase::READY:
            service_output(index, now);
            break;
        case Phase::STOPPED:
        case Phase::MISMATCH:
            break;
        }
    }

    const hal::Can::Statistics bus = config_.bus.statistics();
    statistics_.rx_dropped_count = bus.rx_dropped_count;
    statistics_.rx_hardware_loss_event_count = bus.rx_hardware_loss_event_count;
    statistics_.bus_off_count = bus.bus_off_count;
}

bool DmMotor::can_enable(std::size_t index) const noexcept
{
    return ready_ && index < config_.count && slots_[index].phase == Phase::READY;
}

bool DmMotor::enable(std::size_t index) noexcept
{
    if (!can_enable(index))
    {
        return false;
    }
    Slot &slot = slots_[index];
    if (slot.output_requested)
    {
        return true;
    }
    drop_output(slot);
    slot.output_requested = true;
    slot.enable_failed = false;
    return true;
}

bool DmMotor::disable(std::size_t index) noexcept
{
    if (index >= config_.count)
    {
        return false;
    }
    drop_output(slots_[index]);
    return true;
}

bool DmMotor::set_command(std::size_t index, const protocol::DmEsc::MitCommand &command) noexcept
{
    if (!ready_ || index >= config_.count || !slots_[index].output_requested)
    {
        return false;
    }
    if (!finite(command))
    {
        slots_[index].command = {};
        return false;
    }
    slots_[index].command = command;
    return true;
}

bool DmMotor::set_torque(std::size_t index, float newton_meter) noexcept
{
    Dm::MitCommand command{};
    command.torque_newton_meter = newton_meter;
    return set_command(index, command);
}

void DmMotor::clear_all_commands() noexcept
{
    for (Slot &slot : slots_)
    {
        slot.command = {};
    }
}

bool DmMotor::get_snapshot(std::size_t index, Snapshot &snapshot) const noexcept
{
    if (index >= config_.count)
    {
        return false;
    }
    const Slot &slot = slots_[index];
    const Dm::Limits &limits = config_.profile.limits;

    Snapshot result{};
    result.motor = config_.motors[index];
    result.phase = slot.phase;
    result.output_requested = slot.output_requested;
    result.output_active = slot.output_active;
    result.enable_failed = slot.enable_failed;
    result.online = ready_ && slot.phase == Phase::READY &&
                    !hal::os::has_elapsed(slot.last_update_us, LINK_TIMEOUT_US, hal::os::now_us());
    result.has_feedback = slot.has_feedback;
    result.last_update_us = slot.last_update_us;
    result.feedback_count = slot.feedback_count;
    result.state = slot.state;
    if (slot.has_feedback)
    {
        const float radians_per_count = 2.0F * limits.position_rad / static_cast<float>((1UL << POSITION_BITS) - 1UL);
        result.position_rad = static_cast<float>(slot.position_counts) * radians_per_count - limits.position_rad;
        result.position_from_first_feedback_rad =
            static_cast<float>(slot.position_counts - slot.origin_counts) * radians_per_count;
        result.velocity_rad_per_second = Dm::from_unsigned(
            slot.feedback.velocity, -limits.velocity_rad_per_second, limits.velocity_rad_per_second, TWELVE_BITS);
        result.torque_newton_meter = Dm::from_unsigned(
            slot.feedback.torque, -limits.torque_newton_meter, limits.torque_newton_meter, TWELVE_BITS);
        result.mos_temperature_celsius = slot.feedback.mos_temperature_celsius;
        result.rotor_temperature_celsius = slot.feedback.rotor_temperature_celsius;
    }
    result.control_mode = slot.control_mode;
    result.limits = slot.limits;
    result.timeout_ticks = slot.timeout_ticks;
    result.mismatch_register = slot.mismatch_register;
    snapshot = result;
    return true;
}

bool DmMotor::get_statistics(Statistics &statistics) const noexcept
{
    statistics = statistics_;
    return true;
}

float DmMotor::maximum_torque_newton_meter() const noexcept
{
    if (!ready_)
    {
        return 0.0F;
    }
    const Profile &profile = config_.profile;
    return profile.peak_torque_newton_meter < profile.limits.torque_newton_meter ? profile.peak_torque_newton_meter
                                                                                 : profile.limits.torque_newton_meter;
}

bool DmMotor::config_valid() const noexcept
{
    const Profile &profile = config_.profile;
    if (config_.count == 0U || config_.count > MAXIMUM_MOTORS || !Dm::is_valid(profile.limits) ||
        !std::isfinite(profile.peak_torque_newton_meter) || profile.peak_torque_newton_meter <= 0.0F ||
        timeout_ticks_ == 0U)
    {
        return false;
    }
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        const Motor &motor = config_.motors[index];
        if (motor.esc_id == 0U || motor.esc_id > MAXIMUM_ESC_ID || motor.master_id > MAXIMUM_MASTER_ID)
        {
            return false;
        }
        for (std::size_t other = 0U; other < config_.count; ++other)
        {
            // 电机按帧 ID 低 8 位认帧：MST_ID 撞上谁的 ESC_ID，谁就会把反馈当成发给自己的帧
            if ((motor.master_id & FRAME_ID_LOW_MASK) == config_.motors[other].esc_id ||
                (other < index && config_.motors[other].esc_id == motor.esc_id))
            {
                return false;
            }
        }
    }
    return true;
}

void DmMotor::receive_frames(hal::os::Microseconds now) noexcept
{
    hal::Can::Frame frame{};
    while (config_.bus.receive(frame) == hal::Can::ReceiveResult::RECEIVED)
    {
        bool matched = false;
        if (frame.identifier_type == hal::Can::IdentifierType::STANDARD)
        {
            const Dm::Frame dm_frame{frame.identifier, frame.length, frame.data};
            for (std::size_t index = 0U; index < config_.count && !matched; ++index)
            {
                matched = config_.motors[index].master_id == frame.identifier && handle_frame(index, dm_frame, now);
            }
        }
        if (!matched)
        {
            ++statistics_.unknown_frame_count;
        }
    }
}

bool DmMotor::handle_frame(std::size_t index, const protocol::DmEsc::Frame &frame, hal::os::Microseconds now) noexcept
{
    Slot &slot = slots_[index];
    const Motor &motor = config_.motors[index];

    // 先认寄存器应答：应答与反馈共用 MST_ID，按反馈解析会得到乱码（Decision-20261102-01 第 5 条）
    Dm::RegisterReply reply{};
    if (Dm::decode_register_reply(frame, motor.esc_id, reply) && is_verification_register(reply.register_address))
    {
        slot.last_update_us = now;
        ++statistics_.register_reply_count;
        accept_reply(index, reply);
        return true;
    }

    Dm::FeedbackRaw feedback{};
    if (!Dm::decode_feedback(frame, feedback) || feedback.id_low_bits != (motor.esc_id & ESC_ID_MASK))
    {
        return false;
    }
    slot.last_update_us = now;
    slot.state = feedback.state;
    ++slot.feedback_count;
    ++statistics_.feedback_count;
    if (slot.phase == Phase::VERIFYING)
    {
        // 核对时只用反馈确认"失能"已送达；量程还没核对，不换算物理量
        if (slot.step == Step::DISABLE && slot.request_pending)
        {
            slot.step = Step::READ_POSITION_MAX;
            slot.request_pending = false;
        }
    }
    else if (slot.phase == Phase::READY)
    {
        accept_feedback(slot, feedback, now);
    }
    return true;
}

void DmMotor::accept_reply(std::size_t index, const protocol::DmEsc::RegisterReply &reply) noexcept
{
    Slot &slot = slots_[index];
    struct Expected final
    {
        Dm::RegisterOperation operation;
        Dm::Register address;
    };
    Expected expected{Dm::RegisterOperation::READ, Dm::Register::CONTROL_MODE};
    switch (slot.step)
    {
    case Step::READ_CONTROL_MODE:
        break;
    case Step::READ_POSITION_MAX:
        expected.address = Dm::Register::POSITION_MAX;
        break;
    case Step::READ_VELOCITY_MAX:
        expected.address = Dm::Register::VELOCITY_MAX;
        break;
    case Step::READ_TORQUE_MAX:
        expected.address = Dm::Register::TORQUE_MAX;
        break;
    case Step::WRITE_TIMEOUT:
        expected = {Dm::RegisterOperation::WRITE, Dm::Register::TIMEOUT};
        break;
    case Step::DISABLE:
        ++statistics_.stale_reply_count;
        return;
    }
    if (slot.phase != Phase::VERIFYING || !slot.request_pending || reply.operation != expected.operation ||
        reply.register_address != static_cast<std::uint8_t>(expected.address))
    {
        ++statistics_.stale_reply_count;
        return;
    }

    slot.request_pending = false;
    switch (slot.step)
    {
    case Step::READ_CONTROL_MODE:
        slot.control_mode = reply.value;
        if (reply.value < static_cast<std::uint32_t>(Dm::ControlMode::MIT) ||
            reply.value > static_cast<std::uint32_t>(Dm::ControlMode::FORCE_POSITION))
        {
            // 不认识的模式：不知道"失能"该用哪个帧 ID，也不能控制，直接判为不符
            slot.phase = Phase::MISMATCH;
            slot.mismatch_register = static_cast<std::uint8_t>(Dm::Register::CONTROL_MODE);
            return;
        }
        slot.step = Step::DISABLE;
        break;
    case Step::READ_POSITION_MAX:
        slot.limits.position_rad = Dm::value_as_float(reply.value);
        slot.step = Step::READ_VELOCITY_MAX;
        break;
    case Step::READ_VELOCITY_MAX:
        slot.limits.velocity_rad_per_second = Dm::value_as_float(reply.value);
        slot.step = Step::READ_TORQUE_MAX;
        break;
    case Step::READ_TORQUE_MAX:
        slot.limits.torque_newton_meter = Dm::value_as_float(reply.value);
        slot.step = Step::WRITE_TIMEOUT;
        break;
    case Step::WRITE_TIMEOUT:
        slot.timeout_ticks = reply.value;
        finish_verification(slot);
        break;
    case Step::DISABLE:
        break;
    }
}

void DmMotor::finish_verification(Slot &slot) noexcept
{
    const Dm::Limits &expected = config_.profile.limits;
    Dm::Register mismatch = Dm::Register::UNDER_VOLTAGE; // 占位，只在 has_mismatch 时使用
    bool has_mismatch = true;
    if (slot.control_mode != static_cast<std::uint32_t>(Dm::ControlMode::MIT))
    {
        mismatch = Dm::Register::CONTROL_MODE;
    }
    else if (!same_limit(slot.limits.position_rad, expected.position_rad))
    {
        mismatch = Dm::Register::POSITION_MAX;
    }
    else if (!same_limit(slot.limits.velocity_rad_per_second, expected.velocity_rad_per_second))
    {
        mismatch = Dm::Register::VELOCITY_MAX;
    }
    else if (!same_limit(slot.limits.torque_newton_meter, expected.torque_newton_meter))
    {
        mismatch = Dm::Register::TORQUE_MAX;
    }
    else if (slot.timeout_ticks != timeout_ticks_)
    {
        mismatch = Dm::Register::TIMEOUT;
    }
    else
    {
        has_mismatch = false;
    }

    if (has_mismatch)
    {
        slot.phase = Phase::MISMATCH;
        slot.mismatch_register = static_cast<std::uint8_t>(mismatch);
        return;
    }
    slot.phase = Phase::READY;
    drop_output(slot);
}

void DmMotor::accept_feedback(Slot &slot,
                              const protocol::DmEsc::FeedbackRaw &feedback,
                              hal::os::Microseconds now) noexcept
{
    if (slot.has_feedback)
    {
        std::int32_t delta =
            static_cast<std::int32_t>(feedback.position) - static_cast<std::int32_t>(slot.feedback.position);
        if (delta > POSITION_COUNTS / 2)
        {
            delta -= POSITION_COUNTS;
        }
        else if (delta < -POSITION_COUNTS / 2)
        {
            delta += POSITION_COUNTS;
        }
        slot.position_counts += delta;
    }
    else
    {
        slot.position_counts = static_cast<std::int32_t>(feedback.position);
        slot.origin_counts = slot.position_counts;
        slot.has_feedback = true;
    }
    slot.feedback = feedback;

    if (!slot.output_requested || !slot.clear_sent)
    {
        return;
    }
    if (!slot.output_active)
    {
        // 清除错误之前发出的帧，其反馈可能还在路上：守护时间之后的反馈才作数
        if (!hal::os::has_elapsed(slot.clear_sent_us, ENABLE_CONFIRM_GUARD_US, now))
        {
            return;
        }
        if (feedback.state == Dm::State::ENABLED)
        {
            slot.output_active = true;
        }
        else if (is_fault(feedback.state))
        {
            drop_output(slot);
            slot.enable_failed = true;
            ++statistics_.enable_failure_count;
        }
        return;
    }
    if (feedback.state != Dm::State::ENABLED)
    {
        drop_output(slot);
        if (feedback.state == Dm::State::DISABLED)
        {
            // 不是我们让它失能的：电机复位过，RAM 里写的超时已经丢了，重新核对
            restart_verification(slot);
        }
    }
}

void DmMotor::service_verification(std::size_t index, hal::os::Microseconds now) noexcept
{
    Slot &slot = slots_[index];
    if (slot.request_pending)
    {
        if (!hal::os::has_elapsed(slot.request_sent_us, REQUEST_TIMEOUT_US, now))
        {
            return;
        }
        ++statistics_.request_timeout_count;
        slot.request_pending = false;
    }

    const std::uint16_t esc_id = config_.motors[index].esc_id;
    Dm::Frame frame{};
    bool encoded = false;
    switch (slot.step)
    {
    case Step::READ_CONTROL_MODE:
        encoded = Dm::encode_read_register(esc_id, Dm::Register::CONTROL_MODE, frame);
        break;
    case Step::DISABLE:
        // 帧 ID 跟随电机当前的模式（READ_CONTROL_MODE 已确认是 1–4）
        encoded = Dm::encode_system(esc_id,
                                    static_cast<Dm::ControlMode>(static_cast<std::uint8_t>(slot.control_mode)),
                                    Dm::SystemCommand::DISABLE,
                                    frame);
        break;
    case Step::READ_POSITION_MAX:
        encoded = Dm::encode_read_register(esc_id, Dm::Register::POSITION_MAX, frame);
        break;
    case Step::READ_VELOCITY_MAX:
        encoded = Dm::encode_read_register(esc_id, Dm::Register::VELOCITY_MAX, frame);
        break;
    case Step::READ_TORQUE_MAX:
        encoded = Dm::encode_read_register(esc_id, Dm::Register::TORQUE_MAX, frame);
        break;
    case Step::WRITE_TIMEOUT:
        encoded = Dm::encode_write_register(esc_id, Dm::Register::TIMEOUT, timeout_ticks_, frame);
        break;
    }
    if (encoded && send(frame))
    {
        slot.request_pending = true;
        slot.request_sent_us = now;
    }
}

void DmMotor::service_output(std::size_t index, hal::os::Microseconds now) noexcept
{
    Slot &slot = slots_[index];
    if (hal::os::has_elapsed(slot.last_update_us, LINK_TIMEOUT_US, now))
    {
        restart_verification(slot);
        service_verification(index, now);
        return;
    }
    if (slot.output_requested && !slot.output_active && slot.clear_sent &&
        hal::os::has_elapsed(slot.clear_sent_us, ENABLE_TIMEOUT_US, now))
    {
        drop_output(slot);
        slot.enable_failed = true;
        ++statistics_.enable_failure_count;
    }

    const std::uint16_t esc_id = config_.motors[index].esc_id;
    Dm::Frame frame{};
    bool encoded = false;
    const bool clearing = slot.output_requested && !slot.clear_sent;
    if (!slot.output_requested)
    {
        encoded = Dm::encode_system(esc_id, Dm::ControlMode::MIT, Dm::SystemCommand::DISABLE, frame);
    }
    else if (clearing)
    {
        encoded = Dm::encode_system(esc_id, Dm::ControlMode::MIT, Dm::SystemCommand::CLEAR_ERROR, frame);
    }
    else if (!slot.output_active)
    {
        encoded = Dm::encode_system(esc_id, Dm::ControlMode::MIT, Dm::SystemCommand::ENABLE, frame);
    }
    else
    {
        encoded = Dm::encode_mit(esc_id, slot.command, config_.profile.limits, frame);
    }
    if (encoded && send(frame) && clearing)
    {
        slot.clear_sent = true;
        slot.clear_sent_us = now;
    }
}

bool DmMotor::send(const protocol::DmEsc::Frame &frame) noexcept
{
    hal::Can::Frame out{};
    out.identifier = frame.identifier;
    out.identifier_type = hal::Can::IdentifierType::STANDARD;
    out.length = frame.length;
    out.data = frame.data;
    const hal::Can::SendResult result = config_.bus.send(out);
    const std::size_t slot = static_cast<std::size_t>(result);
    std::array<std::uint32_t, hal::Can::SEND_RESULT_COUNT> &counts = statistics_.tx_result_counts;
    ++counts[slot < counts.size() ? slot : counts.size() - 1U];
    return result == hal::Can::SendResult::QUEUED;
}

void DmMotor::restart_verification(Slot &slot) noexcept
{
    drop_output(slot);
    slot.phase = Phase::VERIFYING;
    slot.step = Step::READ_CONTROL_MODE;
    slot.request_pending = false;
    slot.control_mode = 0U;
    slot.limits = {};
    slot.timeout_ticks = 0U;
    slot.mismatch_register = 0U;
}

void DmMotor::drop_output(Slot &slot) noexcept
{
    slot.output_requested = false;
    slot.clear_sent = false;
    slot.output_active = false;
    slot.command = {};
}

} // namespace device
