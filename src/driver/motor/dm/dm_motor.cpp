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

float radians_per_count(const Dm::Limits &limits)
{
    return 2.0F * limits.position_rad / static_cast<float>((1UL << POSITION_BITS) - 1UL);
}

std::uint32_t timeout_ticks(const DmMotor::Profile &profile)
{
    return profile.communication_timeout_us / DmMotor::TIMEOUT_TICK_US;
}

/**
 * 从 stamp 起是否已过 duration。stamp 可能比 now 晚一点：帧在这一轮开始之后才收到，或指令在这一轮开始之后才下，
 * 无符号相减会回绕成很大的数、误判为超时，所以用有符号差，这时按"刚刚"算（2026-11-02 上板发现）。
 */
bool expired(hal::os::Microseconds stamp, hal::os::Microseconds duration, hal::os::Microseconds now)
{
    return static_cast<std::int32_t>(now - stamp) >= static_cast<std::int32_t>(duration);
}

bool model_valid(const DmMotor::Profile &profile)
{
    return Dm::is_valid(profile.limits) && std::isfinite(profile.peak_torque_newton_meter) &&
           profile.peak_torque_newton_meter > 0.0F && timeout_ticks(profile) != 0U;
}

} // namespace

bool DmMotor::init() noexcept
{
    ready_ = false;
    if (!config_valid() || !attach())
    {
        return false;
    }
    statistics_ = {};
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        slots_[index] = Slot{};
        restart_verification(index);
    }
    ready_ = true;
    return true;
}

bool DmMotor::attach() noexcept
{
    if (attached_)
    {
        return true;
    }
    // 先登记到 CanBus：失败时关节还没接上，可以改好配置后重试
    std::array<CanBus::Filter, MAXIMUM_MOTORS> filters{};
    std::size_t filter_count = 0U;
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        const std::uint16_t master_id = config_.motors[index].master_id;
        bool seen = false;
        for (std::size_t filter = 0U; filter < filter_count; ++filter)
        {
            seen = seen || filters[filter].first == master_id;
        }
        if (!seen)
        {
            filters[filter_count++] = CanBus::Filter{master_id, master_id, hal::Can::IdentifierType::STANDARD};
        }
    }
    if (!bus_.attach(CanBus::Client{this, on_frame, on_transmit}, filters.data(), filter_count))
    {
        return false;
    }
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        const Motor &motor = config_.motors[index];
        JointLink::Binding binding{};
        binding.modes = JointLink::TORQUE_AND_IMPEDANCE;
        binding.motor_maximum_torque_newton_meter = motor.profile.maximum_torque_newton_meter();
        binding.position_absolute = true;
        binding.has_temperature = true;
        binding.mechanics = motor.mechanics;
        // config_valid() 已经检查过关节与机构，这里不会失败
        (void)JointLink::bind(*motor.joint, binding);
    }
    attached_ = true;
    return true;
}

bool DmMotor::config_valid() const noexcept
{
    if (config_.count == 0U || config_.count > MAXIMUM_MOTORS)
    {
        return false;
    }
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        const Motor &motor = config_.motors[index];
        const JointMechanics &mechanics = motor.mechanics;
        if (motor.joint == nullptr || (!attached_ && motor.joint->supports(JointMode::TORQUE)) ||
            !model_valid(motor.profile) || motor.esc_id == 0U || motor.esc_id > MAXIMUM_ESC_ID ||
            motor.master_id > MAXIMUM_MASTER_ID || (mechanics.direction != 1 && mechanics.direction != -1) ||
            !std::isfinite(mechanics.gear_ratio) || mechanics.gear_ratio <= 0.0F ||
            !std::isfinite(mechanics.zero_offset_rad))
        {
            return false;
        }
        for (std::size_t other = 0U; other < config_.count; ++other)
        {
            const Motor &peer = config_.motors[other];
            // 电机按帧 ID 低 8 位认帧：MST_ID 撞上谁的 ESC_ID，谁就会把反馈当成发给自己的帧
            if ((motor.master_id & FRAME_ID_LOW_MASK) == peer.esc_id ||
                (other < index && (peer.esc_id == motor.esc_id || peer.joint == motor.joint)))
            {
                return false;
            }
        }
    }
    return true;
}

void DmMotor::on_frame(void *context, const hal::Can::Frame &frame) noexcept
{
    static_cast<DmMotor *>(context)->receive(frame);
}

void DmMotor::on_transmit(void *context, hal::os::Microseconds now) noexcept
{
    static_cast<DmMotor *>(context)->transmit(now);
}

void DmMotor::receive(const hal::Can::Frame &frame) noexcept
{
    if (!ready_)
    {
        return;
    }
    const Dm::Frame dm_frame{frame.identifier, frame.length, frame.data};
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        if (config_.motors[index].master_id == frame.identifier && handle_frame(index, dm_frame, frame.received_us))
        {
            return;
        }
    }
    ++statistics_.unknown_frame_count;
}

void DmMotor::transmit(hal::os::Microseconds now) noexcept
{
    if (!ready_)
    {
        return;
    }
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
        publish_status(index, now);
    }
}

bool DmMotor::get_snapshot(std::size_t index, Snapshot &snapshot) const noexcept
{
    if (index >= config_.count)
    {
        return false;
    }
    const Slot &slot = slots_[index];
    const Motor &motor = config_.motors[index];
    const Dm::Limits &limits = motor.profile.limits;

    Snapshot result{};
    result.esc_id = motor.esc_id;
    result.master_id = motor.master_id;
    result.phase = slot.phase;
    result.output_requested = slot.output_requested;
    result.output_active = slot.output_active;
    result.enable_failed = slot.enable_failed;
    result.online =
        ready_ && slot.phase == Phase::READY && !expired(slot.last_update_us, LINK_TIMEOUT_US, hal::os::now_us());
    result.has_feedback = slot.has_feedback;
    result.last_update_us = slot.last_update_us;
    result.feedback_count = slot.feedback_count;
    result.state = slot.state;
    if (slot.has_feedback)
    {
        result.position_rad = position_rad(index);
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

float DmMotor::position_rad(std::size_t index) const noexcept
{
    const Dm::Limits &limits = config_.motors[index].profile.limits;
    return static_cast<float>(slots_[index].position_counts) * radians_per_count(limits) - limits.position_rad;
}

bool DmMotor::handle_frame(std::size_t index,
                           const protocol::DmEsc::Frame &frame,
                           hal::os::Microseconds received_us) noexcept
{
    Slot &slot = slots_[index];
    const Motor &motor = config_.motors[index];

    // 先认寄存器应答：应答与反馈共用 MST_ID，按反馈解析会得到乱码（Decision-20261102-01 第 5 条）
    Dm::RegisterReply reply{};
    if (Dm::decode_register_reply(frame, motor.esc_id, reply) && is_verification_register(reply.register_address))
    {
        slot.last_update_us = received_us;
        ++statistics_.register_reply_count;
        accept_reply(index, reply);
        return true;
    }

    Dm::FeedbackRaw feedback{};
    if (!Dm::decode_feedback(frame, feedback) || feedback.id_low_bits != (motor.esc_id & ESC_ID_MASK))
    {
        return false;
    }
    slot.last_update_us = received_us;
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
        accept_feedback(index, feedback, received_us);
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
        finish_verification(index);
        break;
    case Step::DISABLE:
        break;
    }
}

void DmMotor::finish_verification(std::size_t index) noexcept
{
    Slot &slot = slots_[index];
    const Profile &profile = config_.motors[index].profile;
    const Dm::Limits &expected = profile.limits;
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
    else if (slot.timeout_ticks != timeout_ticks(profile))
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
    drop_output(index);
}

void DmMotor::accept_feedback(std::size_t index,
                              const protocol::DmEsc::FeedbackRaw &feedback,
                              hal::os::Microseconds received_us) noexcept
{
    Slot &slot = slots_[index];
    const Dm::Limits &limits = config_.motors[index].profile.limits;
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
        slot.has_feedback = true;
    }
    slot.feedback = feedback;

    MotorFeedback motor_feedback{};
    motor_feedback.position_rad = position_rad(index);
    motor_feedback.velocity_rad_per_second = Dm::from_unsigned(
        feedback.velocity, -limits.velocity_rad_per_second, limits.velocity_rad_per_second, TWELVE_BITS);
    motor_feedback.torque_newton_meter =
        Dm::from_unsigned(feedback.torque, -limits.torque_newton_meter, limits.torque_newton_meter, TWELVE_BITS);
    motor_feedback.temperature_celsius = static_cast<float>(feedback.rotor_temperature_celsius);
    motor_feedback.stamp_us = received_us;
    JointLink::publish(*config_.motors[index].joint, motor_feedback);

    if (!slot.output_requested || !slot.clear_sent)
    {
        return;
    }
    if (!slot.output_active)
    {
        // 清除错误之前发出的帧，其反馈可能还在路上：清除错误之后、守护时间以后收到的反馈才作数。
        // 用有符号差：这帧可能早于 clear_sent_us 收到，无符号相减会回绕成很大的数
        if (static_cast<std::int32_t>(received_us - slot.clear_sent_us) <
            static_cast<std::int32_t>(ENABLE_CONFIRM_GUARD_US))
        {
            return;
        }
        if (feedback.state == Dm::State::ENABLED)
        {
            slot.output_active = true;
        }
        else if (is_fault(feedback.state))
        {
            abandon_output(index);
            slot.enable_failed = true;
            ++statistics_.enable_failure_count;
        }
        return;
    }
    if (feedback.state != Dm::State::ENABLED)
    {
        abandon_output(index);
        if (feedback.state == Dm::State::DISABLED)
        {
            // 不是我们让它失能的：电机复位过，RAM 里写的超时已经丢了，重新核对
            restart_verification(index);
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

    const Motor &motor = config_.motors[index];
    const std::uint16_t esc_id = motor.esc_id;
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
        encoded = Dm::encode_write_register(esc_id, Dm::Register::TIMEOUT, timeout_ticks(motor.profile), frame);
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
    const Motor &motor = config_.motors[index];
    Joint &joint = *motor.joint;
    if (expired(slot.last_update_us, LINK_TIMEOUT_US, now))
    {
        abandon_output(index);
        restart_verification(index);
        service_verification(index, now);
        return;
    }

    // 关节的请求：新请求从清错开始；撤销就失能
    const bool requested = JointLink::enable_requested(joint);
    if (requested && !slot.output_requested)
    {
        drop_output(index);
        slot.output_requested = true;
        slot.enable_failed = false;
    }
    else if (!requested && slot.output_requested)
    {
        drop_output(index);
    }
    if (slot.output_requested && !slot.output_active && slot.clear_sent &&
        hal::os::has_elapsed(slot.clear_sent_us, ENABLE_TIMEOUT_US, now))
    {
        abandon_output(index);
        slot.enable_failed = true;
        ++statistics_.enable_failure_count;
    }

    const std::uint16_t esc_id = motor.esc_id;
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
        // 关节的指令（电机输出轴单位）；超时、没有指令时 JointLink 给 0 力矩
        const JointCommand command = JointLink::command(joint, now);
        Dm::MitCommand mit{};
        mit.torque_newton_meter = command.torque_newton_meter;
        if (command.mode == JointMode::IMPEDANCE)
        {
            // 位置给定按电机当前所在的那一圈换算回电机的量程
            const float wrap_rad = position_rad(index) - Dm::from_unsigned(slot.feedback.position,
                                                                           -motor.profile.limits.position_rad,
                                                                           motor.profile.limits.position_rad,
                                                                           POSITION_BITS);
            mit.position_rad = command.position_rad - wrap_rad;
            mit.velocity_rad_per_second = command.velocity_rad_per_second;
            mit.kp = command.stiffness;
            mit.kd = command.damping;
        }
        encoded = Dm::encode_mit(esc_id, mit, motor.profile.limits, frame);
    }
    if (encoded && send(frame) && clearing)
    {
        slot.clear_sent = true;
        slot.clear_sent_us = now;
    }
}

void DmMotor::publish_status(std::size_t index, hal::os::Microseconds now) noexcept
{
    const Slot &slot = slots_[index];
    Joint &joint = *config_.motors[index].joint;
    const bool ready = slot.phase == Phase::READY;
    JointLink::set_available(joint, ready);
    const bool online = ready && !expired(slot.last_update_us, LINK_TIMEOUT_US, now);
    const bool fault = slot.phase == Phase::MISMATCH || slot.enable_failed || is_fault(slot.state);
    JointLink::set_status(joint, online, slot.output_active, fault, static_cast<std::uint8_t>(slot.state));
}

bool DmMotor::send(const protocol::DmEsc::Frame &frame) noexcept
{
    hal::Can::Frame out{};
    out.identifier = frame.identifier;
    out.identifier_type = hal::Can::IdentifierType::STANDARD;
    out.length = frame.length;
    out.data = frame.data;
    const hal::Can::SendResult result = bus_.bus().send(out);
    const std::size_t slot = static_cast<std::size_t>(result);
    std::array<std::uint32_t, hal::Can::SEND_RESULT_COUNT> &counts = statistics_.tx_result_counts;
    ++counts[slot < counts.size() ? slot : counts.size() - 1U];
    return result == hal::Can::SendResult::QUEUED;
}

void DmMotor::restart_verification(std::size_t index) noexcept
{
    drop_output(index);
    Slot &slot = slots_[index];
    slot.phase = Phase::VERIFYING;
    slot.step = Step::READ_CONTROL_MODE;
    slot.request_pending = false;
    slot.control_mode = 0U;
    slot.limits = {};
    slot.timeout_ticks = 0U;
    slot.mismatch_register = 0U;
}

void DmMotor::drop_output(std::size_t index) noexcept
{
    Slot &slot = slots_[index];
    slot.output_requested = false;
    slot.clear_sent = false;
    slot.output_active = false;
}

void DmMotor::abandon_output(std::size_t index) noexcept
{
    // 驱动撤销出力（故障、离线、使能失败）：关节的请求一并清掉，能力要再 enable() 才会重试
    drop_output(index);
    JointLink::drop_enable(*config_.motors[index].joint);
}

} // namespace device
