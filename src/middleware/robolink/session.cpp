#include "middleware/robolink/session.hpp"

#include <algorithm>
#include <limits>

namespace robolink
{

namespace
{

constexpr std::uint32_t MICROSECONDS_PER_MILLISECOND{1000U};

std::uint16_t saturate_u16(std::uint32_t value) noexcept
{
    return static_cast<std::uint16_t>(std::min<std::uint32_t>(value, std::numeric_limits<std::uint16_t>::max()));
}

} // namespace

bool Session::init(const Config &config, std::uint32_t now_us) noexcept
{
    if (config.heartbeat_period_ms == 0U || config.status_period_us == 0U || config.diagnostics_period_us == 0U ||
        config.link_timeout_us == 0U || config.max_frames_per_process == 0U)
    {
        return false;
    }
    config_ = config;
    parser_.reset();
    statistics_ = Statistics{};
    status_ = StatusContent{};
    attitude_ = GimbalAttitude{};
    attitude_pending_ = false;
    receive_overflow_ = 0U;
    last_now_us_ = now_us;
    uptime_us_ = now_us;
    drop_session(LinkState::DISCONNECTED);
    return true;
}

void Session::set_connected(bool connected, std::uint32_t now_us) noexcept
{
    advance_clock(now_us);
    if (!connected)
    {
        if (state_ != LinkState::DISCONNECTED)
        {
            drop_session(LinkState::DISCONNECTED);
        }
        return;
    }
    if (state_ == LinkState::DISCONNECTED)
    {
        drop_session(LinkState::USB_CONNECTED); // 新连接：上次没解析完的字节作废
    }
}

std::size_t Session::receive(const std::uint8_t *data, std::size_t length) noexcept
{
    if (state_ == LinkState::DISCONNECTED)
    {
        return length; // 没连上时收到的字节直接丢掉
    }
    return parser_.push(data, length);
}

std::size_t Session::receive_space() const noexcept
{
    return parser_.free_space();
}

void Session::process(std::uint32_t now_us) noexcept
{
    advance_clock(now_us);
    if (state_ == LinkState::DISCONNECTED)
    {
        return;
    }
    Frame frame{};
    for (std::uint8_t count = 0U; count < config_.max_frames_per_process && parser_.next(frame); ++count)
    {
        ++statistics_.rx_frames;
        last_rx_us_ = now_us; // 任一合法帧都算活
        SlotReader reader{frame.data, frame.size};
        Slot slot{};
        while (reader.next(slot))
        {
            handle_slot(slot, now_us);
        }
    }
    const bool in_session = state_ == LinkState::READY || state_ == LinkState::ACTIVE;
    if (in_session && due(last_rx_us_, config_.link_timeout_us + 1U, now_us))
    {
        ++statistics_.link_timeouts;
        drop_session(LinkState::USB_CONNECTED);
    }
}

void Session::handle_slot(const Slot &slot, std::uint32_t now_us) noexcept
{
    const bool in_session = state_ == LinkState::READY || state_ == LinkState::ACTIVE;
    switch (static_cast<MessageType>(slot.type))
    {
    case MessageType::HELLO:
    {
        Hello hello{};
        if (!decode_hello(slot.payload, slot.length, hello))
        {
            ++statistics_.invalid_payloads;
        }
        else
        {
            handle_hello(hello, slot.sequence, now_us);
        }
        break;
    }
    case MessageType::CONTROL_SNAPSHOT:
    {
        ControlSnapshot snapshot{};
        if (!in_session)
        {
            ++statistics_.unexpected_messages;
        }
        else if (!decode_control_snapshot(slot.payload, slot.length, snapshot))
        {
            ++statistics_.invalid_payloads;
        }
        else
        {
            control_ = snapshot;
            control_received_us_ = now_us;
            control_pending_ = true;
            last_rx_control_seq_ = snapshot.control_seq;
            ++statistics_.control_snapshots;
            state_ = LinkState::ACTIVE; // §5.1：首次有效控制
        }
        break;
    }
    case MessageType::ERROR:
    {
        ErrorMessage error{};
        if (decode_error(slot.payload, slot.length, error))
        {
            ++statistics_.peer_errors;
        }
        else
        {
            ++statistics_.invalid_payloads;
        }
        break;
    }
    case MessageType::HELLO_ACK: // PC 确认本端的 HELLO：本端不等它（Decision-20261103-01 第 4 点）
    case MessageType::HEARTBEAT:
    case MessageType::ACK: // 本端不发 ACK_REQ，不等 ACK
        break;
    case MessageType::SYSTEM_STATUS:
    case MessageType::GIMBAL_ATTITUDE:
    default:
        ++statistics_.unexpected_messages; // MCU → PC 的消息不该从 PC 来
        break;
    }
}

void Session::handle_hello(const Hello &hello, std::uint16_t sequence, std::uint32_t now_us) noexcept
{
    HelloStatus status = HelloStatus::OK;
    if (hello.device_type != DeviceType::PC)
    {
        ++statistics_.rejected_hellos;
        return; // 不是 PC：不回
    }
    if (hello.proto_ver_major < VERSION)
    {
        status = HelloStatus::VERSION_INCOMPATIBLE; // 本端只会 v2
    }
    else if ((hello.capabilities & CAPABILITIES) == 0U)
    {
        status = HelloStatus::NO_COMMON_CAPABILITY;
    }

    if (status == HelloStatus::OK)
    {
        enter_ready(hello, now_us);
    }
    else
    {
        ++statistics_.rejected_hellos;
        if (state_ == LinkState::READY || state_ == LinkState::ACTIVE)
        {
            drop_session(LinkState::USB_CONNECTED); // PC 换成了不兼容的版本
        }
    }
    handshake_ack_ = HelloAck{sequence, std::min(VERSION, hello.proto_ver_major), status, config_.heartbeat_period_ms};
    handshake_pending_ = true;
}

void Session::enter_ready(const Hello &hello, std::uint32_t now_us) noexcept
{
    // §5.1：进入 READY 时帧序号与全部消息序号归零，此前的状态作废
    drop_session(LinkState::READY);
    peer_ = hello;
    last_rx_us_ = now_us;
    last_tx_us_ = now_us;
    ++statistics_.handshakes;
}

void Session::drop_session(LinkState next) noexcept
{
    state_ = next;
    frame_sequence_ = 0U;
    message_sequence_.fill(0U);
    handshake_pending_ = false;
    full_status_pending_ = false;
    prepared_ = Prepared{};
    peer_ = Hello{};
    control_pending_ = false;
    last_rx_control_seq_ = 0U;
    if (next == LinkState::DISCONNECTED)
    {
        parser_.clear();
    }
}

bool Session::take_control(ControlSnapshot &snapshot, std::uint32_t &received_us) noexcept
{
    if (!control_pending_)
    {
        return false;
    }
    snapshot = control_;
    received_us = control_received_us_;
    control_pending_ = false;
    return true;
}

void Session::set_status(const StatusContent &content) noexcept
{
    status_ = content;
}

void Session::set_attitude(const GimbalAttitude &attitude) noexcept
{
    attitude_ = attitude;
    attitude_pending_ = true;
}

void Session::set_receive_overflow(std::uint32_t dropped_bytes) noexcept
{
    receive_overflow_ = dropped_bytes;
}

std::size_t Session::prepare(std::uint32_t now_us) noexcept
{
    advance_clock(now_us);
    prepared_ = Prepared{};
    if (state_ == LinkState::DISCONNECTED)
    {
        return 0U;
    }

    builder_.begin(frame_sequence_, now_us);
    if (handshake_pending_)
    {
        // 握手应答单独一帧：HELLO_ACK + 本端 HELLO（64 B）。拒绝时只回 HELLO_ACK
        bool added = builder_.add(
            MessageType::HELLO_ACK, message_sequence_[SEQUENCE_HELLO_ACK], encode_hello_ack(handshake_ack_));
        if (handshake_ack_.status == HelloStatus::OK)
        {
            Hello hello{};
            hello.device_type = DeviceType::MCU;
            hello.fw_version = config_.fw_version;
            hello.git_short = config_.git_short;
            hello.capabilities = CAPABILITIES;
            hello.heartbeat_period_ms = config_.heartbeat_period_ms;
            hello.max_frame_bytes = static_cast<std::uint16_t>(MAX_FRAME_SIZE);
            hello.device_id_digest = config_.device_id_digest;
            added = added && builder_.add(MessageType::HELLO, message_sequence_[SEQUENCE_HELLO], encode_hello(hello));
        }
        prepared_.handshake = added;
        return added ? builder_.finish() : 0U;
    }
    if (state_ == LinkState::USB_CONNECTED)
    {
        return 0U; // 没握手：什么都不发
    }

    if (full_status_pending_)
    {
        // 握手后的第一份完整状态（带诊断尾部，76 B），单独一帧
        prepared_.status = builder_.add(
            MessageType::SYSTEM_STATUS, message_sequence_[SEQUENCE_STATUS], encode_system_status(build_status(true)));
        prepared_.diagnostics = prepared_.status;
        return prepared_.status ? builder_.finish() : 0U;
    }

    if (due(last_status_us_, config_.status_period_us, now_us))
    {
        const bool diagnostics = due(last_diagnostics_us_, config_.diagnostics_period_us, now_us);
        prepared_.status = builder_.add(MessageType::SYSTEM_STATUS,
                                        message_sequence_[SEQUENCE_STATUS],
                                        encode_system_status(build_status(diagnostics)));
        prepared_.diagnostics = prepared_.status && diagnostics;
    }
    // 带诊断尾部的状态与姿态放不进同一帧：姿态留到下一轮
    if (attitude_pending_ && peer_accepts_attitude() && builder_.fits(GIMBAL_ATTITUDE_SIZE))
    {
        prepared_.attitude = builder_.add(
            MessageType::GIMBAL_ATTITUDE, message_sequence_[SEQUENCE_ATTITUDE], encode_gimbal_attitude(attitude_));
    }
    if (builder_.slot_count() == 0U)
    {
        if (!due(last_tx_us_,
                 static_cast<std::uint32_t>(config_.heartbeat_period_ms) * MICROSECONDS_PER_MILLISECOND,
                 now_us))
        {
            return 0U;
        }
        prepared_.keepalive = true; // 零槽保活帧
    }
    return builder_.finish();
}

void Session::commit(std::uint32_t now_us) noexcept
{
    const Prepared sent = prepared_;
    prepared_ = Prepared{};
    if (!sent.handshake && !sent.status && !sent.attitude && !sent.keepalive)
    {
        return; // 没有组好的帧
    }
    ++frame_sequence_;
    ++statistics_.tx_frames;
    last_tx_us_ = now_us;
    if (sent.handshake)
    {
        handshake_pending_ = false;
        ++message_sequence_[SEQUENCE_HELLO_ACK];
        if (handshake_ack_.status == HelloStatus::OK)
        {
            ++message_sequence_[SEQUENCE_HELLO];
            full_status_pending_ = true; // §5.1：进入 READY 后先发一次完整状态
        }
    }
    if (sent.status)
    {
        ++message_sequence_[SEQUENCE_STATUS];
        ++statistics_.statuses_sent;
        last_status_us_ = now_us;
        if (sent.diagnostics)
        {
            last_diagnostics_us_ = now_us;
        }
        if (full_status_pending_ && !sent.handshake)
        {
            full_status_pending_ = false;
            state_ = LinkState::ACTIVE; // §5.1：首次状态帧发出
        }
    }
    if (sent.attitude)
    {
        ++message_sequence_[SEQUENCE_ATTITUDE];
        ++statistics_.attitudes_sent;
        attitude_pending_ = false;
    }
    if (sent.keepalive)
    {
        ++statistics_.keepalives_sent;
    }
}

void Session::report_busy() noexcept
{
    prepared_ = Prepared{};
    ++statistics_.tx_busy_drop;
}

Session::Statistics Session::statistics() const noexcept
{
    Statistics statistics = statistics_;
    const FrameParser::Statistics &parsed = parser_.statistics();
    statistics.crc_errors = parsed.crc_errors;
    statistics.header_errors = parsed.header_errors;
    statistics.frame_errors = parsed.frame_errors;
    statistics.discarded_bytes = parsed.discarded_bytes;
    return statistics;
}

void Session::advance_clock(std::uint32_t now_us) noexcept
{
    // 有符号差：同一轮里先后传进来的时刻可能有先有后，倒退的不计
    const auto elapsed = static_cast<std::int32_t>(now_us - last_now_us_);
    if (elapsed > 0)
    {
        uptime_us_ += static_cast<std::uint32_t>(elapsed);
        last_now_us_ = now_us;
    }
}

SystemStatus Session::build_status(bool diagnostics) const noexcept
{
    SystemStatus status{};
    status.uptime_ms = static_cast<std::uint32_t>(uptime_us_ / MICROSECONDS_PER_MILLISECOND);
    status.mode = status_.mode;
    status.control_source = status_.control_source;
    status.safety_state = status_.safety_state;
    status.last_rx_control_seq = last_rx_control_seq_;
    status.last_accepted_control_seq = status_.last_accepted_control_seq;
    status.last_applied_control_seq = status_.last_applied_control_seq;
    status.control_msg_age_ms = status_.control_msg_age_ms;
    status.link_state = state_;
    status.fault_flags = status_.fault_flags;
    status.has_diagnostics = diagnostics;
    if (diagnostics)
    {
        const Statistics totals = statistics();
        status.diagnostics.rx_frames = totals.rx_frames;
        status.diagnostics.tx_frames = totals.tx_frames;
        status.diagnostics.crc_errors = totals.crc_errors;
        status.diagnostics.rx_overflow = saturate_u16(receive_overflow_);
        status.diagnostics.tx_busy_drop = saturate_u16(totals.tx_busy_drop);
        status.diagnostics.queue_drop = 0U; // 事件只有握手应答，排队等发，不丢
        status.diagnostics.event_queue_depth = handshake_pending_ ? 1U : 0U;
    }
    return status;
}

bool Session::due(std::uint32_t since_us, std::uint32_t period_us, std::uint32_t now_us) const noexcept
{
    // 有符号差：since 可能比 now 晚一点（commit 的时刻晚于下一轮开头取的 now），这时还没到期
    const auto elapsed = static_cast<std::int32_t>(now_us - since_us);
    return elapsed >= 0 && static_cast<std::uint32_t>(elapsed) >= period_us;
}

} // namespace robolink
