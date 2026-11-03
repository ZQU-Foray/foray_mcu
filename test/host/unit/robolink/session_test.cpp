// middleware_robolink 的连接会话（Decision-20261103-01 第 4–8 点）：测试扮演 PC，按协议 §5 核对
// 握手、序号归零、发送调度、帧长上限、判活、控制快照与计数。握手应答与标准帧逐字节对照。
#include "middleware/robolink/messages.hpp"
#include "middleware/robolink/session.hpp"
#include "middleware/robolink/wire.hpp"
#include "unit/robolink/golden.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace
{

using robolink::LinkState;
using robolink::MessageType;
using robolink::Session;

constexpr std::uint32_t MS{1000U};

struct SlotCopy
{
    std::uint8_t type;
    std::uint16_t sequence;
    std::vector<std::uint8_t> payload;
};

struct McuFrame
{
    std::vector<std::uint8_t> bytes;
    robolink::FrameHeader header{};
    std::vector<SlotCopy> slots;

    [[nodiscard]] bool has(MessageType type) const
    {
        for (const SlotCopy &slot : slots)
        {
            if (slot.type == static_cast<std::uint8_t>(type))
            {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] const SlotCopy &get(MessageType type) const
    {
        for (const SlotCopy &slot : slots)
        {
            if (slot.type == static_cast<std::uint8_t>(type))
            {
                return slot;
            }
        }
        assert(false);
        return slots.front();
    }
};

// PC 端：组一帧发给 MCU
std::vector<std::uint8_t> pc_frame(std::uint16_t sequence,
                                   MessageType type,
                                   std::uint16_t message_sequence,
                                   const std::uint8_t *payload,
                                   std::size_t length)
{
    std::array<std::uint8_t, robolink::MAX_FRAME_SIZE> buffer{};
    robolink::FrameBuilder builder{buffer.data(), buffer.size()};
    builder.begin(sequence, 777U);
    const bool added = builder.add(type, message_sequence, payload, length);
    assert(added);
    (void)added;
    builder.finish();
    return std::vector<std::uint8_t>(builder.data(), builder.data() + builder.size());
}

std::vector<std::uint8_t> pc_hello(std::uint16_t message_sequence,
                                   std::uint8_t major = 2U,
                                   bool attitude = true,
                                   robolink::DeviceType device = robolink::DeviceType::PC)
{
    robolink::Hello hello{};
    hello.proto_ver_major = major;
    hello.device_type = device;
    hello.capabilities = Session::CAPABILITIES;
    if (!attitude)
    {
        hello.capabilities &= ~robolink::capability_bit(MessageType::GIMBAL_ATTITUDE);
    }
    hello.heartbeat_period_ms = 20U;
    const auto payload = robolink::encode_hello(hello);
    return pc_frame(0U, MessageType::HELLO, message_sequence, payload.bytes.data(), payload.size);
}

std::vector<std::uint8_t> pc_control(std::uint32_t control_seq, std::uint16_t message_sequence)
{
    robolink::ControlSnapshot control{};
    control.control_seq = control_seq;
    control.source = robolink::ControlSource::PC;
    control.mode = robolink::Mode::VISION_TRACK;
    control.flags = robolink::control_flag::GIMBAL;
    control.gimbal_yaw = 0.75F;
    const auto payload = robolink::encode_control_snapshot(control);
    return pc_frame(1U, MessageType::CONTROL_SNAPSHOT, message_sequence, payload.bytes.data(), payload.size);
}

std::vector<std::uint8_t> pc_keepalive()
{
    std::array<std::uint8_t, robolink::MAX_FRAME_SIZE> buffer{};
    robolink::FrameBuilder builder{buffer.data(), buffer.size()};
    builder.begin(9U, 0U);
    builder.finish();
    return std::vector<std::uint8_t>(builder.data(), builder.data() + builder.size());
}

void feed(Session &session, const std::vector<std::uint8_t> &bytes, std::uint32_t now)
{
    const std::size_t accepted = session.receive(bytes.data(), bytes.size());
    assert(accepted == bytes.size());
    (void)accepted;
    session.process(now);
}

// MCU 端组一帧并按发出处理；没有要发的返回空帧
McuFrame transmit(Session &session, std::uint32_t now)
{
    McuFrame frame{};
    const std::size_t size = session.prepare(now);
    if (size == 0U)
    {
        return frame;
    }
    assert(size <= robolink::MCU_MAX_FRAME_SIZE); // §6：MCU 发送的帧不超过 96 B
    frame.bytes.assign(session.frame(), session.frame() + size);
    assert(robolink::check_frame(frame.bytes.data(), size) == robolink::FrameError::NONE);
    const bool parsed = robolink::parse_header(frame.bytes.data(), frame.header);
    assert(parsed);
    (void)parsed;
    assert(frame.header.timestamp_us == now);
    robolink::SlotReader reader{frame.bytes.data(), size};
    robolink::Slot slot{};
    while (reader.next(slot))
    {
        frame.slots.push_back(SlotCopy{slot.type, slot.sequence, {slot.payload, slot.payload + slot.length}});
    }
    session.commit(now);
    return frame;
}

robolink::SystemStatus status_of(const McuFrame &frame)
{
    const SlotCopy &slot = frame.get(MessageType::SYSTEM_STATUS);
    robolink::SystemStatus status{};
    const bool decoded = robolink::decode_system_status(slot.payload.data(), slot.payload.size(), status);
    assert(decoded);
    (void)decoded;
    return status;
}

Session::Config default_config()
{
    Session::Config config{};
    config.fw_version = {0U, 1U, 0U};
    config.git_short = 0x1234ABCDU;
    return config;
}

// 握手到 ACTIVE：返回下一轮的时刻
std::uint32_t handshake(Session &session, std::uint32_t now, bool attitude = true)
{
    session.set_connected(true, now);
    feed(session, pc_hello(0U, 2U, attitude), now);
    assert(session.state() == LinkState::READY);
    const McuFrame reply = transmit(session, now);
    assert(reply.has(MessageType::HELLO_ACK));
    const McuFrame full = transmit(session, now + 1U);
    assert(status_of(full).has_diagnostics);
    assert(session.state() == LinkState::ACTIVE);
    return now + 1U;
}

void silent_until_hello()
{
    Session session;
    assert(session.init(default_config(), 0U));
    assert(session.state() == LinkState::DISCONNECTED);
    assert(session.prepare(0U) == 0U);
    const std::uint8_t junk[3]{1U, 2U, 3U};
    assert(session.receive(junk, sizeof(junk)) == sizeof(junk)); // 没连上：丢掉
    session.set_connected(true, 10U);
    assert(session.state() == LinkState::USB_CONNECTED);
    feed(session, pc_keepalive(), 10U);
    feed(session, pc_control(1U, 0U), 10U);  // 没握手的控制快照
    assert(session.prepare(1000000U) == 0U); // 没握手：什么都不发，保活也不发
    robolink::ControlSnapshot control{};
    std::uint32_t received = 0U;
    assert(!session.take_control(control, received));
    assert(session.statistics().unexpected_messages == 1U);
    assert(session.state() == LinkState::USB_CONNECTED); // 没握手时不判活
}

void handshake_matches_the_golden_frame(const golden::Table &table)
{
    Session session;
    assert(session.init(default_config(), 0U));
    session.set_connected(true, 500U);
    feed(session, pc_hello(7U), 500U);
    assert(session.state() == LinkState::READY && session.peer_accepts_attitude());
    assert(session.statistics().handshakes == 1U);

    // 传输层忙：同一帧下一轮重发
    assert(session.prepare(900U) == 64U);
    session.report_busy();
    assert(session.statistics().tx_busy_drop == 1U);
    const McuFrame reply = transmit(session, 1000U);
    assert(reply.bytes == golden::get(table, "mcu_handshake").bytes); // HELLO_ACK + 本端 HELLO，帧序号 0
    assert(session.state() == LinkState::READY);

    // 接着发完整状态（带诊断尾部），发出后进入 ACTIVE
    const McuFrame full = transmit(session, 2000U);
    assert(full.header.sequence == 1U && full.slots.size() == 1U && full.bytes.size() == 76U);
    const robolink::SystemStatus status = status_of(full);
    assert(status.has_diagnostics && status.link_state == LinkState::READY);
    assert(status.uptime_ms == 2U && status.mode == robolink::Mode::IDLE);
    assert(status.control_msg_age_ms == 0xFFFFU);
    assert(status.diagnostics.rx_frames == 1U && status.diagnostics.tx_frames == 1U);
    assert(status.diagnostics.tx_busy_drop == 1U);
    assert(session.state() == LinkState::ACTIVE);
}

void schedules_status_and_attitude()
{
    Session session;
    assert(session.init(default_config(), 0U));
    const std::uint32_t now = handshake(session, 0U);

    // 5 ms 内没有新姿态：不发（还没到保活时间）
    assert(transmit(session, now + 1U * MS).bytes.empty());

    // 新姿态：单独一帧 60 B
    robolink::GimbalAttitude attitude{};
    attitude.sample_time_us = now + 1500U;
    attitude.quaternion_wxyz = {1.0F, 0.0F, 0.0F, 0.0F};
    attitude.imu_state = robolink::ImuState::READY;
    attitude.attitude_valid = true;
    session.set_attitude(attitude);
    McuFrame frame = transmit(session, now + 2U * MS);
    assert(frame.bytes.size() == 60U && frame.has(MessageType::GIMBAL_ATTITUDE) && frame.header.sequence == 2U);
    assert(frame.get(MessageType::GIMBAL_ATTITUDE).sequence == 0U);
    assert(transmit(session, now + 3U * MS).bytes.empty()); // 同一份姿态不重发

    // 状态到期（距完整状态 5 ms）且有新姿态：同一帧 94 B
    session.set_attitude(attitude);
    robolink::Session::StatusContent content{};
    content.fault_flags = 0x5U;
    session.set_status(content);
    frame = transmit(session, now + 5U * MS);
    assert(frame.bytes.size() == 94U && frame.has(MessageType::SYSTEM_STATUS) &&
           frame.has(MessageType::GIMBAL_ATTITUDE));
    robolink::SystemStatus status = status_of(frame);
    assert(!status.has_diagnostics && status.link_state == LinkState::ACTIVE && status.fault_flags == 0x5U);
    assert(frame.get(MessageType::SYSTEM_STATUS).sequence == 1U &&
           frame.get(MessageType::GIMBAL_ATTITUDE).sequence == 1U);

    // 只有状态：52 B（协议 §4.6 "常态整帧 52 B"）
    frame = transmit(session, now + 10U * MS);
    assert(frame.bytes.size() == 52U && frame.slots.size() == 1U);

    // 1 kHz 跑 1 s：每 5 ms 一份状态、每 1 s 附一次诊断尾部；那一轮姿态顺延
    std::uint32_t statuses = 0U;
    std::uint32_t attitudes = 0U;
    std::uint32_t diagnostics = 0U;
    bool deferred_once = false;
    for (std::uint32_t tick = 11U; tick <= 1100U; ++tick)
    {
        session.set_attitude(attitude);
        frame = transmit(session, now + tick * MS);
        assert(!frame.bytes.empty());
        statuses += frame.has(MessageType::SYSTEM_STATUS) ? 1U : 0U;
        attitudes += frame.has(MessageType::GIMBAL_ATTITUDE) ? 1U : 0U;
        if (frame.has(MessageType::SYSTEM_STATUS) && status_of(frame).has_diagnostics)
        {
            ++diagnostics;
            assert(!frame.has(MessageType::GIMBAL_ATTITUDE) && frame.bytes.size() == 76U);
            deferred_once = true;
        }
    }
    assert(diagnostics == 1U && deferred_once);
    assert(statuses == 1090U / 5U);
    assert(attitudes == 1090U - 1U); // 只有附诊断尾部那一轮没发姿态（这里不调用 process()，不涉及判活）
}

void keepalive_when_idle()
{
    Session::Config config = default_config();
    config.status_period_us = 1000U * MS; // 状态 1 s 一次，空闲时靠保活帧
    Session session;
    assert(session.init(config, 0U));
    const std::uint32_t now = handshake(session, 0U);
    assert(transmit(session, now + 19U * MS).bytes.empty());
    const McuFrame frame = transmit(session, now + 20U * MS);
    assert(frame.bytes.size() == 18U && frame.slots.empty());
    assert((frame.header.flags & robolink::frame_flag::KEEPALIVE) != 0U);
    assert(transmit(session, now + 21U * MS).bytes.empty());
    assert(session.statistics().keepalives_sent == 1U);
}

void attitude_only_when_declared()
{
    Session session;
    assert(session.init(default_config(), 0U));
    const std::uint32_t now = handshake(session, 0U, false);
    assert(!session.peer_accepts_attitude());
    session.set_attitude(robolink::GimbalAttitude{});
    for (std::uint32_t tick = 1U; tick <= 20U; ++tick)
    {
        assert(!transmit(session, now + tick * MS).has(MessageType::GIMBAL_ATTITUDE));
    }
    assert(session.statistics().attitudes_sent == 0U);
}

void link_timeout_and_rehandshake()
{
    Session session;
    assert(session.init(default_config(), 0U));
    std::uint32_t now = handshake(session, 0U);
    // PC 每 100 ms 来一帧保活：一直活着
    for (int round = 0; round < 10; ++round)
    {
        now += 100U * MS;
        feed(session, pc_keepalive(), now);
        assert(session.state() == LinkState::ACTIVE);
    }
    // 500 ms 整还算活，再多 1 µs 就超时
    session.process(now + 500U * MS);
    assert(session.state() == LinkState::ACTIVE);
    session.process(now + 500U * MS + 1U);
    assert(session.state() == LinkState::USB_CONNECTED && session.statistics().link_timeouts == 1U);
    assert(session.prepare(now + 600U * MS) == 0U); // 停发

    // PC 重新握手：序号归零
    now += 700U * MS;
    feed(session, pc_hello(3U), now);
    assert(session.state() == LinkState::READY && session.statistics().handshakes == 2U);
    McuFrame frame = transmit(session, now);
    assert(frame.header.sequence == 0U && frame.get(MessageType::HELLO_ACK).sequence == 0U);
    robolink::HelloAck ack{};
    const auto &payload = frame.get(MessageType::HELLO_ACK).payload;
    assert(robolink::decode_hello_ack(payload.data(), payload.size(), ack) && ack.acked_msg_seq == 3U);
    frame = transmit(session, now + 1U);
    assert(frame.header.sequence == 1U && frame.get(MessageType::SYSTEM_STATUS).sequence == 0U);

    // ACTIVE 时 PC 重启又发 HELLO：重新握手
    feed(session, pc_hello(0U), now + 10U * MS);
    assert(session.state() == LinkState::READY && session.statistics().handshakes == 3U);
    frame = transmit(session, now + 10U * MS);
    assert(frame.header.sequence == 0U && frame.has(MessageType::HELLO_ACK));
}

void rejects_bad_hellos(const golden::Table &table)
{
    Session session;
    assert(session.init(default_config(), 0U));
    session.set_connected(true, 0U);

    // 主版本 1：回 HELLO_ACK（版本不兼容），不带本端 HELLO，不进入 READY
    feed(session, golden::get(table, "pc_hello_major_1").bytes, 10U);
    assert(session.state() == LinkState::USB_CONNECTED);
    McuFrame frame = transmit(session, 20U);
    assert(frame.slots.size() == 1U && frame.has(MessageType::HELLO_ACK));
    robolink::HelloAck ack{};
    const auto &payload = frame.get(MessageType::HELLO_ACK).payload;
    assert(robolink::decode_hello_ack(payload.data(), payload.size(), ack));
    assert(ack.status == robolink::HelloStatus::VERSION_INCOMPATIBLE && ack.negotiated_major == 1U);
    assert(transmit(session, 30U).bytes.empty()); // 之后不发别的

    // 能力无交集
    robolink::Hello hello{};
    hello.device_type = robolink::DeviceType::PC;
    hello.capabilities = std::uint64_t{1U}; // 只有 bit 0：不对应任何已注册的消息
    const auto encoded = robolink::encode_hello(hello);
    feed(session, pc_frame(0U, MessageType::HELLO, 1U, encoded.bytes.data(), encoded.size), 40U);
    frame = transmit(session, 50U);
    assert(robolink::decode_hello_ack(frame.get(MessageType::HELLO_ACK).payload.data(), 8U, ack));
    assert(ack.status == robolink::HelloStatus::NO_COMMON_CAPABILITY);

    // 设备类型是 MCU：不回
    feed(session, pc_hello(0U, 2U, true, robolink::DeviceType::MCU), 60U);
    assert(transmit(session, 70U).bytes.empty());
    // 设备类型越界：消息内容拒收
    feed(session, golden::get(table, "hello_bad_device_type").bytes, 80U);
    assert(transmit(session, 90U).bytes.empty());
    assert(session.statistics().rejected_hellos == 3U && session.statistics().invalid_payloads == 1U);
    assert(session.state() == LinkState::USB_CONNECTED);

    // 已经 ACTIVE 时收到不兼容的 HELLO：回到 USB_CONNECTED
    handshake(session, 100U);
    feed(session, golden::get(table, "pc_hello_major_1").bytes, 200U);
    assert(session.state() == LinkState::USB_CONNECTED);
}

void control_snapshots(const golden::Table &table)
{
    Session session;
    assert(session.init(default_config(), 0U));
    std::uint32_t now = handshake(session, 0U);

    feed(session, pc_control(41U, 0U), now + 1U * MS);
    feed(session, pc_control(42U, 1U), now + 2U * MS); // 只留最新的
    robolink::ControlSnapshot control{};
    std::uint32_t received = 0U;
    assert(session.take_control(control, received));
    assert(control.control_seq == 42U && received == now + 2U * MS && control.gimbal_yaw == 0.75F);
    assert(!session.take_control(control, received));

    feed(session, golden::get(table, "control_nan").bytes, now + 3U * MS); // 内容拒收
    assert(!session.take_control(control, received));
    assert(session.statistics().control_snapshots == 2U && session.statistics().invalid_payloads == 1U);

    // last_rx_control_seq 进状态；accepted、applied 由产品给
    Session::StatusContent content{};
    content.last_accepted_control_seq = 40U;
    session.set_status(content);
    const McuFrame frame = transmit(session, now + 5U * MS);
    const robolink::SystemStatus status = status_of(frame);
    assert(status.last_rx_control_seq == 42U && status.last_accepted_control_seq == 40U);
    assert(status.last_applied_control_seq == 0U);

    // MCU → PC 的消息从 PC 来：方向不对
    const auto wrong = robolink::encode_system_status(robolink::SystemStatus{});
    feed(session, pc_frame(2U, MessageType::SYSTEM_STATUS, 0U, wrong.bytes.data(), wrong.size), now + 6U * MS);
    assert(session.statistics().unexpected_messages == 1U);

    // 重新握手后，上一连接的快照作废
    feed(session, pc_control(43U, 2U), now + 7U * MS);
    feed(session, pc_hello(0U), now + 8U * MS);
    assert(!session.take_control(control, received));
    now += 8U * MS;
    transmit(session, now);
    assert(status_of(transmit(session, now + 1U)).last_rx_control_seq == 0U);
}

void control_activates_ready_session()
{
    Session session;
    assert(session.init(default_config(), 0U));
    session.set_connected(true, 0U);
    feed(session, pc_hello(0U), 0U);
    feed(session, pc_control(1U, 0U), 1U); // 应答还没发出就来了控制快照
    assert(session.state() == LinkState::ACTIVE);
    McuFrame frame = transmit(session, 2U);
    assert(frame.has(MessageType::HELLO_ACK)); // 握手应答照样先发
    frame = transmit(session, 3U);
    assert(status_of(frame).has_diagnostics); // 然后完整状态
}

void disconnect_and_reconnect()
{
    Session session;
    assert(session.init(default_config(), 0U));
    const std::uint32_t now = handshake(session, 0U);
    // 半帧留在解析器里
    const std::vector<std::uint8_t> hello = pc_hello(0U);
    assert(session.receive(hello.data(), 10U) == 10U);
    session.set_connected(false, now + 1U);
    assert(session.state() == LinkState::DISCONNECTED && session.prepare(now + 2U) == 0U);
    session.set_connected(true, now + 3U);
    assert(session.state() == LinkState::USB_CONNECTED);
    // 剩下的半帧接不上：丢掉，不握手
    feed(session, std::vector<std::uint8_t>(hello.begin() + 10, hello.end()), now + 4U);
    assert(session.state() == LinkState::USB_CONNECTED);
    feed(session, hello, now + 5U);
    assert(session.state() == LinkState::READY);
}

void frames_per_process_are_bounded()
{
    Session session;
    assert(session.init(default_config(), 0U));
    const std::uint32_t now = handshake(session, 0U);
    const std::uint32_t before = session.statistics().rx_frames;
    std::vector<std::uint8_t> many;
    for (int copy = 0; copy < 10; ++copy)
    {
        const auto keepalive = pc_keepalive();
        many.insert(many.end(), keepalive.begin(), keepalive.end());
    }
    assert(session.receive(many.data(), many.size()) == many.size());
    session.process(now);
    assert(session.statistics().rx_frames - before == 8U); // §1.4：每轮最多 8 帧
    session.process(now);
    assert(session.statistics().rx_frames - before == 10U);
}

void uptime_and_diagnostics()
{
    Session session;
    const std::uint32_t start = 0xFFFF0000U; // 时基快回绕时启动（已开机约 71.6 分钟）
    assert(session.init(default_config(), start));
    session.set_connected(true, start);
    feed(session, pc_hello(0U), start);
    transmit(session, start);
    session.set_receive_overflow(70000U);         // 超过 u16：饱和
    const std::uint32_t later = start + 0x20000U; // 跨过回绕
    const robolink::SystemStatus status = status_of(transmit(session, later));
    assert(status.uptime_ms == static_cast<std::uint32_t>((0xFFFF0000ULL + 0x20000ULL) / 1000ULL));
    assert(status.diagnostics.rx_overflow == 0xFFFFU);
    // 同一轮里先后传进来的时刻有先有后：开机时长不倒退，也不暴涨
    session.process(later - 5U);
    const robolink::SystemStatus again = status_of(transmit(session, later + 5U * MS));
    assert(again.uptime_ms == status.uptime_ms + 5U);
}

// commit() 用发帧那一刻的时间，下一轮开头取的 now 可能比它早一点：不能当成"过了很久"
void stamps_slightly_in_the_past()
{
    Session session;
    assert(session.init(default_config(), 0U));
    const std::uint32_t now = handshake(session, 0U);
    const McuFrame frame = transmit(session, now + 5U * MS);
    assert(frame.has(MessageType::SYSTEM_STATUS));
    assert(session.prepare(now + 5U * MS - 10U) == 0U); // 状态、保活都没到期
    assert(session.prepare(now + 5U * MS + 10U) == 0U);
    feed(session, pc_keepalive(), now + 5U * MS - 20U); // 判活同理
    session.process(now + 5U * MS - 30U);
    assert(session.state() == LinkState::ACTIVE);
}

void rejects_bad_config()
{
    Session session;
    Session::Config config = default_config();
    config.status_period_us = 0U;
    assert(!session.init(config, 0U));
    config = default_config();
    config.max_frames_per_process = 0U;
    assert(!session.init(config, 0U));
    config = default_config();
    config.heartbeat_period_ms = 0U;
    assert(!session.init(config, 0U));
}

} // namespace

int main()
{
    const golden::Table table = golden::load();
    silent_until_hello();
    handshake_matches_the_golden_frame(table);
    schedules_status_and_attitude();
    keepalive_when_idle();
    attitude_only_when_declared();
    link_timeout_and_rehandshake();
    rejects_bad_hellos(table);
    control_snapshots(table);
    control_activates_ready_session();
    disconnect_and_reconnect();
    frames_per_process_are_bounded();
    uptime_and_diagnostics();
    stamps_slightly_in_the_past();
    rejects_bad_config();
    std::puts("robolink_session: 全部通过");
    return 0;
}
