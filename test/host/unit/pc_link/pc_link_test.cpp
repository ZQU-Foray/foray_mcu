// pc_link::PcLink（Decision-20261103-01）在 platform/mock 上：USB 替身扮演主机打开串口、发 HELLO 与控制快照，
// 测试核对 MCU 经 USB 发出的帧：握手应答、完整状态、姿态（来自 imu.attitude）、USB 忙时重组、控制快照发布到
// pc_link.control、判活超时、断开。
#include "middleware/robolink/messages.hpp"
#include "middleware/robolink/session.hpp"
#include "middleware/robolink/wire.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"
#include "pc_link/pc_link.hpp"
#include "topics/imu.hpp"
#include "topics/pc_link.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace
{

using robolink::MessageType;

constexpr hal::os::Microseconds MS{1000U};

std::vector<std::uint8_t>
pc_frame(MessageType type, std::uint16_t sequence, const std::uint8_t *payload, std::size_t length)
{
    std::array<std::uint8_t, robolink::MAX_FRAME_SIZE> buffer{};
    robolink::FrameBuilder builder{buffer.data(), buffer.size()};
    builder.begin(0U, 0U);
    const bool added = builder.add(type, sequence, payload, length);
    assert(added);
    (void)added;
    builder.finish();
    return std::vector<std::uint8_t>(builder.data(), builder.data() + builder.size());
}

void inject(const std::vector<std::uint8_t> &bytes)
{
    assert(mock::usb_cdc::inject(bytes.data(), bytes.size()) == bytes.size());
}

void send_hello(bool attitude)
{
    robolink::Hello hello{};
    hello.device_type = robolink::DeviceType::PC;
    hello.capabilities = robolink::Session::CAPABILITIES;
    if (!attitude)
    {
        hello.capabilities &= ~robolink::capability_bit(MessageType::GIMBAL_ATTITUDE);
    }
    const auto payload = robolink::encode_hello(hello);
    inject(pc_frame(MessageType::HELLO, 0U, payload.bytes.data(), payload.size));
}

struct Sent
{
    std::vector<std::uint8_t> bytes;
    robolink::FrameHeader header{};
    std::vector<robolink::Slot> slots;

    [[nodiscard]] const robolink::Slot *find(MessageType type) const
    {
        for (const robolink::Slot &slot : slots)
        {
            if (slot.type == static_cast<std::uint8_t>(type))
            {
                return &slot;
            }
        }
        return nullptr;
    }
};

// 取出 MCU 这一轮发出的一帧并结束这次发送
bool take_frame(Sent &sent)
{
    std::array<std::uint8_t, 512> buffer{};
    const std::size_t length = mock::usb_cdc::take_written(buffer.data(), buffer.size());
    if (length == 0U)
    {
        return false;
    }
    assert(mock::usb_cdc::write_pending());
    assert(mock::usb_cdc::complete_write());
    sent.bytes.assign(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(length));
    assert(length <= robolink::MCU_MAX_FRAME_SIZE);
    assert(robolink::check_frame(sent.bytes.data(), length) == robolink::FrameError::NONE);
    assert(robolink::parse_header(sent.bytes.data(), sent.header));
    sent.slots.clear();
    robolink::SlotReader reader{sent.bytes.data(), length};
    robolink::Slot slot{};
    while (reader.next(slot))
    {
        sent.slots.push_back(slot);
    }
    return true;
}

pc_link::PcLink::Parameters parameters()
{
    pc_link::PcLink::Parameters parameters{};
    parameters.protocol.fw_version = {0U, 1U, 0U};
    parameters.max_reads_per_step = 8U;
    return parameters;
}

} // namespace

int main()
{
    mock::reset();
    mock::usb_cdc::reset();
    mock::set_now_us(5000U);

    pc_link::PcLink link{hal::UsbCdc{0U}};
    pc_link::PcLink::Parameters bad = parameters();
    bad.max_reads_per_step = 0U;
    assert(!link.init(bad));
    assert(link.init(parameters()));
    auto attitude_publisher = topics::imu::attitude.claim_publisher();
    assert(attitude_publisher);
    topic::Subscriber<topics::pc_link::Control> control_topic{topics::pc_link::control};
    Sent sent{};

    // 主机没打开串口：什么都不做
    link.step(hal::os::now_us());
    assert(link.report().state == robolink::LinkState::DISCONNECTED && !take_frame(sent));

    // 打开串口：等 HELLO，不发
    mock::usb_cdc::set_connected(true);
    mock::advance_us(MS);
    link.step(hal::os::now_us());
    assert(link.report().state == robolink::LinkState::USB_CONNECTED && !take_frame(sent));

    // HELLO → 同一轮回 HELLO_ACK + 本端 HELLO
    send_hello(true);
    mock::advance_us(MS);
    link.step(hal::os::now_us());
    assert(take_frame(sent) && sent.find(MessageType::HELLO_ACK) != nullptr &&
           sent.find(MessageType::HELLO) != nullptr);
    assert(sent.header.sequence == 0U && sent.header.timestamp_us == hal::os::now_us());
    robolink::Hello mcu_hello{};
    const robolink::Slot *hello_slot = sent.find(MessageType::HELLO);
    assert(robolink::decode_hello(hello_slot->payload, hello_slot->length, mcu_hello));
    assert(mcu_hello.device_type == robolink::DeviceType::MCU && mcu_hello.fw_version[1] == 1U);
    assert(link.report().peer_accepts_attitude);

    // 下一轮：完整状态（带诊断尾部），IDLE / NONE / 未收到快照 / age 65535
    mock::advance_us(MS);
    link.step(hal::os::now_us());
    assert(take_frame(sent));
    robolink::SystemStatus status{};
    const robolink::Slot *status_slot = sent.find(MessageType::SYSTEM_STATUS);
    assert(status_slot != nullptr && robolink::decode_system_status(status_slot->payload, status_slot->length, status));
    assert(status.has_diagnostics && status.mode == robolink::Mode::IDLE);
    assert(status.control_source == robolink::ControlSource::NONE);
    assert(status.safety_state == robolink::SafetyState::NO_SNAPSHOT && status.control_msg_age_ms == 0xFFFFU);
    assert(link.report().state == robolink::LinkState::ACTIVE);

    // imu.attitude 的新姿态 → GIMBAL_ATTITUDE
    topics::imu::Attitude attitude{};
    attitude.quaternion = {0.5F, 0.5F, 0.5F, -0.5F};
    attitude.gyro_rad_per_sec = {0.25F, 0.0F, -1.0F};
    attitude.timestamp_us = hal::os::now_us() + 200U;
    attitude.state = topics::imu::State::READY;
    attitude.valid = true;
    mock::advance_us(MS);
    attitude_publisher.publish(attitude);
    link.step(hal::os::now_us());
    assert(take_frame(sent));
    robolink::GimbalAttitude message{};
    const robolink::Slot *attitude_slot = sent.find(MessageType::GIMBAL_ATTITUDE);
    assert(attitude_slot != nullptr &&
           robolink::decode_gimbal_attitude(attitude_slot->payload, attitude_slot->length, message));
    assert(message.sample_time_us == attitude.timestamp_us && message.attitude_valid);
    assert(message.quaternion_wxyz[3] == -0.5F && message.gyro_rad_per_sec[2] == -1.0F);
    assert(message.imu_state == robolink::ImuState::READY);
    // 帧头时刻晚于采样时刻
    assert(static_cast<std::int32_t>(sent.header.timestamp_us - message.sample_time_us) >= 0);

    // 无效的姿态：四元数与角速度清零，有效位为 0
    attitude.valid = false;
    attitude.state = topics::imu::State::SETTLING;
    mock::advance_us(MS);
    attitude_publisher.publish(attitude);
    link.step(hal::os::now_us());
    assert(take_frame(sent));
    attitude_slot = sent.find(MessageType::GIMBAL_ATTITUDE);
    assert(attitude_slot != nullptr &&
           robolink::decode_gimbal_attitude(attitude_slot->payload, attitude_slot->length, message));
    assert(!message.attitude_valid && message.quaternion_wxyz[0] == 0.0F && message.gyro_rad_per_sec[0] == 0.0F);
    assert(message.imu_state == robolink::ImuState::SETTLING);

    // USB 上一次发送没结束：这一轮不算发出，结束后下一轮按最新值重组
    attitude.valid = true;
    mock::advance_us(MS);
    attitude_publisher.publish(attitude);
    link.step(hal::os::now_us()); // 发出，但不结束
    std::array<std::uint8_t, 512> scratch{};
    assert(mock::usb_cdc::take_written(scratch.data(), scratch.size()) > 0U && mock::usb_cdc::write_pending());
    mock::advance_us(5U * MS);
    attitude.timestamp_us = hal::os::now_us();
    attitude_publisher.publish(attitude);
    link.step(hal::os::now_us()); // 状态到期 + 新姿态，但 USB 忙
    assert(link.report().write_busy == 1U && link.report().session.tx_busy_drop == 1U);
    assert(mock::usb_cdc::complete_write());
    mock::advance_us(MS);
    link.step(hal::os::now_us());
    assert(take_frame(sent) && sent.find(MessageType::SYSTEM_STATUS) != nullptr);
    attitude_slot = sent.find(MessageType::GIMBAL_ATTITUDE);
    assert(attitude_slot != nullptr &&
           robolink::decode_gimbal_attitude(attitude_slot->payload, attitude_slot->length, message));
    assert(message.sample_time_us == attitude.timestamp_us); // 最新的那份

    // 控制快照 → pc_link.control
    robolink::ControlSnapshot control{};
    control.control_seq = 77U;
    control.source = robolink::ControlSource::PC;
    control.mode = robolink::Mode::VISION_TRACK;
    control.flags = robolink::control_flag::GIMBAL;
    control.gimbal_yaw = -1.25F;
    control.gimbal_pitch_rate = 0.5F;
    control.validity_hint_ms = 20U;
    const auto control_payload = robolink::encode_control_snapshot(control);
    inject(pc_frame(MessageType::CONTROL_SNAPSHOT, 0U, control_payload.bytes.data(), control_payload.size));
    mock::advance_us(MS);
    link.step(hal::os::now_us());
    topics::pc_link::Control received{};
    assert(control_topic.update(received));
    assert(received.control_seq == 77U && received.source == 2U && received.mode == 2U && received.flags == 0x0002U);
    assert(received.gimbal_yaw == -1.25F && received.gimbal_pitch_rate == 0.5F && received.validity_hint_ms == 20U);
    assert(received.received_us == hal::os::now_us());
    assert(link.report().controls == 1U);
    (void)take_frame(sent);

    // 状态里回报 last_rx_control_seq
    for (int round = 0; round < 6; ++round)
    {
        mock::advance_us(MS);
        link.step(hal::os::now_us());
        if (take_frame(sent) && sent.find(MessageType::SYSTEM_STATUS) != nullptr)
        {
            status_slot = sent.find(MessageType::SYSTEM_STATUS);
            assert(robolink::decode_system_status(status_slot->payload, status_slot->length, status));
            break;
        }
    }
    assert(status.last_rx_control_seq == 77U && status.last_accepted_control_seq == 0U);

    // 500 ms 收不到 PC 的帧：判活超时，停发
    for (int round = 0; round < 510; ++round)
    {
        mock::advance_us(MS);
        link.step(hal::os::now_us());
        (void)take_frame(sent);
    }
    assert(link.report().state == robolink::LinkState::USB_CONNECTED && link.report().session.link_timeouts == 1U);
    mock::advance_us(MS);
    link.step(hal::os::now_us());
    assert(!take_frame(sent));

    // PC 重新握手（这次不声明姿态）：不发姿态
    send_hello(false);
    mock::advance_us(MS);
    link.step(hal::os::now_us());
    assert(take_frame(sent) && sent.find(MessageType::HELLO_ACK) != nullptr);
    for (int round = 0; round < 20; ++round)
    {
        mock::advance_us(MS);
        attitude_publisher.publish(attitude);
        link.step(hal::os::now_us());
        if (take_frame(sent))
        {
            assert(sent.find(MessageType::GIMBAL_ATTITUDE) == nullptr);
        }
    }

    // 主机关串口：回到 DISCONNECTED
    mock::usb_cdc::set_connected(false);
    mock::advance_us(MS);
    link.step(hal::os::now_us());
    assert(link.report().state == robolink::LinkState::DISCONNECTED && !link.report().connected);
    assert(link.report().session.crc_errors == 0U && link.report().session.frame_errors == 0U);
    std::puts("pc_link: 全部通过");
    return 0;
}
