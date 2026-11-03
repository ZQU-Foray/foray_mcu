// middleware_robolink 的帧层与槽位层：用独立生成的标准帧逐字节核对组帧，
// 逐条核对拒收原因；字节流解析的半帧、粘包、假同步、最长帧与按块喂入。
#include "middleware/robolink/messages.hpp"
#include "middleware/robolink/wire.hpp"
#include "unit/robolink/golden.hpp"
#include "utils/crc/crc32.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace
{

using robolink::FrameError;
using robolink::MessageType;

std::vector<std::uint8_t> bytes_of(const robolink::FrameBuilder &builder)
{
    return std::vector<std::uint8_t>(builder.data(), builder.data() + builder.size());
}

FrameError expected_error(const std::string &expect)
{
    if (expect == "ok" || expect == "payload")
    {
        return FrameError::NONE;
    }
    if (expect == "header")
    {
        return FrameError::HEADER;
    }
    if (expect == "crc")
    {
        return FrameError::CRC;
    }
    if (expect == "layout")
    {
        return FrameError::LAYOUT;
    }
    if (expect == "unknown_type")
    {
        return FrameError::UNKNOWN_TYPE;
    }
    if (expect == "slot_flags")
    {
        return FrameError::SLOT_FLAGS;
    }
    if (expect == "class")
    {
        return FrameError::CLASS_MISMATCH;
    }
    if (expect == "short")
    {
        return FrameError::PAYLOAD_TOO_SHORT;
    }
    if (expect == "ack_flag")
    {
        return FrameError::ACK_FLAG;
    }
    std::fprintf(stderr, "未知的期望 %s\n", expect.c_str());
    assert(false);
    return FrameError::NONE;
}

void crc_is_iso_hdlc()
{
    const std::uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    assert(crc::crc32_calc(text, sizeof(text)) == 0xCBF43926U); // §1.3 的校验值
}

void every_vector_is_classified(const golden::Table &table)
{
    for (const auto &[name, vector] : table)
    {
        const FrameError error = robolink::check_frame(vector.bytes.data(), vector.bytes.size());
        if (error != expected_error(vector.expect))
        {
            std::fprintf(
                stderr, "%s：期望 %s，实际 %u\n", name.c_str(), vector.expect.c_str(), static_cast<unsigned>(error));
            assert(false);
        }
    }
}

void builds_mcu_frames_byte_for_byte(const golden::Table &table)
{
    std::array<std::uint8_t, robolink::MCU_MAX_FRAME_SIZE> buffer{};
    robolink::FrameBuilder builder{buffer.data(), buffer.size()};

    builder.begin(0x1234U, 0x89ABCDEFU);
    assert(builder.finish() == 18U);
    assert(bytes_of(builder) == golden::get(table, "keepalive").bytes);

    // 握手应答：HELLO_ACK + 本端 HELLO
    robolink::HelloAck ack{7U, 2U, robolink::HelloStatus::OK, 20U};
    robolink::Hello hello{};
    hello.device_type = robolink::DeviceType::MCU;
    hello.fw_version = {0U, 1U, 0U};
    hello.git_short = 0x1234ABCDU;
    for (const MessageType type : {MessageType::HELLO,
                                   MessageType::HELLO_ACK,
                                   MessageType::HEARTBEAT,
                                   MessageType::ACK,
                                   MessageType::ERROR,
                                   MessageType::SYSTEM_STATUS,
                                   MessageType::CONTROL_SNAPSHOT,
                                   MessageType::GIMBAL_ATTITUDE})
    {
        hello.capabilities |= robolink::capability_bit(type);
    }
    hello.heartbeat_period_ms = 20U;
    hello.max_frame_bytes = 512U;
    builder.begin(0U, 1000U);
    assert(builder.add(MessageType::HELLO_ACK, 0U, robolink::encode_hello_ack(ack)));
    assert(builder.add(MessageType::HELLO, 0U, robolink::encode_hello(hello)));
    builder.finish();
    assert(bytes_of(builder) == golden::get(table, "mcu_handshake").bytes);

    // 状态核心 + 姿态同帧：94 B，不超过 MCU 的 96 B
    robolink::SystemStatus status{};
    status.uptime_ms = 123456U;
    status.mode = robolink::Mode::IDLE;
    status.control_source = robolink::ControlSource::NONE;
    status.safety_state = robolink::SafetyState::NO_SNAPSHOT;
    status.last_rx_control_seq = 41U;
    status.control_msg_age_ms = 65535U;
    status.link_state = robolink::LinkState::ACTIVE;
    status.fault_flags = 0x80000001U;
    robolink::GimbalAttitude attitude{};
    attitude.sample_time_us = 0xFFFFFE00U;
    attitude.quaternion_wxyz = {0.5F, -0.5F, 0.5F, 0.5F};
    attitude.gyro_rad_per_sec = {0.125F, -2.5F, 3.0F};
    attitude.imu_state = robolink::ImuState::READY;
    attitude.attitude_valid = true;
    builder.begin(0x0102U, 0xFFFFFF00U);
    assert(builder.add(MessageType::SYSTEM_STATUS, 0xFFFFU, robolink::encode_system_status(status)));
    assert(builder.add(MessageType::GIMBAL_ATTITUDE, 5U, robolink::encode_gimbal_attitude(attitude)));
    assert(builder.finish() == 94U);
    assert(bytes_of(builder) == golden::get(table, "mcu_status_attitude").bytes);

    // 带诊断尾部的状态：76 B，与姿态放不进同一帧
    robolink::SystemStatus full{};
    full.uptime_ms = 2U;
    full.control_msg_age_ms = 65535U;
    full.link_state = robolink::LinkState::READY;
    full.has_diagnostics = true;
    full.diagnostics = robolink::SystemDiagnostics{1000U, 2000U, 3U, 4U, 5U, 6U, 7U};
    builder.begin(1U, 2000U);
    assert(builder.add(MessageType::SYSTEM_STATUS, 0U, robolink::encode_system_status(full)));
    assert(!builder.fits(robolink::GIMBAL_ATTITUDE_SIZE));
    assert(!builder.add(MessageType::GIMBAL_ATTITUDE, 0U, robolink::encode_gimbal_attitude(attitude)));
    assert(builder.finish() == 76U);
    assert(bytes_of(builder) == golden::get(table, "mcu_status_full").bytes);

    robolink::GimbalAttitude invalid{};
    invalid.sample_time_us = 69000U;
    invalid.quaternion_wxyz = {0.0F, 0.0F, 0.0F, 0.0F};
    builder.begin(9U, 70000U);
    assert(builder.add(MessageType::GIMBAL_ATTITUDE, 1U, robolink::encode_gimbal_attitude(invalid)));
    builder.finish();
    assert(bytes_of(builder) == golden::get(table, "mcu_attitude_invalid").bytes);
}

void builds_pc_frames_byte_for_byte(const golden::Table &table)
{
    // PC → MCU 的帧也能用同一个组帧器组出来（宿主测试扮演 PC 时用）
    std::array<std::uint8_t, robolink::MAX_FRAME_SIZE> buffer{};
    robolink::FrameBuilder builder{buffer.data(), buffer.size()};

    robolink::ControlSnapshot control{};
    control.control_seq = 2U;
    control.source = robolink::ControlSource::PC;
    control.mode = robolink::Mode::VISION_TRACK;
    control.flags = robolink::control_flag::GIMBAL;
    control.chassis_vx = 0.5F;
    control.chassis_vy = -0.25F;
    control.chassis_wz = 1.5F;
    control.gimbal_yaw = -3.0F;
    control.gimbal_pitch = 0.125F;
    control.gimbal_yaw_rate = 0.5F;
    control.gimbal_pitch_rate = -0.0625F;
    control.friction_rads = 600.0F;
    control.feeder_rads = 12.5F;
    control.validity_hint_ms = 30U;
    builder.begin(3U, 123456789U);
    assert(builder.add(MessageType::CONTROL_SNAPSHOT, 2U, robolink::encode_control_snapshot(control)));
    builder.finish();
    assert(bytes_of(builder) == golden::get(table, "pc_control").bytes);

    builder.begin(5U, 123458789U);
    assert(builder.add(MessageType::HEARTBEAT, 0U, nullptr, 0U));
    builder.finish();
    assert(bytes_of(builder) == golden::get(table, "pc_heartbeat_slot").bytes);

    // ACK 槽位置 ACK_PRESENT
    builder.begin(6U, 123459789U);
    assert(builder.add(
        MessageType::ACK,
        1U,
        robolink::encode_ack(robolink::Ack{0x10U, 0x0203U, robolink::AckState::ACCEPTED, robolink::ErrorCode::NONE})));
    builder.finish();
    assert(bytes_of(builder) == golden::get(table, "pc_ack").bytes);

    robolink::ErrorMessage error{};
    error.error_code = 2U;
    error.detail = {'l', 'e', 'n'};
    error.detail_length = 3U;
    builder.begin(7U, 123460789U);
    assert(builder.add(MessageType::ERROR, 0U, robolink::encode_error(error)));
    builder.finish();
    assert(bytes_of(builder) == golden::get(table, "pc_error").bytes);
}

void builder_rejects_what_does_not_fit()
{
    std::array<std::uint8_t, robolink::MAX_FRAME_SIZE> buffer{};
    robolink::FrameBuilder small{buffer.data(), robolink::MCU_MAX_FRAME_SIZE};
    const std::array<std::uint8_t, robolink::MAX_PAYLOAD_SIZE + 1U> payload{};

    assert(small.finish() == 0U); // 没有 begin()
    small.begin(0U, 0U);
    assert(!small.add(static_cast<MessageType>(0x06U), 0U, nullptr, 0U)); // 未注册
    assert(!small.add(MessageType::ERROR, 0U, nullptr, 2U));              // 有长度没数据
    assert(small.fits(96U - 18U - 6U));
    assert(!small.fits(96U - 18U - 6U + 1U));
    assert(!small.add(MessageType::ERROR, 0U, payload.data(), 96U - 18U - 6U + 1U));
    assert(small.slot_count() == 0U);
    assert(small.finish() == 18U);                               // 什么都没加：保活帧
    assert(!small.add(MessageType::HEARTBEAT, 0U, nullptr, 0U)); // 已经 finish()

    robolink::FrameBuilder large{buffer.data(), 4096U}; // 按 512 截
    large.begin(0U, 0U);
    assert(!large.fits(robolink::MAX_PAYLOAD_SIZE + 1U));
    assert(large.add(MessageType::ERROR, 0U, payload.data(), robolink::MAX_PAYLOAD_SIZE));
    assert(large.finish() == 14U + 6U + robolink::MAX_PAYLOAD_SIZE + 4U);
    assert(robolink::check_frame(large.data(), large.size()) == FrameError::NONE);
}

void slot_reader_walks_the_frame(const golden::Table &table)
{
    const auto &frame = golden::get(table, "mcu_handshake").bytes;
    robolink::SlotReader reader{frame.data(), frame.size()};
    robolink::Slot slot{};
    assert(reader.next(slot) && slot.type == 0x02U && slot.flags == 0x0CU && slot.length == 8U);
    assert(slot.payload == frame.data() + 14U + 6U);
    assert(reader.next(slot) && slot.type == 0x01U && slot.flags == 0x04U && slot.length == 26U);
    assert(!reader.next(slot));

    const auto &keepalive = golden::get(table, "keepalive").bytes;
    robolink::SlotReader empty{keepalive.data(), keepalive.size()};
    assert(!empty.next(slot));
}

// 期望交出的帧：标准帧里帧层合法的（ok、payload）
std::vector<std::vector<std::uint8_t>> valid_frames(const golden::Table &table)
{
    std::vector<std::vector<std::uint8_t>> frames;
    for (const auto &[name, vector] : table)
    {
        (void)name;
        if (vector.expect == "ok" || vector.expect == "payload")
        {
            frames.push_back(vector.bytes);
        }
    }
    return frames;
}

struct Counts
{
    std::uint32_t crc{0U};
    std::uint32_t frame{0U};
    std::uint32_t unknown{0U};
};

Counts expected_counts(const golden::Table &table)
{
    Counts counts{};
    for (const auto &[name, vector] : table)
    {
        (void)name;
        const FrameError error = expected_error(vector.expect);
        counts.crc += error == FrameError::CRC ? 1U : 0U;
        counts.frame +=
            (error != FrameError::NONE && error != FrameError::CRC && error != FrameError::HEADER) ? 1U : 0U;
        counts.unknown += error == FrameError::UNKNOWN_TYPE ? 1U : 0U;
    }
    return counts;
}

// 把全部标准帧（含坏帧）连同中间的垃圾字节按 chunk 字节一块喂进去，取出的帧应正好是合法的那些，顺序不变
void parser_recovers_from_everything(const golden::Table &table, std::size_t chunk)
{
    std::vector<std::uint8_t> stream;
    const std::vector<std::uint8_t> garbage{0x00U, 0xD3U, 0x00U, 0xD3U, 0x91U, 0x07U, 0xFFU, 0xD3U};
    for (const auto &[name, vector] : table)
    {
        (void)name;
        stream.insert(stream.end(), vector.bytes.begin(), vector.bytes.end());
        stream.insert(stream.end(), garbage.begin(), garbage.end());
    }

    robolink::FrameParser parser;
    std::vector<std::vector<std::uint8_t>> got;
    std::size_t offset = 0U;
    while (offset < stream.size())
    {
        const std::size_t want = std::min(chunk, stream.size() - offset);
        const std::size_t accepted = parser.push(stream.data() + offset, std::min(want, parser.free_space()));
        offset += accepted;
        robolink::Frame frame{};
        while (parser.next(frame))
        {
            assert(frame.size == frame.header.length);
            got.emplace_back(frame.data, frame.data + frame.size);
        }
        // next() 交不出帧时缓冲区里只剩不完整的候选帧，总有空间收后续字节，否则解析器卡死
        assert(parser.free_space() > 0U);
    }
    assert(got == valid_frames(table));
    const Counts counts = expected_counts(table);
    assert(parser.statistics().frames == got.size());
    assert(parser.statistics().crc_errors == counts.crc);
    assert(parser.statistics().frame_errors == counts.frame);
    assert(parser.statistics().unknown_types == counts.unknown);
    assert(parser.statistics().header_errors >= 8U); // 8 个帧头坏帧，加上垃圾里的假魔数
    assert(parser.statistics().discarded_bytes > 0U);
}

void parser_handles_the_longest_frame()
{
    // 512 B：HEARTBEAT 槽 + 482 B 的 ERROR 槽
    std::array<std::uint8_t, robolink::MAX_FRAME_SIZE> buffer{};
    std::array<std::uint8_t, 482> detail{};
    for (std::size_t index = 0U; index < detail.size(); ++index)
    {
        detail[index] = static_cast<std::uint8_t>(index * 7U); // 含 0xD3、0x91
    }
    robolink::FrameBuilder builder{buffer.data(), buffer.size()};
    builder.begin(1U, 2U);
    assert(builder.add(MessageType::HEARTBEAT, 0U, nullptr, 0U));
    assert(builder.add(MessageType::ERROR, 0U, detail.data(), detail.size()));
    assert(builder.finish() == robolink::MAX_FRAME_SIZE);

    robolink::FrameParser parser;
    const std::uint8_t noise[] = {0x91U, 0xD3U, 0xD3U, 0x91U, 0x02U};
    assert(parser.push(noise, sizeof(noise)) == sizeof(noise));
    robolink::Frame frame{};
    assert(!parser.next(frame)); // 不够一个帧头
    std::size_t fed = 0U;
    while (fed < builder.size())
    {
        assert(!parser.next(frame));
        fed += parser.push(builder.data() + fed, 1U);
    }
    assert(parser.next(frame) && frame.size == robolink::MAX_FRAME_SIZE);
    assert(std::equal(frame.data, frame.data + frame.size, builder.data()));
    assert(!parser.next(frame));
    assert(parser.free_space() == robolink::MAX_FRAME_SIZE);
}

void parser_keeps_a_trailing_magic_byte(const golden::Table &table)
{
    const auto &control = golden::get(table, "pc_control").bytes;
    robolink::FrameParser parser;
    robolink::Frame frame{};
    const std::uint8_t junk[] = {0x55U, 0xAAU};
    assert(parser.push(junk, sizeof(junk)) == 2U);
    assert(parser.push(control.data(), 1U) == 1U); // 只到 0xD3
    assert(!parser.next(frame));
    assert(parser.statistics().discarded_bytes == 2U);
    assert(parser.push(control.data() + 1U, control.size() - 1U) == control.size() - 1U);
    assert(parser.next(frame) && frame.size == control.size());

    // 缓冲区满时只收下装得下的部分
    parser.reset();
    std::vector<std::uint8_t> many;
    for (int copy = 0; copy < 20; ++copy)
    {
        many.insert(many.end(), control.begin(), control.end());
    }
    assert(parser.push(many.data(), many.size()) == robolink::MAX_FRAME_SIZE);
    std::size_t frames = 0U;
    while (parser.next(frame))
    {
        ++frames;
    }
    assert(frames == robolink::MAX_FRAME_SIZE / control.size());
}

// §1.4：CRC 失败丢弃整帧，从帧尾之后继续扫描。藏在坏帧载荷里的合法帧不得交出
void parser_skips_the_whole_bad_frame(const golden::Table &table)
{
    const auto &inner = golden::get(table, "keepalive").bytes;
    std::array<std::uint8_t, robolink::MAX_FRAME_SIZE> buffer{};
    robolink::FrameBuilder builder{buffer.data(), buffer.size()};
    builder.begin(1U, 2U);
    assert(builder.add(MessageType::ERROR, 0U, inner.data(), inner.size()));
    builder.finish();
    std::vector<std::uint8_t> stream(builder.data(), builder.data() + builder.size());
    stream.back() ^= 0x01U; // 外层帧 CRC 错
    const auto &after = golden::get(table, "pc_control").bytes;
    stream.insert(stream.end(), after.begin(), after.end());

    robolink::FrameParser parser;
    assert(parser.push(stream.data(), stream.size()) == stream.size());
    robolink::Frame frame{};
    assert(parser.next(frame) && frame.size == after.size());
    assert(std::equal(frame.data, frame.data + frame.size, after.begin()));
    assert(!parser.next(frame));
    assert(parser.statistics().crc_errors == 1U && parser.statistics().frames == 1U);
}

void header_fields_are_parsed(const golden::Table &table)
{
    const auto &frame = golden::get(table, "mcu_status_attitude").bytes;
    robolink::FrameHeader header{};
    assert(robolink::parse_header(frame.data(), header));
    assert(header.flags == 0U && header.length == 94U && header.sequence == 0x0102U);
    assert(header.timestamp_us == 0xFFFFFF00U && header.slot_count == 2U);
    assert(!robolink::parse_header(nullptr, header));
    assert(robolink::check_frame(frame.data(), frame.size() - 1U) == FrameError::HEADER); // 长度与帧头不符
    assert(robolink::check_frame(nullptr, 0U) == FrameError::HEADER);
}

} // namespace

int main()
{
    const golden::Table table = golden::load();
    crc_is_iso_hdlc();
    every_vector_is_classified(table);
    builds_mcu_frames_byte_for_byte(table);
    builds_pc_frames_byte_for_byte(table);
    builder_rejects_what_does_not_fit();
    slot_reader_walks_the_frame(table);
    for (const std::size_t chunk : {1U, 2U, 3U, 7U, 64U, 100U, 512U, 4096U})
    {
        parser_recovers_from_everything(table, chunk);
    }
    parser_handles_the_longest_frame();
    parser_keeps_a_trailing_magic_byte(table);
    parser_skips_the_whole_bad_frame(table);
    header_fields_are_parsed(table);
    std::puts("robolink_wire: 全部通过");
    return 0;
}
