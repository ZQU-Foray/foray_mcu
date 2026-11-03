// middleware_robolink 的消息载荷：解码独立生成的标准帧并逐字段核对，
// 协议规定拒收或改写的取值，编码后再解码得到原值。
#include "middleware/robolink/messages.hpp"
#include "middleware/robolink/wire.hpp"
#include "unit/robolink/golden.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

namespace
{

using robolink::MessageType;

// 标准帧里唯一（或第 index 个）槽位
robolink::Slot slot_of(const golden::Vector &vector, std::size_t index = 0U)
{
    assert(robolink::check_frame(vector.bytes.data(), vector.bytes.size()) == robolink::FrameError::NONE);
    robolink::SlotReader reader{vector.bytes.data(), vector.bytes.size()};
    robolink::Slot slot{};
    for (std::size_t skipped = 0U; skipped <= index; ++skipped)
    {
        const bool found = reader.next(slot);
        assert(found);
        (void)found;
    }
    return slot;
}

void decodes_pc_hello(const golden::Table &table)
{
    robolink::Hello hello{};
    const robolink::Slot slot = slot_of(golden::get(table, "pc_hello"));
    assert(slot.type == static_cast<std::uint8_t>(MessageType::HELLO) && slot.sequence == 0U);
    assert(robolink::decode_hello(slot.payload, slot.length, hello));
    assert(hello.proto_ver_major == 2U && hello.proto_ver_minor == 0U);
    assert(hello.device_type == robolink::DeviceType::PC);
    assert(hello.git_short == 0U && hello.heartbeat_period_ms == 20U && hello.max_frame_bytes == 512U);
    assert((hello.capabilities & robolink::capability_bit(MessageType::GIMBAL_ATTITUDE)) != 0U);
    assert(hello.capabilities == 0x000100010001003EULL);

    const robolink::Slot old = slot_of(golden::get(table, "pc_hello_without_attitude"));
    assert(robolink::decode_hello(old.payload, old.length, hello));
    assert(old.sequence == 3U);
    assert((hello.capabilities & robolink::capability_bit(MessageType::GIMBAL_ATTITUDE)) == 0U);
    assert(hello.fw_version == (std::array<std::uint8_t, 3>{1U, 2U, 3U}));
    assert(hello.git_short == 0xDEADBEEFU && hello.heartbeat_period_ms == 50U && hello.max_frame_bytes == 96U);
    assert(hello.device_id_digest == 9U);

    const robolink::Slot major_1 = slot_of(golden::get(table, "pc_hello_major_1"));
    assert(robolink::decode_hello(major_1.payload, major_1.length, hello));
    assert(hello.proto_ver_major == 1U);
    assert(hello.heartbeat_period_ms == robolink::DEFAULT_HEARTBEAT_PERIOD_MS); // 0 按 50

    const robolink::Slot bad = slot_of(golden::get(table, "hello_bad_device_type"));
    assert(!robolink::decode_hello(bad.payload, bad.length, hello));
    assert(!robolink::decode_hello(slot.payload, slot.length - 1U, hello));
    assert(!robolink::decode_hello(nullptr, 26U, hello));
}

void decodes_control_snapshot(const golden::Table &table)
{
    robolink::ControlSnapshot control{};
    const robolink::Slot slot = slot_of(golden::get(table, "pc_control"));
    assert(slot.type == static_cast<std::uint8_t>(MessageType::CONTROL_SNAPSHOT) && slot.sequence == 2U);
    assert(robolink::decode_control_snapshot(slot.payload, slot.length, control));
    assert(control.control_seq == 2U && control.source == robolink::ControlSource::PC);
    assert(control.mode == robolink::Mode::VISION_TRACK && control.flags == robolink::control_flag::GIMBAL);
    // 云台使能：云台字段原样；底盘、射击未使能，视为缺失，解码为 0
    assert(control.gimbal_yaw == -3.0F && control.gimbal_pitch == 0.125F);
    assert(control.gimbal_yaw_rate == 0.5F && control.gimbal_pitch_rate == -0.0625F);
    assert(control.chassis_vx == 0.0F && control.chassis_vy == 0.0F && control.chassis_wz == 0.0F);
    assert(control.friction_rads == 0.0F && control.feeder_rads == 0.0F);
    assert(control.validity_hint_ms == 30U);

    // §7.1：末尾追加的字段按长度截断读取
    const robolink::Slot extended = slot_of(golden::get(table, "pc_control_extended"));
    assert(extended.length == 56U && extended.sequence == 3U);
    assert(robolink::decode_control_snapshot(extended.payload, extended.length, control));
    assert(control.gimbal_yaw == -3.0F);

    for (const char *name : {"control_reserved_flags", "control_reserved_tail", "control_nan"})
    {
        const robolink::Slot bad = slot_of(golden::get(table, name));
        assert(!robolink::decode_control_snapshot(bad.payload, bad.length, control));
    }
    assert(!robolink::decode_control_snapshot(slot.payload, 47U, control));
}

void control_snapshot_normalizes_values()
{
    robolink::ControlSnapshot sent{};
    sent.flags = robolink::control_flag::CHASSIS | robolink::control_flag::SHOOTER;
    sent.chassis_vx = 1.0F;
    sent.chassis_vy = -2.0F;
    sent.chassis_wz = 0.25F;
    sent.friction_rads = 500.0F;
    sent.feeder_rads = -1.0F;
    sent.gimbal_yaw = std::numeric_limits<float>::quiet_NaN(); // 云台未使能：忽略，不拒收
    sent.validity_hint_ms = 200U;
    auto payload = robolink::encode_control_snapshot(sent);
    payload.bytes[4] = 9U; // 未知 source → NONE
    payload.bytes[5] = 8U; // 未知 mode → IDLE

    robolink::ControlSnapshot got{};
    assert(robolink::decode_control_snapshot(payload.bytes.data(), payload.size, got));
    assert(got.source == robolink::ControlSource::NONE && got.mode == robolink::Mode::IDLE);
    assert(got.chassis_vx == 1.0F && got.chassis_vy == -2.0F && got.chassis_wz == 0.25F);
    assert(got.friction_rads == 500.0F && got.feeder_rads == -1.0F);
    assert(got.gimbal_yaw == 0.0F);
    assert(got.validity_hint_ms == robolink::MAX_VALIDITY_HINT_MS); // ≥ 50 按 50

    sent.chassis_wz = std::numeric_limits<float>::infinity(); // 底盘使能：非有限值拒收
    const auto infinite = robolink::encode_control_snapshot(sent);
    assert(!robolink::decode_control_snapshot(infinite.bytes.data(), infinite.size, got));
}

void decodes_other_pc_messages(const golden::Table &table)
{
    robolink::Ack ack{};
    const robolink::Slot ack_slot = slot_of(golden::get(table, "pc_ack"));
    assert(robolink::decode_ack(ack_slot.payload, ack_slot.length, ack));
    assert(ack.acked_msg_type == 0x10U && ack.acked_msg_seq == 0x0203U);
    assert(ack.ack_state == robolink::AckState::ACCEPTED && ack.error_code == robolink::ErrorCode::NONE);

    robolink::ErrorMessage error{};
    const robolink::Slot error_slot = slot_of(golden::get(table, "pc_error"));
    assert(robolink::decode_error(error_slot.payload, error_slot.length, error));
    assert(error.error_code == 2U && error.detail_length == 3U);
    assert(error.detail[0] == 'l' && error.detail[1] == 'e' && error.detail[2] == 'n');

    robolink::HelloAck hello_ack{};
    const robolink::Slot ack_ok = slot_of(golden::get(table, "pc_hello_ack"));
    assert(robolink::decode_hello_ack(ack_ok.payload, ack_ok.length, hello_ack));
    assert(hello_ack.acked_msg_seq == 0U && hello_ack.negotiated_major == 2U);
    assert(hello_ack.status == robolink::HelloStatus::OK && hello_ack.heartbeat_period_ms == 20U);
    const robolink::Slot ack_bad = slot_of(golden::get(table, "hello_ack_reserved"));
    assert(!robolink::decode_hello_ack(ack_bad.payload, ack_bad.length, hello_ack));

    // 越界的取值
    std::array<std::uint8_t, robolink::ACK_SIZE> raw{0x10U, 0U, 0U, 4U, 0U, 0U, 0U};
    assert(!robolink::decode_ack(raw.data(), raw.size(), ack)); // ack_state 4
    raw[3] = 3U;
    raw[4] = 6U;
    assert(!robolink::decode_ack(raw.data(), raw.size(), ack)); // error_code 6
    raw[4] = 5U;
    raw[6] = 1U;
    assert(!robolink::decode_ack(raw.data(), raw.size(), ack)); // reserved
    raw[6] = 0U;
    assert(robolink::decode_ack(raw.data(), raw.size(), ack) && ack.error_code == robolink::ErrorCode::EXEC_FAILED);

    // ERROR 详情超过 64 B 时截断
    std::array<std::uint8_t, 2U + 80U> long_error{};
    long_error[0] = 1U;
    assert(robolink::decode_error(long_error.data(), long_error.size(), error) && error.detail_length == 64U);
    assert(!robolink::decode_error(long_error.data(), 1U, error));
}

void mcu_messages_round_trip(const golden::Table &table)
{
    robolink::SystemStatus status{};
    const golden::Vector &both = golden::get(table, "mcu_status_attitude");
    const robolink::Slot status_slot = slot_of(both, 0U);
    assert(robolink::decode_system_status(status_slot.payload, status_slot.length, status));
    assert(status.uptime_ms == 123456U && status.mode == robolink::Mode::IDLE);
    assert(status.last_rx_control_seq == 41U && status.control_msg_age_ms == 65535U);
    assert(status.link_state == robolink::LinkState::ACTIVE && status.fault_flags == 0x80000001U);
    assert(!status.has_diagnostics);

    robolink::GimbalAttitude attitude{};
    const robolink::Slot attitude_slot = slot_of(both, 1U);
    assert(robolink::decode_gimbal_attitude(attitude_slot.payload, attitude_slot.length, attitude));
    assert(attitude.sample_time_us == 0xFFFFFE00U && attitude.attitude_valid);
    assert(attitude.quaternion_wxyz == (std::array<float, 4>{0.5F, -0.5F, 0.5F, 0.5F}));
    assert(attitude.gyro_rad_per_sec == (std::array<float, 3>{0.125F, -2.5F, 3.0F}));
    assert(attitude.imu_state == robolink::ImuState::READY);

    const robolink::Slot full = slot_of(golden::get(table, "mcu_status_full"));
    assert(robolink::decode_system_status(full.payload, full.length, status) && status.has_diagnostics);
    assert(status.diagnostics.rx_frames == 1000U && status.diagnostics.tx_frames == 2000U);
    assert(status.diagnostics.crc_errors == 3U && status.diagnostics.rx_overflow == 4U);
    assert(status.diagnostics.tx_busy_drop == 5U && status.diagnostics.queue_drop == 6U);
    assert(status.diagnostics.event_queue_depth == 7U && status.link_state == robolink::LinkState::READY);

    // 改写与拒收
    auto raw = robolink::encode_system_status(status);
    raw.bytes[4] = 8U; // 未知 mode → SAFETY
    raw.bytes[5] = 5U; // 未知来源 → NONE
    raw.bytes[6] = 4U; // 未知安全状态 → 控制失联
    assert(robolink::decode_system_status(raw.bytes.data(), raw.size, status));
    assert(status.mode == robolink::Mode::SAFETY && status.control_source == robolink::ControlSource::NONE);
    assert(status.safety_state == robolink::SafetyState::CONTROL_LOST);
    raw.bytes[22] = 4U; // link_state 越界
    assert(!robolink::decode_system_status(raw.bytes.data(), raw.size, status));

    auto attitude_raw = robolink::encode_gimbal_attitude(attitude);
    attitude_raw.bytes[32] = 7U;    // 未知 → SETTLING
    attitude_raw.bytes[33] = 0xFEU; // 只有保留位
    assert(robolink::decode_gimbal_attitude(attitude_raw.bytes.data(), attitude_raw.size, attitude));
    assert(attitude.imu_state == robolink::ImuState::SETTLING && !attitude.attitude_valid);
}

void encoded_sizes_follow_the_protocol()
{
    assert(robolink::encode_hello(robolink::Hello{}).size == 26U);
    assert(robolink::encode_hello_ack(robolink::HelloAck{}).size == 8U);
    assert(robolink::encode_ack(robolink::Ack{}).size == 7U);
    assert(robolink::encode_error(robolink::ErrorMessage{}).size == 2U);
    robolink::ErrorMessage error{};
    error.detail_length = 200U; // 发送侧截断到 64
    assert(robolink::encode_error(error).size == 66U);
    robolink::SystemStatus status{};
    assert(robolink::encode_system_status(status).size == 28U);
    status.has_diagnostics = true;
    assert(robolink::encode_system_status(status).size == 52U);
    assert(robolink::encode_control_snapshot(robolink::ControlSnapshot{}).size == 48U);
    assert(robolink::encode_gimbal_attitude(robolink::GimbalAttitude{}).size == 36U);
}

} // namespace

int main()
{
    const golden::Table table = golden::load();
    decodes_pc_hello(table);
    decodes_control_snapshot(table);
    control_snapshot_normalizes_values();
    decodes_other_pc_messages(table);
    mcu_messages_round_trip(table);
    encoded_sizes_follow_the_protocol();
    std::puts("robolink_messages: 全部通过");
    return 0;
}
