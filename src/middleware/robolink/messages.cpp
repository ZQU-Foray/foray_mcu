#include "middleware/robolink/messages.hpp"

#include "middleware/robolink/bytes.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace robolink
{

namespace
{

using bytes::get_f32;
using bytes::get_u16;
using bytes::get_u32;
using bytes::get_u64;
using bytes::put_f32;
using bytes::put_u16;
using bytes::put_u32;
using bytes::put_u64;

constexpr std::uint8_t MODE_COUNT{8U};
constexpr std::uint8_t SOURCE_COUNT{5U};
constexpr std::uint8_t SAFETY_STATE_COUNT{4U};
constexpr std::uint8_t LINK_STATE_COUNT{4U};
constexpr std::uint8_t IMU_STATE_COUNT{3U};
constexpr std::uint8_t HELLO_STATUS_COUNT{3U};
constexpr std::uint8_t ACK_STATE_COUNT{4U};
constexpr std::uint16_t ERROR_CODE_COUNT{6U};

std::uint16_t heartbeat_or_default(std::uint16_t value) noexcept
{
    return value == 0U ? DEFAULT_HEARTBEAT_PERIOD_MS : value;
}

// 控制快照里一个子系统的浮点：使能时必须是有限值，未使能时视为缺失（解码为 0）
bool read_subsystem(
    const std::uint8_t *payload, std::size_t offset, std::size_t count, bool enabled, float *const *fields) noexcept
{
    for (std::size_t index = 0U; index < count; ++index)
    {
        const float value = get_f32(payload + offset + 4U * index);
        if (enabled && !std::isfinite(value))
        {
            return false;
        }
        *fields[index] = enabled ? value : 0.0F;
    }
    return true;
}

} // namespace

// 编码

Encoded<HELLO_SIZE> encode_hello(const Hello &message) noexcept
{
    Encoded<HELLO_SIZE> out{};
    std::uint8_t *p = out.bytes.data();
    p[0] = message.proto_ver_major;
    p[1] = message.proto_ver_minor;
    p[2] = static_cast<std::uint8_t>(message.device_type);
    std::memcpy(p + 3, message.fw_version.data(), message.fw_version.size());
    put_u32(p + 6, message.git_short);
    put_u64(p + 10, message.capabilities);
    put_u16(p + 18, message.heartbeat_period_ms);
    put_u16(p + 20, message.max_frame_bytes);
    put_u32(p + 22, message.device_id_digest);
    out.size = HELLO_SIZE;
    return out;
}

Encoded<HELLO_ACK_SIZE> encode_hello_ack(const HelloAck &message) noexcept
{
    Encoded<HELLO_ACK_SIZE> out{};
    std::uint8_t *p = out.bytes.data();
    put_u16(p, message.acked_msg_seq);
    p[2] = message.negotiated_major;
    p[3] = static_cast<std::uint8_t>(message.status);
    put_u16(p + 4, message.heartbeat_period_ms);
    put_u16(p + 6, 0U); // reserved
    out.size = HELLO_ACK_SIZE;
    return out;
}

Encoded<ACK_SIZE> encode_ack(const Ack &message) noexcept
{
    Encoded<ACK_SIZE> out{};
    std::uint8_t *p = out.bytes.data();
    p[0] = message.acked_msg_type;
    put_u16(p + 1, message.acked_msg_seq);
    p[3] = static_cast<std::uint8_t>(message.ack_state);
    put_u16(p + 4, static_cast<std::uint16_t>(message.error_code));
    p[6] = 0U; // reserved
    out.size = ACK_SIZE;
    return out;
}

Encoded<ERROR_HEADER_SIZE + ERROR_MAX_DETAIL> encode_error(const ErrorMessage &message) noexcept
{
    Encoded<ERROR_HEADER_SIZE + ERROR_MAX_DETAIL> out{};
    const std::size_t detail = std::min<std::size_t>(message.detail_length, ERROR_MAX_DETAIL); // 超长由发送侧截断
    put_u16(out.bytes.data(), message.error_code);
    std::memcpy(out.bytes.data() + ERROR_HEADER_SIZE, message.detail.data(), detail);
    out.size = ERROR_HEADER_SIZE + detail;
    return out;
}

Encoded<SYSTEM_STATUS_FULL_SIZE> encode_system_status(const SystemStatus &message) noexcept
{
    Encoded<SYSTEM_STATUS_FULL_SIZE> out{};
    std::uint8_t *p = out.bytes.data();
    put_u32(p, message.uptime_ms);
    p[4] = static_cast<std::uint8_t>(message.mode);
    p[5] = static_cast<std::uint8_t>(message.control_source);
    p[6] = static_cast<std::uint8_t>(message.safety_state);
    p[7] = 0U; // reserved
    put_u32(p + 8, message.last_rx_control_seq);
    put_u32(p + 12, message.last_accepted_control_seq);
    put_u32(p + 16, message.last_applied_control_seq);
    put_u16(p + 20, message.control_msg_age_ms);
    p[22] = static_cast<std::uint8_t>(message.link_state);
    p[23] = 0U; // reserved
    put_u32(p + 24, message.fault_flags);
    out.size = SYSTEM_STATUS_CORE_SIZE;
    if (message.has_diagnostics)
    {
        const SystemDiagnostics &d = message.diagnostics;
        put_u32(p + 28, d.rx_frames);
        put_u32(p + 32, d.tx_frames);
        put_u32(p + 36, d.crc_errors);
        put_u16(p + 40, d.rx_overflow);
        put_u16(p + 42, d.tx_busy_drop);
        put_u16(p + 44, d.queue_drop);
        p[46] = d.event_queue_depth;
        p[47] = 0U;          // reserved
        put_u32(p + 48, 0U); // reserved_wide
        out.size = SYSTEM_STATUS_FULL_SIZE;
    }
    return out;
}

Encoded<CONTROL_SNAPSHOT_SIZE> encode_control_snapshot(const ControlSnapshot &message) noexcept
{
    Encoded<CONTROL_SNAPSHOT_SIZE> out{};
    std::uint8_t *p = out.bytes.data();
    put_u32(p, message.control_seq);
    p[4] = static_cast<std::uint8_t>(message.source);
    p[5] = static_cast<std::uint8_t>(message.mode);
    put_u16(p + 6, message.flags);
    const std::array<float, 9> values{message.chassis_vx,
                                      message.chassis_vy,
                                      message.chassis_wz,
                                      message.gimbal_yaw,
                                      message.gimbal_pitch,
                                      message.gimbal_yaw_rate,
                                      message.gimbal_pitch_rate,
                                      message.friction_rads,
                                      message.feeder_rads};
    for (std::size_t index = 0U; index < values.size(); ++index)
    {
        put_f32(p + 8U + 4U * index, values[index]);
    }
    put_u16(p + 44, message.validity_hint_ms);
    put_u16(p + 46, 0U); // reserved
    out.size = CONTROL_SNAPSHOT_SIZE;
    return out;
}

Encoded<GIMBAL_ATTITUDE_SIZE> encode_gimbal_attitude(const GimbalAttitude &message) noexcept
{
    Encoded<GIMBAL_ATTITUDE_SIZE> out{};
    std::uint8_t *p = out.bytes.data();
    put_u32(p, message.sample_time_us);
    for (std::size_t index = 0U; index < message.quaternion_wxyz.size(); ++index)
    {
        put_f32(p + 4U + 4U * index, message.quaternion_wxyz[index]);
    }
    for (std::size_t index = 0U; index < message.gyro_rad_per_sec.size(); ++index)
    {
        put_f32(p + 20U + 4U * index, message.gyro_rad_per_sec[index]);
    }
    p[32] = static_cast<std::uint8_t>(message.imu_state);
    p[33] = message.attitude_valid ? attitude_flag::ATTITUDE_VALID : 0U;
    put_u16(p + 34, 0U); // reserved
    out.size = GIMBAL_ATTITUDE_SIZE;
    return out;
}

// 解码

bool decode_hello(const std::uint8_t *payload, std::size_t length, Hello &out) noexcept
{
    if (payload == nullptr || length < HELLO_SIZE || payload[2] > static_cast<std::uint8_t>(DeviceType::PC))
    {
        return false;
    }
    Hello message{};
    message.proto_ver_major = payload[0];
    message.proto_ver_minor = payload[1];
    message.device_type = static_cast<DeviceType>(payload[2]);
    std::memcpy(message.fw_version.data(), payload + 3, message.fw_version.size());
    message.git_short = get_u32(payload + 6);
    message.capabilities = get_u64(payload + 10);
    message.heartbeat_period_ms = heartbeat_or_default(get_u16(payload + 18));
    message.max_frame_bytes = get_u16(payload + 20);
    message.device_id_digest = get_u32(payload + 22);
    out = message;
    return true;
}

bool decode_hello_ack(const std::uint8_t *payload, std::size_t length, HelloAck &out) noexcept
{
    if (payload == nullptr || length < HELLO_ACK_SIZE || payload[3] >= HELLO_STATUS_COUNT || get_u16(payload + 6) != 0U)
    {
        return false;
    }
    out = HelloAck{
        get_u16(payload), payload[2], static_cast<HelloStatus>(payload[3]), heartbeat_or_default(get_u16(payload + 4))};
    return true;
}

bool decode_ack(const std::uint8_t *payload, std::size_t length, Ack &out) noexcept
{
    if (payload == nullptr || length < ACK_SIZE || payload[3] >= ACK_STATE_COUNT ||
        get_u16(payload + 4) >= ERROR_CODE_COUNT || payload[6] != 0U)
    {
        return false;
    }
    out = Ack{payload[0],
              get_u16(payload + 1),
              static_cast<AckState>(payload[3]),
              static_cast<ErrorCode>(get_u16(payload + 4))};
    return true;
}

bool decode_error(const std::uint8_t *payload, std::size_t length, ErrorMessage &out) noexcept
{
    if (payload == nullptr || length < ERROR_HEADER_SIZE)
    {
        return false;
    }
    ErrorMessage message{};
    message.error_code = get_u16(payload);
    const std::size_t detail = std::min(length - ERROR_HEADER_SIZE, ERROR_MAX_DETAIL);
    std::memcpy(message.detail.data(), payload + ERROR_HEADER_SIZE, detail);
    message.detail_length = static_cast<std::uint8_t>(detail);
    out = message;
    return true;
}

bool decode_system_status(const std::uint8_t *payload, std::size_t length, SystemStatus &out) noexcept
{
    if (payload == nullptr || length < SYSTEM_STATUS_CORE_SIZE || payload[7] != 0U || payload[23] != 0U ||
        payload[22] >= LINK_STATE_COUNT)
    {
        return false;
    }
    SystemStatus message{};
    message.uptime_ms = get_u32(payload);
    message.mode = payload[4] < MODE_COUNT ? static_cast<Mode>(payload[4]) : Mode::SAFETY;
    message.control_source = payload[5] < SOURCE_COUNT ? static_cast<ControlSource>(payload[5]) : ControlSource::NONE;
    message.safety_state =
        payload[6] < SAFETY_STATE_COUNT ? static_cast<SafetyState>(payload[6]) : SafetyState::CONTROL_LOST;
    message.last_rx_control_seq = get_u32(payload + 8);
    message.last_accepted_control_seq = get_u32(payload + 12);
    message.last_applied_control_seq = get_u32(payload + 16);
    message.control_msg_age_ms = get_u16(payload + 20);
    message.link_state = static_cast<LinkState>(payload[22]);
    message.fault_flags = get_u32(payload + 24);
    if (length >= SYSTEM_STATUS_FULL_SIZE)
    {
        message.has_diagnostics = true;
        message.diagnostics = SystemDiagnostics{get_u32(payload + 28),
                                                get_u32(payload + 32),
                                                get_u32(payload + 36),
                                                get_u16(payload + 40),
                                                get_u16(payload + 42),
                                                get_u16(payload + 44),
                                                payload[46]};
    }
    out = message;
    return true;
}

bool decode_control_snapshot(const std::uint8_t *payload, std::size_t length, ControlSnapshot &out) noexcept
{
    if (payload == nullptr || length < CONTROL_SNAPSHOT_SIZE)
    {
        return false;
    }
    ControlSnapshot message{};
    message.control_seq = get_u32(payload);
    message.source = payload[4] < SOURCE_COUNT ? static_cast<ControlSource>(payload[4]) : ControlSource::NONE;
    message.mode = payload[5] < MODE_COUNT ? static_cast<Mode>(payload[5]) : Mode::IDLE;
    message.flags = get_u16(payload + 6);
    if ((message.flags & static_cast<std::uint16_t>(~control_flag::KNOWN)) != 0U || get_u16(payload + 46) != 0U)
    {
        return false;
    }
    float *const chassis[3]{&message.chassis_vx, &message.chassis_vy, &message.chassis_wz};
    float *const gimbal[4]{
        &message.gimbal_yaw, &message.gimbal_pitch, &message.gimbal_yaw_rate, &message.gimbal_pitch_rate};
    float *const shooter[2]{&message.friction_rads, &message.feeder_rads};
    if (!read_subsystem(payload, 8U, 3U, (message.flags & control_flag::CHASSIS) != 0U, chassis) ||
        !read_subsystem(payload, 20U, 4U, (message.flags & control_flag::GIMBAL) != 0U, gimbal) ||
        !read_subsystem(payload, 36U, 2U, (message.flags & control_flag::SHOOTER) != 0U, shooter))
    {
        return false;
    }
    message.validity_hint_ms = std::min(get_u16(payload + 44), MAX_VALIDITY_HINT_MS);
    out = message;
    return true;
}

bool decode_gimbal_attitude(const std::uint8_t *payload, std::size_t length, GimbalAttitude &out) noexcept
{
    if (payload == nullptr || length < GIMBAL_ATTITUDE_SIZE)
    {
        return false;
    }
    GimbalAttitude message{};
    message.sample_time_us = get_u32(payload);
    for (std::size_t index = 0U; index < message.quaternion_wxyz.size(); ++index)
    {
        message.quaternion_wxyz[index] = get_f32(payload + 4U + 4U * index);
    }
    for (std::size_t index = 0U; index < message.gyro_rad_per_sec.size(); ++index)
    {
        message.gyro_rad_per_sec[index] = get_f32(payload + 20U + 4U * index);
    }
    message.imu_state = payload[32] < IMU_STATE_COUNT ? static_cast<ImuState>(payload[32]) : ImuState::SETTLING;
    message.attitude_valid = (payload[33] & attitude_flag::ATTITUDE_VALID) != 0U;
    out = message;
    return true;
}

} // namespace robolink
