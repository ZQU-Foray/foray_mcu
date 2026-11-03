#ifndef MIDDLEWARE_ROBOLINK_MESSAGES_HPP
#define MIDDLEWARE_ROBOLINK_MESSAGES_HPP

#include "middleware/robolink/wire.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

// RoboLink v0.2 的消息载荷（foray_interfaces protocol/robolink.md §4）。
// 字段名与协议表相同；单位全 SI（m/s、rad、rad/s），时间是 MCU 的单调 µs。
// 编码：按协议的偏移逐字节写小端，保留字段写 0。
// 解码：载荷不得短于协议长度（更长的是末尾追加的新字段，忽略）；按协议"缺失/无效语义"列拒收或改写：
//   - 拒收（返回 false）：保留字段非 0、取值越界且协议写明拒收、控制快照里生效子系统的浮点不是有限值；
//   - 改写：未知的模式、来源、IMU 状态等按协议给的默认值，心跳周期 0 按 50 ms。

namespace robolink
{

inline constexpr std::size_t HELLO_SIZE{26U};
inline constexpr std::size_t HELLO_ACK_SIZE{8U};
inline constexpr std::size_t ACK_SIZE{7U};
inline constexpr std::size_t ERROR_HEADER_SIZE{2U};
inline constexpr std::size_t ERROR_MAX_DETAIL{64U};
inline constexpr std::size_t SYSTEM_STATUS_CORE_SIZE{28U};
inline constexpr std::size_t SYSTEM_STATUS_DIAGNOSTICS_SIZE{24U};
inline constexpr std::size_t SYSTEM_STATUS_FULL_SIZE{SYSTEM_STATUS_CORE_SIZE + SYSTEM_STATUS_DIAGNOSTICS_SIZE};
inline constexpr std::size_t CONTROL_SNAPSHOT_SIZE{48U};
inline constexpr std::size_t GIMBAL_ATTITUDE_SIZE{36U};

/** HELLO 的心跳周期为 0 时按这个值（§4.1【补注】）。 */
inline constexpr std::uint16_t DEFAULT_HEARTBEAT_PERIOD_MS{50U};
/** 控制快照有效期提示的上限（§4.7）。 */
inline constexpr std::uint16_t MAX_VALIDITY_HINT_MS{50U};

enum class DeviceType : std::uint8_t
{
    MCU = 0U,
    PC = 1U
};

/** §4.6 mode（SYSTEM_STATUS 与 CONTROL_SNAPSHOT 共用）。 */
enum class Mode : std::uint8_t
{
    MANUAL = 0U,
    AUTO_NAV = 1U,
    VISION_TRACK = 2U,
    PATROL = 3U,
    GIMBAL_ONLY = 4U,
    SHOOTER_ONLY = 5U,
    SAFETY = 6U,
    IDLE = 7U
};

/** §4.6 control_source、§4.7 source。 */
enum class ControlSource : std::uint8_t
{
    NONE = 0U,
    REMOTE = 1U,
    PC = 2U,
    AUTO = 3U,
    SAFETY = 4U
};

/** §4.6 safety_state。 */
enum class SafetyState : std::uint8_t
{
    NO_SNAPSHOT = 0U,   // 未收到快照
    PC_CONTROL = 1U,    // 快照有效（PC 控制中）
    CONTROL_LOST = 2U,  // 控制失联
    EMERGENCY_STOP = 3U // 急停
};

/** §4.6 link_state（§5.1 的连接状态）。 */
enum class LinkState : std::uint8_t
{
    DISCONNECTED = 0U,
    USB_CONNECTED = 1U,
    READY = 2U,
    ACTIVE = 3U
};

/** §4.8 imu_state。 */
enum class ImuState : std::uint8_t
{
    SETTLING = 0U, // 恒温或零偏标定中：roll/pitch 可用，yaw 零偏未定，不接受世界系云台目标
    READY = 1U,    // 零偏已标定
    DEGRADED = 2U  // 标定超时，按未标定运行
};

/** §4.2 HELLO_ACK status。 */
enum class HelloStatus : std::uint8_t
{
    OK = 0U,
    VERSION_INCOMPATIBLE = 1U,
    NO_COMMON_CAPABILITY = 2U
};

/** §4.4 ack_state。 */
enum class AckState : std::uint8_t
{
    RECEIVED = 0U,
    ACCEPTED = 1U,
    APPLIED = 2U,
    REJECTED = 3U
};

/** §4.4 error_code（ERROR 同）。 */
enum class ErrorCode : std::uint16_t
{
    NONE = 0U,
    UNSUPPORTED = 1U,
    LENGTH_MISMATCH = 2U,
    CLASS_MISMATCH = 3U,
    DIRECTION_VIOLATION = 4U,
    EXEC_FAILED = 5U
};

/** §4.7 flags。 */
namespace control_flag
{
inline constexpr std::uint16_t CHASSIS{0x0001U};
inline constexpr std::uint16_t GIMBAL{0x0002U};
inline constexpr std::uint16_t SHOOTER{0x0004U};
inline constexpr std::uint16_t EMERGENCY_STOP{0x0008U};
inline constexpr std::uint16_t KNOWN{0x000FU};
} // namespace control_flag

/** §4.8 flags。 */
namespace attitude_flag
{
inline constexpr std::uint8_t ATTITUDE_VALID{0x01U};
} // namespace attitude_flag

/** §4.1 HELLO（26 B）。 */
struct Hello
{
    std::uint8_t proto_ver_major{VERSION};
    std::uint8_t proto_ver_minor{0U};
    DeviceType device_type{DeviceType::MCU};
    std::array<std::uint8_t, 3> fw_version{}; // 主.次.修订，全 0 表示没有版本信息
    std::uint32_t git_short{0U};              // 提交号低 32 位，0 表示未知
    std::uint64_t capabilities{0U};           // 支持的消息类型位图
    std::uint16_t heartbeat_period_ms{DEFAULT_HEARTBEAT_PERIOD_MS};
    std::uint16_t max_frame_bytes{static_cast<std::uint16_t>(MAX_FRAME_SIZE)}; // 本端可接收的最大帧
    std::uint32_t device_id_digest{0U};                                        // 0 表示未提供
};

/** §4.2 HELLO_ACK（8 B）。 */
struct HelloAck
{
    std::uint16_t acked_msg_seq{0U};
    std::uint8_t negotiated_major{VERSION};
    HelloStatus status{HelloStatus::OK};
    std::uint16_t heartbeat_period_ms{DEFAULT_HEARTBEAT_PERIOD_MS};
};

/** §4.4 ACK（7 B）。 */
struct Ack
{
    std::uint8_t acked_msg_type{0U};
    std::uint16_t acked_msg_seq{0U};
    AckState ack_state{AckState::RECEIVED};
    ErrorCode error_code{ErrorCode::NONE};
};

/** §4.5 ERROR（2 B + 至多 64 B 详情）。 */
struct ErrorMessage
{
    std::uint16_t error_code{0U}; // 取值同 ErrorCode；按原值保存
    std::array<std::uint8_t, ERROR_MAX_DETAIL> detail{};
    std::uint8_t detail_length{0U};
};

/** §4.6 诊断尾部（可选，24 B）。 */
struct SystemDiagnostics
{
    std::uint32_t rx_frames{0U};
    std::uint32_t tx_frames{0U};
    std::uint32_t crc_errors{0U};
    std::uint16_t rx_overflow{0U};
    std::uint16_t tx_busy_drop{0U};
    std::uint16_t queue_drop{0U};
    std::uint8_t event_queue_depth{0U};
};

/** §4.6 SYSTEM_STATUS（28 B 核心 + 可选 24 B 诊断）。 */
struct SystemStatus
{
    std::uint32_t uptime_ms{0U};
    Mode mode{Mode::IDLE};
    ControlSource control_source{ControlSource::NONE};
    SafetyState safety_state{SafetyState::NO_SNAPSHOT};
    std::uint32_t last_rx_control_seq{0U};
    std::uint32_t last_accepted_control_seq{0U};
    std::uint32_t last_applied_control_seq{0U};
    std::uint16_t control_msg_age_ms{0U};
    LinkState link_state{LinkState::DISCONNECTED};
    std::uint32_t fault_flags{0U};
    bool has_diagnostics{false};
    SystemDiagnostics diagnostics{};
};

/** §4.7 CONTROL_SNAPSHOT（48 B）。未使能子系统的字段解码为 0（协议：视为缺失）。 */
struct ControlSnapshot
{
    std::uint32_t control_seq{0U};
    ControlSource source{ControlSource::NONE};
    Mode mode{Mode::IDLE};
    std::uint16_t flags{0U};
    float chassis_vx{0.0F};             // m/s，机体系 x 前
    float chassis_vy{0.0F};             // m/s，机体系 y 左
    float chassis_wz{0.0F};             // rad/s，绕 z 逆时针为正
    float gimbal_yaw{0.0F};             // rad，世界系，向左为正，不区分圈数
    float gimbal_pitch{0.0F};           // rad，世界系，向下为正
    float gimbal_yaw_rate{0.0F};        // rad/s，yaw 目标角速度前馈
    float gimbal_pitch_rate{0.0F};      // rad/s，pitch 目标角速度前馈
    float friction_rads{0.0F};          // rad/s，摩擦轮目标角速度
    float feeder_rads{0.0F};            // rad/s，拨弹盘目标角速度
    std::uint16_t validity_hint_ms{0U}; // 0 = 无提示；不超过 50
};

/** §4.8 GIMBAL_ATTITUDE（36 B）。 */
struct GimbalAttitude
{
    std::uint32_t sample_time_us{0U};                             // IMU 采样时刻，与帧头 Timestamp 同一时钟
    std::array<float, 4> quaternion_wxyz{1.0F, 0.0F, 0.0F, 0.0F}; // 云台系 → 世界系，标量在前
    std::array<float, 3> gyro_rad_per_sec{};                      // 云台系角速度，已扣除零偏
    ImuState imu_state{ImuState::SETTLING};
    bool attitude_valid{false};
};

[[nodiscard]] Encoded<HELLO_SIZE> encode_hello(const Hello &message) noexcept;
[[nodiscard]] Encoded<HELLO_ACK_SIZE> encode_hello_ack(const HelloAck &message) noexcept;
[[nodiscard]] Encoded<ACK_SIZE> encode_ack(const Ack &message) noexcept;
[[nodiscard]] Encoded<ERROR_HEADER_SIZE + ERROR_MAX_DETAIL> encode_error(const ErrorMessage &message) noexcept;
/** 带诊断尾部时 52 B，否则 28 B。 */
[[nodiscard]] Encoded<SYSTEM_STATUS_FULL_SIZE> encode_system_status(const SystemStatus &message) noexcept;
[[nodiscard]] Encoded<CONTROL_SNAPSHOT_SIZE> encode_control_snapshot(const ControlSnapshot &message) noexcept;
[[nodiscard]] Encoded<GIMBAL_ATTITUDE_SIZE> encode_gimbal_attitude(const GimbalAttitude &message) noexcept;

/** device_type 不是 0 或 1 时拒收；心跳周期 0 按 50 ms。 */
[[nodiscard]] bool decode_hello(const std::uint8_t *payload, std::size_t length, Hello &out) noexcept;
/** status 不是 0–2、reserved 非 0 时拒收；心跳周期 0 按 50 ms。 */
[[nodiscard]] bool decode_hello_ack(const std::uint8_t *payload, std::size_t length, HelloAck &out) noexcept;
/** ack_state、error_code 越界或 reserved 非 0 时拒收。 */
[[nodiscard]] bool decode_ack(const std::uint8_t *payload, std::size_t length, Ack &out) noexcept;
/** 详情超过 64 B 时截断。 */
[[nodiscard]] bool decode_error(const std::uint8_t *payload, std::size_t length, ErrorMessage &out) noexcept;
/** 保留字节非 0 或 link_state 越界时拒收；剩余 ≥ 24 B 时读诊断尾部。未知 mode 按 SAFETY、来源按 NONE、
 *  安全状态按控制失联。 */
[[nodiscard]] bool decode_system_status(const std::uint8_t *payload, std::size_t length, SystemStatus &out) noexcept;
/** flags 保留位或 reserved 非 0、生效子系统的浮点不是有限值时拒收。未知 source 按 NONE、mode 按 IDLE；
 *  validity_hint_ms ≥ 50 按 50。 */
[[nodiscard]] bool
decode_control_snapshot(const std::uint8_t *payload, std::size_t length, ControlSnapshot &out) noexcept;
/** 未知 imu_state 按 SETTLING；flags 保留位与 reserved 忽略。 */
[[nodiscard]] bool
decode_gimbal_attitude(const std::uint8_t *payload, std::size_t length, GimbalAttitude &out) noexcept;

} // namespace robolink

#endif // MIDDLEWARE_ROBOLINK_MESSAGES_HPP
