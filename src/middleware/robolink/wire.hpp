#ifndef MIDDLEWARE_ROBOLINK_WIRE_HPP
#define MIDDLEWARE_ROBOLINK_WIRE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

// RoboLink v0.2 的帧层与槽位层（foray_interfaces protocol/robolink.md §1–§3）。
// 本仓唯一组 RoboLink 帧、算链路 CRC 的地方（规则 11）。字节布局由独立生成的标准帧
// test/data/robolink/golden_v0_2.txt 逐字节核对（tool/robolink/make_golden.py）。
// 只处理内存里的字节：不碰外设、不分配内存、不阻塞，任务与宿主测试里都能用；对象不是线程安全的，一个对象只归一个任务。

namespace robolink
{

/** 协议主版本（帧头 Version、HELLO 的 proto_ver_major）。 */
inline constexpr std::uint8_t VERSION{0x02U};
inline constexpr std::array<std::uint8_t, 2> MAGIC{0xD3U, 0x91U};
inline constexpr std::size_t HEADER_SIZE{14U};
inline constexpr std::size_t SLOT_HEADER_SIZE{6U};
inline constexpr std::size_t CRC_SIZE{4U};
inline constexpr std::size_t MIN_FRAME_SIZE{HEADER_SIZE + CRC_SIZE};
inline constexpr std::size_t MAX_FRAME_SIZE{512U};
inline constexpr std::size_t MAX_PAYLOAD_SIZE{486U};
/** §6：MCU 发送路径的整帧上限（MCU 不发长帧）。 */
inline constexpr std::size_t MCU_MAX_FRAME_SIZE{96U};

/** §1.2 帧头 Frame Flags。 */
namespace frame_flag
{
inline constexpr std::uint8_t ACK_PRESENT{0x01U}; // 帧内含 ACK 槽位
inline constexpr std::uint8_t KEEPALIVE{0x02U};   // 零槽保活帧
inline constexpr std::uint8_t KNOWN{0x03U};
} // namespace frame_flag

/** §2.1 槽位头 Slot Flags。 */
namespace slot_flag
{
inline constexpr std::uint8_t ACK_REQ{0x01U};
inline constexpr std::uint8_t LATEST_ONLY{0x02U};
inline constexpr std::uint8_t EVENT{0x04U};
inline constexpr std::uint8_t RESPONSE{0x08U};
inline constexpr std::uint8_t KNOWN{0x0FU};
} // namespace slot_flag

/** §3.1 第一版消息集。 */
enum class MessageType : std::uint8_t
{
    HELLO = 0x01U,
    HELLO_ACK = 0x02U,
    HEARTBEAT = 0x03U,
    ACK = 0x04U,
    ERROR = 0x05U,
    SYSTEM_STATUS = 0x10U,
    CONTROL_SNAPSHOT = 0x20U,
    GIMBAL_ATTITUDE = 0x30U
};

enum class Direction : std::uint8_t
{
    BOTH = 0U,
    MCU_TO_PC = 1U,
    PC_TO_MCU = 2U
};

/** 注册表的一行：槽位标志由消息的类决定（EVENT、LATEST、STATUS，及 RESPONSE），载荷不得短于 min_payload。 */
struct MessageInfo
{
    MessageType type;
    std::uint8_t slot_flags;
    std::uint16_t min_payload;
    Direction direction;
};

/** 查注册表；未注册的类型返回 nullptr。 */
[[nodiscard]] const MessageInfo *find_message(std::uint8_t type) noexcept;

/** HELLO 的 capabilities：bit n ⇔ Type n（Type 必须 < 64）。 */
[[nodiscard]] constexpr std::uint64_t capability_bit(MessageType type) noexcept
{
    return std::uint64_t{1U} << static_cast<std::uint8_t>(type);
}

/** §1.2 帧头的字段（魔数、版本、保留字节已核对，不再保存）。 */
struct FrameHeader
{
    std::uint8_t flags;
    std::uint16_t length; // 整帧字节数，含帧头与 CRC
    std::uint16_t sequence;
    std::uint32_t timestamp_us; // 发送端单调时钟
    std::uint8_t slot_count;
};

/** 整帧被拒的原因。 */
enum class FrameError : std::uint8_t
{
    NONE = 0U,
    HEADER = 1U, // 魔数、版本、未知标志位、长度越界、槽位数与保活位矛盾、保留字节非 0
    CRC = 2U,    // CRC-32 不符
    LAYOUT = 3U, // 槽位越过帧尾、载荷超过 486 B、槽位数与内容不符、帧尾有多余字节
    UNKNOWN_TYPE = 4U,      // 未注册的消息类型
    SLOT_FLAGS = 5U,        // 槽位标志有未知位
    CLASS_MISMATCH = 6U,    // 槽位标志与注册表的类不一致
    PAYLOAD_TOO_SHORT = 7U, // 载荷短于注册表的最小长度
    ACK_FLAG = 8U           // 帧头 ACK_PRESENT 与有没有 ACK 槽位不一致
};

/**
 * @brief 解析并核对 14 B 帧头：魔数、版本、标志位、长度 ∈ [18, 512]、槽位数与 KEEPALIVE、保留字节。
 * @param data 至少 HEADER_SIZE 字节。
 */
[[nodiscard]] bool parse_header(const std::uint8_t *data, FrameHeader &header) noexcept;

/** 核对一整帧：帧头、帧头长度等于 size、CRC、槽位布局与注册表。 */
[[nodiscard]] FrameError check_frame(const std::uint8_t *data, std::size_t size) noexcept;

/** 一个槽位：载荷指向帧内，随帧失效。 */
struct Slot
{
    std::uint8_t type;
    std::uint8_t flags;
    std::uint16_t sequence;
    const std::uint8_t *payload;
    std::uint16_t length;
};

/** 按顺序取出一帧里的槽位。只用于 check_frame() 通过的帧（或 FrameParser 交出的帧）。 */
class SlotReader final
{
  public:
    SlotReader(const std::uint8_t *frame, std::size_t size) noexcept;

    [[nodiscard]] bool next(Slot &slot) noexcept;

  private:
    const std::uint8_t *frame_;
    std::size_t end_;
    std::size_t offset_;
    std::uint8_t remaining_;
};

/**
 * @brief 定长编码结果：bytes 的前 size 个字节有效。
 */
template <std::size_t CAPACITY>
struct Encoded
{
    std::array<std::uint8_t, CAPACITY> bytes{};
    std::size_t size{0U};
};

/**
 * @brief 在调用方给的缓冲区里组一帧：begin() → add() 若干次 → finish()。
 *
 * 槽位标志按注册表填写；帧头的 ACK_PRESENT、KEEPALIVE、长度、槽位数与 CRC 由 finish() 填写。
 */
class FrameBuilder final
{
  public:
    /** capacity 是本端发送的整帧上限（MCU 为 MCU_MAX_FRAME_SIZE），大于 MAX_FRAME_SIZE 时按 MAX_FRAME_SIZE。 */
    FrameBuilder(std::uint8_t *buffer, std::size_t capacity) noexcept;

    void begin(std::uint16_t sequence, std::uint32_t timestamp_us) noexcept;

    /** 还能不能放下一个载荷为 length 字节的槽位（含 CRC）。 */
    [[nodiscard]] bool fits(std::size_t length) const noexcept;

    /**
     * @brief 追加一个槽位。
     * @return 类型未注册、载荷超过 486 B、放不下、或已经 finish() 时返回 false，已写的内容不变。
     */
    [[nodiscard]] bool
    add(MessageType type, std::uint16_t sequence, const std::uint8_t *payload, std::size_t length) noexcept;

    template <std::size_t CAPACITY>
    [[nodiscard]] bool add(MessageType type, std::uint16_t sequence, const Encoded<CAPACITY> &payload) noexcept
    {
        return add(type, sequence, payload.bytes.data(), payload.size);
    }

    /** 写帧头的标志、长度、槽位数与 CRC。返回整帧长度；begin() 之前调用返回 0。 */
    std::size_t finish() noexcept;

    [[nodiscard]] const std::uint8_t *data() const noexcept
    {
        return buffer_;
    }

    [[nodiscard]] std::size_t size() const noexcept
    {
        return size_;
    }

    [[nodiscard]] std::uint8_t slot_count() const noexcept
    {
        return slot_count_;
    }

  private:
    std::uint8_t *buffer_;
    std::size_t capacity_;
    std::size_t size_{0U};
    std::uint8_t slot_count_{0U};
    bool has_ack_{false};
    bool open_{false};
};

/** FrameParser 交出的一帧：data 指向解析器内部缓冲区，有效到下一次 push()、next() 或 reset()。 */
struct Frame
{
    const std::uint8_t *data;
    std::size_t size;
    FrameHeader header;
};

/**
 * @brief 字节流里的帧定界（§1.4）：容忍半帧、粘包、多帧连发。
 *
 * - 先找魔数；帧头不合法时从候选头的第二字节起重新扫描；
 * - CRC 不符或槽位不合法时丢弃整帧，从帧尾之后继续扫描；
 * - 缓冲区正好装下最长的帧：候选帧总在缓冲区开头，等字节到齐时不会溢出。
 *
 * 用法：每轮 push() 收到的字节（一次最多 free_space()），再调用 next() 若干次（§1.4 建议每轮最多 8 帧）。
 */
class FrameParser final
{
  public:
    /** 计数从 reset() 起累计，按 2^32 回绕。 */
    struct Statistics
    {
        std::uint32_t frames{0U};          // 交出的合法帧
        std::uint32_t crc_errors{0U};      // CRC 不符而丢弃的帧
        std::uint32_t header_errors{0U};   // 魔数后的帧头不合法（假同步）
        std::uint32_t frame_errors{0U};    // CRC 正确、槽位层不合法而丢弃的帧
        std::uint32_t unknown_types{0U};   // 其中因未注册类型丢弃的帧
        std::uint32_t discarded_bytes{0U}; // 找魔数时跳过的字节
    };

    /** 追加收到的字节，返回接受了多少（缓冲区满时少于 length，调用方下一轮再给）。 */
    std::size_t push(const std::uint8_t *data, std::size_t length) noexcept;

    [[nodiscard]] std::size_t free_space() const noexcept;

    /** 找下一个合法帧。没有完整的合法帧时返回 false（已收到的半帧留着等后续字节）。 */
    [[nodiscard]] bool next(Frame &frame) noexcept;

    /** 丢掉缓冲区里的字节，计数保留（传输层断开重连）。 */
    void clear() noexcept;

    /** 清空缓冲区与计数。 */
    void reset() noexcept;

    [[nodiscard]] const Statistics &statistics() const noexcept
    {
        return statistics_;
    }

    /** 最近一次丢弃整帧的原因（诊断用）。 */
    [[nodiscard]] FrameError last_error() const noexcept
    {
        return last_error_;
    }

  private:
    void drop(std::size_t count) noexcept;

    std::array<std::uint8_t, MAX_FRAME_SIZE> buffer_{};
    std::size_t length_{0U};
    std::size_t release_{0U}; // 上一次交出的帧的长度，下一次调用时丢掉
    Statistics statistics_{};
    FrameError last_error_{FrameError::NONE};
};

} // namespace robolink

#endif // MIDDLEWARE_ROBOLINK_WIRE_HPP
