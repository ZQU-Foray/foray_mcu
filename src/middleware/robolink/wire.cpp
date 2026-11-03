#include "middleware/robolink/wire.hpp"

#include "middleware/robolink/bytes.hpp"
#include "utils/crc/crc32.hpp"

#include <algorithm>
#include <cstring>

namespace robolink
{

namespace
{

// §3.1 注册表。槽位标志由类决定：EVENT → EVENT 位，LATEST → LATEST_ONLY 位，STATUS → 不置位，"+ RESPONSE" → RESPONSE 位。
// 载荷最小长度（§7.1）：更长的载荷是末尾追加的新字段，接收方按长度截断读取。
constexpr std::array<MessageInfo, 8> MESSAGES{{
    {MessageType::HELLO, slot_flag::EVENT, 26U, Direction::BOTH},
    {MessageType::HELLO_ACK, slot_flag::EVENT | slot_flag::RESPONSE, 8U, Direction::BOTH},
    {MessageType::HEARTBEAT, 0U, 0U, Direction::BOTH},
    {MessageType::ACK, slot_flag::RESPONSE, 7U, Direction::BOTH},
    {MessageType::ERROR, slot_flag::EVENT, 2U, Direction::BOTH},
    {MessageType::SYSTEM_STATUS, 0U, 28U, Direction::MCU_TO_PC},
    {MessageType::CONTROL_SNAPSHOT, slot_flag::LATEST_ONLY, 48U, Direction::PC_TO_MCU},
    {MessageType::GIMBAL_ATTITUDE, 0U, 36U, Direction::MCU_TO_PC},
}};

// 帧头偏移（§1.1）
constexpr std::size_t OFFSET_VERSION{2U};
constexpr std::size_t OFFSET_FLAGS{3U};
constexpr std::size_t OFFSET_LENGTH{4U};
constexpr std::size_t OFFSET_SEQUENCE{6U};
constexpr std::size_t OFFSET_TIMESTAMP{8U};
constexpr std::size_t OFFSET_SLOT_COUNT{12U};
constexpr std::size_t OFFSET_RESERVED{13U};

bool crc_matches(const std::uint8_t *data, std::size_t size) noexcept
{
    const std::size_t covered = size - CRC_SIZE;
    return crc::crc32_calc(data, covered) == bytes::get_u32(data + covered);
}

// 槽位层（§2.1）：逐个核对槽位，最后核对槽位正好铺满到 CRC 前、ACK_PRESENT 与 ACK 槽位一致
FrameError check_slots(const std::uint8_t *data, std::size_t size, const FrameHeader &header) noexcept
{
    const std::size_t end = size - CRC_SIZE;
    std::size_t offset = HEADER_SIZE;
    bool has_ack = false;
    for (std::uint8_t index = 0U; index < header.slot_count; ++index)
    {
        if (end - offset < SLOT_HEADER_SIZE)
        {
            return FrameError::LAYOUT;
        }
        const std::uint8_t type = data[offset];
        const std::uint8_t flags = data[offset + 1U];
        const std::size_t length = bytes::get_u16(data + offset + 4U);
        offset += SLOT_HEADER_SIZE;
        if (length > MAX_PAYLOAD_SIZE || length > end - offset)
        {
            return FrameError::LAYOUT;
        }
        if ((flags & static_cast<std::uint8_t>(~slot_flag::KNOWN)) != 0U)
        {
            return FrameError::SLOT_FLAGS;
        }
        const MessageInfo *info = find_message(type);
        if (info == nullptr)
        {
            return FrameError::UNKNOWN_TYPE;
        }
        if (flags != info->slot_flags)
        {
            return FrameError::CLASS_MISMATCH;
        }
        if (length < info->min_payload)
        {
            return FrameError::PAYLOAD_TOO_SHORT;
        }
        has_ack = has_ack || info->type == MessageType::ACK;
        offset += length;
    }
    if (offset != end)
    {
        return FrameError::LAYOUT;
    }
    if (((header.flags & frame_flag::ACK_PRESENT) != 0U) != has_ack)
    {
        return FrameError::ACK_FLAG;
    }
    return FrameError::NONE;
}

} // namespace

const MessageInfo *find_message(std::uint8_t type) noexcept
{
    for (const MessageInfo &info : MESSAGES)
    {
        if (static_cast<std::uint8_t>(info.type) == type)
        {
            return &info;
        }
    }
    return nullptr;
}

bool parse_header(const std::uint8_t *data, FrameHeader &header) noexcept
{
    if (data == nullptr || data[0] != MAGIC[0] || data[1] != MAGIC[1] || data[OFFSET_VERSION] != VERSION ||
        data[OFFSET_RESERVED] != 0U)
    {
        return false;
    }
    const FrameHeader parsed{data[OFFSET_FLAGS],
                             bytes::get_u16(data + OFFSET_LENGTH),
                             bytes::get_u16(data + OFFSET_SEQUENCE),
                             bytes::get_u32(data + OFFSET_TIMESTAMP),
                             data[OFFSET_SLOT_COUNT]};
    const bool keepalive = (parsed.flags & frame_flag::KEEPALIVE) != 0U;
    // 零槽只允许 KEEPALIVE 帧，有槽位时不得置 KEEPALIVE
    if ((parsed.flags & static_cast<std::uint8_t>(~frame_flag::KNOWN)) != 0U || parsed.length < MIN_FRAME_SIZE ||
        parsed.length > MAX_FRAME_SIZE || keepalive != (parsed.slot_count == 0U))
    {
        return false;
    }
    header = parsed;
    return true;
}

FrameError check_frame(const std::uint8_t *data, std::size_t size) noexcept
{
    FrameHeader header{};
    if (data == nullptr || size < HEADER_SIZE || !parse_header(data, header) || header.length != size)
    {
        return FrameError::HEADER;
    }
    if (!crc_matches(data, size))
    {
        return FrameError::CRC;
    }
    return check_slots(data, size, header);
}

// SlotReader

SlotReader::SlotReader(const std::uint8_t *frame, std::size_t size) noexcept
    : frame_{frame}, end_{size >= MIN_FRAME_SIZE ? size - CRC_SIZE : 0U}, offset_{HEADER_SIZE},
      remaining_{size >= MIN_FRAME_SIZE ? frame[OFFSET_SLOT_COUNT] : std::uint8_t{0U}}
{
}

bool SlotReader::next(Slot &slot) noexcept
{
    if (remaining_ == 0U || end_ < offset_ || end_ - offset_ < SLOT_HEADER_SIZE)
    {
        return false;
    }
    const std::uint8_t *head = frame_ + offset_;
    const std::uint16_t length = bytes::get_u16(head + 4U);
    if (length > end_ - offset_ - SLOT_HEADER_SIZE)
    {
        remaining_ = 0U;
        return false;
    }
    slot = Slot{head[0], head[1], bytes::get_u16(head + 2U), head + SLOT_HEADER_SIZE, length};
    offset_ += SLOT_HEADER_SIZE + length;
    --remaining_;
    return true;
}

// FrameBuilder

FrameBuilder::FrameBuilder(std::uint8_t *buffer, std::size_t capacity) noexcept
    : buffer_{buffer}, capacity_{buffer == nullptr ? 0U : std::min(capacity, MAX_FRAME_SIZE)}
{
}

void FrameBuilder::begin(std::uint16_t sequence, std::uint32_t timestamp_us) noexcept
{
    if (capacity_ < MIN_FRAME_SIZE)
    {
        open_ = false;
        size_ = 0U;
        return;
    }
    buffer_[0] = MAGIC[0];
    buffer_[1] = MAGIC[1];
    buffer_[OFFSET_VERSION] = VERSION;
    buffer_[OFFSET_FLAGS] = 0U;
    bytes::put_u16(buffer_ + OFFSET_LENGTH, 0U);
    bytes::put_u16(buffer_ + OFFSET_SEQUENCE, sequence);
    bytes::put_u32(buffer_ + OFFSET_TIMESTAMP, timestamp_us);
    buffer_[OFFSET_SLOT_COUNT] = 0U;
    buffer_[OFFSET_RESERVED] = 0U;
    size_ = HEADER_SIZE;
    slot_count_ = 0U;
    has_ack_ = false;
    open_ = true;
}

bool FrameBuilder::fits(std::size_t length) const noexcept
{
    return open_ && length <= MAX_PAYLOAD_SIZE && slot_count_ < 0xFFU &&
           capacity_ - size_ >= SLOT_HEADER_SIZE + length + CRC_SIZE;
}

bool FrameBuilder::add(MessageType type,
                       std::uint16_t sequence,
                       const std::uint8_t *payload,
                       std::size_t length) noexcept
{
    const MessageInfo *info = find_message(static_cast<std::uint8_t>(type));
    if (info == nullptr || !fits(length) || (payload == nullptr && length != 0U))
    {
        return false;
    }
    std::uint8_t *head = buffer_ + size_;
    head[0] = static_cast<std::uint8_t>(type);
    head[1] = info->slot_flags;
    bytes::put_u16(head + 2U, sequence);
    bytes::put_u16(head + 4U, static_cast<std::uint16_t>(length));
    if (length != 0U)
    {
        std::memcpy(head + SLOT_HEADER_SIZE, payload, length);
    }
    size_ += SLOT_HEADER_SIZE + length;
    ++slot_count_;
    has_ack_ = has_ack_ || type == MessageType::ACK;
    return true;
}

std::size_t FrameBuilder::finish() noexcept
{
    if (!open_)
    {
        return 0U;
    }
    open_ = false;
    const std::uint8_t flags = static_cast<std::uint8_t>((has_ack_ ? frame_flag::ACK_PRESENT : 0U) |
                                                         (slot_count_ == 0U ? frame_flag::KEEPALIVE : 0U));
    buffer_[OFFSET_FLAGS] = flags;
    buffer_[OFFSET_SLOT_COUNT] = slot_count_;
    const std::size_t total = size_ + CRC_SIZE;
    bytes::put_u16(buffer_ + OFFSET_LENGTH, static_cast<std::uint16_t>(total));
    bytes::put_u32(buffer_ + size_, crc::crc32_calc(buffer_, size_));
    size_ = total;
    return total;
}

// FrameParser

std::size_t FrameParser::push(const std::uint8_t *data, std::size_t length) noexcept
{
    if (release_ != 0U)
    {
        drop(release_);
        release_ = 0U;
    }
    if (data == nullptr)
    {
        return 0U;
    }
    const std::size_t accepted = std::min(length, buffer_.size() - length_);
    std::memcpy(buffer_.data() + length_, data, accepted);
    length_ += accepted;
    return accepted;
}

std::size_t FrameParser::free_space() const noexcept
{
    return buffer_.size() - length_ + release_;
}

bool FrameParser::next(Frame &frame) noexcept
{
    if (release_ != 0U)
    {
        drop(release_);
        release_ = 0U;
    }
    while (length_ > 0U)
    {
        // 找魔数：末尾只剩一个 0xD3 时留着，下一包可能以 0x91 开头
        std::size_t start = 0U;
        while (start + 1U < length_ && !(buffer_[start] == MAGIC[0] && buffer_[start + 1U] == MAGIC[1]))
        {
            ++start;
        }
        if (start + 1U >= length_ && buffer_[start] != MAGIC[0])
        {
            ++start; // 最后一个字节也不是魔数的开头
        }
        if (start != 0U)
        {
            statistics_.discarded_bytes += static_cast<std::uint32_t>(start);
            drop(start);
        }
        if (length_ < HEADER_SIZE)
        {
            return false;
        }

        FrameHeader header{};
        if (!parse_header(buffer_.data(), header))
        {
            // 假同步：从候选头的第二字节起重扫，代价不超过一个帧头
            ++statistics_.header_errors;
            last_error_ = FrameError::HEADER;
            drop(1U);
            continue;
        }
        if (length_ < header.length)
        {
            return false; // 半帧：等后续字节（候选帧在缓冲区开头，最长的帧也放得下）
        }
        if (!crc_matches(buffer_.data(), header.length))
        {
            ++statistics_.crc_errors;
            last_error_ = FrameError::CRC;
            drop(header.length);
            continue;
        }
        const FrameError error = check_slots(buffer_.data(), header.length, header);
        if (error != FrameError::NONE)
        {
            ++statistics_.frame_errors;
            if (error == FrameError::UNKNOWN_TYPE)
            {
                ++statistics_.unknown_types;
            }
            last_error_ = error;
            drop(header.length);
            continue;
        }
        ++statistics_.frames;
        frame = Frame{buffer_.data(), header.length, header};
        release_ = header.length;
        return true;
    }
    return false;
}

void FrameParser::reset() noexcept
{
    length_ = 0U;
    release_ = 0U;
    statistics_ = Statistics{};
    last_error_ = FrameError::NONE;
}

void FrameParser::drop(std::size_t count) noexcept
{
    const std::size_t dropped = std::min(count, length_);
    std::memmove(buffer_.data(), buffer_.data() + dropped, length_ - dropped);
    length_ -= dropped;
}

} // namespace robolink
