#include "Libraries/Protocol/sbus/Sbus.hpp"

namespace protocol
{

void SbusParser::reset() noexcept
{
    buffer_.fill(0U);
    buffered_byte_count_ = 0U;
    statistics_ = {};
}

std::size_t SbusParser::input(const std::uint8_t *data, std::size_t length, Frame &latest_frame) noexcept
{
    if (data == nullptr || length == 0U)
    {
        return 0U;
    }

    std::size_t decoded_frame_count = 0U;
    for (std::size_t input_index = 0U; input_index < length; ++input_index)
    {
        const std::uint8_t byte = data[input_index];
        if (buffered_byte_count_ == 0U && byte != START_BYTE)
        {
            ++statistics_.discarded_byte_count;
            continue;
        }

        buffer_[buffered_byte_count_] = byte;
        ++buffered_byte_count_;
        if (buffered_byte_count_ != FRAME_SIZE)
        {
            continue;
        }

        Frame decoded_frame{};
        if (decode_frame(buffer_, decoded_frame))
        {
            latest_frame = decoded_frame;
            ++decoded_frame_count;
            ++statistics_.accepted_frame_count;
            buffered_byte_count_ = 0U;
            continue;
        }

        ++statistics_.rejected_frame_count;
        recover_after_rejected_frame();
    }

    return decoded_frame_count;
}

bool SbusParser::decode_frame(const std::array<std::uint8_t, FRAME_SIZE> &raw_frame, Frame &frame) noexcept
{
    if (raw_frame[0] != START_BYTE || !is_supported_end_byte(raw_frame[24]))
    {
        return false;
    }

    Frame decoded_frame{};
    for (std::size_t channel = 0U; channel < CHANNEL_COUNT; ++channel)
    {
        const std::size_t bit_offset = channel * 11U;
        const std::size_t byte_offset = 1U + bit_offset / 8U;
        const std::uint32_t packed = static_cast<std::uint32_t>(raw_frame[byte_offset]) |
                                     (static_cast<std::uint32_t>(raw_frame[byte_offset + 1U]) << 8U) |
                                     (static_cast<std::uint32_t>(raw_frame[byte_offset + 2U]) << 16U);
        decoded_frame.channels[channel] = static_cast<std::uint16_t>((packed >> (bit_offset % 8U)) & 0x07FFU);
    }

    const std::uint8_t flags = raw_frame[23];
    decoded_frame.digital_channel_17 = (flags & (1U << 0U)) != 0U;
    decoded_frame.digital_channel_18 = (flags & (1U << 1U)) != 0U;
    decoded_frame.frame_lost = (flags & (1U << 2U)) != 0U;
    decoded_frame.failsafe = (flags & (1U << 3U)) != 0U;
    frame = decoded_frame;
    return true;
}

bool SbusParser::is_supported_end_byte(std::uint8_t byte) noexcept
{
    return byte == 0x00U || byte == 0x04U || byte == 0x14U || byte == 0x24U || byte == 0x34U;
}

void SbusParser::recover_after_rejected_frame() noexcept
{
    std::size_t next_start = 1U;
    while (next_start < FRAME_SIZE && buffer_[next_start] != START_BYTE)
    {
        ++next_start;
    }

    statistics_.discarded_byte_count += static_cast<std::uint32_t>(next_start);
    if (next_start == FRAME_SIZE)
    {
        buffered_byte_count_ = 0U;
        return;
    }

    buffered_byte_count_ = FRAME_SIZE - next_start;
    for (std::size_t index = 0U; index < buffered_byte_count_; ++index)
    {
        buffer_[index] = buffer_[next_start + index];
    }
}

} // namespace protocol
