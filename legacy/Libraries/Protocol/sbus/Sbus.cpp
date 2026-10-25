#include "Libraries/Protocol/sbus/Sbus.hpp"

namespace protocol
{

void SbusParser::Reset() noexcept
{
  buffer_.fill(0U);
  bufferedByteCount_ = 0U;
  statistics_ = {};
}

std::size_t SbusParser::Input(const std::uint8_t *data, std::size_t length, Frame &latestFrame) noexcept
{
  if (data == nullptr || length == 0U)
  {
    return 0U;
  }

  std::size_t decodedFrameCount = 0U;
  for (std::size_t inputIndex = 0U; inputIndex < length; ++inputIndex)
  {
    const std::uint8_t byte = data[inputIndex];
    if (bufferedByteCount_ == 0U && byte != StartByte)
    {
      ++statistics_.discardedByteCount;
      continue;
    }

    buffer_[bufferedByteCount_] = byte;
    ++bufferedByteCount_;
    if (bufferedByteCount_ != FrameSize)
    {
      continue;
    }

    Frame decodedFrame{};
    if (DecodeFrame(buffer_, decodedFrame))
    {
      latestFrame = decodedFrame;
      ++decodedFrameCount;
      ++statistics_.acceptedFrameCount;
      bufferedByteCount_ = 0U;
      continue;
    }

    ++statistics_.rejectedFrameCount;
    RecoverAfterRejectedFrame();
  }

  return decodedFrameCount;
}

bool SbusParser::DecodeFrame(const std::array<std::uint8_t, FrameSize> &rawFrame, Frame &frame) noexcept
{
  if (rawFrame[0] != StartByte || !IsSupportedEndByte(rawFrame[24]))
  {
    return false;
  }

  Frame decodedFrame{};
  for (std::size_t channel = 0U; channel < ChannelCount; ++channel)
  {
    const std::size_t bitOffset = channel * 11U;
    const std::size_t byteOffset = 1U + bitOffset / 8U;
    const std::uint32_t packed = static_cast<std::uint32_t>(rawFrame[byteOffset]) |
                                 (static_cast<std::uint32_t>(rawFrame[byteOffset + 1U]) << 8U) |
                                 (static_cast<std::uint32_t>(rawFrame[byteOffset + 2U]) << 16U);
    decodedFrame.channels[channel] = static_cast<std::uint16_t>((packed >> (bitOffset % 8U)) & 0x07FFU);
  }

  const std::uint8_t flags = rawFrame[23];
  decodedFrame.digitalChannel17 = (flags & (1U << 0U)) != 0U;
  decodedFrame.digitalChannel18 = (flags & (1U << 1U)) != 0U;
  decodedFrame.frameLost = (flags & (1U << 2U)) != 0U;
  decodedFrame.failsafe = (flags & (1U << 3U)) != 0U;
  frame = decodedFrame;
  return true;
}

bool SbusParser::IsSupportedEndByte(std::uint8_t byte) noexcept
{
  return byte == 0x00U || byte == 0x04U || byte == 0x14U || byte == 0x24U || byte == 0x34U;
}

void SbusParser::RecoverAfterRejectedFrame() noexcept
{
  std::size_t nextStart = 1U;
  while (nextStart < FrameSize && buffer_[nextStart] != StartByte)
  {
    ++nextStart;
  }

  statistics_.discardedByteCount += static_cast<std::uint32_t>(nextStart);
  if (nextStart == FrameSize)
  {
    bufferedByteCount_ = 0U;
    return;
  }

  bufferedByteCount_ = FrameSize - nextStart;
  for (std::size_t index = 0U; index < bufferedByteCount_; ++index)
  {
    buffer_[index] = buffer_[nextStart + index];
  }
}

} // namespace protocol
