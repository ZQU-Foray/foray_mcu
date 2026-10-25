#ifndef LIBRARIES_PROTOCOL_SBUS_SBUS_HPP
#define LIBRARIES_PROTOCOL_SBUS_SBUS_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace protocol
{

class SbusParser final
{
public:
  static constexpr std::size_t FrameSize{25U};
  static constexpr std::size_t ChannelCount{16U};
  static constexpr std::uint8_t StartByte{0x0FU};

  struct Frame final
  {
    std::array<std::uint16_t, ChannelCount> channels{};
    bool digitalChannel17{false};
    bool digitalChannel18{false};
    bool frameLost{false};
    bool failsafe{false};
  };

  struct Statistics final
  {
    std::uint32_t acceptedFrameCount{0U};
    std::uint32_t rejectedFrameCount{0U};
    std::uint32_t discardedByteCount{0U};
  };

  SbusParser() = default;

  void Reset() noexcept;

  // 接受任意分段的连续字节流。一次调用可解析多帧，latestFrame 返回
  // 最后一帧，返回值为本次调用成功解析的帧数。
  [[nodiscard]] std::size_t Input(const std::uint8_t *data, std::size_t length, Frame &latestFrame) noexcept;

  [[nodiscard]] const Statistics &GetStatistics() const noexcept
  {
    return statistics_;
  }

  // 仅解析一帧，不修改流解析器状态，供协议测试和离线数据使用。
  [[nodiscard]] static bool DecodeFrame(const std::array<std::uint8_t, FrameSize> &rawFrame, Frame &frame) noexcept;

private:
  [[nodiscard]] static bool IsSupportedEndByte(std::uint8_t byte) noexcept;
  void RecoverAfterRejectedFrame() noexcept;

  std::array<std::uint8_t, FrameSize> buffer_{};
  std::size_t bufferedByteCount_{0U};
  Statistics statistics_{};
};

} // namespace protocol

#endif
