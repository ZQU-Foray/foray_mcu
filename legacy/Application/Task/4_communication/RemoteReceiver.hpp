#ifndef APPLICATION_REMOTE_RECEIVER_HPP
#define APPLICATION_REMOTE_RECEIVER_HPP

#include "Libraries/Protocol/sbus/Sbus.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace application
{

class RemoteReceiver final
{
public:
  static constexpr std::size_t ChannelCount{protocol::SbusParser::ChannelCount};
  static constexpr std::uint16_t ChannelMinimum{321U};
  static constexpr std::uint16_t ChannelMiddle{992U};
  static constexpr std::uint16_t ChannelMaximum{1663U};
  static constexpr std::uint32_t LinkTimeoutMs{100U};

  struct Snapshot final
  {
    std::array<std::uint16_t, ChannelCount> channels{};
    bool digitalChannel17{false};
    bool digitalChannel18{false};
    bool frameLost{false};
    bool failsafe{false};
    bool connected{false};
    std::uint32_t acceptedFrameCount{0U};
    std::uint32_t rejectedFrameCount{0U};
    std::uint32_t discardedByteCount{0U};
    std::uint32_t uartDroppedByteCount{0U};
    std::uint32_t uartErrorEventCount{0U};
  };

  RemoteReceiver() = delete;

  // 在调度器启动前调用；只建立协议状态，不接管 UART DMA。
  [[nodiscard]] static bool Init() noexcept;

  // 由单一任务周期调用，负责清空 UART5 软件队列并输入流解析器。
  static void Process() noexcept;

  [[nodiscard]] static bool IsReady() noexcept;

  // 快照复制接口，可与 Process 所在任务并发调用。
  [[nodiscard]] static bool GetSnapshot(Snapshot &snapshot) noexcept;

  // 使用当前接收机实测标定值，将指定通道钳位并归一化到 [-1, 1]。
  [[nodiscard]] static float NormalizeChannel(const Snapshot &snapshot, std::size_t channel) noexcept;
};

} // namespace application

#endif
