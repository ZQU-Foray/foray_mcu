#ifndef REMOTE_REMOTE_RECEIVER_HPP
#define REMOTE_REMOTE_RECEIVER_HPP

#include "driver/protocol/sbus/sbus.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace application
{

class RemoteReceiver final
{
  public:
    static constexpr std::size_t CHANNEL_COUNT{protocol::SbusParser::CHANNEL_COUNT};
    static constexpr std::uint16_t CHANNEL_MINIMUM{321U};
    static constexpr std::uint16_t CHANNEL_MIDDLE{992U};
    static constexpr std::uint16_t CHANNEL_MAXIMUM{1663U};
    static constexpr std::uint32_t LINK_TIMEOUT_MS{100U};

    struct Snapshot final
    {
        std::array<std::uint16_t, CHANNEL_COUNT> channels{};
        bool digital_channel_17{false};
        bool digital_channel_18{false};
        bool frame_lost{false};
        bool failsafe{false};
        bool connected{false};
        std::uint32_t accepted_frame_count{0U};
        std::uint32_t rejected_frame_count{0U};
        std::uint32_t discarded_byte_count{0U};
        std::uint32_t uart_dropped_byte_count{0U};
        std::uint32_t uart_error_event_count{0U};
    };

    RemoteReceiver() = delete;

    // 在调度器启动前调用；只建立协议状态，不接管 UART DMA。
    [[nodiscard]] static bool init() noexcept;

    // 由单一任务周期调用，负责清空 UART5 软件队列并输入流解析器。
    static void process() noexcept;

    [[nodiscard]] static bool is_ready() noexcept;

    // 快照复制接口，可与 Process 所在任务并发调用。
    [[nodiscard]] static bool get_snapshot(Snapshot &snapshot) noexcept;

    // 使用当前接收机实测标定值，将指定通道钳位并归一化到 [-1, 1]。
    [[nodiscard]] static float normalize_channel(const Snapshot &snapshot, std::size_t channel) noexcept;
};

} // namespace application

#endif
