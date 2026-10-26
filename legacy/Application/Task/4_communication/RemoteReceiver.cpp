#include "Application/Task/4_communication/RemoteReceiver.hpp"
#include "Time.hpp"
#include "Uart.hpp"
#include <array>
#include <atomic>

namespace application
{
namespace
{

using RemoteEndpoint = platform::Uart::Endpoint;
using ReceiveResult = platform::Uart::ReceiveResult;

static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "Remote receiver snapshot requires lock-free 32-bit atomics");

protocol::SbusParser g_parser{};
protocol::SbusParser::Frame g_latest_frame{};
bool g_has_accepted_frame{false};
bool g_link_timed_out{true};
std::uint32_t g_last_accepted_frame_tick{0U};

std::atomic<std::uint32_t> g_snapshot_sequence{0U};
std::array<std::atomic<std::uint32_t>, RemoteReceiver::CHANNEL_COUNT> g_snapshot_channels{};
std::atomic<std::uint32_t> g_snapshot_flags{0U};
std::atomic<std::uint32_t> g_snapshot_connected{0U};
std::atomic<std::uint32_t> g_snapshot_accepted_frame_count{0U};
std::atomic<std::uint32_t> g_snapshot_rejected_frame_count{0U};
std::atomic<std::uint32_t> g_snapshot_discarded_byte_count{0U};
std::atomic<std::uint32_t> g_snapshot_uart_dropped_byte_count{0U};
std::atomic<std::uint32_t> g_snapshot_uart_error_event_count{0U};
std::atomic<std::uint32_t> g_remote_ready{0U};

constexpr std::uint32_t DIGITAL_CHANNEL_17_FLAG{1U << 0U};
constexpr std::uint32_t DIGITAL_CHANNEL_18_FLAG{1U << 1U};
constexpr std::uint32_t FRAME_LOST_FLAG{1U << 2U};
constexpr std::uint32_t FAILSAFE_FLAG{1U << 3U};

void publish_snapshot(const platform::Uart::Statistics &uart_statistics) noexcept
{
    g_snapshot_sequence.fetch_add(1U, std::memory_order_acq_rel);

    for (std::size_t channel = 0U; channel < RemoteReceiver::CHANNEL_COUNT; ++channel)
    {
        g_snapshot_channels[channel].store(g_latest_frame.channels[channel], std::memory_order_relaxed);
    }

    std::uint32_t flags = 0U;
    flags |= g_latest_frame.digital_channel_17 ? DIGITAL_CHANNEL_17_FLAG : 0U;
    flags |= g_latest_frame.digital_channel_18 ? DIGITAL_CHANNEL_18_FLAG : 0U;
    flags |= g_latest_frame.frame_lost ? FRAME_LOST_FLAG : 0U;
    flags |= g_latest_frame.failsafe ? FAILSAFE_FLAG : 0U;
    g_snapshot_flags.store(flags, std::memory_order_relaxed);
    g_snapshot_connected.store(g_has_accepted_frame && !g_link_timed_out && !g_latest_frame.failsafe ? 1U : 0U,
                               std::memory_order_relaxed);

    const protocol::SbusParser::Statistics &parser_statistics = g_parser.get_statistics();
    g_snapshot_accepted_frame_count.store(parser_statistics.accepted_frame_count, std::memory_order_relaxed);
    g_snapshot_rejected_frame_count.store(parser_statistics.rejected_frame_count, std::memory_order_relaxed);
    g_snapshot_discarded_byte_count.store(parser_statistics.discarded_byte_count, std::memory_order_relaxed);
    g_snapshot_uart_dropped_byte_count.store(uart_statistics.dropped_byte_count, std::memory_order_relaxed);
    g_snapshot_uart_error_event_count.store(uart_statistics.error_event_count, std::memory_order_relaxed);

    g_snapshot_sequence.fetch_add(1U, std::memory_order_release);
}

} // namespace

bool RemoteReceiver::init() noexcept
{
    g_remote_ready.store(0U, std::memory_order_relaxed);
    g_parser.reset();
    g_latest_frame = {};
    g_has_accepted_frame = false;
    g_link_timed_out = true;
    g_last_accepted_frame_tick = platform::Time::now_ticks();

    platform::Uart::Statistics uart_statistics{};
    publish_snapshot(uart_statistics);
    if (!platform::Time::is_ready() || !platform::Uart::is_ready(RemoteEndpoint::REMOTE_RECEIVER))
    {
        return false;
    }

    g_remote_ready.store(1U, std::memory_order_release);
    return true;
}

void RemoteReceiver::process() noexcept
{
    if (!is_ready())
    {
        return;
    }

    std::array<std::uint8_t, 64U> received_bytes{};
    bool received_any_bytes = false;
    for (;;)
    {
        std::size_t received_length = 0U;
        const ReceiveResult result =
            platform::Uart::try_read(RemoteEndpoint::REMOTE_RECEIVER, received_bytes, received_length);
        if (result == ReceiveResult::EMPTY)
        {
            break;
        }

        if (result != ReceiveResult::RECEIVED)
        {
            if (result == ReceiveResult::NOT_READY || result == ReceiveResult::ERROR)
            {
                g_remote_ready.store(0U, std::memory_order_release);
            }
            return;
        }

        received_any_bytes = true;
        if (g_parser.input(received_bytes.data(), received_length, g_latest_frame) > 0U)
        {
            g_has_accepted_frame = true;
            g_link_timed_out = false;
            g_last_accepted_frame_tick = platform::Time::now_ticks();
        }
    }

    bool link_state_changed = false;
    if (g_has_accepted_frame && !g_link_timed_out &&
        platform::Time::has_elapsed_ms(g_last_accepted_frame_tick, LINK_TIMEOUT_MS))
    {
        g_link_timed_out = true;
        link_state_changed = true;
    }

    if (!received_any_bytes && !link_state_changed)
    {
        return;
    }

    // 统计为 MAY 能力：Port 未实现时保持全 0 快照，不影响链路状态
    platform::Uart::Statistics uart_statistics{};
    (void)platform::Uart::get_statistics(RemoteEndpoint::REMOTE_RECEIVER, uart_statistics);
    publish_snapshot(uart_statistics);
}

bool RemoteReceiver::is_ready() noexcept
{
    return g_remote_ready.load(std::memory_order_acquire) != 0U && platform::Time::is_ready() &&
           platform::Uart::is_ready(RemoteEndpoint::REMOTE_RECEIVER);
}

bool RemoteReceiver::get_snapshot(Snapshot &snapshot) noexcept
{
    if (!is_ready())
    {
        snapshot = {};
        return false;
    }

    std::uint32_t sequence_before = 0U;
    std::uint32_t sequence_after = 0U;
    std::uint32_t flags = 0U;
    std::uint32_t connected = 0U;
    do
    {
        sequence_before = g_snapshot_sequence.load(std::memory_order_acquire);
        if ((sequence_before & 1U) != 0U)
        {
            continue;
        }

        for (std::size_t channel = 0U; channel < CHANNEL_COUNT; ++channel)
        {
            snapshot.channels[channel] =
                static_cast<std::uint16_t>(g_snapshot_channels[channel].load(std::memory_order_relaxed));
        }
        flags = g_snapshot_flags.load(std::memory_order_relaxed);
        connected = g_snapshot_connected.load(std::memory_order_relaxed);
        snapshot.accepted_frame_count = g_snapshot_accepted_frame_count.load(std::memory_order_relaxed);
        snapshot.rejected_frame_count = g_snapshot_rejected_frame_count.load(std::memory_order_relaxed);
        snapshot.discarded_byte_count = g_snapshot_discarded_byte_count.load(std::memory_order_relaxed);
        snapshot.uart_dropped_byte_count = g_snapshot_uart_dropped_byte_count.load(std::memory_order_relaxed);
        snapshot.uart_error_event_count = g_snapshot_uart_error_event_count.load(std::memory_order_relaxed);
        sequence_after = g_snapshot_sequence.load(std::memory_order_acquire);
    } while (sequence_before != sequence_after || (sequence_after & 1U) != 0U);

    snapshot.digital_channel_17 = (flags & DIGITAL_CHANNEL_17_FLAG) != 0U;
    snapshot.digital_channel_18 = (flags & DIGITAL_CHANNEL_18_FLAG) != 0U;
    snapshot.frame_lost = (flags & FRAME_LOST_FLAG) != 0U;
    snapshot.failsafe = (flags & FAILSAFE_FLAG) != 0U;
    snapshot.connected = connected != 0U;
    return true;
}

float RemoteReceiver::normalize_channel(const Snapshot &snapshot, std::size_t channel) noexcept
{
    if (channel >= CHANNEL_COUNT)
    {
        return 0.0F;
    }

    std::uint16_t value = snapshot.channels[channel];
    if (value < CHANNEL_MINIMUM)
    {
        value = CHANNEL_MINIMUM;
    }
    else if (value > CHANNEL_MAXIMUM)
    {
        value = CHANNEL_MAXIMUM;
    }

    return static_cast<float>(static_cast<std::int32_t>(value) - static_cast<std::int32_t>(CHANNEL_MIDDLE)) /
           static_cast<float>(CHANNEL_MAXIMUM - CHANNEL_MIDDLE);
}

} // namespace application
