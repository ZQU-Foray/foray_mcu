// hal::Can 的宿主替身（Decision-20261101-01）。参数检查、结果优先级与 DM-MC02 的 port 一致。
#include "hal/can.hpp"
#include "hal/os/clock.hpp"
#include "mock_hal.hpp"

#include <array>
#include <deque>
#include <mutex>

namespace
{

constexpr std::uint32_t STANDARD_IDENTIFIER_MAX{0x7FFU};
constexpr std::uint32_t EXTENDED_IDENTIFIER_MAX{0x1FFFFFFFU};

struct Bus
{
    bool ready{true};
    bool bus_off{false};
    bool receiver_claimed{false};
    hal::Can::SendResult send_result{hal::Can::SendResult::QUEUED};
    std::deque<hal::Can::Frame> received{};
    std::deque<hal::Can::Frame> sent{};
    bool has_last_sent{false};
    hal::Can::Frame last_sent{};
    hal::Can::Statistics statistics{};
    hal::Callback notification{nullptr};
    void *context{nullptr};
};

std::mutex g_mutex;
std::array<Bus, mock::can::BUS_COUNT> g_buses{};

} // namespace

namespace hal
{

bool Can::is_ready() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return index_ < g_buses.size() && g_buses[index_].ready && !g_buses[index_].bus_off;
}

bool Can::claim_receiver() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_buses.size() || g_buses[index_].receiver_claimed)
    {
        return false;
    }
    g_buses[index_].receiver_claimed = true;
    return true;
}

Can::SendResult Can::send(const Frame &frame) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    const bool standard = frame.identifier_type == IdentifierType::STANDARD;
    if (index_ >= g_buses.size() || frame.length > MAX_DATA_LENGTH ||
        frame.identifier > (standard ? STANDARD_IDENTIFIER_MAX : EXTENDED_IDENTIFIER_MAX))
    {
        return SendResult::INVALID_ARGUMENT;
    }
    Bus &bus = g_buses[index_];
    if (!bus.ready)
    {
        return SendResult::NOT_READY;
    }
    if (bus.bus_off)
    {
        return SendResult::BUS_OFF;
    }
    if (bus.send_result != SendResult::QUEUED)
    {
        return bus.send_result;
    }
    if (bus.sent.size() >= mock::can::QUEUE_CAPACITY)
    {
        bus.sent.pop_front(); // 记录只保留最近的帧
    }
    bus.sent.push_back(frame);
    bus.last_sent = frame;
    bus.has_last_sent = true;
    return SendResult::QUEUED;
}

Can::ReceiveResult Can::receive(Frame &frame) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_buses.size())
    {
        return ReceiveResult::INVALID_ARGUMENT;
    }
    Bus &bus = g_buses[index_];
    if (!bus.received.empty())
    {
        frame = bus.received.front();
        bus.received.pop_front();
        return ReceiveResult::RECEIVED;
    }
    if (!bus.ready)
    {
        return ReceiveResult::NOT_READY;
    }
    return bus.bus_off ? ReceiveResult::BUS_OFF : ReceiveResult::EMPTY;
}

bool Can::set_receive_notification(Callback callback, void *context) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_buses.size())
    {
        return false;
    }
    g_buses[index_].notification = callback;
    g_buses[index_].context = context;
    return true;
}

Can::Statistics Can::statistics() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return index_ < g_buses.size() ? g_buses[index_].statistics : Statistics{};
}

} // namespace hal

namespace mock::can
{

void reset() noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    for (Bus &bus : g_buses)
    {
        bus = Bus{};
    }
}

void set_ready(std::uint8_t bus, bool ready) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (bus < g_buses.size())
    {
        g_buses[bus].ready = ready;
    }
}

void set_bus_off(std::uint8_t bus, bool bus_off) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (bus < g_buses.size())
    {
        g_buses[bus].bus_off = bus_off;
    }
}

void set_send_result(std::uint8_t bus, hal::Can::SendResult result) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (bus < g_buses.size())
    {
        g_buses[bus].send_result = result;
    }
}

bool inject(std::uint8_t bus, const hal::Can::Frame &frame) noexcept
{
    // 与 port 一样，入队时写接收时刻（模拟时刻）
    hal::Can::Frame stamped = frame;
    stamped.received_us = hal::os::now_us();
    hal::Callback notification = nullptr;
    void *context = nullptr;
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        if (bus >= g_buses.size())
        {
            return false;
        }
        Bus &target = g_buses[bus];
        if (target.received.size() >= QUEUE_CAPACITY)
        {
            ++target.statistics.rx_dropped_count;
            return false;
        }
        target.received.push_back(stamped);
        notification = target.notification;
        context = target.context;
    }
    if (notification != nullptr)
    {
        notification(context); // 与 port 一样在帧入队之后通知；在锁外调用，回调可以再调用 hal::Can
    }
    return true;
}

bool inject(std::uint8_t bus, std::uint32_t identifier, const std::array<std::uint8_t, 8> &data) noexcept
{
    hal::Can::Frame frame{};
    frame.identifier = identifier;
    frame.identifier_type = hal::Can::IdentifierType::STANDARD;
    frame.length = 8U;
    frame.data = data;
    return inject(bus, frame);
}

std::size_t sent_count(std::uint8_t bus) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return bus < g_buses.size() ? g_buses[bus].sent.size() : 0U;
}

bool pop_sent(std::uint8_t bus, hal::Can::Frame &frame) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (bus >= g_buses.size() || g_buses[bus].sent.empty())
    {
        return false;
    }
    frame = g_buses[bus].sent.front();
    g_buses[bus].sent.pop_front();
    return true;
}

bool last_sent(std::uint8_t bus, hal::Can::Frame &frame) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (bus >= g_buses.size() || !g_buses[bus].has_last_sent)
    {
        return false;
    }
    frame = g_buses[bus].last_sent;
    return true;
}

void set_statistics(std::uint8_t bus, const hal::Can::Statistics &statistics) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (bus < g_buses.size())
    {
        g_buses[bus].statistics = statistics;
    }
}

} // namespace mock::can
