// hal::Uart 的宿主替身。参数检查与结果优先级与 DM-MC02 的 port 一致。
#include "hal/uart.hpp"
#include "mock_hal.hpp"

#include <array>
#include <deque>
#include <mutex>

namespace
{

struct Port
{
    bool ready{true};
    bool writable{true};
    std::deque<std::uint8_t> received{};
    std::deque<std::uint8_t> written{};
    bool write_pending{false};
    std::size_t pending_length{0U};
    hal::Uart::Statistics statistics{};
    hal::Callback receive_notification{nullptr};
    void *receive_context{nullptr};
    hal::Callback write_notification{nullptr};
    void *write_context{nullptr};
};

std::mutex g_mutex;
std::array<Port, mock::uart::PORT_COUNT> g_ports{};

} // namespace

namespace hal
{

bool Uart::is_ready() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return index_ < g_ports.size() && g_ports[index_].ready;
}

std::size_t Uart::read(std::uint8_t *data, std::size_t capacity) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_ports.size() || data == nullptr)
    {
        return 0U;
    }
    std::deque<std::uint8_t> &received = g_ports[index_].received;
    std::size_t count = 0U;
    while (count < capacity && !received.empty())
    {
        data[count++] = received.front();
        received.pop_front();
    }
    return count;
}

Uart::WriteResult Uart::write(const std::uint8_t *data, std::size_t length) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_ports.size() || data == nullptr || length == 0U || length > MAX_WRITE_LENGTH)
    {
        return WriteResult::INVALID_ARGUMENT;
    }
    Port &port = g_ports[index_];
    if (!port.writable)
    {
        return WriteResult::UNSUPPORTED;
    }
    if (!port.ready)
    {
        return WriteResult::NOT_READY;
    }
    if (port.write_pending)
    {
        return WriteResult::BUSY;
    }
    port.written.insert(port.written.end(), data, data + length);
    port.write_pending = true;
    port.pending_length = length;
    return WriteResult::STARTED;
}

bool Uart::set_receive_notification(Callback callback, void *context) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_ports.size())
    {
        return false;
    }
    g_ports[index_].receive_notification = callback;
    g_ports[index_].receive_context = context;
    return true;
}

bool Uart::set_write_notification(Callback callback, void *context) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_ports.size())
    {
        return false;
    }
    g_ports[index_].write_notification = callback;
    g_ports[index_].write_context = context;
    return true;
}

Uart::Statistics Uart::statistics() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return index_ < g_ports.size() ? g_ports[index_].statistics : Statistics{};
}

} // namespace hal

namespace mock::uart
{

void reset() noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    for (Port &port : g_ports)
    {
        port = Port{};
    }
}

void set_capabilities(std::uint8_t port, bool ready, bool writable) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (port < g_ports.size())
    {
        g_ports[port].ready = ready;
        g_ports[port].writable = writable;
    }
}

std::size_t inject(std::uint8_t port, const std::uint8_t *data, std::size_t length) noexcept
{
    hal::Callback notification = nullptr;
    void *context = nullptr;
    std::size_t accepted = 0U;
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        if (port >= g_ports.size() || data == nullptr)
        {
            return 0U;
        }
        Port &target = g_ports[port];
        while (accepted < length && target.received.size() < QUEUE_CAPACITY)
        {
            target.received.push_back(data[accepted++]);
        }
        target.statistics.received_byte_count += static_cast<std::uint32_t>(length);
        target.statistics.dropped_byte_count += static_cast<std::uint32_t>(length - accepted);
        notification = accepted > 0U ? target.receive_notification : nullptr;
        context = target.receive_context;
    }
    if (notification != nullptr)
    {
        notification(context);
    }
    return accepted;
}

bool write_pending(std::uint8_t port) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return port < g_ports.size() && g_ports[port].write_pending;
}

bool complete_write(std::uint8_t port, bool error) noexcept
{
    hal::Callback notification = nullptr;
    void *context = nullptr;
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        if (port >= g_ports.size() || !g_ports[port].write_pending)
        {
            return false;
        }
        Port &target = g_ports[port];
        if (error)
        {
            ++target.statistics.transmit_failure_count;
        }
        else
        {
            target.statistics.transmitted_byte_count += static_cast<std::uint32_t>(target.pending_length);
        }
        target.write_pending = false;
        target.pending_length = 0U;
        notification = target.write_notification;
        context = target.write_context;
    }
    if (notification != nullptr)
    {
        notification(context);
    }
    return true;
}

std::size_t take_written(std::uint8_t port, std::uint8_t *data, std::size_t capacity) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (port >= g_ports.size() || data == nullptr)
    {
        return 0U;
    }
    std::deque<std::uint8_t> &written = g_ports[port].written;
    std::size_t count = 0U;
    while (count < capacity && !written.empty())
    {
        data[count++] = written.front();
        written.pop_front();
    }
    return count;
}

} // namespace mock::uart
