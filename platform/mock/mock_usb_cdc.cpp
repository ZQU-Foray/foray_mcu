// hal::UsbCdc 的宿主替身（Decision-20261101-01）。参数检查与结果优先级与 DM-MC02 的 port 一致。
#include "hal/usb_cdc.hpp"
#include "mock_hal.hpp"

#include <deque>
#include <mutex>

namespace
{

struct State
{
    bool connected{false};
    std::deque<std::uint8_t> received{};
    std::deque<std::uint8_t> written{};
    bool write_pending{false};
    std::size_t pending_length{0U};
    hal::UsbCdc::Statistics statistics{};
    hal::Callback receive_notification{nullptr};
    void *receive_context{nullptr};
    hal::Callback write_notification{nullptr};
    void *write_context{nullptr};
};

std::mutex g_mutex;
State g_state{};

} // namespace

namespace hal
{

bool UsbCdc::is_ready() const noexcept
{
    return index_ == 0U;
}

bool UsbCdc::is_connected() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return index_ == 0U && g_state.connected;
}

std::size_t UsbCdc::read(std::uint8_t *data, std::size_t capacity) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ != 0U || data == nullptr)
    {
        return 0U;
    }
    std::size_t count = 0U;
    while (count < capacity && !g_state.received.empty())
    {
        data[count++] = g_state.received.front();
        g_state.received.pop_front();
    }
    return count;
}

UsbCdc::WriteResult UsbCdc::write(const std::uint8_t *data, std::size_t length) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ != 0U || data == nullptr || length == 0U || length > MAX_WRITE_LENGTH)
    {
        return WriteResult::INVALID_ARGUMENT;
    }
    if (!g_state.connected)
    {
        return WriteResult::NOT_READY;
    }
    if (g_state.write_pending)
    {
        return WriteResult::BUSY;
    }
    g_state.written.insert(g_state.written.end(), data, data + length);
    g_state.write_pending = true;
    g_state.pending_length = length;
    return WriteResult::STARTED;
}

bool UsbCdc::set_receive_notification(Callback callback, void *context) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ != 0U)
    {
        return false;
    }
    g_state.receive_notification = callback;
    g_state.receive_context = context;
    return true;
}

bool UsbCdc::set_write_notification(Callback callback, void *context) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ != 0U)
    {
        return false;
    }
    g_state.write_notification = callback;
    g_state.write_context = context;
    return true;
}

UsbCdc::Statistics UsbCdc::statistics() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return index_ == 0U ? g_state.statistics : Statistics{};
}

} // namespace hal

namespace mock::usb_cdc
{

void reset() noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    g_state = State{};
}

void set_connected(bool connected) noexcept
{
    hal::Callback notification = nullptr;
    void *context = nullptr;
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        if (g_state.connected == connected)
        {
            return;
        }
        g_state.connected = connected;
        g_state.received.clear();
        if (connected)
        {
            ++g_state.statistics.connection_count;
        }
        else
        {
            ++g_state.statistics.disconnection_count;
            if (g_state.write_pending)
            {
                g_state.write_pending = false;
                g_state.pending_length = 0U;
                ++g_state.statistics.transmit_failure_count;
                notification = g_state.write_notification;
                context = g_state.write_context;
            }
        }
    }
    if (notification != nullptr)
    {
        notification(context);
    }
}

std::size_t inject(const std::uint8_t *data, std::size_t length) noexcept
{
    hal::Callback notification = nullptr;
    void *context = nullptr;
    std::size_t accepted = 0U;
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        if (data == nullptr)
        {
            return 0U;
        }
        if (!g_state.connected)
        {
            ++g_state.statistics.receive_error_count;
            return 0U;
        }
        while (accepted < length && g_state.received.size() < QUEUE_CAPACITY)
        {
            g_state.received.push_back(data[accepted++]);
        }
        g_state.statistics.received_byte_count += static_cast<std::uint32_t>(length);
        g_state.statistics.dropped_byte_count += static_cast<std::uint32_t>(length - accepted);
        notification = accepted > 0U ? g_state.receive_notification : nullptr;
        context = g_state.receive_context;
    }
    if (notification != nullptr)
    {
        notification(context);
    }
    return accepted;
}

bool write_pending() noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return g_state.write_pending;
}

bool complete_write(bool error) noexcept
{
    hal::Callback notification = nullptr;
    void *context = nullptr;
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        if (!g_state.write_pending)
        {
            return false;
        }
        if (error)
        {
            ++g_state.statistics.transmit_failure_count;
        }
        else
        {
            g_state.statistics.transmitted_byte_count += static_cast<std::uint32_t>(g_state.pending_length);
        }
        g_state.write_pending = false;
        g_state.pending_length = 0U;
        notification = g_state.write_notification;
        context = g_state.write_context;
    }
    if (notification != nullptr)
    {
        notification(context);
    }
    return true;
}

std::size_t take_written(std::uint8_t *data, std::size_t capacity) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (data == nullptr)
    {
        return 0U;
    }
    std::size_t count = 0U;
    while (count < capacity && !g_state.written.empty())
    {
        data[count++] = g_state.written.front();
        g_state.written.pop_front();
    }
    return count;
}

} // namespace mock::usb_cdc
