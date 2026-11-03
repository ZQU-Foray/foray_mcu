// hal::Spi 的宿主替身。参数检查与结果优先级与 DM-MC02 的 port 一致：
// 参数 → 编号 → 初始化 → 总线占用 → 能力。
#include "hal/spi.hpp"
#include "mock_hal.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <mutex>

namespace
{

using Result = hal::Spi::Result;
constexpr std::size_t BLOCKING_MAX_LENGTH{0xFFFFU};
constexpr std::uint8_t OWN_BUS{0xFFU}; // 默认：每个设备单独一条总线，总线号 = 设备号

struct Device
{
    std::uint8_t bus{OWN_BUS};
    bool ready{true};
    bool async{true};
    bool transfer{true};
    mock::spi::Responder responder{nullptr};
    void *responder_context{nullptr};
    Result result{Result::NOT_READY};
    std::uint32_t transaction_count{0U};
    std::array<std::uint8_t, hal::Spi::MAX_ASYNC_LENGTH> last_transmit{};
    std::size_t last_transmit_length{0U};
};

struct Pending
{
    bool active{false};
    std::uint8_t device{0U};
    std::array<std::uint8_t, hal::Spi::MAX_ASYNC_LENGTH> transmit{};
    std::uint8_t *receive{nullptr};
    std::size_t length{0U};
    hal::Callback callback{nullptr};
    void *context{nullptr};
};

std::mutex g_mutex;
std::array<Device, mock::spi::DEVICE_COUNT> g_devices{};
std::array<Pending, mock::spi::DEVICE_COUNT> g_pending{}; // 按总线号
bool g_complete_immediately{false};

std::uint8_t bus_of(std::uint8_t index) noexcept
{
    const std::uint8_t bus = g_devices[index].bus;
    return bus == OWN_BUS ? index : bus;
}

void record_transmit(Device &device, const std::uint8_t *data, std::size_t length) noexcept
{
    ++device.transaction_count;
    device.last_transmit_length = std::min(length, device.last_transmit.size());
    std::memcpy(device.last_transmit.data(), data, device.last_transmit_length);
}

// 检查到"能开始"为止；返回 COMPLETED 表示可以开始。调用时持有锁。
Result admit(std::uint8_t index, bool async, bool transfer) noexcept
{
    if (index >= g_devices.size())
    {
        return Result::INVALID_ARGUMENT;
    }
    const Device &device = g_devices[index];
    if (!device.ready)
    {
        return Result::NOT_READY;
    }
    if (g_pending[bus_of(index)].active)
    {
        return Result::BUSY;
    }
    if ((async && !device.async) || (transfer && !device.transfer))
    {
        return Result::UNSUPPORTED;
    }
    return Result::COMPLETED;
}

Result blocking(
    std::uint8_t index, const std::uint8_t *transmit, std::uint8_t *receive, std::size_t length, bool transfer) noexcept
{
    if (transmit == nullptr || (transfer && receive == nullptr) || length == 0U || length > BLOCKING_MAX_LENGTH)
    {
        return Result::INVALID_ARGUMENT;
    }
    mock::spi::Responder responder = nullptr;
    void *context = nullptr;
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        const Result admitted = admit(index, false, transfer);
        if (admitted != Result::COMPLETED)
        {
            return admitted;
        }
        Device &device = g_devices[index];
        record_transmit(device, transmit, length);
        responder = device.responder;
        context = device.responder_context;
    }
    if (receive != nullptr)
    {
        std::memset(receive, 0, length);
    }
    if (responder != nullptr)
    {
        responder(context, index, transmit, receive, length);
    }
    return Result::COMPLETED;
}

Result start(std::uint8_t index,
             const std::uint8_t *transmit,
             std::uint8_t *receive,
             std::size_t length,
             bool transfer,
             hal::Callback callback,
             void *context) noexcept
{
    if (transmit == nullptr || (transfer && receive == nullptr) || length == 0U || length > hal::Spi::MAX_ASYNC_LENGTH)
    {
        return Result::INVALID_ARGUMENT;
    }
    bool complete_now = false;
    std::uint8_t bus = 0U;
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        const Result admitted = admit(index, true, transfer);
        if (admitted != Result::COMPLETED)
        {
            return admitted;
        }
        Device &device = g_devices[index];
        record_transmit(device, transmit, length);
        device.result = Result::BUSY;
        bus = bus_of(index);
        Pending &pending = g_pending[bus];
        pending.active = true;
        pending.device = index;
        std::memcpy(pending.transmit.data(), transmit, length); // 与 port 一样，发送数据返回前已拷走
        pending.receive = receive;
        pending.length = length;
        pending.callback = callback;
        pending.context = context;
        complete_now = g_complete_immediately;
    }
    if (complete_now)
    {
        (void)mock::spi::complete(bus);
    }
    return Result::STARTED;
}

} // namespace

namespace hal
{

bool Spi::is_ready() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return index_ < g_devices.size() && g_devices[index_].ready && !g_pending[bus_of(index_)].active;
}

Spi::Result Spi::transmit(const std::uint8_t *data, std::size_t length, os::Microseconds) const noexcept
{
    return blocking(index_, data, nullptr, length, false);
}

Spi::Result Spi::transfer(const std::uint8_t *transmit_data,
                          std::uint8_t *receive_data,
                          std::size_t length,
                          os::Microseconds) const noexcept
{
    return blocking(index_, transmit_data, receive_data, length, true);
}

Spi::Result
Spi::start_transmit(const std::uint8_t *data, std::size_t length, Callback callback, void *context) const noexcept
{
    return start(index_, data, nullptr, length, false, callback, context);
}

Spi::Result Spi::start_transfer(const std::uint8_t *transmit_data,
                                std::uint8_t *receive_data,
                                std::size_t length,
                                Callback callback,
                                void *context) const noexcept
{
    return start(index_, transmit_data, receive_data, length, true, callback, context);
}

Spi::Result Spi::async_result() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_devices.size())
    {
        return Result::INVALID_ARGUMENT;
    }
    const Device &device = g_devices[index_];
    if (!device.ready)
    {
        return Result::NOT_READY;
    }
    return device.async ? device.result : Result::UNSUPPORTED;
}

} // namespace hal

namespace mock::spi
{

void reset() noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    g_devices.fill(Device{});
    g_pending.fill(Pending{});
    g_complete_immediately = false;
}

void set_bus(std::uint8_t device, std::uint8_t bus) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (device < g_devices.size() && bus < g_pending.size())
    {
        g_devices[device].bus = bus;
    }
}

void set_capabilities(std::uint8_t device, bool ready, bool async, bool transfer) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (device < g_devices.size())
    {
        g_devices[device].ready = ready;
        g_devices[device].async = async;
        g_devices[device].transfer = transfer;
    }
}

void set_responder(std::uint8_t device, Responder responder, void *context) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (device < g_devices.size())
    {
        g_devices[device].responder = responder;
        g_devices[device].responder_context = context;
    }
}

void set_complete_immediately(bool enabled) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    g_complete_immediately = enabled;
}

bool busy(std::uint8_t bus, std::uint8_t *device) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (bus >= g_pending.size() || !g_pending[bus].active)
    {
        return false;
    }
    if (device != nullptr)
    {
        *device = g_pending[bus].device;
    }
    return true;
}

bool complete(std::uint8_t bus, bool error) noexcept
{
    Pending pending{};
    Responder responder = nullptr;
    void *responder_context = nullptr;
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        if (bus >= g_pending.size() || !g_pending[bus].active)
        {
            return false;
        }
        pending = g_pending[bus];
        responder = g_devices[pending.device].responder;
        responder_context = g_devices[pending.device].responder_context;
    }
    // 应答与数据拷贝在锁外：应答函数可以推进时间、读写测试状态
    if (!error)
    {
        std::array<std::uint8_t, hal::Spi::MAX_ASYNC_LENGTH> received{};
        if (responder != nullptr)
        {
            responder(responder_context,
                      pending.device,
                      pending.transmit.data(),
                      pending.receive != nullptr ? received.data() : nullptr,
                      pending.length);
        }
        if (pending.receive != nullptr)
        {
            std::memcpy(pending.receive, received.data(), pending.length);
        }
    }
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        g_devices[pending.device].result = error ? Result::ERROR : Result::COMPLETED;
        g_pending[bus].active = false; // 总线先释放，回调里就能开始下一个事务
    }
    if (pending.callback != nullptr)
    {
        pending.callback(pending.context);
    }
    return true;
}

std::uint32_t transaction_count(std::uint8_t device) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return device < g_devices.size() ? g_devices[device].transaction_count : 0U;
}

std::size_t last_transmit(std::uint8_t device, std::uint8_t *data, std::size_t capacity) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (device >= g_devices.size() || data == nullptr)
    {
        return 0U;
    }
    const std::size_t length = std::min(capacity, g_devices[device].last_transmit_length);
    std::memcpy(data, g_devices[device].last_transmit.data(), length);
    return length;
}

} // namespace mock::spi
