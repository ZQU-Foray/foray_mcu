// hal::Gpio 的宿主替身（Decision-20261101-01）。
#include "hal/gpio.hpp"
#include "mock_hal.hpp"

#include <array>
#include <mutex>

namespace
{

struct Pin
{
    hal::Gpio::Level level{hal::Gpio::Level::LOW};
    bool output{true};
    bool interrupt{true};
    std::uint32_t write_count{0U};
    hal::Callback callback{nullptr};
    void *context{nullptr};
};

std::mutex g_mutex;
std::array<Pin, mock::gpio::PIN_COUNT> g_pins{};

} // namespace

namespace hal
{

Gpio::Level Gpio::read() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return index_ < g_pins.size() ? g_pins[index_].level : Level::LOW;
}

bool Gpio::write(Level level) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_pins.size() || !g_pins[index_].output)
    {
        return false;
    }
    g_pins[index_].level = level;
    ++g_pins[index_].write_count;
    return true;
}

bool Gpio::toggle() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_pins.size() || !g_pins[index_].output)
    {
        return false;
    }
    Pin &pin = g_pins[index_];
    pin.level = pin.level == Level::HIGH ? Level::LOW : Level::HIGH;
    ++pin.write_count;
    return true;
}

bool Gpio::set_interrupt(Callback callback, void *context) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_pins.size() || !g_pins[index_].interrupt)
    {
        return false;
    }
    g_pins[index_].callback = callback;
    g_pins[index_].context = context;
    return true;
}

} // namespace hal

namespace mock::gpio
{

void reset() noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    g_pins.fill(Pin{});
}

void set_capabilities(std::uint8_t pin, bool output, bool interrupt) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (pin < g_pins.size())
    {
        g_pins[pin].output = output;
        g_pins[pin].interrupt = interrupt;
    }
}

void set_level(std::uint8_t pin, hal::Gpio::Level level) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (pin < g_pins.size())
    {
        g_pins[pin].level = level;
    }
}

hal::Gpio::Level level(std::uint8_t pin) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return pin < g_pins.size() ? g_pins[pin].level : hal::Gpio::Level::LOW;
}

std::uint32_t write_count(std::uint8_t pin) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return pin < g_pins.size() ? g_pins[pin].write_count : 0U;
}

bool trigger_interrupt(std::uint8_t pin) noexcept
{
    hal::Callback callback = nullptr;
    void *context = nullptr;
    {
        const std::lock_guard<std::mutex> lock{g_mutex};
        if (pin < g_pins.size())
        {
            callback = g_pins[pin].callback;
            context = g_pins[pin].context;
        }
    }
    if (callback == nullptr)
    {
        return false;
    }
    callback(context); // 回调可能再调用 hal::Gpio，所以在锁外执行
    return true;
}

} // namespace mock::gpio
