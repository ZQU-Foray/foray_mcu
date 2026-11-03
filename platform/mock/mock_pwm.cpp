// hal::Pwm 的宿主替身。参数检查与 DM-MC02 的 port 一致，频率上限不检查。
#include "hal/pwm.hpp"
#include "mock_hal.hpp"

#include <array>
#include <mutex>

namespace
{

struct Channel
{
    bool ready{true};
    mock::pwm::Output output{0U, 0U, 0U};
};

std::mutex g_mutex;
std::array<Channel, mock::pwm::CHANNEL_COUNT> g_channels{};

} // namespace

namespace hal
{

bool Pwm::is_ready() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return index_ < g_channels.size() && g_channels[index_].ready;
}

Pwm::Result Pwm::set(std::uint32_t frequency_hz, std::uint16_t duty_permille) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_channels.size() || frequency_hz == 0U || duty_permille > MAX_DUTY_PERMILLE)
    {
        return Result::INVALID_ARGUMENT;
    }
    Channel &channel = g_channels[index_];
    if (!channel.ready)
    {
        return Result::NOT_READY;
    }
    channel.output.frequency_hz = frequency_hz;
    channel.output.duty_permille = duty_permille;
    ++channel.output.set_count;
    return Result::COMPLETED;
}

Pwm::Result Pwm::off() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (index_ >= g_channels.size())
    {
        return Result::INVALID_ARGUMENT;
    }
    Channel &channel = g_channels[index_];
    if (!channel.ready)
    {
        return Result::NOT_READY;
    }
    channel.output.duty_permille = 0U;
    ++channel.output.set_count;
    return Result::COMPLETED;
}

} // namespace hal

namespace mock::pwm
{

void reset() noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    g_channels.fill(Channel{});
}

void set_ready(std::uint8_t channel, bool ready) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (channel < g_channels.size())
    {
        g_channels[channel].ready = ready;
    }
}

Output output(std::uint8_t channel) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return channel < g_channels.size() ? g_channels[channel].output : Output{0U, 0U, 0U};
}

} // namespace mock::pwm
