#include "mock_os.hpp"

#include "hal/os/critical_section.hpp"

namespace
{

constexpr std::uint32_t DEFAULT_TICK_FREQUENCY_HZ{1000000U};

hal::os::Tick g_ticks{0U};
std::uint32_t g_tick_frequency_hz{DEFAULT_TICK_FREQUENCY_HZ};
std::uint32_t g_critical_depth{0U};
std::uint32_t g_critical_entry_count{0U};

} // namespace

namespace hal::os
{

Tick now_ticks() noexcept
{
    return g_ticks;
}

std::uint32_t tick_frequency_hz() noexcept
{
    return g_tick_frequency_hz;
}

// 与真实平台的"保存并屏蔽、析构时恢复"一致：保存的是进入前的嵌套深度，析构时原样恢复。
CriticalSection::CriticalSection() noexcept : saved_state_{g_critical_depth}
{
    ++g_critical_depth;
    ++g_critical_entry_count;
}

CriticalSection::~CriticalSection()
{
    g_critical_depth = saved_state_;
}

} // namespace hal::os

namespace mock
{

void reset() noexcept
{
    g_ticks = 0U;
    g_tick_frequency_hz = DEFAULT_TICK_FREQUENCY_HZ;
    g_critical_depth = 0U;
    g_critical_entry_count = 0U;
}

void set_ticks(hal::os::Tick ticks) noexcept
{
    g_ticks = ticks;
}

void advance_ticks(hal::os::Tick delta) noexcept
{
    g_ticks += delta;
}

void set_tick_frequency_hz(std::uint32_t frequency_hz) noexcept
{
    g_tick_frequency_hz = frequency_hz;
}

std::uint32_t critical_depth() noexcept
{
    return g_critical_depth;
}

std::uint32_t critical_entry_count() noexcept
{
    return g_critical_entry_count;
}

} // namespace mock
