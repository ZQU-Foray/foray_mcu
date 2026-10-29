// platform_mock 对 hal_os 契约的实现：时基可由测试控制，临界区可嵌套且逐层恢复。
#include "hal/os/clock.hpp"
#include "hal/os/critical_section.hpp"
#include "mock_os.hpp"

#include <cassert>
#include <cstdint>

namespace
{

void clock_follows_test_control()
{
    mock::reset();
    assert(hal::os::now_ticks() == 0U);
    assert(hal::os::tick_frequency_hz() == 1000000U);

    mock::set_ticks(100U);
    mock::advance_ticks(50U);
    assert(hal::os::now_ticks() == 150U);

    mock::set_tick_frequency_hz(480000000U);
    assert(hal::os::tick_frequency_hz() == 480000000U);
}

void elapsed_ticks_survive_wraparound()
{
    mock::reset();
    mock::set_ticks(0xFFFFFFF0U);
    const hal::os::Tick start = hal::os::now_ticks();
    mock::advance_ticks(0x20U);
    // 契约约定：按 2^32 回绕，无符号减法得到经过的计数。
    assert(static_cast<hal::os::Tick>(hal::os::now_ticks() - start) == 0x20U);
}

void critical_sections_nest_and_restore()
{
    mock::reset();
    assert(mock::critical_depth() == 0U);
    {
        const hal::os::CriticalSection outer;
        assert(mock::critical_depth() == 1U);
        {
            const hal::os::CriticalSection inner;
            assert(mock::critical_depth() == 2U);
        }
        // 内层退出只恢复到外层的状态，不会提前离开临界区。
        assert(mock::critical_depth() == 1U);
    }
    assert(mock::critical_depth() == 0U);
    assert(mock::critical_entry_count() == 2U);
}

} // namespace

int main()
{
    clock_follows_test_control();
    elapsed_ticks_survive_wraparound();
    critical_sections_nest_and_restore();
    return 0;
}
