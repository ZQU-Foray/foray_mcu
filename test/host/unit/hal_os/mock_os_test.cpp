// hal_os 的时基与临界区：契约头里的换算辅助，以及 platform_mock 的实现
// （时间由测试控制；临界区可嵌套、逐层恢复，多线程下互斥）。
#include "hal/os/clock.hpp"
#include "hal/os/critical_section.hpp"
#include "mock_os.hpp"

#include <cassert>
#include <cstdint>
#include <thread>

namespace
{

// 换算辅助是 constexpr，编译期就能检查
static_assert(hal::os::elapsed_us(0xFFFFFFF0U, 0x10U) == 0x20U, "跨回绕的时长");
static_assert(hal::os::has_elapsed(0xFFFFFFF0U, 0x20U, 0x10U), "跨回绕正好到期");
static_assert(!hal::os::has_elapsed(0xFFFFFFF0U, 0x21U, 0x10U), "跨回绕差 1 µs 未到期");
static_assert(hal::os::has_elapsed(5U, 0U, 5U), "时长 0 立即到期");
static_assert(hal::os::to_seconds(hal::os::MICROSECONDS_PER_SECOND) == 1.0F, "1 s");
static_assert(hal::os::to_seconds(625U) == 0.000625F, "BMI088 加速度计 1600 Hz 的采样间隔");

void clock_follows_test_control()
{
    mock::reset();
    assert(hal::os::now_us() == 0U);

    mock::set_now_us(100U);
    mock::advance_us(50U);
    assert(hal::os::now_us() == 150U);

    mock::reset();
    assert(hal::os::now_us() == 0U);
}

void helpers_read_the_clock_by_default()
{
    mock::reset();
    mock::set_now_us(0xFFFFFFF0U);
    const hal::os::Microseconds start = hal::os::now_us();
    mock::advance_us(0x1FU);
    assert(hal::os::elapsed_us(start) == 0x1FU);
    assert(!hal::os::has_elapsed(start, 0x20U));
    mock::advance_us(1U);
    assert(hal::os::has_elapsed(start, 0x20U));
    // 回绕周期内最长的超时：2^32 − 1 µs
    mock::set_now_us(start + 0xFFFFFFFEU);
    assert(!hal::os::has_elapsed(start, 0xFFFFFFFFU));
    mock::advance_us(1U);
    assert(hal::os::has_elapsed(start, 0xFFFFFFFFU));
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

void critical_sections_exclude_other_threads()
{
    mock::reset();
    constexpr std::uint32_t ROUNDS = 200000U;
    // 普通变量：只靠临界区互斥。缺了互斥，两个线程的"读、加一、写"会互相覆盖，总数变少。
    std::uint32_t counter = 0U;
    const auto work = [&counter]()
    {
        for (std::uint32_t i = 0U; i < ROUNDS; ++i)
        {
            const hal::os::CriticalSection section;
            const std::uint32_t value = counter;
            counter = value + 1U;
        }
        // 深度是每个线程自己的
        assert(mock::critical_depth() == 0U);
    };
    std::thread other{work};
    work();
    other.join();
    assert(counter == 2U * ROUNDS);
    assert(mock::critical_entry_count() == 2U * ROUNDS);
}

} // namespace

int main()
{
    clock_follows_test_control();
    helpers_read_the_clock_by_default();
    critical_sections_nest_and_restore();
    critical_sections_exclude_other_threads();
    return 0;
}
