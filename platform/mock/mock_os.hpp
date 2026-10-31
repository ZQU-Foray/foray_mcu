#ifndef PLATFORM_MOCK_MOCK_OS_HPP
#define PLATFORM_MOCK_MOCK_OS_HPP

#include "hal/os/clock.hpp"

#include <cstdint>

// 宿主测试控制 hal_os 替身的接口（Decision-20261031-01～03）。全部函数线程安全。
// - 时间由测试推进，不随真实时间走；推进时间会叫醒正在等待的线程，让它们重新检查超时。
// - 临界区是一把进程内的递归锁：同一时刻只有一个线程在临界区内，对应单核上关中断的效果。
// - Signal 检查误用：在临界区内调用 wait() 或 notify()、两个线程同时等同一个 Signal 时，打印原因并 abort()。
// - wait() 睡了 10 s 真实时间还没被叫醒，说明测试漏了 notify() 或推进时间，打印原因并 abort()，避免测试挂起。
// 状态是进程内全局的，测试开始时调用 reset()。

namespace mock
{

/** 恢复初始状态：时间 0、进入临界区的次数清零。调用时不能有线程在临界区内或在等待。 */
void reset() noexcept;

/** 设置当前时刻，并叫醒正在等待的线程。用于设定起点，如测试回绕。 */
void set_now_us(hal::os::Microseconds now) noexcept;

/** 时间前进 delta，并叫醒正在等待的线程。 */
void advance_us(hal::os::Microseconds delta) noexcept;

/** 调用线程的临界区嵌套深度，0 表示不在临界区内。 */
[[nodiscard]] std::uint32_t critical_depth() noexcept;

/** reset() 之后所有线程进入临界区的总次数。 */
[[nodiscard]] std::uint32_t critical_entry_count() noexcept;

/** 正在 Signal::wait() 里睡眠的线程数。测试用它确认等待者已经睡下，再 notify() 或推进时间。 */
[[nodiscard]] std::uint32_t waiting_count() noexcept;

} // namespace mock

#endif // PLATFORM_MOCK_MOCK_OS_HPP
