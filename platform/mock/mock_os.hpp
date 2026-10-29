#ifndef PLATFORM_MOCK_MOCK_OS_HPP
#define PLATFORM_MOCK_MOCK_OS_HPP

#include "hal/os/clock.hpp"

#include <cstdint>

// 宿主测试控制 hal_os 替身的接口：时间由测试推进，临界区只记录嵌套深度。
// 状态是进程内全局的，测试开始时调用 reset()。不是线程安全的：多线程测试不要使用。

namespace mock
{

/** 恢复初始状态：计数 0、频率 1 MHz、不在临界区内、进入次数清零。 */
void reset() noexcept;

void set_ticks(hal::os::Tick ticks) noexcept;
void advance_ticks(hal::os::Tick delta) noexcept;
void set_tick_frequency_hz(std::uint32_t frequency_hz) noexcept;

/** 当前临界区嵌套深度，0 表示不在临界区内。 */
[[nodiscard]] std::uint32_t critical_depth() noexcept;

/** reset() 之后进入临界区的总次数。 */
[[nodiscard]] std::uint32_t critical_entry_count() noexcept;

} // namespace mock

#endif // PLATFORM_MOCK_MOCK_OS_HPP
