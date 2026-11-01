#ifndef PLATFORM_ST_DM_MC02_PORT_OS_OS_HPP
#define PLATFORM_ST_DM_MC02_PORT_OS_OS_HPP

// hal::os 在 DM-MC02 上的实现（port 私有，Decision-20261031-10）。
// 契约函数本身（now_us、CriticalSection、Signal）的定义在同目录的 .cpp 里，这里只放 port 内部的初始化。

namespace port::os
{

/**
 * @brief 启动时基：调试暂停时冻结 TIM2，然后启动计数器。
 * @return TIM2 启动成功返回 true。
 * @note 由 hal::init() 调用（F2 第 4 步），在内核启动前、外设初始化之后。之前 now_us() 一直读到 0。
 */
[[nodiscard]] bool start_clock() noexcept;

} // namespace port::os

#endif // PLATFORM_ST_DM_MC02_PORT_OS_OS_HPP
