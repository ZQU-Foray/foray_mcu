#ifndef HAL_OS_SIGNAL_HPP
#define HAL_OS_SIGNAL_HPP

#include "hal/os/clock.hpp"

// 最小 OS 契约：二值信号（Decision-20261031-02）。
// 一个任务在 wait() 上睡眠，别的任务或中断用 notify() 叫醒它。
// topic 的阻塞等待、按 topic 触发的任务、驱动等 DMA 完成都用它。
// 实现：RTOS 板用任务通知，裸机板用标志位加 WFI（F2 起）；宿主测试由 platform/mock 提供。

namespace hal::os
{

/** wait() 的超时：不睡眠，只检查有没有待处理的通知。 */
inline constexpr Microseconds NO_WAIT{0U};
/** wait() 的超时：一直等到通知。 */
inline constexpr Microseconds WAIT_FOREVER{0xFFFFFFFFU};

/**
 * @brief 二值信号：notify() 留下一个通知，wait() 取走它；没有通知时 wait() 睡眠到通知或超时。
 *
 * - 通知不丢：notify() 早于 wait() 时，下一次 wait() 立即返回 true。
 * - 合并：上一次 wait() 返回之后的多次 notify() 只算一次。需要计数时用队列。
 * - 一次一个等待者：每次 wait() 时登记当前任务，返回时注销。不同任务可以先后等同一个 Signal，
 *   一个任务也可以先后等多个 Signal；两个任务同时等同一个 Signal 是误用。
 * - wait() 返回 true 只说明"上次取走之后有过 notify()"，对应的事件可能已经被调用方处理过。
 *   调用方醒来后要重新检查条件（如 topic 的版本号）。
 * - 可以常量初始化，作全局对象时不依赖构造顺序；不可拷贝、不可移动。
 */
class Signal final
{
  public:
    constexpr Signal() noexcept = default;

    Signal(const Signal &) = delete;
    Signal &operator=(const Signal &) = delete;
    Signal(Signal &&) = delete;
    Signal &operator=(Signal &&) = delete;

    /**
     * @brief 留下一个通知；有任务正在等，就叫醒它。
     * @note 任务与框架中断里都能调用，由实现区分上下文；不阻塞。不能在 CriticalSection 内调用。
     */
    void notify() noexcept;

    /**
     * @brief 取走待处理的通知；没有时睡眠，直到 notify() 或超时。
     * @param timeout 最长等待时间（µs）。NO_WAIT 只检查不睡眠，WAIT_FOREVER 一直等。
     *        超时返回时实际等待不少于 timeout，按实现的调度粒度向上取整（DM-MC02 上是 1 ms 的 RTOS 节拍）。
     * @return true：取走了一个通知；false：超时。
     * @note 只能在任务上下文调用（裸机板为主循环）；不能在中断里、CriticalSection 内调用。
     */
    bool wait(Microseconds timeout) noexcept;

  private:
    // 实现私有的状态，只由 port 或 mock 读写，读写时由实现保证互斥（如在临界区内）。
    void *waiter_{nullptr}; // 正在等待的任务（RTOS 上是任务句柄），没有时为空
    bool pending_{false};   // 有没有待处理的通知
};

} // namespace hal::os

#endif // HAL_OS_SIGNAL_HPP
