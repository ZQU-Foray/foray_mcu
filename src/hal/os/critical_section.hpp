#ifndef HAL_OS_CRITICAL_SECTION_HPP
#define HAL_OS_CRITICAL_SECTION_HPP

#include <cstdint>

// 最小 OS 契约：临界区（Decision-20261031-03）。
// 用 RAII 守卫而不是成对的 enter/exit，避免提前返回时漏掉恢复。
//
// 屏蔽范围：全部框架中断，不屏蔽框架线以上的中断。
//   - 框架线每块板一条，RTOS 板与 RTOS 允许调用其中断 API 的最高优先级相同（DM-MC02 为优先级 5）；
//   - 框架线以上的中断（DM-MC02 为 0–4）不得调用 RTOS，也不得使用框架（hal::os、topic）；
//   - Cortex-M3/M4/M7 用 BASEPRI；Cortex-M0+ 没有 BASEPRI，用 PRIMASK 全部屏蔽；Cortex-R5 屏蔽 IRQ，FIQ 在线以上。
// 实现放在 platform 的 port（F2）；宿主测试由 platform/mock 提供。

namespace hal::os
{

/**
 * @brief 临界区守卫：构造时保存原状态并屏蔽框架中断，析构时恢复原状态。
 * @note 任务与框架中断里都能用；可以嵌套，内层析构只恢复到外层进入时的状态，不会提前打开中断。
 * @note 只保护很短的代码段。临界区内不得阻塞，不得调用 RTOS API，也不得调用 Signal：
 *       RTOS 的任务级 API 退出它自己的临界区时可能直接打开中断（Decision-20261031-03）。
 */
class CriticalSection final
{
  public:
    CriticalSection() noexcept;
    ~CriticalSection();

    CriticalSection(const CriticalSection &) = delete;
    CriticalSection &operator=(const CriticalSection &) = delete;
    CriticalSection(CriticalSection &&) = delete;
    CriticalSection &operator=(CriticalSection &&) = delete;

  private:
    std::uint32_t saved_state_; // 进入前的屏蔽状态（BASEPRI、PRIMASK 或 CPSR），由实现解释
};

} // namespace hal::os

#endif // HAL_OS_CRITICAL_SECTION_HPP
