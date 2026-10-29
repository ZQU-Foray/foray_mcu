#ifndef HAL_OS_CRITICAL_SECTION_HPP
#define HAL_OS_CRITICAL_SECTION_HPP

#include <cstdint>

// 最小 OS 契约：临界区（评审报告 §5.7、Q10）。
// 用 RAII 守卫而不是成对的 enter/exit，避免提前返回时漏掉恢复。
// 各平台屏蔽中断的方式不同（Cortex-M 为 PRIMASK/BASEPRI，Cortex-R 为 CPSR 的 I/F 位），
// 所以只约定行为，实现放在 platform 的 port（P3a）；宿主测试由 platform/mock 提供。
// P1 草案，P3a 冻结契约 v1 时可以调整。

namespace hal::os
{

/**
 * @brief 临界区守卫：构造时屏蔽可能抢占当前上下文的中断并保存原状态，析构时恢复原状态。
 * @note 可以嵌套：内层析构只恢复到外层进入时的状态，不会提前打开中断。
 * @note 只保护很短的代码段；临界区内不得阻塞，也不得调用 RTOS API。
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
    std::uint32_t saved_state_;
};

} // namespace hal::os

#endif // HAL_OS_CRITICAL_SECTION_HPP
