// hal::os::Signal 在 DM-MC02（FreeRTOS）上的实现。
//
// - 待处理状态 pending_ 与等待者 waiter_ 存在 Signal 里，在临界区内读写；
// - FreeRTOS 的任务通知只用来叫醒任务，用 1 号通知槽（0 号给 CMSIS-RTOS2 的线程标志与 stream buffer）；
// - 叫醒是"可能有通知"的提示：任务醒来后在临界区内检查 pending_，不是自己的就接着睡。所以一个任务先后等
//   几个 Signal 互不干扰，迟到的任务通知也只会让下一次等待多醒一次；
// - 超时按 now_us() 计算，换成 RTOS 节拍时向上取整，醒来后按 µs 再核对一次，实际等待不少于 timeout。
#include "hal/os/signal.hpp"

#include "hal/os/clock.hpp"
#include "hal/os/critical_section.hpp"

#include "FreeRTOS.h"
#include "stm32h7xx.h" // CMSIS：__get_IPSR、__get_BASEPRI
#include "task.h"

// 1 号通知槽要求每个任务至少有 2 个槽，由板级 CMake 的编译宏统一设置（CubeMX 没有这一项）
static_assert(configTASK_NOTIFICATION_ARRAY_ENTRIES >= 2, "Signal 用 1 号任务通知槽，需要至少 2 个槽");

namespace
{

constexpr UBaseType_t NOTIFY_INDEX{1U};
constexpr hal::os::Microseconds MICROSECONDS_PER_TICK{hal::os::MICROSECONDS_PER_SECOND / configTICK_RATE_HZ};
static_assert(hal::os::MICROSECONDS_PER_SECOND % configTICK_RATE_HZ == 0U, "RTOS 节拍必须是整数微秒");

bool in_interrupt() noexcept
{
    return __get_IPSR() != 0U;
}

// 还要等 remaining µs，换成 RTOS 节拍并向上取整；remaining 最大 2^32 − 2，结果远小于 portMAX_DELAY
TickType_t to_ticks(hal::os::Microseconds remaining) noexcept
{
    return (remaining / MICROSECONDS_PER_TICK) + ((remaining % MICROSECONDS_PER_TICK != 0U) ? 1U : 0U);
}

} // namespace

namespace hal::os
{

void Signal::notify() noexcept
{
    // 不能在 CriticalSection 内调用：叫醒任务要调用 RTOS，RTOS 退出它自己的临界区时会把 BASEPRI 清零
    configASSERT(__get_BASEPRI() == 0U);
    void *waiter = nullptr;
    {
        const CriticalSection critical_section;
        pending_ = true;
        waiter = waiter_;
    }
    if (waiter == nullptr)
    {
        return; // 没人在等：通知留在 pending_ 里，下一次 wait() 立即返回
    }
    TaskHandle_t const task = static_cast<TaskHandle_t>(waiter);
    if (in_interrupt())
    {
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveIndexedFromISR(task, NOTIFY_INDEX, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
    else
    {
        static_cast<void>(xTaskNotifyGiveIndexed(task, NOTIFY_INDEX));
    }
}

bool Signal::wait(Microseconds timeout) noexcept
{
    // 只能在任务里、临界区外调用
    configASSERT(!in_interrupt());
    configASSERT(__get_BASEPRI() == 0U);
    if (timeout == NO_WAIT)
    {
        const CriticalSection critical_section;
        const bool notified = pending_;
        pending_ = false;
        return notified;
    }

    const Microseconds start = now_us();
    {
        const CriticalSection critical_section;
        configASSERT(waiter_ == nullptr); // 两个任务同时等同一个 Signal 是误用
        if (pending_)
        {
            pending_ = false;
            return true;
        }
        waiter_ = xTaskGetCurrentTaskHandle();
    }

    for (;;)
    {
        TickType_t ticks = portMAX_DELAY;
        if (timeout != WAIT_FOREVER)
        {
            const Microseconds elapsed = elapsed_us(start);
            if (elapsed >= timeout)
            {
                break;
            }
            ticks = to_ticks(timeout - elapsed);
        }
        // 返回值只说明"有过任务通知"，可能是别的 Signal 迟到的，以 pending_ 为准
        static_cast<void>(ulTaskNotifyTakeIndexed(NOTIFY_INDEX, pdTRUE, ticks));

        const CriticalSection critical_section;
        if (pending_)
        {
            pending_ = false;
            waiter_ = nullptr;
            return true;
        }
    }

    // 超时：注销等待者；与超时同时到来的通知也算取到
    const CriticalSection critical_section;
    const bool notified = pending_;
    pending_ = false;
    waiter_ = nullptr;
    return notified;
}

} // namespace hal::os
