// hal::os 临界区在 DM-MC02 上的实现。
// 屏蔽框架中断：把 BASEPRI 抬到框架线（FreeRTOS 的 configMAX_SYSCALL_INTERRUPT_PRIORITY，优先级 5），
// 优先级 0–4 的中断照常响应。等价于 FreeRTOS 的 portSET_INTERRUPT_MASK_FROM_ISR /
// portCLEAR_INTERRUPT_MASK_FROM_ISR：保存原值、析构时写回，所以可以嵌套，任务与中断里都能用。
// 不用 taskENTER_CRITICAL：它只能在任务里用，而且按嵌套计数开关中断，不保存原状态。
#include "hal/os/critical_section.hpp"

#include "FreeRTOS.h"  // configMAX_SYSCALL_INTERRUPT_PRIORITY
#include "stm32h7xx.h" // CMSIS：__get_BASEPRI、__set_BASEPRI_MAX、__set_BASEPRI

namespace hal::os
{

CriticalSection::CriticalSection() noexcept : saved_state_{__get_BASEPRI()}
{
    // BASEPRI_MAX 只会把屏蔽抬高、不会降低：已经在更严的屏蔽里时保持不变
    __set_BASEPRI_MAX(configMAX_SYSCALL_INTERRUPT_PRIORITY);
    // 与 FreeRTOS 的 ulPortRaiseBASEPRI 相同：保证后面的指令执行时屏蔽已经生效
    __ISB();
    __DSB();
}

CriticalSection::~CriticalSection()
{
    __set_BASEPRI(saved_state_);
}

} // namespace hal::os
