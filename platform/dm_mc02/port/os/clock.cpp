// hal::os 时基在 DM-MC02 上的实现（Decision-20261031-01、-04、-10）。
// TIM2 是 32 位定时器，CubeMX 配成 240 MHz / 240 = 1 MHz 自由计数、自动重装 0xFFFFFFFF、不开中断，
// 计数值就是当前时刻（µs），回绕周期正好是 2^32 µs，与契约一致。
#include "hal/os/clock.hpp"
#include "os.hpp"

#include "tim.h" // htim2（CubeMX 生成）

namespace hal::os
{

Microseconds now_us() noexcept
{
    // 32 位寄存器，一次读取就是原子的；任务与中断里都能调用，不进临界区
    return TIM2->CNT;
}

} // namespace hal::os

namespace port::os
{

bool start_clock() noexcept
{
    // 调试暂停时冻结 TIM2：RTOS 节拍（SysTick）在内核暂停时也停，两者一致，
    // 断点恢复后不会因为时基跳了一大截而集体超时（Decision-20261031-01 的建议）
    __HAL_DBGMCU_FREEZE_TIM2();
    return HAL_TIM_Base_Start(&htim2) == HAL_OK;
}

} // namespace port::os
