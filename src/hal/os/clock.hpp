#ifndef HAL_OS_CLOCK_HPP
#define HAL_OS_CLOCK_HPP

#include <cstdint>

// 最小 OS 契约：单调时基（评审报告 §5.7、Q10）。
// 实现：固件由 platform 的 port 提供（P3a），宿主测试由 platform/mock 提供。
// P1 草案，P3a 冻结契约 v1 时可以调整。

namespace hal::os
{

/** 单调计数值，按 2^32 回绕；两次读数做无符号减法即得经过的计数。 */
using Tick = std::uint32_t;

/**
 * @brief 读取当前单调计数。
 * @note 可在任务与中断上下文调用，不阻塞。
 */
[[nodiscard]] Tick now_ticks() noexcept;

/**
 * @brief 计数频率（Hz）。
 * @note 运行期间不变；调用方据此把计数差换算为时间。
 */
[[nodiscard]] std::uint32_t tick_frequency_hz() noexcept;

} // namespace hal::os

#endif // HAL_OS_CLOCK_HPP
