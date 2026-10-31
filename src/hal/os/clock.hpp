#ifndef HAL_OS_CLOCK_HPP
#define HAL_OS_CLOCK_HPP

#include <cstdint>

// 最小 OS 契约：单调时基（Decision-20261031-01）。
// 单位固定为 1 µs，各平台一致，换算都是编译期常量。
// 实现：固件由 platform 的 port 提供（F2），宿主测试由 platform/mock 提供。

namespace hal::os
{

/**
 * 时刻与时长，单位 µs，按 2^32 回绕（约 71.6 分钟）。
 * 两个时刻做无符号减法得到经过的时长，跨回绕也正确，前提是两者相隔不到一个回绕周期。
 */
using Microseconds = std::uint32_t;

inline constexpr Microseconds MICROSECONDS_PER_MILLISECOND{1000U};
inline constexpr Microseconds MICROSECONDS_PER_SECOND{1000000U};

/**
 * @brief 读取当前时刻。
 * @note 任务与中断里都能调用，不阻塞，不进临界区。
 * @note hal::init() 返回后开始单调递增，之前的读数没有意义。
 */
[[nodiscard]] Microseconds now_us() noexcept;

/**
 * @brief 从 start 到 now 经过的时长。
 * @note now 省略时读当前时刻；一次判断里要多次使用同一时刻时，先读一次再传进来。
 */
[[nodiscard]] constexpr Microseconds elapsed_us(Microseconds start, Microseconds now = now_us()) noexcept
{
    return now - start;
}

/**
 * @brief 从 start 起是否已经过了 duration。
 * @note duration 最大为 2^32 − 1 µs；两次判断之间的间隔要小于一个回绕周期，否则会漏判。
 */
[[nodiscard]] constexpr bool
has_elapsed(Microseconds start, Microseconds duration, Microseconds now = now_us()) noexcept
{
    return now - start >= duration;
}

/**
 * @brief 把时长换算成秒，用于控制周期 dt 这类浮点计算。
 * @note 约 16.8 s（2^24 µs）以内只有除法的一次舍入。
 */
[[nodiscard]] constexpr float to_seconds(Microseconds duration) noexcept
{
    return static_cast<float>(duration) / static_cast<float>(MICROSECONDS_PER_SECOND);
}

} // namespace hal::os

#endif // HAL_OS_CLOCK_HPP
