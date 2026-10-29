#ifndef HAL_TIME_HPP
#define HAL_TIME_HPP

#include "hal/detail/time.h"
#include <cstdint>
#include <limits>

namespace platform
{

class Time final
{
  public:
    using Tick = std::uint32_t;

    /**
   * @brief 禁止创建 Time 实例，所有能力均通过静态方法访问。
   */
    Time() = delete;

    /**
   * @brief 查询平台高精度周期计数器是否已初始化并保持运行。
   * @return 计数器可用时返回 true，否则返回 false。
   */
    [[nodiscard]] static bool is_ready() noexcept
    {
        return TimePort_IsReady();
    }

    /**
   * @brief 获取高精度周期计数器的当前 32 位计数值。
   * @return 计数器可用时返回当前 Tick，不可用时返回 0。
   * @note 计数值会自然回绕，时间差应使用无符号减法计算。
   */
    [[nodiscard]] static Tick now_ticks() noexcept
    {
        return TimePort_NowTicks();
    }

    /**
   * @brief 获取高精度周期计数器每秒产生的 Tick 数。
   * @return 计数器可用时返回频率，单位为 Hz；不可用时返回 0。
   */
    [[nodiscard]] static std::uint32_t tick_frequency_hz() noexcept
    {
        return TimePort_FrequencyHz();
    }

    /**
   * @brief 将一段 Tick 数转换为秒数。
   * @param elapsed_ticks 需要转换的 Tick 数。
   * @return 按当前计数频率换算的秒数；计数器不可用时返回 0。
   */
    [[nodiscard]] static float ticks_to_seconds(Tick elapsed_ticks) noexcept
    {
        const std::uint32_t frequency_hz = tick_frequency_hz();
        if (frequency_hz == 0U)
        {
            return 0.0F;
        }

        return static_cast<float>(elapsed_ticks) / static_cast<float>(frequency_hz);
    }

    /**
   * @brief 计算从指定起始 Tick 到当前时刻经过的秒数。
   * @param start_tick 起始时刻的 Tick 值。
   * @return 按无符号回绕规则计算的经过秒数。
   */
    [[nodiscard]] static float elapsed_seconds(Tick start_tick) noexcept
    {
        return ticks_to_seconds(now_ticks() - start_tick);
    }

    /**
   * @brief 计算距上次记录经过的秒数，并把记录更新为当前 Tick。
   * @param previous_tick 保存上次 Tick 的引用，调用后更新为当前值。
   * @return 两次记录之间按无符号回绕规则计算的秒数。
   */
    [[nodiscard]] static float delta_seconds(Tick &previous_tick) noexcept
    {
        const Tick current_tick = now_ticks();
        const float delta_seconds = ticks_to_seconds(current_tick - previous_tick);
        previous_tick = current_tick;
        return delta_seconds;
    }

    /**
   * @brief 判断从指定起始 Tick 起是否已经达到给定毫秒超时。
   * @param start_tick 起始时刻的 Tick 值。
   * @param timeout_ms 需要判断的超时时长，单位为毫秒。
   * @return 已达到超时时长时返回 true；计数器不可用或时长无法表示时返回 false。
   */
    [[nodiscard]] static bool has_elapsed_ms(Tick start_tick, std::uint32_t timeout_ms) noexcept
    {
        const std::uint32_t frequency_hz = tick_frequency_hz();
        if (frequency_hz == 0U)
        {
            return false;
        }

        if (timeout_ms == 0U)
        {
            return true;
        }

        constexpr std::uint64_t MILLISECONDS_PER_SECOND = 1000ULL;
        const std::uint64_t timeout_ticks =
            (static_cast<std::uint64_t>(timeout_ms) * frequency_hz + MILLISECONDS_PER_SECOND - 1ULL) /
            MILLISECONDS_PER_SECOND;

        if (timeout_ticks > std::numeric_limits<Tick>::max())
        {
            return false;
        }

        return static_cast<Tick>(now_ticks() - start_tick) >= static_cast<Tick>(timeout_ticks);
    }

    /**
   * @brief 执行阻塞式微秒延时。
   * @param delay_us 请求延时的微秒数，传入 0 时立即返回。
   * @note 仅用于确实需要忙等待的短时序，不应在中断或长周期任务中滥用。
   */
    static void delay_us(std::uint32_t delay_us) noexcept
    {
        TimePort_DelayUs(delay_us);
    }
};

} // namespace platform

#endif
