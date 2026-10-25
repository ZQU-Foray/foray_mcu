#ifndef PLATFORM_INTERFACE_TIME_HPP
#define PLATFORM_INTERFACE_TIME_HPP

#include "Detail/Time.h"
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
  [[nodiscard]] static bool IsReady() noexcept
  {
    return TimePort_IsReady();
  }

  /**
   * @brief 获取高精度周期计数器的当前 32 位计数值。
   * @return 计数器可用时返回当前 Tick，不可用时返回 0。
   * @note 计数值会自然回绕，时间差应使用无符号减法计算。
   */
  [[nodiscard]] static Tick NowTicks() noexcept
  {
    return TimePort_NowTicks();
  }

  /**
   * @brief 获取高精度周期计数器每秒产生的 Tick 数。
   * @return 计数器可用时返回频率，单位为 Hz；不可用时返回 0。
   */
  [[nodiscard]] static std::uint32_t TickFrequencyHz() noexcept
  {
    return TimePort_FrequencyHz();
  }

  /**
   * @brief 将一段 Tick 数转换为秒数。
   * @param elapsedTicks 需要转换的 Tick 数。
   * @return 按当前计数频率换算的秒数；计数器不可用时返回 0。
   */
  [[nodiscard]] static float TicksToSeconds(Tick elapsedTicks) noexcept
  {
    const std::uint32_t frequencyHz = TickFrequencyHz();
    if (frequencyHz == 0U)
    {
      return 0.0F;
    }

    return static_cast<float>(elapsedTicks) / static_cast<float>(frequencyHz);
  }

  /**
   * @brief 计算从指定起始 Tick 到当前时刻经过的秒数。
   * @param startTick 起始时刻的 Tick 值。
   * @return 按无符号回绕规则计算的经过秒数。
   */
  [[nodiscard]] static float ElapsedSeconds(Tick startTick) noexcept
  {
    return TicksToSeconds(NowTicks() - startTick);
  }

  /**
   * @brief 计算距上次记录经过的秒数，并把记录更新为当前 Tick。
   * @param previousTick 保存上次 Tick 的引用，调用后更新为当前值。
   * @return 两次记录之间按无符号回绕规则计算的秒数。
   */
  [[nodiscard]] static float DeltaSeconds(Tick &previousTick) noexcept
  {
    const Tick currentTick = NowTicks();
    const float deltaSeconds = TicksToSeconds(currentTick - previousTick);
    previousTick = currentTick;
    return deltaSeconds;
  }

  /**
   * @brief 判断从指定起始 Tick 起是否已经达到给定毫秒超时。
   * @param startTick 起始时刻的 Tick 值。
   * @param timeoutMs 需要判断的超时时长，单位为毫秒。
   * @return 已达到超时时长时返回 true；计数器不可用或时长无法表示时返回 false。
   */
  [[nodiscard]] static bool HasElapsedMs(Tick startTick, std::uint32_t timeoutMs) noexcept
  {
    const std::uint32_t frequencyHz = TickFrequencyHz();
    if (frequencyHz == 0U)
    {
      return false;
    }

    if (timeoutMs == 0U)
    {
      return true;
    }

    constexpr std::uint64_t millisecondsPerSecond = 1000ULL;
    const std::uint64_t timeoutTicks =
        (static_cast<std::uint64_t>(timeoutMs) * frequencyHz + millisecondsPerSecond - 1ULL) / millisecondsPerSecond;

    if (timeoutTicks > std::numeric_limits<Tick>::max())
    {
      return false;
    }

    return static_cast<Tick>(NowTicks() - startTick) >= static_cast<Tick>(timeoutTicks);
  }

  /**
   * @brief 执行阻塞式微秒延时。
   * @param delayUs 请求延时的微秒数，传入 0 时立即返回。
   * @note 仅用于确实需要忙等待的短时序，不应在中断或长周期任务中滥用。
   */
  static void DelayUs(std::uint32_t delayUs) noexcept
  {
    TimePort_DelayUs(delayUs);
  }
};

} // namespace platform

#endif
