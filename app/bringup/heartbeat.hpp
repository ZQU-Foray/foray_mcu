#ifndef BRINGUP_HEARTBEAT_HPP
#define BRINGUP_HEARTBEAT_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/bringup.hpp"

#include <cstdint>

namespace bringup
{

/**
 * @brief 心跳：每轮发布一次 bringup.heartbeat（序号 + 本轮时刻），验证周期任务与 topic 发布（Decision-20261031-12）。
 * @note 能力零 RTOS：由组合根放进周期任务，或在宿主测试里直接调用 step()。
 */
class Heartbeat final
{
  public:
    /** 取得 bringup.heartbeat 的发布句柄；已被别处取得时返回 false。 */
    [[nodiscard]] bool init() noexcept;

    /** 发布一条心跳。 */
    void step(hal::os::Microseconds now) noexcept;

    /** 已发布的条数。 */
    [[nodiscard]] std::uint32_t sequence() const noexcept
    {
        return sequence_;
    }

  private:
    topic::Publisher<topics::bringup::Heartbeat> publisher_{};
    std::uint32_t sequence_{0U};
};

} // namespace bringup

#endif // BRINGUP_HEARTBEAT_HPP
