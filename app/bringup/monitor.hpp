#ifndef BRINGUP_MONITOR_HPP
#define BRINGUP_MONITOR_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/bringup.hpp"

#include <cstdint>

namespace bringup
{

/**
 * @brief 心跳监视：读 bringup.heartbeat，统计收到、丢失、乱序与延迟，验证 topic 的订阅与按 topic 触发的任务
 *        （Decision-20261031-12）。
 * @note 能力零 RTOS：由组合根放进按 bringup.heartbeat 触发的任务，或在宿主测试里直接调用 step()。
 */
class Monitor final
{
  public:
    struct Config
    {
        hal::os::Microseconds latency_limit_us; // 延迟超过它就计入 late
    };

    /** 统计结果，调试器可读（只有运行 step() 的任务写）。 */
    struct Report
    {
        std::uint32_t received;     // 读到的心跳条数
        std::uint32_t lost;         // 两次读取之间被覆盖、没读到的条数（Subscriber::lost_count）
        std::uint32_t out_of_order; // 序号不比上一条大的次数，应为 0
        std::uint32_t late;         // 延迟超过 latency_limit_us 的次数
        hal::os::Microseconds last_latency_us;
        hal::os::Microseconds max_latency_us;
        std::uint32_t last_sequence;
    };

    [[nodiscard]] bool init(const Config &config) noexcept;

    /** 有新心跳就读出并统计；延迟 = now − 心跳的发布时刻。 */
    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    topic::Subscriber<topics::bringup::Heartbeat> heartbeat_{topics::bringup::heartbeat};
    Config config_{};
    Report report_{};
};

} // namespace bringup

#endif // BRINGUP_MONITOR_HPP
