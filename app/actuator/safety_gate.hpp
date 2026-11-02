#ifndef ACTUATOR_SAFETY_GATE_HPP
#define ACTUATOR_SAFETY_GATE_HPP

#include "driver/motor/joint.hpp"
#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/safety.hpp"

#include <cstddef>
#include <cstdint>

namespace actuator
{

/**
 * @brief 安全门（Decision-20261102-03 D7）：按 safety.stop 集中决定一组关节能不能出力，能力不必各写一遍。
 *
 * - 上电时锁住：关节被禁止（Joint::set_inhibited(true)），能力 enable() 都失败；
 * - 锁住后，只有 safety.stop 不是急停、且 arm_sequence 比锁住时变了（操作者确认过）才放开；
 * - 放开后，收到急停，或超过 stop_timeout_us 收不到 safety.stop（发布者挂了），立即锁住。被禁止的关节随即撤销出力，
 *   能力要在放开后重新 enable()。
 *
 * @note 与这些关节、它们的 CanBus 在同一个任务里（D3），step 排在能力的 step 之后、CanBus::transmit() 之前。
 */
class SafetyGate final
{
  public:
    enum class State : std::uint8_t
    {
        LOCKED = 0U,
        RELEASED = 1U
    };

    /** 来自 profile 的参数。 */
    struct Parameters
    {
        hal::os::Microseconds stop_timeout_us; // 这么久收不到 safety.stop 就当急停
    };

    /** 组合根注入：管哪些关节（指向组合根里的数组）与参数。 */
    struct Config
    {
        device::Joint *const *joints;
        std::size_t count;
        Parameters parameters;
    };

    struct Report
    {
        State state;
        std::uint32_t lock_count;    // 从放开变为锁住的次数（含超时）
        std::uint32_t timeout_count; // 其中因超时收不到 safety.stop 的次数
        std::uint32_t release_count;
        bool has_message;
        hal::os::Microseconds last_message_us; // 最后一次收到 safety.stop 的时刻
    };

    /** 检查配置并锁住全部关节。关节为空或参数无效时返回 false。 */
    [[nodiscard]] bool init(const Config &config) noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    void apply(bool locked) noexcept;

    Config config_{};
    Report report_{};
    topic::Subscriber<topics::safety::Stop> stop_{topics::safety::stop};
    topics::safety::Stop message_{};
    std::uint32_t arm_at_lock_{0U};
    bool arm_known_{false}; // 锁住之后是否已经记下当时的 arm_sequence
};

} // namespace actuator

#endif // ACTUATOR_SAFETY_GATE_HPP
