#include "actuator/safety_gate.hpp"

namespace actuator
{

namespace
{

/** 有符号差：消息可能在本轮开始之后才收到（与驱动里同样的考虑）。 */
bool expired(hal::os::Microseconds stamp, hal::os::Microseconds duration, hal::os::Microseconds now)
{
    return static_cast<std::int32_t>(now - stamp) >= static_cast<std::int32_t>(duration);
}

} // namespace

bool SafetyGate::init(const Config &config) noexcept
{
    config_ = config;
    report_ = Report{};
    message_ = {};
    arm_known_ = false;
    if (config.joints == nullptr || config.count == 0U || config.parameters.stop_timeout_us == 0U)
    {
        return false;
    }
    for (std::size_t index = 0U; index < config.count; ++index)
    {
        if (config.joints[index] == nullptr)
        {
            return false;
        }
    }
    report_.state = State::LOCKED;
    apply(true);
    return true;
}

void SafetyGate::step(hal::os::Microseconds now) noexcept
{
    if (config_.joints == nullptr)
    {
        return;
    }
    topics::safety::Stop message{};
    if (stop_.update(message))
    {
        message_ = message;
        report_.has_message = true;
        report_.last_message_us = now;
    }
    const bool stale =
        !report_.has_message || expired(report_.last_message_us, config_.parameters.stop_timeout_us, now);
    const bool stop = stale || message_.stop;

    if (report_.state == State::RELEASED)
    {
        if (stop)
        {
            report_.state = State::LOCKED;
            ++report_.lock_count;
            report_.timeout_count += stale ? 1U : 0U;
            arm_known_ = !stale;
            arm_at_lock_ = message_.arm_sequence;
        }
    }
    else if (report_.has_message && !stale)
    {
        // 锁住后第一次看到消息：记下当时的确认计数，要等它变化（操作者确认）才放开
        if (!arm_known_)
        {
            arm_known_ = true;
            arm_at_lock_ = message_.arm_sequence;
        }
        else if (!message_.stop && message_.arm_sequence != arm_at_lock_)
        {
            report_.state = State::RELEASED;
            ++report_.release_count;
        }
    }
    apply(report_.state == State::LOCKED);
}

void SafetyGate::apply(bool locked) noexcept
{
    for (std::size_t index = 0U; index < config_.count; ++index)
    {
        config_.joints[index]->set_inhibited(locked);
    }
}

} // namespace actuator
