#include "bringup/monitor.hpp"

namespace bringup
{

bool Monitor::init(const Config &config) noexcept
{
    config_ = config;
    report_ = Report{};
    return true;
}

void Monitor::step(hal::os::Microseconds now) noexcept
{
    topics::bringup::Heartbeat message{};
    if (!heartbeat_.update(message))
    {
        return; // 多次发布合并成一轮、上一轮已经读过：没有新数据
    }
    if (report_.received != 0U && message.sequence <= report_.last_sequence)
    {
        ++report_.out_of_order;
    }
    ++report_.received;
    report_.last_sequence = message.sequence;
    report_.lost = heartbeat_.lost_count();

    const hal::os::Microseconds latency = hal::os::elapsed_us(message.timestamp_us, now);
    report_.last_latency_us = latency;
    if (latency > report_.max_latency_us)
    {
        report_.max_latency_us = latency;
    }
    if (latency > config_.latency_limit_us)
    {
        ++report_.late;
    }
}

} // namespace bringup
