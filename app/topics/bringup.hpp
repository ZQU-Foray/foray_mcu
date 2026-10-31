#ifndef TOPICS_BRINGUP_HPP
#define TOPICS_BRINGUP_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"

#include <cstdint>

// bringup 能力发布的 topic（Decision-20261031-12）：框架验证用，不含 RM 业务。

namespace topics::bringup
{

/** 心跳：bringup::Heartbeat 每轮发布一次。 */
struct Heartbeat
{
    std::uint32_t sequence;             // 从 1 开始，每次发布加 1
    hal::os::Microseconds timestamp_us; // 发布时刻（发布任务本轮开始的时刻）
};

inline topic::Topic<Heartbeat> heartbeat{"bringup.heartbeat"};

} // namespace topics::bringup

#endif // TOPICS_BRINGUP_HPP
