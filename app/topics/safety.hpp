#ifndef TOPICS_SAFETY_HPP
#define TOPICS_SAFETY_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"

#include <cstdint>

// 安全相关的 topic（Decision-20261102-03 D7）。发布者是负责安全的能力（遥控、裁判系统，阶段 A 重建）；
// 安全门 actuator::SafetyGate 订阅。

namespace topics::safety
{

/** 急停与重新确认。发布者要按周期发布：安全门在超时内收不到就当急停（发布者挂了）。 */
struct Stop
{
    bool stop; // true：所有电机立即停止出力
    std::uint32_t arm_sequence; // 操作者每确认一次加 1（例如遥控拨杆回中再拨上）；急停解除后要看到它变化才恢复
    hal::os::Microseconds timestamp_us; // 发布时刻
};

inline topic::Topic<Stop> stop{"safety.stop"};

} // namespace topics::safety

#endif // TOPICS_SAFETY_HPP
