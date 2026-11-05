#ifndef TOPICS_ALL_HPP
#define TOPICS_ALL_HPP

#include "middleware/topic/topic.hpp"
#include "topics/bringup.hpp"
#include "topics/command.hpp"
#include "topics/gimbal.hpp"
#include "topics/imu.hpp"
#include "topics/pc_link.hpp"
#include "topics/remote.hpp"
#include "topics/safety.hpp"

#include <array>

// 本产品的全部 topic，给调试通道列出名字与观测计数（app/topics/README.md）。新增 topic 时同步加入。

namespace topics
{

inline constexpr std::array<const topic::TopicBase *, 11> ALL{
    &bringup::heartbeat,
    &bringup::imu_event,
    &bringup::imu,
    &bringup::sbus,
    &safety::stop,
    &imu::event,
    &imu::attitude,
    &pc_link::control,
    &remote::input,
    &command::gimbal,
    &gimbal::state,
};

} // namespace topics

#endif // TOPICS_ALL_HPP
