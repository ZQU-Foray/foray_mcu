#ifndef TOPICS_ALL_HPP
#define TOPICS_ALL_HPP

#include "middleware/topic/topic.hpp"
#include "topics/bringup.hpp"

#include <array>

// 本产品的全部 topic，给调试通道列出名字与观测计数（app/topics/README.md）。新增 topic 时同步加入。

namespace topics
{

inline constexpr std::array<const topic::TopicBase *, 1> ALL{&bringup::heartbeat};

} // namespace topics

#endif // TOPICS_ALL_HPP
