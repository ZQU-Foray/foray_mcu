#ifndef TOPICS_REMOTE_HPP
#define TOPICS_REMOTE_HPP

#include "middleware/topic/topic.hpp"
#include "topics/operator_input.hpp"

// SBUS 遥控能力 remote::Remote 发布的 topic。

namespace topics::remote
{

/** 遥控的最新状态，类型与其他输入来源共用（topics::OperatorInput）。 */
inline topic::Topic<OperatorInput> input{"remote.input"};

} // namespace topics::remote

#endif
