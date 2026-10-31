// 框架独立构建用（Decision-20261031-06）：显式实例化框架里只有头文件的模板。
// 这些模板没有使用方时根本不会被编译，框架独立构建也就检查不到它们。这里把每个成员函数都实例化一遍：
//   - 宿主（framework preset）：只靠 src 与标准库能编译，带 -Wall -Wextra -Werror；
//   - Cortex-M0+（framework-m0plus preset）：M0+ 没有 LDREX/STREX，原子读改写会编译成 __atomic_*、__sync_*
//     库调用。CI 用 tool/ci/arch_check.py atomics 检查目标文件里没有这类调用（规则 13）。
// 新增只有头文件的并发原语或 topic 相关模板时，在这里补上实例化。
#include "middleware/topic/topic.hpp"
#include "utils/concurrency/latest_value.hpp"
#include "utils/concurrency/spsc_queue.hpp"

#include <array>
#include <cstdint>

namespace template_check
{

// 大小不是 4 的倍数，LatestValue 的最后一个字有空余字节
struct Message
{
    std::uint32_t sequence;
    std::array<float, 7> values;
    std::uint8_t flags;
};

} // namespace template_check

template class concurrency::SpscQueue<std::uint8_t, 64U>;
template class concurrency::SpscQueue<template_check::Message, 4U>;
template class concurrency::LatestValue<template_check::Message>;
template class topic::Topic<template_check::Message>;
template class topic::Publisher<template_check::Message>;
template class topic::Subscriber<template_check::Message>;
