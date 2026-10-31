#ifndef PARAMS_HPP
#define PARAMS_HPP

// bringup_dm_mc02 的参数（只含数据，规则 4）。

#include "bringup/monitor.hpp"

namespace params
{

// 心跳从发布到被监视任务读到的延迟上限：监视任务的优先级更高，正常应在几十 µs 内
inline constexpr bringup::Monitor::Config MONITOR{100U};

} // namespace params

#endif // PARAMS_HPP
