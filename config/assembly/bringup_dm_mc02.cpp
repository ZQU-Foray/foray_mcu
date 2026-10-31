// 组合根：bringup_dm_mc02（框架验证，Decision-20261031-12）。
// 构造能力对象、注入参数、定义任务表、实现启动入口 boot_*；不含行为。
#include "boot/boot.h"
#include "bringup/heartbeat.hpp"
#include "bringup/monitor.hpp"
#include "common/task_runner.hpp"
#include "hal/init.hpp"
#include "hal/os/clock.hpp"
#include "params.hpp"
#include "topics/bringup.hpp"

#include <array>

namespace
{

bringup::Heartbeat g_heartbeat;
bringup::Monitor g_monitor;

// 任务表里的 step：每个只调一个能力的 step(now)
void heartbeat_step(hal::os::Microseconds now)
{
    g_heartbeat.step(now);
}

void monitor_step(hal::os::Microseconds now)
{
    g_monitor.step(now);
}

// 读者（monitor）的优先级高于写者（heartbeat）：Decision-20261029-02 修好的情形。CubeMX 的默认任务是 24。
constexpr std::array<common::TaskSpec, 2> TASKS{{
    {"heartbeat", 256U, 25U, common::every_ms(1U), {heartbeat_step}},
    {"monitor", 256U, 26U, common::on_topic(topics::bringup::heartbeat), {monitor_step}},
}};

common::TaskRunner<TASKS> g_runner;

} // namespace

bool boot_init(void)
{
    return hal::init() && g_heartbeat.init() && g_monitor.init(params::MONITOR) && g_runner.init();
}

bool boot_init_rtos(void)
{
    return g_runner.start();
}

void boot_run(void)
{
    g_runner.sample_stack_health(hal::os::now_us());
}
