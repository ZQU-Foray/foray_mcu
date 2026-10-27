#include "tasks/chassis_task.hpp"
#include "support/mock_motor.hpp"
#include <cassert>
#include <cmath>
#include <string>
#include <cstdlib>

namespace
{
unsigned g_delays = 0;
std::string g_scenario;
} // namespace
extern "C"
{
    bool TimePort_Init()
    {
        return true;
    }
    bool TimePort_IsReady()
    {
        return true;
    }
    std::uint32_t TimePort_NowTicks()
    {
        return g_delays;
    }
    std::uint32_t TimePort_FrequencyHz()
    {
        return 1000;
    }
    void TimePort_DelayUs(std::uint32_t) {}
}
[[noreturn]] void osThreadExit()
{
    std::abort();
}
std::int32_t osDelay(std::uint32_t ticks)
{
    assert(ticks == 1);
    ++g_delays;
    using Task = application::task::ChassisTask;
    if (g_delays == 1)
    {
        Task::set_mode(Task::WheelControlMode::TORQUE);
        Task::set_wheel_target(0, 0.3F);
    }
    if (g_delays == 2)
    {
        assert(mock::g_processes == 1 && mock::g_sent[0] == 0);
        assert(std::fabs(mock::g_staged[0] - 0.3F) < 1e-6F);
        mock::g_read_failure = g_scenario == "read_failure";
        mock::g_command_failure = g_scenario == "command_failure";
    }
    if (g_delays == 3)
    {
        assert(mock::g_processes == 2);
        assert(std::fabs(mock::g_sent[0] - 0.3F) < 1e-6F); // previous cycle's command
        if (mock::g_read_failure || mock::g_command_failure)
            assert(mock::g_staged[0] == 0);
        mock::g_read_failure = false;
        mock::g_command_failure = false;
        Task::set_wheel_target(0, 0.3F); // recover after failure/reset
    }
    if (g_delays == 4)
    {
        assert(mock::g_processes == 3);
        if (g_scenario != "normal")
            assert(mock::g_sent[0] == 0);
        assert(std::fabs(mock::g_staged[0] - 0.3F) < 1e-6F);
        Task::set_enabled(false);
    }
    if (g_delays == 5)
    {
        assert(mock::g_processes == 4 && mock::g_staged[0] == 0);
    }
    if (g_delays == 6)
    {
        assert(mock::g_processes == 5 && mock::g_sent[0] == 0);
        if (g_scenario == "command_failure")
            assert(mock::g_clears == 1);
        std::exit(0);
    }
    return 0;
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    g_scenario = argv[1];
    using Task = application::task::ChassisTask;
    if (g_scenario == "init_failure")
        mock::g_init_result = false;
    if (g_scenario == "init_failure")
    {
        assert(!Task::init());
        assert(!Task::is_ready() && mock::g_clears == 1);
        mock::g_init_result = true;
        assert(Task::init());
        return 0;
    }
    assert(Task::init());
    assert(Task::init());
    // a2cfd16 起底盘上电默认失能、由上层使能（Run 不再自行使能），这里模拟上层使能。
    Task::set_enabled(true);
    Task::run(nullptr);
}
