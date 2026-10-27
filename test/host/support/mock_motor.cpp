#include "support/mock_motor.hpp"
#include <cmath>

namespace mock
{
bool Adapter::init() noexcept
{
    g_ready = g_init_result;
    return g_ready;
}
void Adapter::process() noexcept
{
    ++g_processes;
    g_sent = g_staged;
}
bool Adapter::read_state(std::size_t id, device::MotorState &state) const noexcept
{
    state = {};
    if (!g_ready || id >= g_staged.size() || g_read_failure)
    {
        return false;
    }
    state.feedback_valid = true;
    state.online = true;
    return true;
}
bool Adapter::set_torque(std::size_t id, float value) noexcept
{
    ++g_writes;
    if (!g_ready || id >= g_staged.size() || g_command_failure)
    {
        return false;
    }
    if (!std::isfinite(value))
    {
        g_staged[id] = 0;
        return false;
    }
    g_staged[id] = value;
    return true;
}
void Adapter::clear_commands() noexcept
{
    ++g_clears;
    g_staged = {};
}
float Adapter::maximum_torque_newton_meter() const noexcept
{
    return g_ready ? g_maximum_torque : 0.0F;
}
} // namespace mock
