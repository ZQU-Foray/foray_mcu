// device::Joint 与 JointLink（Decision-20261102-03）：机构换算、出力门控、指令超时、安全门禁止、全组使能、绑定检查。
// 驱动那一侧由测试直接调 JointLink 扮演。
#include "driver/motor/joint.hpp"
#include "mock_os.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <limits>

namespace
{

using device::Joint;
using device::JointCommand;
using device::JointLink;
using device::JointMechanics;
using device::JointMode;
using device::MotorFeedback;

bool near(float actual, float expected, float tolerance = 1e-5F)
{
    return std::fabs(actual - expected) <= tolerance;
}

JointLink::Binding binding(std::uint8_t modes, const JointMechanics &mechanics = JointMechanics{})
{
    JointLink::Binding result{};
    result.modes = modes;
    result.motor_maximum_torque_newton_meter = 10.0F;
    result.position_absolute = true;
    result.has_temperature = true;
    result.mechanics = mechanics;
    return result;
}

void test_binding()
{
    Joint joint;
    assert(!joint.available() && !joint.enable() && !joint.supports(JointMode::TORQUE));
    assert(joint.maximum_torque_newton_meter() == 0.0F);
    JointMechanics bad{};
    bad.direction = 0;
    assert(!JointLink::bind(joint, binding(JointLink::TORQUE_ONLY, bad)));
    bad = JointMechanics{};
    bad.gear_ratio = 0.0F;
    assert(!JointLink::bind(joint, binding(JointLink::TORQUE_ONLY, bad)));
    bad.gear_ratio = std::numeric_limits<float>::infinity();
    assert(!JointLink::bind(joint, binding(JointLink::TORQUE_ONLY, bad)));
    bad = JointMechanics{};
    bad.zero_offset_rad = std::numeric_limits<float>::quiet_NaN();
    assert(!JointLink::bind(joint, binding(JointLink::TORQUE_ONLY, bad)));
    assert(!JointLink::bind(joint, binding(0U))); // 至少要支持力矩

    JointMechanics mechanics{};
    mechanics.gear_ratio = 2.0F;
    assert(JointLink::bind(joint, binding(JointLink::TORQUE_ONLY, mechanics)));
    assert(!JointLink::bind(joint, binding(JointLink::TORQUE_ONLY))); // 一个关节只接一台电机
    assert(joint.supports(JointMode::TORQUE) && !joint.supports(JointMode::IMPEDANCE));
    assert(near(joint.maximum_torque_newton_meter(), 20.0F)); // 电机 10 N·m × 外接 2:1
    assert(joint.state().position_absolute && joint.state().has_temperature && !joint.state().feedback_valid);
    assert(!joint.available()); // 驱动还没就绪
}

void test_transform()
{
    mock::reset();
    Joint joint;
    JointMechanics mechanics{};
    mechanics.direction = -1;
    mechanics.gear_ratio = 2.0F;
    mechanics.zero_offset_rad = 1.0F;
    assert(JointLink::bind(joint, binding(JointLink::TORQUE_AND_IMPEDANCE, mechanics)));

    // 反馈：q = −(p − 1) / 2，q̇ = −v / 2，τj = −τ × 2
    JointLink::publish(joint, MotorFeedback{3.0F, 4.0F, 0.5F, 41.0F, 777U});
    const auto &state = joint.state();
    assert(near(state.position_rad, -1.0F) && near(state.velocity_rad_per_second, -2.0F));
    assert(near(state.torque_newton_meter, -1.0F) && near(state.temperature_celsius, 41.0F));
    assert(state.stamp_us == 777U && state.feedback_valid);
    JointLink::set_status(joint, true, true, false, 0x1U);
    assert(state.online && state.enabled && !state.fault && state.fault_code == 0x1U);

    // 指令：电机侧 τ = −τj / 2；p = 1 − 2q；v = −2q̇；K、D 除以 4
    JointLink::set_available(joint, true);
    assert(joint.enable());
    assert(joint.set_torque(1.0F));
    JointCommand command = JointLink::command(joint, 0U);
    assert(command.mode == JointMode::TORQUE && near(command.torque_newton_meter, -0.5F));
    assert(joint.set_impedance(0.5F, 1.0F, 8.0F, 0.4F, 2.0F));
    command = JointLink::command(joint, 0U);
    assert(command.mode == JointMode::IMPEDANCE && near(command.position_rad, 0.0F));
    assert(near(command.velocity_rad_per_second, -2.0F) && near(command.stiffness, 2.0F) &&
           near(command.damping, 0.1F));
    assert(near(command.torque_newton_meter, -1.0F));
}

void test_gating_and_timeout()
{
    mock::reset();
    Joint joint;
    assert(JointLink::bind(joint, binding(JointLink::TORQUE_ONLY)));
    assert(!joint.enable()); // 驱动没就绪（达妙还在核对）
    JointLink::set_available(joint, true);
    assert(!joint.set_torque(1.0F));                            // 没有 enable()
    assert(!joint.set_impedance(0.0F, 0.0F, 1.0F, 0.1F, 0.0F)); // 型号不支持阻抗
    assert(joint.enable() && joint.enable() && JointLink::enable_requested(joint));
    // 使能后还没给指令：0 力矩
    assert(JointLink::command(joint, 0U).torque_newton_meter == 0.0F);

    mock::set_now_us(10000U);
    assert(joint.set_torque(2.0F));
    assert(near(JointLink::command(joint, 12999U).torque_newton_meter, 2.0F));
    // 默认 3 ms 没刷新：0 力矩，只计一次
    assert(JointLink::command(joint, 13000U).torque_newton_meter == 0.0F);
    assert(JointLink::command(joint, 14000U).torque_newton_meter == 0.0F && joint.command_timeout_count() == 1U);
    mock::set_now_us(14000U);
    assert(joint.set_torque(2.0F) && near(JointLink::command(joint, 14000U).torque_newton_meter, 2.0F));

    // 指令在这一轮开始之后才下（时间戳晚于驱动发送时用的 now）：按新指令发，不算超时（2026-11-02 上板发现）
    mock::set_now_us(20500U);
    assert(joint.set_torque(3.0F));
    assert(near(JointLink::command(joint, 20000U).torque_newton_meter, 3.0F) && joint.command_timeout_count() == 1U);
    mock::set_now_us(14000U);
    assert(joint.set_torque(2.0F));

    // 非有限值：清掉指令
    assert(!joint.set_torque(std::numeric_limits<float>::quiet_NaN()));
    assert(JointLink::command(joint, 14000U).torque_newton_meter == 0.0F && JointLink::enable_requested(joint));

    // 驱动撤销出力（故障）：要再 enable()
    assert(joint.set_torque(1.0F));
    JointLink::drop_enable(joint);
    assert(!JointLink::enable_requested(joint) && !joint.set_torque(1.0F));
    assert(joint.enable() && joint.set_torque(1.0F));
    // 驱动不就绪了（重新核对）：同样撤销
    JointLink::set_available(joint, false);
    assert(!JointLink::enable_requested(joint) && !joint.enable());
    JointLink::set_available(joint, true);

    // disable()：撤销请求、清掉指令
    assert(joint.enable() && joint.set_torque(1.0F));
    joint.disable();
    assert(!JointLink::enable_requested(joint) && JointLink::command(joint, 14000U).torque_newton_meter == 0.0F);
}

void test_inhibit_and_groups()
{
    mock::reset();
    std::array<Joint, 3> joints{};
    for (Joint &joint : joints)
    {
        assert(JointLink::bind(joint, binding(JointLink::TORQUE_ONLY)));
        JointLink::set_available(joint, true);
    }
    const std::array<Joint *, 3> group{&joints[0], &joints[1], &joints[2]};

    // 安全门禁止：立即撤销，enable() 失败，直到解除
    assert(joints[0].enable() && joints[0].set_torque(1.0F));
    joints[0].set_inhibited(true);
    assert(joints[0].inhibited() && !JointLink::enable_requested(joints[0]) && !joints[0].enable());
    assert(!joints[0].set_torque(1.0F));
    // 全组使能：有一个不能就一个都不请求
    assert(!device::enable_all(group.data(), group.size()));
    assert(!JointLink::enable_requested(joints[1]) && !JointLink::enable_requested(joints[2]));
    joints[0].set_inhibited(false);
    assert(device::enable_all(group.data(), group.size()));
    for (Joint &joint : joints)
    {
        assert(JointLink::enable_requested(joint));
    }
    device::disable_all(group.data(), group.size());
    for (Joint &joint : joints)
    {
        assert(!JointLink::enable_requested(joint));
    }
    assert(!device::enable_all(nullptr, 1U));
}

} // namespace

int main()
{
    test_binding();
    test_transform();
    test_gating_and_timeout();
    test_inhibit_and_groups();
    return 0;
}
