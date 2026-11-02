// actuator::SafetyGate（Decision-20261102-03 D7）：上电锁住、确认后放开、急停与超时立即锁住、恢复要重新确认。
// 关节的驱动那一侧由测试经 JointLink 扮演。
#include "actuator/safety_gate.hpp"
#include "driver/motor/joint.hpp"
#include "mock_os.hpp"
#include "topics/safety.hpp"

#include <array>
#include <cassert>
#include <cstdint>

namespace
{

using actuator::SafetyGate;
using device::Joint;
using device::JointLink;
using State = SafetyGate::State;

constexpr hal::os::Microseconds TIMEOUT_US{50000U};

topic::Publisher<topics::safety::Stop> g_publisher{};

void publish(bool stop, std::uint32_t arm)
{
    g_publisher.publish(topics::safety::Stop{stop, arm, hal::os::now_us()});
}

void step(SafetyGate &gate)
{
    gate.step(hal::os::now_us());
    mock::advance_us(1000U);
}

} // namespace

int main()
{
    mock::reset();
    g_publisher = topics::safety::stop.claim_publisher();
    assert(g_publisher);

    std::array<Joint, 2> joints{};
    for (Joint &joint : joints)
    {
        JointLink::Binding binding{};
        binding.modes = JointLink::TORQUE_ONLY;
        binding.motor_maximum_torque_newton_meter = 1.0F;
        assert(JointLink::bind(joint, binding));
        JointLink::set_available(joint, true);
    }
    const std::array<Joint *, 2> group{&joints[0], &joints[1]};

    SafetyGate gate;
    assert(!gate.init(SafetyGate::Config{nullptr, 2U, {TIMEOUT_US}}));
    assert(!gate.init(SafetyGate::Config{group.data(), 2U, {0U}}));
    const std::array<Joint *, 1> broken{nullptr};
    assert(!gate.init(SafetyGate::Config{broken.data(), 1U, {TIMEOUT_US}}));
    assert(gate.init(SafetyGate::Config{group.data(), group.size(), {TIMEOUT_US}}));

    // 上电锁住：没有消息、或有消息但还没确认，都不能出力
    assert(gate.report().state == State::LOCKED && joints[0].inhibited() && !joints[0].enable());
    step(gate);
    publish(false, 7U);
    step(gate);
    assert(gate.report().state == State::LOCKED && !joints[1].enable());

    // 操作者确认（计数变化）：放开，能力可以使能
    publish(false, 8U);
    step(gate);
    assert(gate.report().state == State::RELEASED && gate.report().release_count == 1U);
    assert(!joints[0].inhibited() && joints[0].enable() && joints[0].set_torque(0.5F));
    assert(device::enable_all(group.data(), group.size()));

    // 急停：立即锁住，关节的出力请求被撤销
    publish(true, 8U);
    step(gate);
    assert(gate.report().state == State::LOCKED && gate.report().lock_count == 1U);
    assert(!JointLink::enable_requested(joints[0]) && !joints[0].set_torque(0.5F) && !joints[0].enable());
    // 急停解除但没有再确认：仍锁住
    publish(false, 8U);
    step(gate);
    assert(gate.report().state == State::LOCKED);
    publish(false, 9U);
    step(gate);
    assert(gate.report().state == State::RELEASED && joints[0].enable());

    // 发布者挂了：超时后锁住；恢复发布后同样要重新确认
    for (hal::os::Microseconds elapsed = 0U; elapsed < TIMEOUT_US; elapsed += 1000U)
    {
        step(gate);
    }
    assert(gate.report().state == State::LOCKED && gate.report().timeout_count == 1U && gate.report().lock_count == 2U);
    assert(!JointLink::enable_requested(joints[0]));
    publish(false, 9U);
    step(gate);
    step(gate);
    assert(gate.report().state == State::LOCKED);
    publish(false, 10U);
    step(gate);
    assert(gate.report().state == State::RELEASED && gate.report().release_count == 3U);

    // 放开时持续发布（不超时）：保持放开
    for (int round = 0; round < 100; ++round)
    {
        publish(false, 10U);
        step(gate);
    }
    assert(gate.report().state == State::RELEASED && joints[1].enable());
    return 0;
}
