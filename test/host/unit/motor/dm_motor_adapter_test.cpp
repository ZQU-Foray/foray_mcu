// 达妙电机接入电机组契约（DmMotorAdapter + MotorGroup<DM_J4310, N>）在 platform/mock 上（Decision-20261102-01）：
// 方向、角度与转速换算、全组使能、故障映射、总线占用。电机由 dm_fake_motor.hpp 扮演。
#include "driver/motor/dm/dm_motor_model_map.hpp"
#include "driver/motor/motor_group.hpp"
#include "unit/motor/dm_fake_motor.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace
{

using device::DmMotor;
using device::MotorGroup;
using device::MotorModel;
using device::MotorState;
using dm_test::Bench;
using dm_test::cycle;
using dm_test::Dm;
using dm_test::FakeMotor;
using Gimbal = MotorGroup<MotorModel::DM_J4310, 2>;
static_assert(Gimbal::count() == 2U);
static_assert(!std::is_copy_constructible_v<Gimbal> && !std::is_move_constructible_v<Gimbal>);

constexpr std::uint8_t BUS{0U};
constexpr std::uint8_t OTHER_BUS{1U};
constexpr std::size_t VERIFY_ROUNDS{8U};

void reset()
{
    mock::reset();
    mock::can::reset();
    mock::set_now_us(1000U);
}

MotorState state_of(const Gimbal &group, std::size_t id)
{
    MotorState state{};
    assert(group.read_state(id, state));
    return state;
}

bool near(float actual, float expected, float tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

/** MIT 帧里的力矩（12 位，±TMAX 10 N·m）。 */
float mit_torque(const hal::Can::Frame &frame)
{
    const auto raw = static_cast<std::uint16_t>(((frame.data[6] & 0x0FU) << 8U) | frame.data[7]);
    return Dm::from_unsigned(raw, -10.0F, 10.0F, 12U);
}

void test_group()
{
    reset();
    FakeMotor yaw{BUS, 1U, 0x11U};
    FakeMotor pitch{BUS, 2U, 0x12U};
    Bench bench{BUS};
    bench.add(yaw);
    bench.add(pitch);
    Gimbal gimbal{hal::Can{BUS}, {{{1U, 1, 0x11U}, {2U, -1, 0x12U}}}};
    assert(gimbal.maximum_torque_newton_meter() == 0.0F && !gimbal.enable());
    assert(gimbal.init() && gimbal.init());
    assert(near(gimbal.maximum_torque_newton_meter(), 10.0F, 0.0F));

    // 核对完成前：能读状态但无效，不能使能
    MotorState state = state_of(gimbal, 0U);
    assert(!state.feedback_valid && !state.online && !state.enabled && !state.fault);
    assert(!gimbal.enable() && !gimbal.set_torque(0U, 1.0F));
    cycle(gimbal, bench, VERIFY_ROUNDS);
    state = state_of(gimbal, 1U);
    assert(state.feedback_valid && state.online && !state.enabled && !state.fault && state.angle_degrees == 0.0F);
    assert(!gimbal.set_torque(0U, 1.0F)); // 还没有 enable()

    assert(gimbal.enable());
    assert(gimbal.set_torque(0U, 2.5F) && gimbal.set_torque(1U, 2.5F));
    cycle(gimbal, bench, 3U);
    assert(state_of(gimbal, 0U).enabled && state_of(gimbal, 1U).enabled);
    assert(near(mit_torque(yaw.last_mit), 2.5F, 0.01F));
    assert(near(mit_torque(pitch.last_mit), -2.5F, 0.01F)); // 方向 −1 作用于力矩
    assert(yaw.last_mit.identifier == 0x001U && pitch.last_mit.identifier == 0x002U);

    // 角度从核对后的第一帧反馈起算；方向作用于角度与转速
    yaw.position = static_cast<std::uint16_t>(32767U + 2621U); // 约 +1 rad
    pitch.position = static_cast<std::uint16_t>(32767U + 2621U);
    yaw.velocity = 0x732U; // −3 rad/s
    pitch.velocity = 0x732U;
    cycle(gimbal, bench, 2U);
    const MotorState yaw_state = state_of(gimbal, 0U);
    const MotorState pitch_state = state_of(gimbal, 1U);
    const float degrees = 2621.0F * 25.0F / 65535.0F * 180.0F / 3.14159265F;
    assert(near(yaw_state.angle_degrees, degrees, 0.01F) && near(pitch_state.angle_degrees, -degrees, 0.01F));
    assert(near(yaw_state.speed_rpm, -3.0F * 60.0F / (2.0F * 3.14159265F), 0.2F));
    assert(near(pitch_state.speed_rpm, 3.0F * 60.0F / (2.0F * 3.14159265F), 0.2F));
    assert(yaw_state.last_update_us != 0U);

    // 非有限力矩：清零该电机的指令
    assert(!gimbal.set_torque(0U, std::numeric_limits<float>::quiet_NaN()));
    cycle(gimbal, bench);
    assert(near(mit_torque(yaw.last_mit), 0.0F, 0.01F) && near(mit_torque(pitch.last_mit), -2.5F, 0.01F));
    assert(!gimbal.set_torque(2U, 1.0F));
    assert(!gimbal.read_state(2U, state) && !state.feedback_valid);
    gimbal.clear_commands();
    cycle(gimbal, bench);
    assert(near(mit_torque(pitch.last_mit), 0.0F, 0.01F) && state_of(gimbal, 1U).enabled);

    // 一台报故障：只有它停下，另一台照常；能力看到 fault 后决定怎么办
    yaw.state = Dm::State::COIL_OVER_TEMPERATURE;
    cycle(gimbal, bench, 2U);
    assert(state_of(gimbal, 0U).fault && !state_of(gimbal, 0U).enabled);
    assert(!state_of(gimbal, 1U).fault && state_of(gimbal, 1U).enabled);
    assert(!gimbal.set_torque(0U, 1.0F) && gimbal.set_torque(1U, 1.0F));
    // 再 enable()：已在出力的不受影响，故障的那台先清错再使能
    assert(gimbal.enable());
    cycle(gimbal, bench, 3U);
    assert(state_of(gimbal, 0U).enabled && !state_of(gimbal, 0U).fault && state_of(gimbal, 1U).enabled);

    // disable()：全组失能
    gimbal.disable();
    assert(!gimbal.set_torque(1U, 1.0F));
    cycle(gimbal, bench, 2U);
    assert(!state_of(gimbal, 0U).enabled && !state_of(gimbal, 1U).enabled);
    assert(yaw.state == Dm::State::DISABLED && pitch.state == Dm::State::DISABLED);

    // 同一条总线的接收只能被占用一次
    Gimbal conflict{hal::Can{BUS}, {{{3U, 1, 0x13U}, {4U, 1, 0x14U}}}};
    assert(!conflict.init() && !conflict.enable());
}

void test_enable_is_all_or_nothing()
{
    reset();
    FakeMotor yaw{OTHER_BUS, 1U, 0x11U};
    FakeMotor pitch{OTHER_BUS, 2U, 0x12U};
    pitch.torque_max = 7.0F; // 调试助手里被改过量程
    Bench bench{OTHER_BUS};
    bench.add(yaw);
    bench.add(pitch);
    Gimbal gimbal{hal::Can{OTHER_BUS}, {{{1U, 1, 0x11U}, {2U, 1, 0x12U}}}};
    assert(gimbal.init());
    cycle(gimbal, bench, VERIFY_ROUNDS);
    assert(!state_of(gimbal, 0U).fault && state_of(gimbal, 1U).fault);
    assert(!state_of(gimbal, 1U).feedback_valid && !state_of(gimbal, 1U).online);
    assert(!gimbal.enable()); // 一台核对不符：一台都不使能
    cycle(gimbal, bench, 5U);
    assert(!state_of(gimbal, 0U).enabled && yaw.state == Dm::State::DISABLED);
}

void test_adapter_config()
{
    reset();
    using Adapter = device::DmMotorAdapter;
    Adapter::Config config{hal::Can{BUS}, DmMotor::Profile::j4310(), {}, 1U};
    config.mappings[0] = {1U, 0x11U, 0};
    Adapter zero_direction{config};
    assert(!zero_direction.init());
    config.mappings[0].direction = 2;
    Adapter bad_direction{config};
    assert(!bad_direction.init());
    config.mappings[0] = {1U, 0x01U, 1}; // MST_ID 与 ESC_ID 冲突：驱动拒绝
    Adapter bad_master{config};
    assert(!bad_master.init());
    config.count = 0U;
    Adapter empty{config};
    assert(!empty.init());
    // 以上都没有占住总线
    config = {hal::Can{BUS}, DmMotor::Profile::j4310(), {}, 1U};
    config.mappings[0] = {1U, 0x11U, -1};
    Adapter valid{config};
    assert(valid.init());
    DmMotor::Snapshot snapshot{};
    assert(valid.get_snapshot(0U, snapshot) && snapshot.phase == DmMotor::Phase::VERIFYING);
    assert(!valid.get_snapshot(1U, snapshot));
}

} // namespace

int main()
{
    test_group();
    test_enable_is_all_or_nothing();
    test_adapter_config();
    return 0;
}
