// gimbal::Gimbal 在 platform/mock 上：两台达妙电机（FakeMotor）、真的 CanBus、DmMotor、SafetyGate，
// 假的 imu.attitude、safety.stop 与 pc_link.control 发布者。
// 每轮顺序与组合根一致：bus.receive() → gimbal.step() → gate.step() → bus.transmit() → bench.respond() → 时间 +1 ms。
// 每个 topic 只能有一个发布者，场景按参数分进程运行（与 imu、remote 相同）。
#include "actuator/safety_gate.hpp"
#include "driver/can_bus/can_bus.hpp"
#include "driver/motor/dm/dm_models.hpp"
#include "driver/motor/dm/dm_motor.hpp"
#include "gimbal/gimbal.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"
#include "topics/imu.hpp"
#include "topics/pc_link.hpp"
#include "topics/safety.hpp"
#include "unit/motor/dm_fake_motor.hpp"
#include "utils/math/quaternion.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{

using dm_test::Bench;
using dm_test::Dm;
using dm_test::FakeMotor;

constexpr std::uint8_t BUS{0U};
constexpr std::uint16_t YAW_ESC{1U};
constexpr std::uint16_t YAW_MASTER{0x11U};
constexpr std::uint16_t PITCH_ESC{2U};
constexpr std::uint16_t PITCH_MASTER{0x12U};
constexpr hal::os::Microseconds ATTITUDE_TIMEOUT{10000U};
constexpr hal::os::Microseconds ENABLE_TIMEOUT{200000U};
constexpr hal::os::Microseconds TARGET_TIMEOUT{100000U};
constexpr hal::os::Microseconds STOP_TIMEOUT{50000U};
constexpr std::uint16_t GIMBAL_FLAG{0x0002U};

// 测试参数：pitch 限位 ±0.5 rad，yaw 限位 ±1.5 rad；增益取仿真里能收敛的一组
constexpr gimbal::Gimbal::Parameters PARAMETERS{
    gimbal::Gimbal::AxisParameters{gimbal::Gimbal::LoopGains{30.0F, 0.0F, 0.0F, 6.0F},
                                   gimbal::Gimbal::LoopGains{2.0F, 10.0F, 1.0F, 5.0F},
                                   true,
                                   -1.5F,
                                   1.5F},
    gimbal::Gimbal::AxisParameters{gimbal::Gimbal::LoopGains{30.0F, 0.0F, 0.0F, 6.0F},
                                   gimbal::Gimbal::LoopGains{2.0F, 10.0F, 1.0F, 5.0F},
                                   true,
                                   -0.5F,
                                   0.5F},
    0.0F, // 重力前馈
    5.0F, // 总限幅
    ATTITUDE_TIMEOUT,
    ENABLE_TIMEOUT,
    TARGET_TIMEOUT,
};

device::DmMotor::Config motor_config(device::Joint &yaw, device::Joint &pitch)
{
    device::DmMotor::Config config{};
    config.count = 2U;
    config.motors[0] =
        device::DmMotor::Motor{&yaw, device::dm::DM_J4310, YAW_ESC, YAW_MASTER, device::JointMechanics{}};
    config.motors[1] =
        device::DmMotor::Motor{&pitch, device::dm::DM_J4310, PITCH_ESC, PITCH_MASTER, device::JointMechanics{}};
    return config;
}

bool near(float actual, float expected, float tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

/** 一轮里发出的帧：有没有"使能"系统命令、有没有 MIT 力矩帧。 */
struct RoundFrames
{
    bool enable{false};
    bool system{false};
    bool mit{false};
    float yaw_torque{0.0F};
    float pitch_torque{0.0F};
};

RoundFrames analyze(const Bench &bench)
{
    RoundFrames round{};
    for (std::size_t index = 0U; index < bench.sent_count(); ++index)
    {
        const hal::Can::Frame &frame = bench.sent(index);
        if (dm_test::is_system_command(frame, Dm::SystemCommand::ENABLE))
        {
            round.enable = true;
            round.system = true;
            continue;
        }
        if (dm_test::is_system_command(frame, Dm::SystemCommand::DISABLE) ||
            dm_test::is_system_command(frame, Dm::SystemCommand::CLEAR_ERROR))
        {
            round.system = true;
            continue;
        }
        if (frame.length != 8U)
        {
            continue;
        }
        round.mit = true;
        // MIT 帧：D[5..7] 里是 Kd 的低 8 位与力矩
        const auto torque_raw = static_cast<std::uint16_t>(((frame.data[6] & 0x0FU) << 8U) | frame.data[7]);
        const float torque = Dm::from_unsigned(torque_raw, -10.0F, 10.0F, 12U);
        if ((frame.identifier & 0xFFU) == YAW_ESC)
        {
            round.yaw_torque = torque;
        }
        else if ((frame.identifier & 0xFFU) == PITCH_ESC)
        {
            round.pitch_torque = torque;
        }
    }
    return round;
}

/** 三个 topic 的发布句柄：一个进程里每个 topic 只能取一次，main() 取好后交给场景。 */
struct Publishers
{
    topic::Publisher<topics::imu::Attitude> attitude;
    topic::Publisher<topics::safety::Stop> stop;
    topic::Publisher<topics::pc_link::Control> control;
};

class Rig final
{
  public:
    Rig(topic::Publisher<topics::imu::Attitude> &attitude,
        topic::Publisher<topics::safety::Stop> &stop,
        topic::Publisher<topics::pc_link::Control> &control) noexcept
        : driver_{bus_, motor_config(yaw_, pitch_)}, attitude_{attitude}, stop_{stop}, control_{control}
    {
    }

    bool init()
    {
        mock::reset();
        mock::can::reset();
        mock::uart::reset();
        mock::set_now_us(1000U);
        bench_.add(yaw_motor_);
        bench_.add(pitch_motor_);
        if (!bus_.init() || !driver_.init())
        {
            return false;
        }
        if (!gate_.init(actuator::SafetyGate::Config{joints_.data(), joints_.size(), {STOP_TIMEOUT}}))
        {
            return false;
        }
        return gimbal_.init(gimbal::Gimbal::Config{&yaw_, &pitch_, PARAMETERS});
    }

    void publish_attitude(float yaw, float pitch, topics::imu::State state, bool valid)
    {
        const math::Quaternion quaternion = math::from_euler_zyx_radians(math::Vector3{yaw, pitch, 0.0F});
        topics::imu::Attitude message{};
        message.quaternion = {quaternion.w, quaternion.x, quaternion.y, quaternion.z};
        message.gyro_rad_per_sec = {0.0F, 0.0F, 0.0F};
        message.timestamp_us = hal::os::now_us();
        message.state = state;
        message.valid = valid;
        attitude_.publish(message);
    }

    void publish_stop(bool stop)
    {
        topics::safety::Stop message{};
        message.stop = stop;
        message.arm_sequence = arm_sequence_;
        message.timestamp_us = hal::os::now_us();
        stop_.publish(message);
    }

    void bump_arm()
    {
        ++arm_sequence_;
    }

    void publish_control(float yaw, float pitch, float yaw_rate = 0.0F, float pitch_rate = 0.0F)
    {
        topics::pc_link::Control message{};
        message.control_seq = ++control_sequence_;
        message.source = 2U;
        message.mode = 1U;
        message.flags = GIMBAL_FLAG;
        message.gimbal_yaw = yaw;
        message.gimbal_pitch = pitch;
        message.gimbal_yaw_rate = yaw_rate;
        message.gimbal_pitch_rate = pitch_rate;
        message.validity_hint_ms = 0U;
        message.received_us = hal::os::now_us();
        control_.publish(message);
    }

    void tick(hal::os::Microseconds delta_us = 1000U)
    {
        const hal::os::Microseconds now = hal::os::now_us();
        bus_.receive();
        gimbal_.step(now);
        gate_.step(now);
        bus_.transmit(now);
        bench_.respond();
        round_ = analyze(bench_);
        mock::advance_us(delta_us);
    }

    /** 每轮发布姿态与"不急停"，让安全门一直放开、姿态一直新鲜。 */
    void tick_alive(float yaw = 0.0F, float pitch = 0.0F, topics::imu::State state = topics::imu::State::READY)
    {
        publish_attitude(yaw, pitch, state, true);
        publish_stop(false);
        tick();
    }

    /** 跑到两台电机核对通过（READY）。 */
    bool run_until_ready()
    {
        for (std::size_t round = 0U; round < 200U; ++round)
        {
            if (motor_ready())
            {
                return true;
            }
            tick_alive();
        }
        return motor_ready();
    }

    /** 先让安全门看到两帧：第一帧记下确认计数，第二帧计数变了才放开。 */
    void release_gate()
    {
        publish_stop(false);
        tick();
        bump_arm();
        publish_stop(false);
        tick();
        assert(gate_.report().state == actuator::SafetyGate::State::RELEASED);
    }

    /** 等使能确认、进入 HOLD。 */
    bool run_until_holding(float yaw = 0.0F, float pitch = 0.0F)
    {
        for (std::size_t round = 0U; round < 200U; ++round)
        {
            if (gimbal_.report().mode == gimbal::Gimbal::Mode::HOLD)
            {
                return true;
            }
            tick_alive(yaw, pitch);
        }
        return gimbal_.report().mode == gimbal::Gimbal::Mode::HOLD;
    }

    bool motor_ready() const
    {
        device::DmMotor::Snapshot yaw{};
        device::DmMotor::Snapshot pitch{};
        return driver_.get_snapshot(0U, yaw) && driver_.get_snapshot(1U, pitch) &&
               yaw.phase == device::DmMotor::Phase::READY && pitch.phase == device::DmMotor::Phase::READY;
    }

    FakeMotor &yaw_motor() noexcept
    {
        return yaw_motor_;
    }
    FakeMotor &pitch_motor() noexcept
    {
        return pitch_motor_;
    }
    device::Joint &yaw() noexcept
    {
        return yaw_;
    }
    device::Joint &pitch() noexcept
    {
        return pitch_;
    }
    actuator::SafetyGate &gate() noexcept
    {
        return gate_;
    }
    gimbal::Gimbal &gimbal() noexcept
    {
        return gimbal_;
    }
    const RoundFrames &round() const noexcept
    {
        return round_;
    }

  private:
    FakeMotor yaw_motor_{BUS, YAW_ESC, YAW_MASTER};
    FakeMotor pitch_motor_{BUS, PITCH_ESC, PITCH_MASTER};
    Bench bench_{BUS};
    device::CanBus bus_{hal::Can{BUS}};
    device::Joint yaw_{};
    device::Joint pitch_{};
    device::DmMotor driver_;
    actuator::SafetyGate gate_{};
    gimbal::Gimbal gimbal_{};
    std::array<device::Joint *, 2> joints_{&yaw_, &pitch_};
    topic::Publisher<topics::imu::Attitude> &attitude_;
    topic::Publisher<topics::safety::Stop> &stop_;
    topic::Publisher<topics::pc_link::Control> &control_;
    std::uint32_t arm_sequence_{0U};
    std::uint32_t control_sequence_{0U};
    RoundFrames round_{};
};

// IMU 在 SETTLING：两台电机只收到"失能"，没有使能、没有 MIT 力矩
int scenario_settling(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    for (std::size_t round = 0U; round < 50U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::SETTLING, true);
        rig.publish_stop(false);
        rig.tick();
        assert(!rig.round().enable && !rig.round().mit);
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::DISABLED);
    assert(rig.gimbal().report().last_disable_reason == gimbal::Gimbal::DisableReason::IMU_SETTLING);
    assert(!rig.yaw_motor().has_mit && !rig.pitch_motor().has_mit);
    return 0;
}

// READY，但安全门锁住：同样不出力
int scenario_locked(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());
    for (std::size_t round = 0U; round < 50U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(true); // 一直是急停
        rig.tick();
        assert(!rig.round().enable && !rig.round().mit);
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::DISABLED);
    assert(rig.gimbal().report().last_disable_reason == gimbal::Gimbal::DisableReason::JOINT_UNAVAILABLE);
    return 0;
}

// 安全门放开：两台都使能，之后每轮都有 MIT 力矩帧；姿态与目标一致时力矩约等于 0（不甩）
int scenario_enable(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());
    assert(rig.yaw().state().enabled && rig.pitch().state().enabled);
    for (std::size_t round = 0U; round < 50U; ++round)
    {
        rig.tick_alive();
        assert(rig.round().mit && !rig.round().enable && !rig.round().system);
        // 12 位量化：±10 N·m 的一格约 0.0049 N·m，0 力矩的帧解出来是 −0.0024
        assert(near(rig.round().yaw_torque, 0.0F, 5.0e-3F));
        assert(near(rig.round().pitch_torque, 0.0F, 5.0e-3F));
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::HOLD);
    return 0;
}

// PC 目标有效：TRACK；yaw 比目标偏右时 yaw 力矩为正；yaw 的误差回绕（不会绕远路）
int scenario_track(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    // 测量 0、目标 +0.1：yaw 力矩为正
    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_control(0.1F, 0.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    assert(rig.gimbal().report().target_valid);
    assert(rig.round().yaw_torque > 0.0F);
    assert(near(rig.gimbal().report().yaw_target_rad, 0.1F, 1.0e-4F));

    // 测量 3.1、目标 −3.1：回绕之后目标在测量值附近，误差只有 0.083 rad，力矩仍为正
    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(3.1F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_control(-3.1F, 0.0F);
        rig.tick();
    }
    const gimbal::Gimbal::Report &report = rig.gimbal().report();
    assert(std::fabs(report.yaw_target_rad - report.yaw_rad) <= 3.14159265F); // 不绕远路
    assert(rig.round().yaw_torque > 0.0F);
    return 0;
}

// 急停：当轮两台都撤销出力，之后回到 DISABLED
int scenario_estop(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
    rig.bump_arm();
    rig.publish_stop(true);
    rig.tick();
    assert(rig.yaw().inhibited() && rig.pitch().inhibited());
    assert(!rig.round().mit); // 这一轮就不再有力矩帧
    rig.tick_alive();
    assert(!rig.yaw().state().enabled && !rig.pitch().state().enabled);
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::DISABLED);
    return 0;
}

// 一台电机报故障：两台都撤销，回 DISABLED
int scenario_fault(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    rig.pitch_motor().state = Dm::State::COIL_OVER_TEMPERATURE;
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.tick_alive();
    }
    assert(rig.pitch().state().fault);
    assert(!rig.yaw().state().enabled && !rig.pitch().state().enabled);
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::DISABLED);
    assert(rig.gimbal().report().last_disable_reason == gimbal::Gimbal::DisableReason::JOINT_FAULT);
    return 0;
}

// 姿态停发超过超时：回 DISABLED
int scenario_attitude_timeout(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    for (std::size_t round = 0U; round < 20U; ++round)
    {
        rig.publish_stop(false); // 安全门不放，只是姿态不发了
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::DISABLED);
    assert(rig.gimbal().report().last_disable_reason == gimbal::Gimbal::DisableReason::IMU_STALE);
    assert(!rig.yaw().state().enabled && !rig.pitch().state().enabled);
    return 0;
}

// PC 目标有效 → 过期：TRACK → HOLD，目标保持最后一个
int scenario_target_expiry(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_control(0.2F, 0.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    assert(near(rig.gimbal().report().yaw_target_rad, 0.2F, 1.0e-4F));

    // 不再发目标，等过默认有效期（100 ms）
    for (std::size_t round = 0U; round < 120U; ++round)
    {
        rig.tick_alive();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::HOLD);
    assert(!rig.gimbal().report().target_valid);
    assert(near(rig.gimbal().report().yaw_target_rad, 0.2F, 1.0e-4F)); // 目标保持最后一个
    return 0;
}

// 有限位的 yaw、底座倾斜（偏移）不为 0：目标先在关节角里回绕再钳位，不会跑到另一头的限位
int scenario_yaw_limit(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    // 底座倾斜 2.5 rad（世界系 2.5、关节约 0），小 yaw 限位 ±1.5：允许的世界系范围约 [1.0, 4.0]。
    // PC 给 −3.0，与 3.283 等价：对应的关节角约 0.78，在限位内，云台应该往正方向转一点。
    // 先钳世界系目标再回绕的写法会把它钳到 1.0（关节角 −1.5），方向相反。
    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(2.5F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_control(-3.0F, 0.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    const float target = rig.gimbal().report().yaw_target_rad;
    assert(near(target, 3.2832F, 0.01F));
    assert(target > rig.gimbal().report().yaw_rad);
    assert(rig.round().yaw_torque > 0.0F);
    // 关节目标落在限位内（tilt 用「世界系 − 关节角」估算）
    const float tilt = rig.gimbal().report().yaw_rad - rig.yaw().state().position_rad;
    assert(std::fabs(target - tilt) <= 1.5F + 0.01F);
    return 0;
}

// 参数检查：负的重力前馈合法（枪管一侧偏重），总限幅超过关节能力、时间参数为 0、限位颠倒才拒绝
int scenario_reject(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());

    gimbal::Gimbal probe;
    gimbal::Gimbal::Parameters parameters = PARAMETERS;
    parameters.pitch_gravity_newton_meter = -0.8F;
    assert(probe.init(gimbal::Gimbal::Config{&rig.yaw(), &rig.pitch(), parameters}));

    parameters = PARAMETERS;
    parameters.torque_limit_newton_meter = 11.0F; // 超过 J4310 的 10 N·m
    assert(!probe.init(gimbal::Gimbal::Config{&rig.yaw(), &rig.pitch(), parameters}));

    parameters = PARAMETERS;
    parameters.attitude_timeout_us = 0U;
    assert(!probe.init(gimbal::Gimbal::Config{&rig.yaw(), &rig.pitch(), parameters}));

    parameters = PARAMETERS;
    parameters.yaw.minimum_rad = 1.0F;
    parameters.yaw.maximum_rad = -1.0F;
    assert(!probe.init(gimbal::Gimbal::Config{&rig.yaw(), &rig.pitch(), parameters}));

    parameters = PARAMETERS;
    assert(!probe.init(gimbal::Gimbal::Config{nullptr, &rig.pitch(), parameters}));
    return 0;
}

// PC 目标的 pitch 超出限位：目标被钳在限位内
int scenario_pitch_limit(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.control};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_control(0.0F, 2.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    assert(near(rig.gimbal().report().pitch_target_rad, PARAMETERS.pitch.maximum_rad, 0.01F));

    // 目标在测量值下方 0.2 rad（在限位内）：pitch 力矩为正（低头为正）
    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_control(0.0F, 0.2F);
        rig.tick();
    }
    assert(rig.round().pitch_torque > 0.0F);
    return 0;
}

} // namespace

int main(int argc, char **argv)
{
    Publishers publishers{topics::imu::attitude.claim_publisher(),
                          topics::safety::stop.claim_publisher(),
                          topics::pc_link::control.claim_publisher()};
    assert(publishers.attitude && publishers.stop && publishers.control);
    if (argc != 2)
    {
        std::fprintf(stderr,
                     "用法：%s settling|locked|enable|track|estop|fault|attitude_timeout|target_expiry|pitch_limit\n",
                     argv[0]);
        return 2;
    }
    const char *scenario = argv[1];
    if (std::strcmp(scenario, "settling") == 0)
    {
        return scenario_settling(publishers);
    }
    if (std::strcmp(scenario, "locked") == 0)
    {
        return scenario_locked(publishers);
    }
    if (std::strcmp(scenario, "enable") == 0)
    {
        return scenario_enable(publishers);
    }
    if (std::strcmp(scenario, "track") == 0)
    {
        return scenario_track(publishers);
    }
    if (std::strcmp(scenario, "estop") == 0)
    {
        return scenario_estop(publishers);
    }
    if (std::strcmp(scenario, "fault") == 0)
    {
        return scenario_fault(publishers);
    }
    if (std::strcmp(scenario, "attitude_timeout") == 0)
    {
        return scenario_attitude_timeout(publishers);
    }
    if (std::strcmp(scenario, "target_expiry") == 0)
    {
        return scenario_target_expiry(publishers);
    }
    if (std::strcmp(scenario, "pitch_limit") == 0)
    {
        return scenario_pitch_limit(publishers);
    }
    if (std::strcmp(scenario, "yaw_limit") == 0)
    {
        return scenario_yaw_limit(publishers);
    }
    if (std::strcmp(scenario, "reject") == 0)
    {
        return scenario_reject(publishers);
    }
    std::fprintf(stderr, "未知场景：%s\n", scenario);
    return 2;
}
