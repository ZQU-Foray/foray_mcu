// gimbal::Gimbal 在 platform/mock 上：两台达妙电机（FakeMotor）、真的 CanBus、DmMotor、SafetyGate，
// 假的 imu.attitude、safety.stop 与 command.gimbal 发布者。
// 每轮顺序与组合根一致：bus.receive() → gimbal.step() → gate.step() → bus.transmit() → bench.respond() → 时间 +1 ms。
// 每个 topic 只能有一个发布者，场景按参数分进程运行（与 imu、remote 相同）。
#include "actuator/safety_gate.hpp"
#include "driver/can_bus/can_bus.hpp"
#include "driver/motor/dm/dm_models.hpp"
#include "driver/motor/dm/dm_motor.hpp"
#include "gimbal/gimbal.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"
#include "topics/command.hpp"
#include "topics/gimbal.hpp"
#include "topics/imu.hpp"
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
#include <limits>

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

// 测试参数：pitch 限位 ±0.5 rad，yaw 限位 ±3.0 rad；增益取仿真里能收敛的一组
constexpr gimbal::Gimbal::Parameters PARAMETERS{
    gimbal::Gimbal::AxisParameters{gimbal::AxisLoop::Parameters{gimbal::AxisLoop::Gains{30.0F, 0.0F, 0.0F, 6.0F},
                                                                gimbal::AxisLoop::Gains{2.0F, 10.0F, 1.0F, 5.0F}},
                                   true,
                                   -3.0F,
                                   3.0F,
                                   1000.0F},
    gimbal::Gimbal::AxisParameters{gimbal::AxisLoop::Parameters{gimbal::AxisLoop::Gains{30.0F, 0.0F, 0.0F, 6.0F},
                                                                gimbal::AxisLoop::Gains{2.0F, 10.0F, 1.0F, 5.0F}},
                                   true,
                                   -0.5F,
                                   0.5F,
                                   1000.0F},
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
    topic::Publisher<topics::command::Gimbal> command;
};

class Rig final
{
  public:
    Rig(topic::Publisher<topics::imu::Attitude> &attitude,
        topic::Publisher<topics::safety::Stop> &stop,
        topic::Publisher<topics::command::Gimbal> &command,
        const gimbal::Gimbal::Parameters &parameters = PARAMETERS,
        bool create_gimbal = true,
        gimbal::Gimbal::CaptureSample *capture_buffer = nullptr,
        std::size_t capture_capacity = 0U) noexcept
        : driver_{bus_, motor_config(yaw_, pitch_)}, parameters_{parameters}, create_gimbal_{create_gimbal},
          capture_buffer_{capture_buffer}, capture_capacity_{capture_capacity}, attitude_{attitude}, stop_{stop},
          command_{command}
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
        return !create_gimbal_ ||
               gimbal_.init(gimbal::Gimbal::Config{&yaw_, &pitch_, parameters_, capture_buffer_, capture_capacity_});
    }

    void publish_attitude(
        float yaw, float pitch, topics::imu::State state, bool valid, hal::os::Microseconds stamp_offset_us = 0U)
    {
        const math::Quaternion quaternion = math::from_euler_zyx_radians(math::Vector3{yaw, pitch, 0.0F});
        topics::imu::Attitude message{};
        message.quaternion = {quaternion.w, quaternion.x, quaternion.y, quaternion.z};
        message.gyro_rad_per_sec = {0.0F, 0.0F, 0.0F};
        message.timestamp_us = hal::os::now_us() + stamp_offset_us;
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

    /** 发布一份云台指令。默认 ABSOLUTE（自瞄），HOLD 时其余字段忽略。 */
    void publish_command(topics::command::GimbalMode mode,
                         float yaw = 0.0F,
                         float pitch = 0.0F,
                         float yaw_rate = 0.0F,
                         float pitch_rate = 0.0F)
    {
        topics::command::Gimbal message{};
        message.mode = mode;
        message.yaw_rad = yaw;
        message.pitch_rad = pitch;
        message.yaw_rate = yaw_rate;
        message.pitch_rate = pitch_rate;
        message.timestamp_us = hal::os::now_us();
        command_.publish(message);
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
    void release_gate(float yaw = 0.0F, float pitch = 0.0F)
    {
        publish_attitude(yaw, pitch, topics::imu::State::READY, true);
        publish_stop(false);
        tick();
        bump_arm();
        publish_attitude(yaw, pitch, topics::imu::State::READY, true);
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
    gimbal::Gimbal::Parameters parameters_{};
    bool create_gimbal_{true};
    gimbal::Gimbal::CaptureSample *capture_buffer_{nullptr};
    std::size_t capture_capacity_{0U};
    actuator::SafetyGate gate_{};
    gimbal::Gimbal gimbal_{};
    std::array<device::Joint *, 2> joints_{&yaw_, &pitch_};
    topic::Publisher<topics::imu::Attitude> &attitude_;
    topic::Publisher<topics::safety::Stop> &stop_;
    topic::Publisher<topics::command::Gimbal> &command_;
    std::uint32_t arm_sequence_{0U};
    RoundFrames round_{};
};

// IMU 在 SETTLING：两台电机只收到"失能"，没有使能、没有 MIT 力矩
int scenario_settling(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
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
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
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
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
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
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    // 测量 0、目标 +0.1：yaw 力矩为正
    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, 0.1F, 0.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    assert(rig.gimbal().report().command == topics::command::GimbalMode::ABSOLUTE);
    assert(rig.round().yaw_torque > 0.0F);
    assert(near(rig.gimbal().report().yaw_target_rad, 0.1F, 1.0e-4F));

    // 测量 3.1、目标 −3.1：回绕之后目标在测量值附近，误差只有 0.083 rad，力矩仍为正
    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(3.1F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, -3.1F, 0.0F);
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
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
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
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
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
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
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

// 指令超时：TRACK → HOLD，目标保持最后一个
int scenario_command_timeout(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, 0.2F, 0.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    assert(near(rig.gimbal().report().yaw_target_rad, 0.2F, 1.0e-4F));

    // 不再发指令，等过 command_timeout_us（测试参数 100 ms）
    for (std::size_t round = 0U; round < 120U; ++round)
    {
        rig.tick_alive();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::HOLD);
    assert(rig.gimbal().report().command == topics::command::GimbalMode::HOLD);
    assert(near(rig.gimbal().report().yaw_target_rad, 0.2F, 1.0e-4F)); // 目标保持最后一个
    return 0;
}

// 有限位的 yaw、底座倾斜（偏移）不为 0：目标先在关节角里回绕再钳位，不会跑到另一头的限位
int scenario_yaw_limit(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
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
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, -3.0F, 0.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    const float target = rig.gimbal().report().yaw_target_rad;
    assert(near(target, 3.2832F, 0.01F));
    assert(target > rig.gimbal().report().yaw_rad);
    assert(rig.round().yaw_torque > 0.0F);
    // 关节目标落在限位内（tilt 用「世界系 − 关节角」估算）
    const float tilt = rig.gimbal().report().yaw_rad - rig.yaw().state().position_rad;
    assert(target - tilt <= PARAMETERS.yaw.maximum_rad + 0.01F);
    assert(target - tilt >= PARAMETERS.yaw.minimum_rad - 0.01F);
    return 0;
}

// 手动：恒定角速度 1 s，目标走了大约「角速度 × 1 s」；回中后不再走
int scenario_manual_rate(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    const float start = rig.gimbal().report().yaw_target_rad;
    for (std::size_t round = 0U; round < 1000U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::RATE, 0.0F, 0.0F, 1.0F, 0.0F);
        rig.tick();
    }
    const gimbal::Gimbal::Report &report = rig.gimbal().report();
    assert(report.mode == gimbal::Gimbal::Mode::MANUAL);
    assert(near(report.yaw_target_rad - start, 1.0F, 0.02F)); // 1 rad/s × 1 s

    const float held = report.yaw_target_rad;
    for (std::size_t round = 0U; round < 20U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::RATE, 0.0F, 0.0F, 0.0F, 0.0F);
        rig.tick();
    }
    assert(near(rig.gimbal().report().yaw_target_rad, held, 1.0e-4F)); // 回中：不再走
    return 0;
}

// 手动往限位外推一秒，再反向：目标停在限位上，反向后立刻离开（没有积分饱和）
int scenario_manual_clamp(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    for (std::size_t round = 0U; round < 1000U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::RATE, 0.0F, 0.0F, 6.0F, 0.0F);
        rig.tick();
    }
    const float clamped = rig.gimbal().report().yaw_target_rad;
    assert(near(clamped, PARAMETERS.yaw.maximum_rad, 0.01F)); // 目标停在限位上

    rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
    rig.publish_stop(false);
    rig.publish_command(topics::command::GimbalMode::RATE, 0.0F, 0.0F, -1.0F, 0.0F);
    rig.tick();
    // 写回了钳完的目标：一拍就离开限位，而不是先把限位外积的那段"退"完
    assert(rig.gimbal().report().yaw_target_rad < clamped - 0.0005F);
    return 0;
}

// 目标转动速度上限：ABSOLUTE 跳 0.5 rad，实际用的目标每拍最多走 maximum_target_rate × dt
int scenario_absolute_rate_limit(Publishers &publishers)
{
    gimbal::Gimbal::Parameters parameters = PARAMETERS;
    parameters.yaw.maximum_target_rate = 2.0F; // rad/s
    Rig rig{publishers.attitude, publishers.stop, publishers.command, parameters};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
    rig.publish_stop(false);
    rig.publish_command(topics::command::GimbalMode::ABSOLUTE, 0.5F, 0.0F);
    rig.tick();
    // 第一拍最多走 2 rad/s × 1 ms = 0.002 rad
    assert(rig.gimbal().report().yaw_target_rad > 0.0F);
    assert(rig.gimbal().report().yaw_target_rad < 0.003F);

    for (std::size_t round = 0U; round < 300U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, 0.5F, 0.0F);
        rig.tick();
    }
    assert(near(rig.gimbal().report().yaw_target_rad, 0.5F, 0.01F)); // 之后到位
    return 0;
}

// gimbal.state 与 Report 一致；模式是这一轮的指令模式
int scenario_state(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    topic::Subscriber<topics::gimbal::State> state{topics::gimbal::state};
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, 0.1F, 0.05F);
        rig.tick();
    }
    topics::gimbal::State message{};
    assert(state.update(message));
    const gimbal::Gimbal::Report &report = rig.gimbal().report();
    assert(message.mode == topics::command::GimbalMode::ABSOLUTE);
    assert(near(message.yaw_rad, report.yaw_rad, 1.0e-6F));
    assert(near(message.pitch_rad, report.pitch_rad, 1.0e-6F));
    assert(near(message.yaw_target_rad, report.yaw_target_rad, 1.0e-6F));
    assert(near(message.pitch_target_rad, report.pitch_target_rad, 1.0e-6F));
    assert(near(message.yaw_joint_rad, report.yaw_joint_rad, 1.0e-6F));
    return 0;
}

// MANUAL 中一台电机失能：两台都撤销，原因是 JOINT_DROPPED（不是等 set_torque 失败才撤）
int scenario_manual_drop(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    for (std::size_t round = 0U; round < 3U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::RATE, 0.0F, 0.0F, 1.0F, 0.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::MANUAL);

    // yaw 电机自己失能（例如电机复位）：在"刚掉"的那一拍看原因
    rig.yaw_motor().state = Dm::State::DISABLED;
    bool dropped = false;
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::RATE, 0.0F, 0.0F, 1.0F, 0.0F);
        rig.tick();
        if (rig.gimbal().report().mode == gimbal::Gimbal::Mode::DISABLED)
        {
            dropped = true;
            break;
        }
    }
    assert(dropped);
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::DISABLED);
    assert(rig.gimbal().report().last_disable_reason == gimbal::Gimbal::DisableReason::JOINT_DROPPED);
    assert(rig.gimbal().report().output_stop_reason == gimbal::Gimbal::DisableReason::JOINT_DROPPED);
    assert(!rig.yaw().state().enabled && !rig.pitch().state().enabled);

    // "掉出来那一刻"的原因单独保留：后面几拍驱动重新核对、关节变成不可用，
    // last_disable_reason 会换成"不可用"，但这个不换
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.tick_alive();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::DISABLED);
    assert(rig.gimbal().report().last_disable_reason == gimbal::Gimbal::DisableReason::JOINT_UNAVAILABLE);
    assert(rig.gimbal().report().output_stop_reason == gimbal::Gimbal::DisableReason::JOINT_DROPPED);
    return 0;
}

// 一台掉了、另一台同时报故障：先报故障（两台之间也按"故障优先"）
int scenario_fault_beats_drop(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    rig.yaw_motor().state = Dm::State::DISABLED;                // yaw 自己失能
    rig.pitch_motor().state = Dm::State::COIL_OVER_TEMPERATURE; // pitch 报故障
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.tick_alive();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::DISABLED);
    assert(rig.gimbal().report().last_disable_reason == gimbal::Gimbal::DisableReason::JOINT_FAULT);
    assert(!rig.yaw().state().enabled && !rig.pitch().state().enabled);
    return 0;
}

// 模式切换从当前目标接着走：ABSOLUTE 到 0.5 后切 RATE（角速度 0）与 HOLD，目标都保持 0.5，
// 不会跳回测量值（0）
int scenario_mode_switch(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, 0.5F, 0.0F);
        rig.tick();
    }
    assert(near(rig.gimbal().report().yaw_target_rad, 0.5F, 1.0e-3F));

    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::RATE, 0.0F, 0.0F, 0.0F, 0.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::MANUAL);
    assert(near(rig.gimbal().report().yaw_target_rad, 0.5F, 1.0e-3F));

    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::HOLD);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::HOLD);
    assert(near(rig.gimbal().report().yaw_target_rad, 0.5F, 1.0e-3F));
    return 0;
}

// 参数检查：负的重力前馈合法（枪管一侧偏重）、限位总宽正好 2π 合法；
// 总限幅超过关节能力、时间参数为 0、限位颠倒或超过 2π、目标速度上限非正、关节为空都拒绝
int scenario_reject(Publishers &publishers)
{
    // 不建 Rig 的云台：Gimbal::init 会占掉 gimbal.state 的发布句柄，一个进程只能成功一次
    Rig rig{publishers.attitude, publishers.stop, publishers.command, PARAMETERS, false};
    assert(rig.init());
    assert(rig.run_until_ready());

    gimbal::Gimbal probe;
    gimbal::Gimbal::Parameters parameters = PARAMETERS;
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
    parameters.yaw.minimum_rad = -3.14159265F;
    parameters.yaw.maximum_rad = 3.2F; // 总宽超过 2π
    assert(!probe.init(gimbal::Gimbal::Config{&rig.yaw(), &rig.pitch(), parameters}));

    parameters = PARAMETERS;
    parameters.pitch.maximum_target_rate = 0.0F;
    assert(!probe.init(gimbal::Gimbal::Config{&rig.yaw(), &rig.pitch(), parameters}));

    parameters = PARAMETERS;
    assert(!probe.init(gimbal::Gimbal::Config{nullptr, &rig.pitch(), parameters}));

    // 唯一的正例：负的重力前馈 + 限位总宽正好 2π
    parameters = PARAMETERS;
    parameters.pitch_gravity_newton_meter = -0.8F;
    parameters.yaw.minimum_rad = -3.14159265F;
    parameters.yaw.maximum_rad = 3.14159265F;
    assert(probe.init(gimbal::Gimbal::Config{&rig.yaw(), &rig.pitch(), parameters}));
    return 0;
}

// 限位总宽超过半圈：合法的远路不能被回绕改走近路。关节在 2.5、偏移 0、目标 −2.5，
// 应该是 −2.5（走 5 rad 的远路），不能是 3.78（走近路穿过限位）
int scenario_yaw_limit_wide(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    // 假电机的位置在第一次反馈之前设好，关节就停在 2.5 rad
    rig.yaw_motor().position = Dm::to_unsigned(2.5F, -12.5F, 12.5F, 16U);
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate(2.5F);
    assert(rig.run_until_holding(2.5F, 0.0F));
    // 核对期间的反馈不换算物理量，READY 之后才发布关节位置
    assert(near(rig.yaw().state().position_rad, 2.5F, 0.01F));

    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(2.5F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, -2.5F, 0.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    assert(near(rig.gimbal().report().yaw_target_rad, -2.5F, 0.01F));
    assert(rig.gimbal().report().yaw_target_rad < 0.0F);
    return 0;
}

// 目标被限位钳住时，这一拍不加角速度前馈：角度环不参与（kp = 0），力矩应该仍是 0
int scenario_pitch_feedforward(Publishers &publishers)
{
    gimbal::Gimbal::Parameters parameters = PARAMETERS;
    parameters.pitch.controller.angle.kp = 0.0F;
    parameters.pitch.controller.angle.ki = 0.0F;
    Rig rig{publishers.attitude, publishers.stop, publishers.command, parameters};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    // 目标 2.0 超出限位（钳在 0.5），前馈 +1 rad/s 朝外
    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, 0.0F, 2.0F, 0.0F, 1.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    assert(near(rig.gimbal().report().pitch_target_rad, PARAMETERS.pitch.maximum_rad, 0.01F));
    assert(near(rig.gimbal().report().pitch_diagnostics.rate_target, 0.0F, 1.0e-6F));
    assert(near(rig.round().pitch_torque, 0.0F, 5.0e-3F));
    return 0;
}

// ENABLING 超时：电机一直不确认出力，超时后回 DISABLED 并计数
int scenario_enable_timeout(Publishers &publishers)
{
    gimbal::Gimbal::Parameters parameters = PARAMETERS;
    // 比达妙驱动自己的使能超时（50 ms）短，先撞到云台这一条
    parameters.enable_timeout_us = 20000U;
    Rig rig{publishers.attitude, publishers.stop, publishers.command, parameters};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.yaw_motor().refuse_enable = true; // 电机收不到使能，永远不确认
    rig.release_gate();

    bool timed_out = false;
    for (std::size_t round = 0U; round < 100U; ++round)
    {
        rig.tick_alive();
        if (rig.gimbal().report().last_disable_reason == gimbal::Gimbal::DisableReason::ENABLE_TIMEOUT)
        {
            timed_out = true;
            break;
        }
    }
    assert(timed_out);
    assert(rig.gimbal().report().enable_failure_count >= 1U);
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::DISABLED);
    assert(!rig.yaw().state().enabled && !rig.pitch().state().enabled);
    return 0;
}

// 采样时刻晚于本轮 now：IMU 任务优先级更高，可能在云台取好 now 之后插进来发布。
// 无符号相减会下溢成一个巨大的数，不能因此判成姿态过期
int scenario_attitude_ahead(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    for (std::size_t round = 0U; round < 20U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true, 500U);
        rig.publish_stop(false);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::HOLD);
    assert(rig.gimbal().report().last_disable_reason != gimbal::Gimbal::DisableReason::IMU_STALE);
    assert(rig.gimbal().report().enable_failure_count == 0U);
    assert(rig.yaw().state().enabled && rig.pitch().state().enabled);
    return 0;
}

// 这一轮没有新姿态：不重复积分，重发上一拍的力矩
int scenario_attitude_gap(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    // 目标比测量值大一点，积分每拍都在涨
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, 0.02F, 0.0F);
        rig.tick();
    }
    const float compensation = rig.gimbal().report().yaw_diagnostics.compensation_newton_meter;
    assert(compensation > 0.0F);

    // 停发姿态 5 拍（还在 10 ms 的超时以内）：补偿力矩不能再涨
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.publish_stop(false);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    assert(rig.round().mit); // 每轮照样下发指令
    assert(near(rig.gimbal().report().yaw_diagnostics.compensation_newton_meter, compensation, 1.0e-4F));
    return 0;
}

// 跨 ±π：IMU 的 yaw 从 3.10 匀速走到 3.20，控制律拿到的是连续的角，力矩不跳变
int scenario_yaw_cross_pi(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate(3.1F);
    assert(rig.run_until_holding(3.1F, 0.0F));

    float previous_torque = rig.gimbal().report().yaw_torque_newton_meter;
    bool has_previous = false;
    for (std::size_t index = 0U; index <= 20U; ++index)
    {
        const float yaw = 3.10F + 0.005F * static_cast<float>(index); // 3.10 → 3.20，跨过 π
        rig.publish_attitude(yaw, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.tick();
        const gimbal::Gimbal::Report &report = rig.gimbal().report();
        if (has_previous)
        {
            assert(std::fabs(report.yaw_torque_newton_meter - previous_torque) < 1.0F);
        }
        previous_torque = report.yaw_torque_newton_meter;
        has_previous = true;
    }
    const gimbal::Gimbal::Report &report = rig.gimbal().report();
    assert(report.mode == gimbal::Gimbal::Mode::HOLD);
    assert(report.yaw_continuous_rad > 3.19F); // 连续累计过了 π
    assert(report.yaw_rad < 0.0F);             // 回绕后的值已经翻到负的一侧
    // 实际用的目标也是按连续 yaw 算的：回绕后的目标会得到 −3.18
    assert(report.yaw_target_rad > 3.0F);
    assert(rig.yaw().state().enabled && rig.pitch().state().enabled);
    return 0;
}

// PC 目标的 pitch 超出限位：目标被钳在限位内
int scenario_pitch_limit(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, 0.0F, 2.0F);
        rig.tick();
    }
    assert(rig.gimbal().report().mode == gimbal::Gimbal::Mode::TRACK);
    assert(near(rig.gimbal().report().pitch_target_rad, PARAMETERS.pitch.maximum_rad, 0.01F));

    // 目标在测量值下方 0.2 rad（在限位内）：pitch 力矩为正（低头为正）
    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.publish_attitude(0.0F, 0.0F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::ABSOLUTE, 0.0F, 0.2F);
        rig.tick();
    }
    assert(rig.round().pitch_torque > 0.0F);
    return 0;
}

// 录制：request 开始录、每份新姿态一条、录到条数或容量就停、再请求从头录；没有新姿态的轮次不录
int scenario_capture(Publishers &publishers)
{
    std::array<gimbal::Gimbal::CaptureSample, 8U> buffer{};
    Rig rig{publishers.attitude, publishers.stop, publishers.command, PARAMETERS, true, buffer.data(), buffer.size()};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    // 没有请求时不录
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.tick_alive();
    }
    assert(rig.gimbal().capture().count == 0U);
    assert(rig.gimbal().capture().sequence == 0U);

    // 请求 5 条：5 份新姿态、每份一条，角度与力矩与当拍 report() 一致
    rig.gimbal().request_capture(5U);
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        rig.tick_alive();
        const std::uint32_t count = rig.gimbal().capture().count;
        assert(count == round + 1U);
        const gimbal::Gimbal::CaptureSample &sample = buffer[count - 1U];
        const gimbal::Gimbal::Report &report = rig.gimbal().report();
        assert(sample.mode == static_cast<std::uint32_t>(report.mode));
        assert(near(sample.yaw_rad, report.yaw_continuous_rad, 1.0e-6F));
        assert(near(sample.pitch_rad, report.pitch_rad, 1.0e-6F));
        assert(near(sample.yaw_torque_newton_meter, report.yaw_torque_newton_meter, 1.0e-6F));
        assert(near(sample.pitch_torque_newton_meter, report.pitch_torque_newton_meter, 1.0e-6F));
        assert(std::isfinite(sample.yaw_feedback_torque_newton_meter));
        assert(std::isfinite(sample.pitch_feedback_torque_newton_meter));
        if (count >= 2U)
        {
            assert(sample.time_us > buffer[count - 2U].time_us); // 采样时刻递增
        }
    }
    assert(rig.gimbal().capture().target == 5U);
    assert(rig.gimbal().capture().request == 0U);
    assert(rig.gimbal().capture().sequence == 1U);
    rig.tick_alive();
    assert(rig.gimbal().capture().count == 5U); // 到条数就停

    // 请求 100 条、容量只有 8：录满 8 条就停，sequence 加 1
    rig.gimbal().request_capture(100U);
    for (std::size_t round = 0U; round < 10U; ++round)
    {
        rig.tick_alive();
    }
    assert(rig.gimbal().capture().count == 8U);
    assert(rig.gimbal().capture().target == 8U);
    assert(rig.gimbal().capture().sequence == 2U);

    // 没有新姿态的轮次不录：请求先留着，下一份新姿态才开始
    rig.gimbal().request_capture(1U);
    rig.tick();
    assert(rig.gimbal().capture().count == 8U);
    assert(rig.gimbal().capture().request == 1U);
    rig.tick_alive();
    assert(rig.gimbal().capture().count == 1U);
    assert(rig.gimbal().capture().sequence == 3U);
    return 0;
}

// 没有存储：request 没有效果，count 一直是 0
int scenario_capture_off(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());
    rig.gimbal().request_capture(5U);
    for (std::size_t round = 0U; round < 20U; ++round)
    {
        rig.tick_alive();
    }
    assert(rig.gimbal().capture().count == 0U);
    assert(rig.gimbal().capture().sequence == 0U);
    assert(rig.gimbal().capture().request == 0U);
    return 0;
}

// 在线调参：改完 sequence 加 1，下一份新姿态生效；积分不清；没有新姿态的轮次不处理
int scenario_tuning(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    // 恒定的 pitch 偏差：角速度环积分涨到限幅（1 N·m），力矩稳定在 −2.2 N·m 左右
    for (std::size_t round = 0U; round < 500U; ++round)
    {
        rig.publish_attitude(0.0F, 0.02F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::HOLD);
        rig.tick();
    }
    const float pitch_before = rig.gimbal().report().pitch_torque_newton_meter;
    const float compensation_before = rig.gimbal().report().pitch_diagnostics.compensation_newton_meter;
    assert(std::fabs(compensation_before) > 0.9F); // 积分确实攒到了限幅
    assert(pitch_before < -1.5F);

    // 改 pitch 角度环 kp（30 → 1）与 yaw 角度环 kp（30 → 1），sequence 加 1
    gimbal::AxisLoop::Parameters yaw_parameters = rig.gimbal().tuning().yaw;
    gimbal::AxisLoop::Parameters pitch_parameters = rig.gimbal().tuning().pitch;
    yaw_parameters.angle.kp = 1.0F;
    pitch_parameters.angle.kp = 1.0F;
    const float gravity = rig.gimbal().tuning().pitch_gravity_newton_meter;
    rig.gimbal().request_tuning(yaw_parameters, pitch_parameters, gravity);

    // 没有新姿态的轮次不处理：applied_sequence 不动
    const std::uint32_t applied = rig.gimbal().tuning().applied_sequence;
    assert(rig.gimbal().tuning().sequence == applied + 1U);
    rig.tick();
    assert(rig.gimbal().tuning().applied_sequence == applied);

    // 下一份新姿态生效：力矩按新 kp 变小，积分没有被清掉
    rig.publish_attitude(0.0F, 0.02F, topics::imu::State::READY, true);
    rig.publish_stop(false);
    rig.publish_command(topics::command::GimbalMode::HOLD);
    rig.tick();
    assert(rig.gimbal().tuning().applied_sequence == rig.gimbal().tuning().sequence);
    assert(rig.gimbal().tuning().rejected_count == 0U);
    assert(near(rig.gimbal().tuning().pitch.angle.kp, 1.0F, 1.0e-6F));
    const float pitch_after = rig.gimbal().report().pitch_torque_newton_meter;
    assert(std::fabs(pitch_after) < std::fabs(pitch_before) - 0.5F);
    assert(std::fabs(rig.gimbal().report().pitch_diagnostics.compensation_newton_meter) > 0.9F);

    // yaw 也一样：偏 0.1 rad，新 kp 下力矩很小（旧 kp 会被总限幅钳住）
    rig.publish_attitude(0.1F, 0.02F, topics::imu::State::READY, true);
    rig.publish_stop(false);
    rig.publish_command(topics::command::GimbalMode::HOLD);
    rig.tick();
    assert(std::fabs(rig.gimbal().report().yaw_torque_newton_meter) < 1.0F);
    return 0;
}

// 调参被拒：任何一项无效时整块都不生效，值改回当前生效的，行为照旧
int scenario_tuning_reject(Publishers &publishers)
{
    Rig rig{publishers.attitude, publishers.stop, publishers.command};
    assert(rig.init());
    assert(rig.run_until_ready());
    rig.release_gate();
    assert(rig.run_until_holding());

    // 两轴都偏 0.1 rad：旧增益下力矩被总限幅钳在 −5 N·m
    const auto publish_offset = [&rig]()
    {
        rig.publish_attitude(0.1F, 0.1F, topics::imu::State::READY, true);
        rig.publish_stop(false);
        rig.publish_command(topics::command::GimbalMode::HOLD);
        rig.tick();
    };
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        publish_offset();
    }
    assert(near(rig.gimbal().report().yaw_torque_newton_meter, -5.0F, 1.0e-3F));
    assert(near(rig.gimbal().report().pitch_torque_newton_meter, -5.0F, 1.0e-3F));
    const gimbal::AxisLoop::Parameters original_yaw = rig.gimbal().tuning().yaw;
    const gimbal::AxisLoop::Parameters original_pitch = rig.gimbal().tuning().pitch;

    // 1) yaw 有效、pitch 的角速度环 kp 给了负数：整块拒绝，yaw 也不生效
    gimbal::AxisLoop::Parameters yaw_parameters = original_yaw;
    gimbal::AxisLoop::Parameters pitch_parameters = original_pitch;
    yaw_parameters.angle.kp = 1.0F;
    pitch_parameters.rate.kp = -1.0F;
    rig.gimbal().request_tuning(yaw_parameters, pitch_parameters, 0.0F);
    publish_offset();
    assert(rig.gimbal().tuning().rejected_count == 1U);
    assert(rig.gimbal().tuning().applied_sequence == rig.gimbal().tuning().sequence);
    assert(near(rig.gimbal().tuning().yaw.angle.kp, original_yaw.angle.kp, 1.0e-6F));
    assert(near(rig.gimbal().tuning().pitch.rate.kp, original_pitch.rate.kp, 1.0e-6F));
    assert(near(rig.gimbal().report().yaw_torque_newton_meter, -5.0F, 1.0e-3F));

    // 2) pitch 的 kp 是 NaN
    pitch_parameters = original_pitch;
    pitch_parameters.angle.kp = std::numeric_limits<float>::quiet_NaN();
    rig.gimbal().request_tuning(original_yaw, pitch_parameters, 0.0F);
    publish_offset();
    assert(rig.gimbal().tuning().rejected_count == 2U);
    assert(near(rig.gimbal().tuning().pitch.angle.kp, original_pitch.angle.kp, 1.0e-6F));

    // 3) 重力前馈超过总限幅
    rig.gimbal().request_tuning(original_yaw, original_pitch, 6.0F);
    publish_offset();
    assert(rig.gimbal().tuning().rejected_count == 3U);
    assert(rig.gimbal().tuning().pitch_gravity_newton_meter == PARAMETERS.pitch_gravity_newton_meter);
    assert(near(rig.gimbal().report().pitch_torque_newton_meter, -5.0F, 1.0e-3F));

    // 4) 全部有效：生效，两轴的力矩都按新 kp 变小
    yaw_parameters = original_yaw;
    pitch_parameters = original_pitch;
    yaw_parameters.angle.kp = 1.0F;
    pitch_parameters.angle.kp = 1.0F;
    rig.gimbal().request_tuning(yaw_parameters, pitch_parameters, 0.0F);
    publish_offset();
    assert(rig.gimbal().tuning().rejected_count == 3U);
    assert(rig.gimbal().tuning().applied_sequence == rig.gimbal().tuning().sequence);
    assert(std::fabs(rig.gimbal().report().pitch_torque_newton_meter) < 4.0F);
    assert(std::fabs(rig.gimbal().report().yaw_torque_newton_meter) < 1.0F);
    return 0;
}

} // namespace

int main(int argc, char **argv)
{
    Publishers publishers{topics::imu::attitude.claim_publisher(),
                          topics::safety::stop.claim_publisher(),
                          topics::command::gimbal.claim_publisher()};
    assert(publishers.attitude && publishers.stop && publishers.command);
    if (argc != 2)
    {
        std::fprintf(stderr,
                     "用法：%s settling|locked|enable|track|estop|fault|attitude_timeout|command_timeout|pitch_limit|"
                     "yaw_limit|yaw_limit_wide|pitch_feedforward|enable_timeout|attitude_ahead|attitude_gap|"
                     "yaw_cross_pi|manual_rate|manual_clamp|manual_drop|fault_beats_drop|absolute_rate_limit|state|"
                     "mode_switch|reject|capture|capture_off|tuning|tuning_reject\n",
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
    if (std::strcmp(scenario, "command_timeout") == 0)
    {
        return scenario_command_timeout(publishers);
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
    if (std::strcmp(scenario, "yaw_limit_wide") == 0)
    {
        return scenario_yaw_limit_wide(publishers);
    }
    if (std::strcmp(scenario, "pitch_feedforward") == 0)
    {
        return scenario_pitch_feedforward(publishers);
    }
    if (std::strcmp(scenario, "enable_timeout") == 0)
    {
        return scenario_enable_timeout(publishers);
    }
    if (std::strcmp(scenario, "attitude_ahead") == 0)
    {
        return scenario_attitude_ahead(publishers);
    }
    if (std::strcmp(scenario, "attitude_gap") == 0)
    {
        return scenario_attitude_gap(publishers);
    }
    if (std::strcmp(scenario, "yaw_cross_pi") == 0)
    {
        return scenario_yaw_cross_pi(publishers);
    }
    if (std::strcmp(scenario, "manual_rate") == 0)
    {
        return scenario_manual_rate(publishers);
    }
    if (std::strcmp(scenario, "manual_clamp") == 0)
    {
        return scenario_manual_clamp(publishers);
    }
    if (std::strcmp(scenario, "absolute_rate_limit") == 0)
    {
        return scenario_absolute_rate_limit(publishers);
    }
    if (std::strcmp(scenario, "state") == 0)
    {
        return scenario_state(publishers);
    }
    if (std::strcmp(scenario, "mode_switch") == 0)
    {
        return scenario_mode_switch(publishers);
    }
    if (std::strcmp(scenario, "fault_beats_drop") == 0)
    {
        return scenario_fault_beats_drop(publishers);
    }
    if (std::strcmp(scenario, "manual_drop") == 0)
    {
        return scenario_manual_drop(publishers);
    }
    if (std::strcmp(scenario, "capture") == 0)
    {
        return scenario_capture(publishers);
    }
    if (std::strcmp(scenario, "capture_off") == 0)
    {
        return scenario_capture_off(publishers);
    }
    if (std::strcmp(scenario, "tuning") == 0)
    {
        return scenario_tuning(publishers);
    }
    if (std::strcmp(scenario, "tuning_reject") == 0)
    {
        return scenario_tuning_reject(publishers);
    }
    std::fprintf(stderr, "未知场景：%s\n", scenario);
    return 2;
}
