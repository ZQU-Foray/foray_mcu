// device::DmMotor 在 platform/mock 上：启动回读核对、使能、反馈、故障与离线（Decision-20261102-01）；
// 登记到 CanBus 收帧、经关节 Joint 控制（Decision-20261102-03）。
// 电机由 dm_fake_motor.hpp 按说明书的帧格式扮演；ESC_ID 1、MST_ID 0x11 与 2026-11-01 实机的设置相同。
#include "driver/can_bus/can_bus.hpp"
#include "driver/motor/dm/dm_models.hpp"
#include "driver/motor/dm/dm_motor.hpp"
#include "driver/motor/joint.hpp"
#include "unit/motor/dm_fake_motor.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace
{

using device::DmMotor;
using device::Joint;
using device::JointMechanics;
using dm_test::Bench;
using dm_test::Bytes;
using dm_test::Dm;
using dm_test::FakeMotor;
using dm_test::is_register_request;
using dm_test::is_system_command;
using Phase = DmMotor::Phase;

constexpr std::uint8_t BUS{0U};
constexpr std::size_t VERIFY_ROUNDS{7U}; // 读模式、失能、三个量程、写超时，第 7 轮收到回显
// 多数测试只下一次指令再看好几轮，把关节的指令超时放宽到 1 s；默认 3 ms 的超时单独测
constexpr JointMechanics LONG_TIMEOUT{1, 1.0F, 0.0F, 1000000U};

// 构造函数是 constexpr：作全局对象时常量初始化，不依赖全局构造顺序
device::CanBus g_constant_bus{hal::Can{BUS}};
Joint g_constant_joint;
constexpr DmMotor::Config constant_config()
{
    DmMotor::Config config{};
    config.count = 1U;
    config.motors[0] = DmMotor::Motor{&g_constant_joint, device::dm::DM_J4310, 1U, 0x11U, JointMechanics{}};
    return config;
}
constexpr DmMotor CONSTANT_MOTOR{g_constant_bus, constant_config()};
static_assert(CONSTANT_MOTOR.count() == 1U);
static_assert(device::dm::DM_J4310.maximum_torque_newton_meter() == 10.0F); // 峰值 12.5 与 TMAX 10 取小
static_assert(DmMotor::frames_per_second(3U, 1000U) == 6000U);
static_assert(device::can_load_percent(DmMotor::frames_per_second(3U, 1000U)) == 81U);

void reset()
{
    mock::reset();
    mock::can::reset();
    mock::set_now_us(1000U);
}

DmMotor::Motor motor_of(std::uint16_t esc_id, std::uint16_t master_id, const JointMechanics &mechanics = LONG_TIMEOUT)
{
    return DmMotor::Motor{nullptr, device::dm::DM_J4310, esc_id, master_id, mechanics};
}

DmMotor::Config one_motor(std::uint16_t esc_id = 1U, std::uint16_t master_id = 0x11U)
{
    DmMotor::Config config{};
    config.count = 1U;
    config.motors[0] = motor_of(esc_id, master_id);
    return config;
}

/** 一条总线：CanBus、关节、驱动、模拟电机。配置里没给关节的电机按顺序接到 joints[i]。 */
struct Rig final
{
    explicit Rig(const DmMotor::Config &config) noexcept : driver{bus, with_joints(config, joints)} {}

    [[nodiscard]] bool start() noexcept
    {
        return bus.init() && driver.init();
    }
    void cycle(std::size_t rounds = 1U) noexcept
    {
        dm_test::cycle(loop, bench, rounds);
    }
    [[nodiscard]] DmMotor::Snapshot snapshot(std::size_t index = 0U) const noexcept
    {
        DmMotor::Snapshot result{};
        assert(driver.get_snapshot(index, result));
        return result;
    }
    [[nodiscard]] DmMotor::Statistics statistics() const noexcept
    {
        DmMotor::Statistics result{};
        assert(driver.get_statistics(result));
        return result;
    }

    static DmMotor::Config with_joints(DmMotor::Config config, std::array<Joint, DmMotor::MAXIMUM_MOTORS> &joints)
    {
        for (std::size_t index = 0U; index < config.motors.size(); ++index)
        {
            if (config.motors[index].joint == nullptr)
            {
                config.motors[index].joint = &joints[index];
            }
        }
        return config;
    }

    Bench bench{BUS};
    device::CanBus bus{hal::Can{BUS}};
    std::array<Joint, DmMotor::MAXIMUM_MOTORS> joints{};
    DmMotor driver;
    dm_test::BusLoop loop{bus};
};

bool near(float actual, float expected, float tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

const hal::Can::Frame &only_sent(const Bench &bench)
{
    assert(bench.sent_count() == 1U);
    return bench.sent(0U);
}

bool sent_system(const Bench &bench, std::uint32_t identifier, Dm::SystemCommand command)
{
    const hal::Can::Frame &frame = only_sent(bench);
    return frame.identifier == identifier && is_system_command(frame, command);
}

/** 走完核对，返回时 READY，且已收到核对后的第一帧反馈。 */
void verify(Rig &rig)
{
    rig.cycle(VERIFY_ROUNDS + 1U);
    assert(rig.snapshot().phase == Phase::READY);
}

/** 关节 enable() 并走到电机确认使能：清除错误、使能，第 3 轮开始发 MIT。 */
void enable_and_confirm(Rig &rig, std::size_t index = 0U)
{
    assert(rig.joints[index].enable());
    rig.cycle(3U);
    assert(rig.snapshot(index).output_active && rig.joints[index].state().enabled);
}

void test_verification_sequence()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Rig rig{one_motor()};
    rig.bench.add(motor);
    Joint &joint = rig.joints[0];
    assert(rig.snapshot().phase == Phase::STOPPED);
    rig.loop.process(); // 没有 init() 不发送
    assert(mock::can::sent_count(BUS) == 0U);
    assert(rig.start());
    assert(rig.snapshot().phase == Phase::VERIFYING);
    // 关节已接上：支持力矩与阻抗，绝对位置，有温度；最大力矩 10 N·m；核对完成前不能使能
    assert(joint.supports(device::JointMode::TORQUE) && joint.supports(device::JointMode::IMPEDANCE));
    assert(joint.state().position_absolute && joint.state().has_temperature);
    assert(joint.maximum_torque_newton_meter() == 10.0F && !joint.enable());

    rig.cycle();
    assert(is_register_request(only_sent(rig.bench), 1U, Dm::RegisterOperation::READ, Dm::Register::CONTROL_MODE));
    assert(only_sent(rig.bench).length == 4U && !joint.available());
    rig.cycle();
    assert(sent_system(rig.bench, 0x001U, Dm::SystemCommand::DISABLE)); // 按读到的模式（MIT）发失能
    rig.cycle();
    assert(is_register_request(only_sent(rig.bench), 1U, Dm::RegisterOperation::READ, Dm::Register::POSITION_MAX));
    rig.cycle();
    assert(is_register_request(only_sent(rig.bench), 1U, Dm::RegisterOperation::READ, Dm::Register::VELOCITY_MAX));
    rig.cycle();
    assert(is_register_request(only_sent(rig.bench), 1U, Dm::RegisterOperation::READ, Dm::Register::TORQUE_MAX));
    rig.cycle();
    // 型号的 50 ms = 1000 个 50 µs 计数，写进电机 RAM
    const hal::Can::Frame &write = only_sent(rig.bench);
    assert(is_register_request(write, 1U, Dm::RegisterOperation::WRITE, Dm::Register::TIMEOUT));
    assert(write.length == 8U && write.data[4] == 0xE8U && write.data[5] == 0x03U && write.data[6] == 0U);
    assert(motor.timeout == 1000U);
    assert(rig.snapshot().phase == Phase::VERIFYING);

    rig.cycle(); // 收到回显：READY，关节可以使能；没有请求出力，发失能作保活
    DmMotor::Snapshot snapshot = rig.snapshot();
    assert(snapshot.phase == Phase::READY && joint.available());
    assert(snapshot.control_mode == 1U && snapshot.timeout_ticks == 1000U && snapshot.mismatch_register == 0U);
    assert(snapshot.limits.position_rad == 12.5F && snapshot.limits.velocity_rad_per_second == 30.0F &&
           snapshot.limits.torque_newton_meter == 10.0F);
    assert(sent_system(rig.bench, 0x001U, Dm::SystemCommand::DISABLE));
    assert(!snapshot.has_feedback && !joint.state().feedback_valid); // 核对时的反馈不换算

    motor.velocity = 0x732U; // −3 rad/s
    motor.torque = 0x9FFU;   // 2.5 N·m
    rig.cycle();
    snapshot = rig.snapshot();
    assert(snapshot.has_feedback && snapshot.online && snapshot.state == Dm::State::DISABLED);
    assert(near(snapshot.position_rad, 0.0F, 0.001F));
    assert(snapshot.mos_temperature_celsius == 30U && snapshot.rotor_temperature_celsius == 35U);
    assert(sent_system(rig.bench, 0x001U, Dm::SystemCommand::DISABLE));
    rig.cycle();
    snapshot = rig.snapshot();
    assert(near(snapshot.velocity_rad_per_second, -3.0F, 0.02F) && near(snapshot.torque_newton_meter, 2.5F, 0.01F));
    // 关节的状态：同样的物理量（机构是 1:1），带反馈的接收时刻，线圈温度
    const device::JointState &state = joint.state();
    assert(state.feedback_valid && state.online && !state.enabled && !state.fault);
    assert(near(state.velocity_rad_per_second, -3.0F, 0.02F) && near(state.torque_newton_meter, 2.5F, 0.01F));
    assert(state.temperature_celsius == 35.0F && state.stamp_us == snapshot.last_update_us);

    const DmMotor::Statistics statistics = rig.statistics();
    assert(statistics.register_reply_count == 5U && statistics.request_timeout_count == 0U);
    assert(statistics.unknown_frame_count == 0U && statistics.stale_reply_count == 0U);
    assert(statistics.tx_result_counts[static_cast<std::size_t>(hal::Can::SendResult::QUEUED)] == 9U);
}

void test_retry_and_late_power()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    motor.powered = false;
    Rig rig{one_motor()};
    rig.bench.add(motor);
    assert(rig.start());

    // 没有应答：每 20 ms 重发一次
    std::size_t sent = 0U;
    for (std::size_t round = 0U; round < 45U; ++round)
    {
        rig.cycle();
        sent += rig.bench.sent_count();
    }
    assert(sent == 3U && rig.statistics().request_timeout_count == 2U);

    // 发送队列满：不算发出，下一轮就重试，不等 20 ms
    mock::can::set_send_result(BUS, hal::Can::SendResult::QUEUE_FULL);
    mock::advance_us(DmMotor::REQUEST_TIMEOUT_US);
    rig.cycle(3U);
    assert(rig.statistics().tx_result_counts[static_cast<std::size_t>(hal::Can::SendResult::QUEUE_FULL)] == 3U);
    mock::can::set_send_result(BUS, hal::Can::SendResult::QUEUED);

    // 电机上电后接上
    motor.powered = true;
    rig.cycle();
    assert(is_register_request(only_sent(rig.bench), 1U, Dm::RegisterOperation::READ, Dm::Register::CONTROL_MODE));
    verify(rig);

    // 重发造成的重复应答：丢弃，不当成反馈
    const DmMotor::Snapshot before = rig.snapshot();
    dm_test::inject(BUS, 0x11U, 8U, Bytes{0x01U, 0x00U, 0x55U, 0x09U, 0xE8U, 0x03U, 0x00U, 0x00U});
    dm_test::inject(BUS, 0x11U, 8U, Bytes{0x01U, 0x00U, 0x33U, 0x0AU, 0x01U, 0x00U, 0x00U, 0x00U});
    rig.cycle();
    const DmMotor::Snapshot after = rig.snapshot();
    assert(rig.statistics().stale_reply_count == 2U);
    assert(after.phase == Phase::READY && after.feedback_count == before.feedback_count + 1U);
    assert(after.position_rad == before.position_rad);
}

/** 用给定的电机设置走完核对，返回最终快照。 */
DmMotor::Snapshot verify_with(FakeMotor &motor, std::size_t rounds = VERIFY_ROUNDS + 2U)
{
    Rig rig{one_motor()};
    rig.bench.add(motor);
    assert(rig.start());
    rig.cycle(rounds);
    const DmMotor::Snapshot snapshot = rig.snapshot();
    if (snapshot.phase == Phase::MISMATCH)
    {
        Joint &joint = rig.joints[0];
        assert(!joint.available() && !joint.enable() && !joint.set_torque(1.0F) && joint.state().fault);
        rig.cycle(5U); // 核对不符：不再发任何帧
        assert(rig.bench.sent_count() == 0U && mock::can::sent_count(BUS) == 0U);
    }
    return snapshot;
}

void test_mismatch()
{
    {
        reset();
        FakeMotor motor{BUS, 1U, 0x11U};
        motor.control_mode = static_cast<std::uint32_t>(Dm::ControlMode::VELOCITY);
        motor.state = Dm::State::ENABLED; // 单片机复位前电机还在使能
        const DmMotor::Snapshot snapshot = verify_with(motor);
        assert(snapshot.phase == Phase::MISMATCH && snapshot.mismatch_register == 0x0AU && snapshot.control_mode == 3U);
        // 失能按电机当前的模式发（速度模式 0x200 + ID），电机确实停了
        assert(motor.last_system_identifier == 0x201U && motor.last_system_command == Dm::SystemCommand::DISABLE);
        assert(motor.state == Dm::State::DISABLED);
        assert(snapshot.limits.position_rad == 12.5F); // 其余照常读出，便于排查
    }
    {
        reset();
        FakeMotor motor{BUS, 1U, 0x11U};
        motor.position_max = 25.0F;
        const DmMotor::Snapshot snapshot = verify_with(motor);
        assert(snapshot.phase == Phase::MISMATCH && snapshot.mismatch_register == 0x15U);
        assert(snapshot.limits.position_rad == 25.0F);
    }
    {
        reset();
        FakeMotor motor{BUS, 1U, 0x11U};
        motor.velocity_max = 45.0F;
        assert(verify_with(motor).mismatch_register == 0x16U);
    }
    {
        reset();
        FakeMotor motor{BUS, 1U, 0x11U};
        motor.torque_max = 9.99F;
        assert(verify_with(motor).mismatch_register == 0x17U);
    }
    {
        reset();
        FakeMotor motor{BUS, 1U, 0x11U};
        motor.echo_wrong_timeout = true;
        const DmMotor::Snapshot snapshot = verify_with(motor);
        assert(snapshot.mismatch_register == 0x09U && snapshot.timeout_ticks == 1001U);
    }
    {
        reset();
        FakeMotor motor{BUS, 1U, 0x11U};
        motor.position_max = 12.50001F; // 相对误差 8e-7：调试助手写入的十进制换算误差，算一致
        assert(verify_with(motor).phase == Phase::READY);
    }
    {
        // 不认识的模式：不知道失能该用哪个帧 ID，读完模式就判为不符
        reset();
        FakeMotor motor{BUS, 1U, 0x11U};
        motor.control_mode = 0U;
        const DmMotor::Snapshot snapshot = verify_with(motor, 2U);
        assert(snapshot.phase == Phase::MISMATCH && snapshot.mismatch_register == 0x0AU);
        assert(motor.frames_received == 1U);
    }
}

void test_config_validation()
{
    reset();
    const auto rejects = [](const DmMotor::Config &config)
    {
        Rig rig{config};
        assert(rig.bus.init() && !rig.driver.init());
        rig.loop.process();
        assert(mock::can::sent_count(BUS) == 0U);
        assert(!rig.joints[0].supports(device::JointMode::TORQUE)); // 失败时关节没被接上
        mock::can::reset();
    };
    DmMotor::Config config = one_motor();
    config.count = 0U;
    rejects(config);
    config.count = DmMotor::MAXIMUM_MOTORS + 1U;
    rejects(config);
    rejects(one_motor(0U, 0x11U));
    rejects(one_motor(16U, 0x20U)); // 反馈 D0 只有 4 位放 ESC_ID
    rejects(one_motor(1U, 0x7FFU)); // 寄存器请求的帧 ID
    rejects(one_motor(1U, 0x001U)); // MST_ID 低 8 位等于 ESC_ID
    rejects(one_motor(1U, 0x101U));
    config = one_motor();
    config.count = 2U;
    config.motors[1] = motor_of(1U, 0x12U); // ESC_ID 重复
    rejects(config);
    config.motors[1] = motor_of(2U, 0x201U); // 低 8 位等于第一台的 ESC_ID
    rejects(config);
    config = one_motor();
    config.motors[0].profile.limits.position_rad = 0.0F;
    rejects(config);
    config = one_motor();
    config.motors[0].profile.peak_torque_newton_meter = std::numeric_limits<float>::quiet_NaN();
    rejects(config);
    config = one_motor();
    config.motors[0].profile.communication_timeout_us = DmMotor::TIMEOUT_TICK_US - 1U;
    rejects(config);
    config = one_motor();
    config.motors[0].mechanics.direction = 0;
    rejects(config);
    config = one_motor();
    config.motors[0].mechanics.gear_ratio = 0.0F;
    rejects(config);

    // 一个关节只能接一台电机：同组里重复，或已被别的驱动接走
    Joint shared;
    config = one_motor();
    config.count = 2U;
    config.motors[0].joint = &shared;
    config.motors[1] = motor_of(2U, 0x12U);
    config.motors[1].joint = &shared;
    rejects(config);
    {
        device::CanBus other_bus{hal::Can{1U}};
        DmMotor::Config other = one_motor();
        other.motors[0].joint = &shared;
        DmMotor owner{other_bus, other};
        assert(owner.init());
        Rig rig{other};
        assert(!rig.driver.init());
    }

    // 同一条 CanBus 上可以有两组达妙电机，MST_ID 不能重叠
    reset();
    Rig rig{one_motor()};
    assert(rig.start() && rig.driver.init()); // 重复 init() 不再登记
    Joint second_joint;
    DmMotor::Config second = one_motor(2U, 0x12U);
    second.motors[0].joint = &second_joint;
    DmMotor neighbor{rig.bus, second};
    assert(neighbor.init());
    Joint third_joint;
    DmMotor::Config third = one_motor(3U, 0x11U); // 与第一组的 MST_ID 重叠
    third.motors[0].joint = &third_joint;
    DmMotor overlapping{rig.bus, third};
    assert(!overlapping.init() && !third_joint.supports(device::JointMode::TORQUE));
    assert(rig.driver.count() == 1U);
    DmMotor::Snapshot snapshot{};
    assert(!rig.driver.get_snapshot(1U, snapshot));
}

void test_enable_flow()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Rig rig{one_motor()};
    rig.bench.add(motor);
    Joint &joint = rig.joints[0];
    assert(rig.start());
    verify(rig);

    assert(!joint.set_torque(1.0F)); // 没有请求出力
    assert(joint.enable());
    assert(joint.set_torque(2.5F));
    rig.cycle();
    assert(sent_system(rig.bench, 0x001U, Dm::SystemCommand::CLEAR_ERROR));
    rig.cycle();
    assert(sent_system(rig.bench, 0x001U, Dm::SystemCommand::ENABLE));
    assert(motor.state == Dm::State::ENABLED && !rig.snapshot().output_active);
    rig.cycle(); // 收到"使能"反馈，开始发指令
    DmMotor::Snapshot snapshot = rig.snapshot();
    assert(snapshot.output_requested && snapshot.output_active && snapshot.state == Dm::State::ENABLED);
    assert(only_sent(rig.bench).identifier == 0x001U);
    assert((only_sent(rig.bench).data == Bytes{0x7FU, 0xFFU, 0x7FU, 0xF0U, 0x00U, 0x00U, 0x09U, 0xFFU}));
    assert(joint.enable()); // 已经请求过：不重新清错
    rig.cycle();
    assert(motor.has_mit && motor.last_system_command == Dm::SystemCommand::ENABLE && joint.state().enabled);

    // 阻抗指令就是完整的 MIT；非有限值清掉指令
    assert(joint.set_impedance(1.0F, -3.0F, 50.0F, 1.0F, 2.5F));
    rig.cycle();
    assert((only_sent(rig.bench).data == Bytes{0x8AU, 0x3CU, 0x73U, 0x21U, 0x99U, 0x33U, 0x39U, 0xFFU}));
    assert(!joint.set_torque(std::numeric_limits<float>::infinity()));
    rig.cycle();
    assert((only_sent(rig.bench).data == Bytes{0x7FU, 0xFFU, 0x7FU, 0xF0U, 0x00U, 0x00U, 0x07U, 0xFFU}));

    // 失能：撤销请求、清零指令，之后每周期发失能
    assert(joint.set_torque(1.0F));
    joint.disable();
    assert(!joint.set_torque(1.0F));
    rig.cycle();
    assert(sent_system(rig.bench, 0x001U, Dm::SystemCommand::DISABLE) && motor.state == Dm::State::DISABLED);
    rig.cycle();
    snapshot = rig.snapshot();
    assert(!snapshot.output_requested && !snapshot.output_active && snapshot.phase == Phase::READY);
    assert(!joint.state().enabled);

    // 再使能时指令从零开始
    enable_and_confirm(rig);
    assert(only_sent(rig.bench).data[6] == 0x07U && only_sent(rig.bench).data[7] == 0xFFU);
}

void test_command_timeout()
{
    // 关节默认的 3 ms 指令超时：能力停止刷新指令，驱动按 0 力矩发，电机仍使能
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    DmMotor::Config config = one_motor();
    config.motors[0].mechanics = JointMechanics{};
    Rig rig{config};
    rig.bench.add(motor);
    Joint &joint = rig.joints[0];
    assert(rig.start());
    verify(rig);
    enable_and_confirm(rig);
    assert(joint.set_torque(2.5F));
    rig.cycle(3U);
    assert(only_sent(rig.bench).data[6] == 0x09U); // 2.5 N·m 还在
    rig.cycle();
    assert(only_sent(rig.bench).data[6] == 0x07U && only_sent(rig.bench).data[7] == 0xFFU);
    assert(joint.command_timeout_count() == 1U && rig.snapshot().output_active);
}

void test_stamps_after_round_start()
{
    // 上板时一轮开始（now）之后，帧才收到、指令才下，时间戳都比 now 晚一点（2026-11-02 上板发现）：
    // 不能被当成离线，也不能被当成指令超时
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    DmMotor::Config config = one_motor();
    config.motors[0].mechanics = JointMechanics{}; // 默认 3 ms 指令超时
    Rig rig{config};
    rig.bench.add(motor);
    Joint &joint = rig.joints[0];
    assert(rig.start());
    verify(rig);
    enable_and_confirm(rig);
    for (std::size_t round = 0U; round < 5U; ++round)
    {
        const hal::os::Microseconds now = hal::os::now_us(); // 这一轮开始
        mock::advance_us(50U);                               // 之后才取帧
        rig.bus.receive();
        mock::advance_us(20U); // 再之后才算出指令
        assert(joint.set_torque(2.5F));
        rig.bus.transmit(now);
        rig.bench.respond();
        mock::advance_us(930U);
    }
    const DmMotor::Snapshot snapshot = rig.snapshot();
    assert(snapshot.phase == Phase::READY && snapshot.output_active && snapshot.online);
    assert(only_sent(rig.bench).data[6] == 0x09U && joint.command_timeout_count() == 0U); // 2.5 N·m 发出去了

    // 帧在这一轮开始之后才收到：不判离线
    const hal::os::Microseconds now = hal::os::now_us();
    mock::advance_us(100U);
    motor.send_feedback();
    rig.bus.receive();
    rig.bus.transmit(now);
    assert(rig.snapshot().phase == Phase::READY && rig.snapshot().online && joint.state().online);
}

void test_enable_confirm_guard()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Rig rig{one_motor()};
    rig.bench.add(motor);
    assert(rig.start());
    verify(rig);
    assert(rig.joints[0].enable());
    rig.loop.process();
    rig.bench.respond();
    assert(sent_system(rig.bench, 0x001U, Dm::SystemCommand::CLEAR_ERROR));
    // 清除错误之前发出的帧，其"使能"反馈在守护时间内晚到：不算确认
    motor.state = Dm::State::ENABLED;
    motor.send_feedback();
    motor.state = Dm::State::DISABLED;
    mock::advance_us(1000U);
    rig.cycle();
    assert(!rig.snapshot().output_active);
    assert(sent_system(rig.bench, 0x001U, Dm::SystemCommand::ENABLE));
}

void test_enable_failure()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Rig rig{one_motor()};
    rig.bench.add(motor);
    Joint &joint = rig.joints[0];
    assert(rig.start());
    verify(rig);

    // 电机一直不使能：50 ms 后放弃，关节的请求被撤销
    motor.refuse_enable = true;
    assert(joint.enable());
    rig.cycle(49U);
    assert(rig.snapshot().output_requested && sent_system(rig.bench, 0x001U, Dm::SystemCommand::ENABLE));
    rig.cycle(2U);
    DmMotor::Snapshot snapshot = rig.snapshot();
    assert(!snapshot.output_requested && snapshot.enable_failed && !joint.set_torque(1.0F));
    assert(joint.state().fault && !joint.state().enabled);
    assert(sent_system(rig.bench, 0x001U, Dm::SystemCommand::DISABLE));
    assert(rig.statistics().enable_failure_count == 1U);

    // 故障清不掉（一直欠压）：守护时间之后看到故障就放弃，不等 50 ms
    motor.refuse_enable = false;
    motor.state = Dm::State::UNDER_VOLTAGE;
    motor.fault_clearable = false;
    rig.cycle();
    assert(joint.enable());
    rig.cycle(4U);
    snapshot = rig.snapshot();
    assert(!snapshot.output_requested && snapshot.enable_failed && snapshot.state == Dm::State::UNDER_VOLTAGE);
    assert(DmMotor::is_fault(snapshot.state) && rig.statistics().enable_failure_count == 2U);
    assert(joint.state().fault_code == static_cast<std::uint8_t>(Dm::State::UNDER_VOLTAGE));

    // 故障消失后再 enable()：先清错，再使能
    motor.fault_clearable = true;
    enable_and_confirm(rig);
    assert(!rig.snapshot().enable_failed && motor.state == Dm::State::ENABLED && !joint.state().fault);
}

void test_fault_and_reset_while_enabled()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Rig rig{one_motor()};
    rig.bench.add(motor);
    Joint &joint = rig.joints[0];
    assert(rig.start());
    verify(rig);
    enable_and_confirm(rig);
    assert(joint.set_torque(1.0F));

    // 运行中报故障：撤销请求，改发失能，不自动重试
    motor.state = Dm::State::MOS_OVER_TEMPERATURE;
    rig.cycle(2U);
    DmMotor::Snapshot snapshot = rig.snapshot();
    assert(!snapshot.output_requested && !snapshot.output_active && snapshot.phase == Phase::READY);
    assert(snapshot.state == Dm::State::MOS_OVER_TEMPERATURE && !joint.set_torque(1.0F) && joint.state().fault);
    assert(sent_system(rig.bench, 0x001U, Dm::SystemCommand::DISABLE));
    rig.cycle(10U);
    assert(motor.state == Dm::State::MOS_OVER_TEMPERATURE); // 驱动不清错
    enable_and_confirm(rig);                                // 能力再次 enable()：清错后恢复

    // 运行中电机自己变成失能（复位过）：撤销请求并重新核对，RAM 里的超时要重写
    motor.state = Dm::State::DISABLED;
    motor.timeout = 0U;
    rig.cycle(2U);
    snapshot = rig.snapshot();
    assert(snapshot.phase == Phase::VERIFYING && !snapshot.output_requested && !joint.available());
    assert(is_register_request(only_sent(rig.bench), 1U, Dm::RegisterOperation::READ, Dm::Register::CONTROL_MODE));
    rig.cycle(VERIFY_ROUNDS);
    assert(rig.snapshot().phase == Phase::READY && motor.timeout == 1000U);
    assert(!rig.snapshot().output_requested && joint.available()); // 不自动重新使能
}

void test_link_timeout()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Rig rig{one_motor()};
    rig.bench.add(motor);
    assert(rig.start());
    verify(rig);
    enable_and_confirm(rig);

    // 最后一帧反馈的接收时刻起算 50 ms（Decision-20261102-04）：48 轮后还在线，再两轮判出离线
    motor.powered = false;
    rig.cycle(48U);
    DmMotor::Snapshot snapshot = rig.snapshot();
    assert(snapshot.phase == Phase::READY && snapshot.output_active && snapshot.online);
    rig.cycle(2U);
    snapshot = rig.snapshot();
    assert(snapshot.phase == Phase::VERIFYING && !snapshot.output_requested && !snapshot.online);
    assert(snapshot.has_feedback && !rig.joints[0].state().online); // 最后的物理量保留
    assert(is_register_request(only_sent(rig.bench), 1U, Dm::RegisterOperation::READ, Dm::Register::CONTROL_MODE));
    motor.powered = true;
    rig.cycle(VERIFY_ROUNDS + 20U);
    assert(rig.snapshot().phase == Phase::READY && rig.snapshot().online);
}

void test_position_wrap_and_mechanics()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    DmMotor::Config config = one_motor();
    config.motors[0].mechanics = JointMechanics{-1, 2.0F, 0.5F, 1000000U}; // 反向、外接 2:1、零点 0.5 rad
    Rig rig{config};
    rig.bench.add(motor);
    Joint &joint = rig.joints[0];
    assert(rig.start());
    verify(rig); // 第一帧：32767（约 0 rad）
    constexpr float RADIANS_PER_COUNT{25.0F / 65535.0F};

    motor.position = 65000U;
    rig.cycle(2U);
    motor.position = 500U; // 越过 +PMAX 回绕到 −PMAX 附近：继续往正方向累计
    rig.cycle(2U);
    const float unwrapped = static_cast<float>(65536 + 500) * RADIANS_PER_COUNT - 12.5F;
    assert(near(rig.snapshot().position_rad, unwrapped, 1e-4F));
    assert(near(joint.state().position_rad, -(unwrapped - 0.5F) / 2.0F, 1e-4F)); // 关节：−(p − 0.5) / 2

    // 阻抗的位置给定：关节 q → 电机 p = 0.5 − 2q，再按电机当前所在的那一圈换算回 ±PMAX
    enable_and_confirm(rig);
    assert(joint.set_impedance(-6.0F, 0.0F, 4.0F, 0.0F, 0.0F)); // 电机 p = 12.5（已越过 +PMAX 一圈）
    rig.cycle();
    const hal::Can::Frame &frame = only_sent(rig.bench);
    const auto position_raw = static_cast<std::uint16_t>((frame.data[0] << 8U) | frame.data[1]);
    const float native = Dm::from_unsigned(position_raw, -12.5F, 12.5F, 16U);
    const float wrap = static_cast<float>(65536) * RADIANS_PER_COUNT;
    assert(near(native, 12.5F - wrap, 0.001F)); // 换算回电机的量程
    const auto kp_raw = static_cast<std::uint16_t>(((frame.data[3] & 0x0FU) << 8U) | frame.data[4]);
    assert(near(Dm::from_unsigned(kp_raw, 0.0F, 500.0F, 12U), 1.0F, 0.2F)); // 关节刚度 4 → 电机 4 / 2² = 1
    motor.position = 65000U;                                                // 反向越回
    rig.cycle(2U);
    assert(near(rig.snapshot().position_rad, static_cast<float>(65000) * RADIANS_PER_COUNT - 12.5F, 1e-4F));
}

void test_shared_master_id()
{
    // 两台电机都用出厂的 MST_ID 0：按 D0–D1（应答）与 D0 低 4 位（反馈）分开
    reset();
    FakeMotor first{BUS, 1U, 0x000U};
    FakeMotor second{BUS, 2U, 0x000U};
    first.position = 40000U;
    second.position = 20000U;
    DmMotor::Config config{};
    config.count = 2U;
    config.motors[0] = motor_of(1U, 0x000U);
    config.motors[1] = motor_of(2U, 0x000U);
    Rig rig{config};
    rig.bench.add(first);
    rig.bench.add(second);
    assert(rig.start());
    rig.cycle(VERIFY_ROUNDS + 1U);
    assert(rig.bench.sent_count() == 2U); // 两台各一帧保活
    const DmMotor::Snapshot one = rig.snapshot(0U);
    const DmMotor::Snapshot two = rig.snapshot(1U);
    assert(one.phase == Phase::READY && two.phase == Phase::READY);
    assert(one.esc_id == 1U && two.esc_id == 2U);
    assert(one.position_rad > 2.0F && two.position_rad < -4.0F);
    assert(first.timeout == 1000U && second.timeout == 1000U);

    // 只使能第二台
    assert(rig.joints[1].enable() && rig.joints[1].set_torque(1.0F));
    rig.cycle(4U);
    assert(!rig.snapshot(0U).output_active && rig.snapshot(1U).output_active);
    assert(first.state == Dm::State::DISABLED && second.state == Dm::State::ENABLED && second.has_mit);
    assert(rig.statistics().unknown_frame_count == 0U);
}

void test_unknown_frames_and_reinit()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Rig rig{one_motor()};
    rig.bench.add(motor);
    assert(rig.start());
    verify(rig);
    dm_test::inject(BUS, 0x205U, 8U, Bytes{});                                                       // 别的设备
    dm_test::inject(BUS, 0x011U, 8U, Bytes{0x12U, 0x80U, 0x00U, 0x7FU, 0xF8U, 0x00U, 0x1AU, 0x1CU}); // 别的 ESC_ID
    hal::Can::Frame extended{};
    extended.identifier = 0x11U;
    extended.identifier_type = hal::Can::IdentifierType::EXTENDED;
    extended.length = 8U;
    assert(mock::can::inject(BUS, extended));
    rig.cycle();
    // 不是本组 MST_ID 的帧由 CanBus 计数；MST_ID 对、内容不对的由驱动计数
    assert(rig.statistics().unknown_frame_count == 1U && rig.bus.statistics().unclaimed_count == 2U);

    // 重新 init()：撤销出力，从头核对
    enable_and_confirm(rig);
    assert(rig.driver.init());
    DmMotor::Snapshot snapshot = rig.snapshot();
    assert(snapshot.phase == Phase::VERIFYING && !snapshot.output_requested && !snapshot.has_feedback);
    rig.cycle();
    assert(!rig.joints[0].available() && !rig.joints[0].state().enabled);
    verify(rig);
}

} // namespace

int main()
{
    test_verification_sequence();
    test_retry_and_late_power();
    test_mismatch();
    test_config_validation();
    test_enable_flow();
    test_command_timeout();
    test_stamps_after_round_start();
    test_enable_confirm_guard();
    test_enable_failure();
    test_fault_and_reset_while_enabled();
    test_link_timeout();
    test_position_wrap_and_mechanics();
    test_shared_master_id();
    test_unknown_frames_and_reinit();
    return 0;
}
