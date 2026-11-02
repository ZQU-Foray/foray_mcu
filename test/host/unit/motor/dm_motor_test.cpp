// device::DmMotor 在 platform/mock 上：启动回读核对、使能、反馈、故障与离线（Decision-20261102-01）。
// 电机由 dm_fake_motor.hpp 按说明书的帧格式扮演；ESC_ID 1、MST_ID 0x11 与 2026-11-01 实机的设置相同。
#include "driver/motor/dm/dm_motor.hpp"
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
using dm_test::Bench;
using dm_test::Bytes;
using dm_test::cycle;
using dm_test::Dm;
using dm_test::FakeMotor;
using dm_test::is_register_request;
using dm_test::is_system_command;
using Phase = DmMotor::Phase;

constexpr std::uint8_t BUS{0U};
constexpr std::size_t VERIFY_ROUNDS{7U}; // 读模式、失能、三个量程、写超时，第 7 轮收到回显

void reset()
{
    mock::reset();
    mock::can::reset();
    mock::set_now_us(1000U);
}

DmMotor::Config one_motor(std::uint16_t esc_id = 1U, std::uint16_t master_id = 0x11U)
{
    DmMotor::Config config{hal::Can{BUS}, DmMotor::Profile::j4310(), {}, 1U};
    config.motors[0] = {esc_id, master_id};
    return config;
}

DmMotor::Snapshot snapshot_of(const DmMotor &driver, std::size_t index = 0U)
{
    DmMotor::Snapshot snapshot{};
    assert(driver.get_snapshot(index, snapshot));
    return snapshot;
}

DmMotor::Statistics statistics_of(const DmMotor &driver)
{
    DmMotor::Statistics statistics{};
    assert(driver.get_statistics(statistics));
    return statistics;
}

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
void verify(DmMotor &driver, Bench &bench)
{
    cycle(driver, bench, VERIFY_ROUNDS + 1U);
    assert(snapshot_of(driver).phase == Phase::READY);
}

/** enable() 并走到电机确认使能：清除错误、使能，第 3 轮开始发 MIT。 */
void enable_and_confirm(DmMotor &driver, Bench &bench, std::size_t index = 0U)
{
    assert(driver.enable(index));
    cycle(driver, bench, 3U);
    assert(snapshot_of(driver, index).output_active);
}

void test_verification_sequence()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{one_motor()};
    assert(snapshot_of(driver).phase == Phase::STOPPED);
    driver.process(); // 没有 init() 不发送
    assert(mock::can::sent_count(BUS) == 0U);
    assert(driver.init());
    assert(snapshot_of(driver).phase == Phase::VERIFYING);
    assert(!driver.can_enable(0U) && !driver.enable(0U));

    cycle(driver, bench);
    assert(is_register_request(only_sent(bench), 1U, Dm::RegisterOperation::READ, Dm::Register::CONTROL_MODE));
    assert(only_sent(bench).length == 4U);
    cycle(driver, bench);
    assert(sent_system(bench, 0x001U, Dm::SystemCommand::DISABLE)); // 按读到的模式（MIT）发失能
    cycle(driver, bench);
    assert(is_register_request(only_sent(bench), 1U, Dm::RegisterOperation::READ, Dm::Register::POSITION_MAX));
    cycle(driver, bench);
    assert(is_register_request(only_sent(bench), 1U, Dm::RegisterOperation::READ, Dm::Register::VELOCITY_MAX));
    cycle(driver, bench);
    assert(is_register_request(only_sent(bench), 1U, Dm::RegisterOperation::READ, Dm::Register::TORQUE_MAX));
    cycle(driver, bench);
    // 档案的 50 ms = 1000 个 50 µs 计数，写进电机 RAM
    const hal::Can::Frame &write = only_sent(bench);
    assert(is_register_request(write, 1U, Dm::RegisterOperation::WRITE, Dm::Register::TIMEOUT));
    assert(write.length == 8U && write.data[4] == 0xE8U && write.data[5] == 0x03U && write.data[6] == 0U);
    assert(motor.timeout == 1000U);
    assert(snapshot_of(driver).phase == Phase::VERIFYING);

    cycle(driver, bench); // 收到回显：READY，没有请求出力，发失能作保活
    DmMotor::Snapshot snapshot = snapshot_of(driver);
    assert(snapshot.phase == Phase::READY && driver.can_enable(0U));
    assert(snapshot.control_mode == 1U && snapshot.timeout_ticks == 1000U && snapshot.mismatch_register == 0U);
    assert(snapshot.limits.position_rad == 12.5F && snapshot.limits.velocity_rad_per_second == 30.0F &&
           snapshot.limits.torque_newton_meter == 10.0F);
    assert(sent_system(bench, 0x001U, Dm::SystemCommand::DISABLE));
    assert(!snapshot.has_feedback); // 核对时的反馈不换算

    motor.velocity = 0x732U; // −3 rad/s
    motor.torque = 0x9FFU;   // 2.5 N·m
    cycle(driver, bench);
    snapshot = snapshot_of(driver);
    assert(snapshot.has_feedback && snapshot.online && snapshot.state == Dm::State::DISABLED);
    assert(near(snapshot.position_rad, 0.0F, 0.001F) && snapshot.position_from_first_feedback_rad == 0.0F);
    assert(snapshot.mos_temperature_celsius == 30U && snapshot.rotor_temperature_celsius == 35U);
    assert(sent_system(bench, 0x001U, Dm::SystemCommand::DISABLE));
    cycle(driver, bench);
    snapshot = snapshot_of(driver);
    assert(near(snapshot.velocity_rad_per_second, -3.0F, 0.02F) && near(snapshot.torque_newton_meter, 2.5F, 0.01F));

    const DmMotor::Statistics statistics = statistics_of(driver);
    assert(statistics.register_reply_count == 5U && statistics.request_timeout_count == 0U);
    assert(statistics.unknown_frame_count == 0U && statistics.stale_reply_count == 0U);
    assert(statistics.tx_result_counts[static_cast<std::size_t>(hal::Can::SendResult::QUEUED)] == 9U);
    assert(near(driver.maximum_torque_newton_meter(), 10.0F, 0.0F)); // 峰值 12.5 与 TMAX 10 取小
}

void test_retry_and_late_power()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    motor.powered = false;
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{one_motor()};
    assert(driver.init());

    // 没有应答：每 20 ms 重发一次
    std::size_t sent = 0U;
    for (std::size_t round = 0U; round < 45U; ++round)
    {
        cycle(driver, bench);
        sent += bench.sent_count();
    }
    assert(sent == 3U && statistics_of(driver).request_timeout_count == 2U);

    // 发送队列满：不算发出，下一轮就重试，不等 20 ms
    mock::can::set_send_result(BUS, hal::Can::SendResult::QUEUE_FULL);
    mock::advance_us(DmMotor::REQUEST_TIMEOUT_US);
    cycle(driver, bench, 3U);
    assert(statistics_of(driver).tx_result_counts[static_cast<std::size_t>(hal::Can::SendResult::QUEUE_FULL)] == 3U);
    mock::can::set_send_result(BUS, hal::Can::SendResult::QUEUED);

    // 电机上电后接上
    motor.powered = true;
    cycle(driver, bench);
    assert(is_register_request(only_sent(bench), 1U, Dm::RegisterOperation::READ, Dm::Register::CONTROL_MODE));
    verify(driver, bench);

    // 重发造成的重复应答：丢弃，不当成反馈
    const DmMotor::Snapshot before = snapshot_of(driver);
    dm_test::inject(BUS, 0x11U, 8U, Bytes{0x01U, 0x00U, 0x55U, 0x09U, 0xE8U, 0x03U, 0x00U, 0x00U});
    dm_test::inject(BUS, 0x11U, 8U, Bytes{0x01U, 0x00U, 0x33U, 0x0AU, 0x01U, 0x00U, 0x00U, 0x00U});
    cycle(driver, bench);
    const DmMotor::Snapshot after = snapshot_of(driver);
    assert(statistics_of(driver).stale_reply_count == 2U);
    assert(after.phase == Phase::READY && after.feedback_count == before.feedback_count + 1U);
    assert(after.position_rad == before.position_rad);
}

/** 用给定的电机设置走完核对，返回最终快照。 */
DmMotor::Snapshot verify_with(FakeMotor &motor, std::size_t rounds = VERIFY_ROUNDS + 2U)
{
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{one_motor()};
    assert(driver.init());
    cycle(driver, bench, rounds);
    const DmMotor::Snapshot snapshot = snapshot_of(driver);
    if (snapshot.phase == Phase::MISMATCH)
    {
        assert(!driver.can_enable(0U) && !driver.enable(0U) && !driver.set_torque(0U, 1.0F));
        cycle(driver, bench, 5U); // 核对不符：不再发任何帧
        assert(bench.sent_count() == 0U && mock::can::sent_count(BUS) == 0U);
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
        DmMotor driver{config};
        assert(!driver.init());
        driver.process();
        assert(mock::can::sent_count(BUS) == 0U);
        assert(!driver.enable(0U) && driver.maximum_torque_newton_meter() == 0.0F);
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
    config.motors[1] = {1U, 0x12U}; // ESC_ID 重复
    rejects(config);
    config.motors[1] = {2U, 0x201U}; // 低 8 位等于第一台的 ESC_ID
    rejects(config);
    config = one_motor();
    config.profile.limits.position_rad = 0.0F;
    rejects(config);
    config = one_motor();
    config.profile.peak_torque_newton_meter = std::numeric_limits<float>::quiet_NaN();
    rejects(config);
    config = one_motor();
    config.profile.communication_timeout_us = DmMotor::TIMEOUT_TICK_US - 1U;
    rejects(config);
    mock::can::set_ready(BUS, false);
    rejects(one_motor());
    mock::can::set_ready(BUS, true);

    // 失败的实例没有占住总线；第二个实例占不到
    DmMotor owner{one_motor()};
    assert(owner.init() && owner.init()); // 重复 init() 不再占用
    DmMotor second{one_motor(2U, 0x12U)};
    assert(!second.init());
    assert(owner.count() == 1U);
    DmMotor::Snapshot snapshot{};
    assert(!owner.get_snapshot(1U, snapshot) && !owner.enable(1U) && !owner.disable(1U));
}

void test_enable_flow()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{one_motor()};
    assert(driver.init());
    verify(driver, bench);

    assert(!driver.set_torque(0U, 1.0F)); // 没有请求出力
    assert(driver.enable(0U));
    assert(driver.set_torque(0U, 2.5F));
    cycle(driver, bench);
    assert(sent_system(bench, 0x001U, Dm::SystemCommand::CLEAR_ERROR));
    cycle(driver, bench);
    assert(sent_system(bench, 0x001U, Dm::SystemCommand::ENABLE));
    assert(motor.state == Dm::State::ENABLED && !snapshot_of(driver).output_active);
    cycle(driver, bench); // 收到"使能"反馈，开始发指令
    DmMotor::Snapshot snapshot = snapshot_of(driver);
    assert(snapshot.output_requested && snapshot.output_active && snapshot.state == Dm::State::ENABLED);
    assert(only_sent(bench).identifier == 0x001U);
    assert((only_sent(bench).data == Bytes{0x7FU, 0xFFU, 0x7FU, 0xF0U, 0x00U, 0x00U, 0x09U, 0xFFU}));
    assert(driver.enable(0U)); // 已经请求过：不重新清错
    cycle(driver, bench);
    assert(motor.has_mit && motor.last_system_command == Dm::SystemCommand::ENABLE);

    // 完整的 MIT 指令；非有限值清零指令
    assert(driver.set_command(0U, Dm::MitCommand{1.0F, -3.0F, 50.0F, 1.0F, 2.5F}));
    cycle(driver, bench);
    assert((only_sent(bench).data == Bytes{0x8AU, 0x3CU, 0x73U, 0x21U, 0x99U, 0x33U, 0x39U, 0xFFU}));
    assert(!driver.set_torque(0U, std::numeric_limits<float>::infinity()));
    cycle(driver, bench);
    assert((only_sent(bench).data == Bytes{0x7FU, 0xFFU, 0x7FU, 0xF0U, 0x00U, 0x00U, 0x07U, 0xFFU}));
    assert(driver.set_torque(0U, -2.5F));
    driver.clear_all_commands(); // 只清指令，不撤销出力
    cycle(driver, bench);
    assert(snapshot_of(driver).output_active && only_sent(bench).data[7] == 0xFFU && only_sent(bench).data[6] == 0x07U);

    // 失能：撤销请求、清零指令，之后每周期发失能
    assert(driver.set_torque(0U, 1.0F));
    assert(driver.disable(0U));
    assert(!driver.set_torque(0U, 1.0F));
    cycle(driver, bench);
    assert(sent_system(bench, 0x001U, Dm::SystemCommand::DISABLE) && motor.state == Dm::State::DISABLED);
    cycle(driver, bench);
    snapshot = snapshot_of(driver);
    assert(!snapshot.output_requested && !snapshot.output_active && snapshot.phase == Phase::READY);

    // 再使能时指令从零开始
    enable_and_confirm(driver, bench);
    assert(only_sent(bench).data[6] == 0x07U && only_sent(bench).data[7] == 0xFFU);
}

void test_enable_confirm_guard()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{one_motor()};
    assert(driver.init());
    verify(driver, bench);
    assert(driver.enable(0U));
    cycle(driver, bench);
    assert(sent_system(bench, 0x001U, Dm::SystemCommand::CLEAR_ERROR));
    // 清除错误之前发出的帧，其"使能"反馈晚到：不算确认
    motor.state = Dm::State::ENABLED;
    motor.send_feedback();
    motor.state = Dm::State::DISABLED;
    cycle(driver, bench);
    assert(!snapshot_of(driver).output_active);
    assert(sent_system(bench, 0x001U, Dm::SystemCommand::ENABLE));
}

void test_enable_failure()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{one_motor()};
    assert(driver.init());
    verify(driver, bench);

    // 电机一直不使能：50 ms 后放弃
    motor.refuse_enable = true;
    assert(driver.enable(0U));
    cycle(driver, bench, 49U);
    assert(snapshot_of(driver).output_requested && sent_system(bench, 0x001U, Dm::SystemCommand::ENABLE));
    cycle(driver, bench, 2U);
    DmMotor::Snapshot snapshot = snapshot_of(driver);
    assert(!snapshot.output_requested && snapshot.enable_failed && !driver.set_torque(0U, 1.0F));
    assert(sent_system(bench, 0x001U, Dm::SystemCommand::DISABLE));
    assert(statistics_of(driver).enable_failure_count == 1U);

    // 故障清不掉（一直欠压）：守护时间之后看到故障就放弃，不等 50 ms
    motor.refuse_enable = false;
    motor.state = Dm::State::UNDER_VOLTAGE;
    motor.fault_clearable = false;
    cycle(driver, bench);
    assert(driver.enable(0U) && !snapshot_of(driver).enable_failed);
    cycle(driver, bench, 4U);
    snapshot = snapshot_of(driver);
    assert(!snapshot.output_requested && snapshot.enable_failed && snapshot.state == Dm::State::UNDER_VOLTAGE);
    assert(DmMotor::is_fault(snapshot.state) && statistics_of(driver).enable_failure_count == 2U);

    // 故障消失后再 enable()：先清错，再使能
    motor.fault_clearable = true;
    enable_and_confirm(driver, bench);
    assert(!snapshot_of(driver).enable_failed && motor.state == Dm::State::ENABLED);
}

void test_fault_and_reset_while_enabled()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{one_motor()};
    assert(driver.init());
    verify(driver, bench);
    enable_and_confirm(driver, bench);
    assert(driver.set_torque(0U, 1.0F));

    // 运行中报故障：撤销请求，改发失能，不自动重试
    motor.state = Dm::State::MOS_OVER_TEMPERATURE;
    cycle(driver, bench, 2U);
    DmMotor::Snapshot snapshot = snapshot_of(driver);
    assert(!snapshot.output_requested && !snapshot.output_active && snapshot.phase == Phase::READY);
    assert(snapshot.state == Dm::State::MOS_OVER_TEMPERATURE && !driver.set_torque(0U, 1.0F));
    assert(sent_system(bench, 0x001U, Dm::SystemCommand::DISABLE));
    cycle(driver, bench, 10U);
    assert(motor.state == Dm::State::MOS_OVER_TEMPERATURE); // 驱动不清错
    enable_and_confirm(driver, bench);                      // 能力再次 enable()：清错后恢复

    // 运行中电机自己变成失能（复位过）：撤销请求并重新核对，RAM 里的超时要重写
    motor.state = Dm::State::DISABLED;
    motor.timeout = 0U;
    cycle(driver, bench, 2U);
    snapshot = snapshot_of(driver);
    assert(snapshot.phase == Phase::VERIFYING && !snapshot.output_requested);
    assert(is_register_request(only_sent(bench), 1U, Dm::RegisterOperation::READ, Dm::Register::CONTROL_MODE));
    cycle(driver, bench, VERIFY_ROUNDS);
    assert(snapshot_of(driver).phase == Phase::READY && motor.timeout == 1000U);
    assert(!snapshot_of(driver).output_requested); // 不自动重新使能
}

void test_link_timeout()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{one_motor()};
    assert(driver.init());
    verify(driver, bench);
    enable_and_confirm(driver, bench);

    motor.powered = false;
    cycle(driver, bench, 49U);
    DmMotor::Snapshot snapshot = snapshot_of(driver);
    assert(snapshot.phase == Phase::READY && snapshot.output_active && snapshot.online);
    cycle(driver, bench, 2U);
    snapshot = snapshot_of(driver);
    assert(snapshot.phase == Phase::VERIFYING && !snapshot.output_requested && !snapshot.online);
    assert(snapshot.has_feedback); // 最后的物理量保留
    assert(is_register_request(only_sent(bench), 1U, Dm::RegisterOperation::READ, Dm::Register::CONTROL_MODE));
    motor.powered = true;
    cycle(driver, bench, VERIFY_ROUNDS + 20U);
    assert(snapshot_of(driver).phase == Phase::READY && snapshot_of(driver).online);
}

void test_position_wrap()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{one_motor()};
    assert(driver.init());
    verify(driver, bench); // 第一帧：32767
    constexpr float RADIANS_PER_COUNT{25.0F / 65535.0F};

    motor.position = 65000U;
    cycle(driver, bench, 2U);
    motor.position = 500U; // 越过 +PMAX 回绕到 −PMAX 附近：继续往正方向累计
    cycle(driver, bench, 2U);
    DmMotor::Snapshot snapshot = snapshot_of(driver);
    assert(near(
        snapshot.position_from_first_feedback_rad, static_cast<float>(65536 + 500 - 32767) * RADIANS_PER_COUNT, 1e-4F));
    assert(near(snapshot.position_rad, static_cast<float>(65536 + 500) * RADIANS_PER_COUNT - 12.5F, 1e-4F));
    motor.position = 65000U; // 反向越回
    cycle(driver, bench, 2U);
    snapshot = snapshot_of(driver);
    assert(
        near(snapshot.position_from_first_feedback_rad, static_cast<float>(65000 - 32767) * RADIANS_PER_COUNT, 1e-4F));
}

void test_shared_master_id()
{
    // 两台电机都用出厂的 MST_ID 0：按 D0–D1（应答）与 D0 低 4 位（反馈）分开
    reset();
    FakeMotor first{BUS, 1U, 0x000U};
    FakeMotor second{BUS, 2U, 0x000U};
    first.position = 40000U;
    second.position = 20000U;
    Bench bench{BUS};
    bench.add(first);
    bench.add(second);
    DmMotor::Config config{hal::Can{BUS}, DmMotor::Profile::j4310(), {}, 2U};
    config.motors[0] = {1U, 0x000U};
    config.motors[1] = {2U, 0x000U};
    DmMotor driver{config};
    assert(driver.init());
    cycle(driver, bench, VERIFY_ROUNDS + 1U);
    assert(bench.sent_count() == 2U); // 两台各一帧保活
    const DmMotor::Snapshot one = snapshot_of(driver, 0U);
    const DmMotor::Snapshot two = snapshot_of(driver, 1U);
    assert(one.phase == Phase::READY && two.phase == Phase::READY);
    assert(one.motor.esc_id == 1U && two.motor.esc_id == 2U);
    assert(one.position_rad > 2.0F && two.position_rad < -4.0F);
    assert(first.timeout == 1000U && second.timeout == 1000U);

    // 只使能第二台
    assert(driver.enable(1U) && driver.set_torque(1U, 1.0F));
    cycle(driver, bench, 4U);
    assert(!snapshot_of(driver, 0U).output_active && snapshot_of(driver, 1U).output_active);
    assert(first.state == Dm::State::DISABLED && second.state == Dm::State::ENABLED && second.has_mit);
    assert(statistics_of(driver).unknown_frame_count == 0U);
}

void test_unknown_frames_and_reinit()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{one_motor()};
    assert(driver.init());
    verify(driver, bench);
    dm_test::inject(BUS, 0x205U, 8U, Bytes{});                                                       // 别的设备
    dm_test::inject(BUS, 0x011U, 8U, Bytes{0x12U, 0x80U, 0x00U, 0x7FU, 0xF8U, 0x00U, 0x1AU, 0x1CU}); // 别的 ESC_ID
    hal::Can::Frame extended{};
    extended.identifier = 0x11U;
    extended.identifier_type = hal::Can::IdentifierType::EXTENDED;
    extended.length = 8U;
    assert(mock::can::inject(BUS, extended));
    cycle(driver, bench);
    assert(statistics_of(driver).unknown_frame_count == 3U);

    // 重新 init()：撤销出力，从头核对
    enable_and_confirm(driver, bench);
    assert(driver.init());
    DmMotor::Snapshot snapshot = snapshot_of(driver);
    assert(snapshot.phase == Phase::VERIFYING && !snapshot.output_requested && !snapshot.has_feedback);
    verify(driver, bench);
}

} // namespace

int main()
{
    test_verification_sequence();
    test_retry_and_late_power();
    test_mismatch();
    test_config_validation();
    test_enable_flow();
    test_enable_confirm_guard();
    test_enable_failure();
    test_fault_and_reset_while_enabled();
    test_link_timeout();
    test_position_wrap();
    test_shared_master_id();
    test_unknown_frames_and_reinit();
    return 0;
}
