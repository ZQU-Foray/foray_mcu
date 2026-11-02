// bringup::DmCheck（Decision-20261102-02）在 platform/mock 上：整个检查流程、跳过转动、没有电机、核对不符、
// 使能失败。电机由 unit/motor/dm_fake_motor.hpp 扮演（含通讯超时与按指令的转速）。
#include "bringup/dm_check.hpp"
#include "unit/motor/dm_fake_motor.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace
{

using bringup::DmCheck;
using device::DmMotor;
using dm_test::Bench;
using dm_test::Dm;
using dm_test::FakeMotor;
using dm_test::is_system_command;
using Stage = DmCheck::Stage;

constexpr std::uint8_t BUS{0U};

// 与 bringup_dm_mc02 的 params.hpp 相同
constexpr DmCheck::Parameters PARAMETERS{
    {1U, 0x11U},
    true,
    0.3F,
    0.3F,
    5000000U,
    500000U,
    200000U,
    1000000U,
    2000000U,
    500000U,
    300000U,
    3000000U,
};

void reset()
{
    mock::reset();
    mock::can::reset();
    mock::set_now_us(1000U);
}

DmMotor::Config motor_config()
{
    return DmMotor::Config{hal::Can{BUS}, DmMotor::Profile::j4310(), {{PARAMETERS.motor}}, 1U};
}

/** 1 ms 一轮跑到结束或失败，最多 rounds 轮；返回跑了多少轮。 */
std::size_t run(DmCheck &check, Bench &bench, std::size_t rounds = 20000U)
{
    std::size_t round = 0U;
    for (; round < rounds; ++round)
    {
        check.step(hal::os::now_us());
        bench.respond();
        mock::advance_us(1000U);
        if (check.report().stage == Stage::DONE || check.report().stage == Stage::FAILED)
        {
            break;
        }
    }
    return round;
}

bool near(float actual, float expected, float tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

void test_full_sequence()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    motor.simulate_motion = true;
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{motor_config()};
    DmCheck check;
    assert(check.init(DmCheck::Config{&driver, PARAMETERS}));
    run(check, bench);

    const DmCheck::Report &report = check.report();
    assert(report.stage == Stage::DONE && report.passed);
    assert(report.verified_after_us < 20000U);
    assert(report.enable_latency_us > 0U && report.enable_latency_us <= 5000U);
    assert(report.zero_torque_drift_rad < 0.001F);
    // 稳态转速 = 0.3 / 0.3 = 1 rad/s，2 s 约转 2 rad
    assert(report.motion_ok &&
           near(report.spin_max_velocity_rad_per_second, 1.0F, 0.05F)); // Kd、力矩、速度都按 12 位量化
    assert(near(report.spin_position_delta_rad, 2.0F, 0.05F));
    assert(near(report.spin_torque_feedback_newton_meter, 0.3F, 0.01F));
    assert(near(report.brake_end_velocity_rad_per_second, 0.0F, 0.05F));
    // 通讯超时：电机按写进去的 50 ms 自己失能并报"通讯丢失"，驱动撤销出力；电机没复位，不必重新核对；
    // 再使能时先清错
    assert(report.silence_fault_state == static_cast<std::uint8_t>(Dm::State::COMMUNICATION_LOST));
    assert(!report.reverified && report.recover_after_us > 0U && report.reenable_latency_us > 0U);
    // 结束：失能，读到的设置与档案一致，保活继续
    assert(report.phase == DmMotor::Phase::READY && report.state == 0U && !report.output_active);
    assert(report.control_mode == 1U && report.timeout_ticks == 1000U && report.mismatch_register == 0U);
    assert(report.position_max_rad == 12.5F && report.velocity_max_rad_per_second == 30.0F &&
           report.torque_max_newton_meter == 10.0F);
    assert(report.tx_queued > 0U && report.tx_rejected == 0U && report.unknown_frames == 0U);
    assert(report.request_timeouts == 0U && report.enable_failures == 0U);
    assert(motor.state == Dm::State::DISABLED);
    const std::uint32_t feedback = report.feedback_count;
    for (std::size_t round = 0U; round < 10U; ++round)
    {
        check.step(hal::os::now_us());
        bench.respond();
        mock::advance_us(1000U);
    }
    assert(check.report().feedback_count == feedback + 10U && check.report().online);
    assert(is_system_command(bench.sent(0U), Dm::SystemCommand::DISABLE));
}

void test_without_spin()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    motor.simulate_motion = true;
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{motor_config()};
    DmCheck::Parameters parameters = PARAMETERS;
    parameters.spin = false;
    DmCheck check;
    assert(check.init(DmCheck::Config{&driver, parameters}));
    run(check, bench);
    const DmCheck::Report &report = check.report();
    assert(report.passed && !report.motion_ok && report.spin_max_velocity_rad_per_second == 0.0F);
    assert(near(report.position_rad, 0.0F, 0.001F)); // 一直没转
}

void test_no_motor()
{
    // 总线上没有电机：停在核对阶段直到超时，只发过寄存器读请求，没发过控制帧
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    motor.powered = false;
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{motor_config()};
    DmCheck check;
    assert(check.init(DmCheck::Config{&driver, PARAMETERS}));
    std::size_t control_frames = 0U;
    for (std::size_t round = 0U; round < 6000U && check.report().stage != Stage::FAILED; ++round)
    {
        check.step(hal::os::now_us());
        bench.respond();
        for (std::size_t index = 0U; index < bench.sent_count(); ++index)
        {
            control_frames += bench.sent(index).identifier == Dm::REGISTER_IDENTIFIER ? 0U : 1U;
        }
        mock::advance_us(1000U);
    }
    const DmCheck::Report &report = check.report();
    assert(report.stage == Stage::FAILED && report.failed_stage == Stage::VERIFY && !report.passed);
    assert(control_frames == 0U && report.request_timeouts > 200U);
}

void test_mismatch()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    motor.torque_max = 7.0F;
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{motor_config()};
    DmCheck check;
    assert(check.init(DmCheck::Config{&driver, PARAMETERS}));
    assert(run(check, bench) < 20U); // 核对一读完就判定
    const DmCheck::Report &report = check.report();
    assert(report.stage == Stage::FAILED && report.failed_stage == Stage::VERIFY);
    assert(report.phase == DmMotor::Phase::MISMATCH && report.mismatch_register == 0x17U);
    assert(report.torque_max_newton_meter == 7.0F);
}

void test_enable_failure()
{
    reset();
    FakeMotor motor{BUS, 1U, 0x11U};
    motor.refuse_enable = true;
    Bench bench{BUS};
    bench.add(motor);
    DmMotor driver{motor_config()};
    DmCheck check;
    assert(check.init(DmCheck::Config{&driver, PARAMETERS}));
    run(check, bench);
    const DmCheck::Report &report = check.report();
    assert(report.stage == Stage::FAILED && report.failed_stage == Stage::ENABLE && report.enable_failures == 1U);
    assert(motor.state == Dm::State::DISABLED && !report.output_active);

    // 驱动初始化失败（总线被占）：init() 返回 false
    DmMotor second{motor_config()};
    DmCheck other;
    assert(!other.init(DmCheck::Config{&second, PARAMETERS}));
    assert(!other.init(DmCheck::Config{nullptr, PARAMETERS}));
}

} // namespace

int main()
{
    test_full_sequence();
    test_without_spin();
    test_no_motor();
    test_mismatch();
    test_enable_failure();
    return 0;
}
