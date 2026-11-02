#include "bringup/dm_check.hpp"

#include <cmath>
#include <cstddef>

namespace bringup
{

namespace
{

using Dm = protocol::DmEsc;
using Motor = device::DmMotor;

constexpr std::size_t MOTOR{0U};
constexpr float MINIMUM_SPIN_VELOCITY{0.05F}; // rad/s：转动时最大速度至少这么大才算动了
constexpr float MINIMUM_SPIN_TRAVEL{0.02F};   // rad：转动时位置至少前进这么多

} // namespace

bool DmCheck::init(const Config &config) noexcept
{
    config_ = config;
    report_ = Report{};
    report_.spin_command_newton_meter = config.parameters.spin_torque_newton_meter;
    report_.spin_damping = config.parameters.damping;
    started_ = false;
    return config.motor != nullptr && config.motor->init();
}

void DmCheck::step(hal::os::Microseconds now) noexcept
{
    if (config_.motor == nullptr)
    {
        return;
    }
    if (!started_)
    {
        started_ = true;
        start_us_ = now;
        enter(Stage::VERIFY, now);
    }

    // 通讯超时一步故意不调 process()：电机收不到帧，应按写进去的超时自己失能
    if (report_.stage != Stage::SILENCE)
    {
        config_.motor->process();
    }
    Motor::Snapshot snapshot{};
    if (!config_.motor->get_snapshot(MOTOR, snapshot))
    {
        return;
    }
    record(snapshot);
    advance(snapshot, now);
}

void DmCheck::advance(const device::DmMotor::Snapshot &snapshot, hal::os::Microseconds now) noexcept
{
    Motor &motor = *config_.motor;
    const Parameters &parameters = config_.parameters;
    const hal::os::Microseconds in_stage = now - stage_start_us_;

    switch (report_.stage)
    {
    case Stage::VERIFY:
        if (snapshot.phase == Motor::Phase::READY)
        {
            report_.verified_after_us = now - start_us_;
            enter(Stage::IDLE, now);
        }
        else if (snapshot.phase == Motor::Phase::MISMATCH || in_stage >= parameters.verify_timeout_us)
        {
            fail();
        }
        break;

    case Stage::IDLE:
        if (in_stage < parameters.idle_us)
        {
            break;
        }
        if (!snapshot.online || !motor.enable(MOTOR))
        {
            fail();
            break;
        }
        enter(Stage::ENABLE, now);
        break;

    case Stage::ENABLE:
        if (snapshot.output_active)
        {
            report_.enable_latency_us = in_stage;
            stage_start_position_rad_ = snapshot.position_rad;
            enter(Stage::ZERO_TORQUE, now);
        }
        else if (snapshot.enable_failed || !snapshot.output_requested || in_stage >= parameters.enable_timeout_us)
        {
            fail();
        }
        break;

    case Stage::ZERO_TORQUE:
        if (!snapshot.output_active || !motor.set_command(MOTOR, Dm::MitCommand{}))
        {
            fail();
            break;
        }
        {
            const float drift = std::fabs(snapshot.position_rad - stage_start_position_rad_);
            report_.zero_torque_drift_rad =
                drift > report_.zero_torque_drift_rad ? drift : report_.zero_torque_drift_rad;
        }
        if (in_stage >= parameters.zero_torque_us)
        {
            stage_start_position_rad_ = snapshot.position_rad;
            enter(parameters.spin ? Stage::SPIN : Stage::SILENCE, now);
        }
        break;

    case Stage::SPIN:
        if (!snapshot.output_active ||
            !motor.set_command(
                MOTOR, Dm::MitCommand{0.0F, 0.0F, 0.0F, parameters.damping, parameters.spin_torque_newton_meter}))
        {
            fail();
            break;
        }
        if (snapshot.velocity_rad_per_second > report_.spin_max_velocity_rad_per_second)
        {
            report_.spin_max_velocity_rad_per_second = snapshot.velocity_rad_per_second;
        }
        if (in_stage >= parameters.spin_us)
        {
            report_.spin_position_delta_rad = snapshot.position_rad - stage_start_position_rad_;
            report_.spin_torque_feedback_newton_meter = snapshot.torque_newton_meter;
            report_.motion_ok = report_.spin_max_velocity_rad_per_second >= MINIMUM_SPIN_VELOCITY &&
                                report_.spin_position_delta_rad >= MINIMUM_SPIN_TRAVEL;
            enter(Stage::BRAKE, now);
        }
        break;

    case Stage::BRAKE:
        if (!snapshot.output_active ||
            !motor.set_command(MOTOR, Dm::MitCommand{0.0F, 0.0F, 0.0F, parameters.damping, 0.0F}))
        {
            fail();
            break;
        }
        if (in_stage >= parameters.brake_us)
        {
            report_.brake_end_velocity_rad_per_second = snapshot.velocity_rad_per_second;
            enter(Stage::SILENCE, now);
        }
        break;

    case Stage::SILENCE:
        if (in_stage >= parameters.silence_us)
        {
            stage_start_feedback_count_ = snapshot.feedback_count;
            enter(Stage::RECOVER, now);
        }
        break;

    case Stage::RECOVER:
        if (snapshot.phase != Motor::Phase::READY)
        {
            report_.reverified = true;
        }
        if (report_.silence_fault_state == 0U && Motor::is_fault(snapshot.state))
        {
            report_.silence_fault_state = static_cast<std::uint8_t>(snapshot.state);
        }
        // 停发期间留在接收队列里的那帧旧反馈先被取出，之后的反馈才是停发以后的：电机应报"通讯丢失"，
        // 驱动随之撤销出力。电机没有复位，RAM 里的设置还在，驱动不必重新核对（复位过的话会重新核对）
        if (snapshot.phase == Motor::Phase::READY && !snapshot.output_requested && snapshot.online &&
            snapshot.feedback_count >= stage_start_feedback_count_ + 2U)
        {
            report_.recover_after_us = in_stage;
            if (!motor.enable(MOTOR))
            {
                fail();
                break;
            }
            enter(Stage::REENABLE, now);
        }
        else if (snapshot.phase == Motor::Phase::MISMATCH || in_stage >= parameters.recover_timeout_us)
        {
            fail();
        }
        break;

    case Stage::REENABLE:
        if (snapshot.output_active)
        {
            report_.reenable_latency_us = in_stage;
            (void)motor.disable(MOTOR);
            enter(Stage::DISABLE, now);
        }
        else if (snapshot.enable_failed || !snapshot.output_requested || in_stage >= parameters.enable_timeout_us)
        {
            fail();
        }
        break;

    case Stage::DISABLE:
        // disable() 之后的第一帧失能保活的反馈才会是"失能"
        if (!snapshot.output_requested && snapshot.state == Dm::State::DISABLED)
        {
            report_.passed = true;
            enter(Stage::DONE, now);
        }
        else if (in_stage >= parameters.enable_timeout_us)
        {
            fail();
        }
        break;

    case Stage::DONE:
    case Stage::FAILED:
        break;
    }
}

void DmCheck::record(const device::DmMotor::Snapshot &snapshot) noexcept
{
    report_.phase = snapshot.phase;
    report_.state = static_cast<std::uint8_t>(snapshot.state);
    report_.online = snapshot.online;
    report_.output_active = snapshot.output_active;
    report_.position_rad = snapshot.position_rad;
    report_.velocity_rad_per_second = snapshot.velocity_rad_per_second;
    report_.torque_newton_meter = snapshot.torque_newton_meter;
    report_.mos_temperature_celsius = snapshot.mos_temperature_celsius;
    report_.rotor_temperature_celsius = snapshot.rotor_temperature_celsius;
    report_.feedback_count = snapshot.feedback_count;
    // 重新核对时读到的值会先清零再读，保留最近一次读全的
    if (snapshot.timeout_ticks != 0U || snapshot.mismatch_register != 0U)
    {
        report_.control_mode = snapshot.control_mode;
        report_.position_max_rad = snapshot.limits.position_rad;
        report_.velocity_max_rad_per_second = snapshot.limits.velocity_rad_per_second;
        report_.torque_max_newton_meter = snapshot.limits.torque_newton_meter;
        report_.timeout_ticks = snapshot.timeout_ticks;
        report_.mismatch_register = snapshot.mismatch_register;
    }

    Motor::Statistics statistics{};
    if (config_.motor->get_statistics(statistics))
    {
        const std::size_t queued = static_cast<std::size_t>(hal::Can::SendResult::QUEUED);
        std::uint32_t rejected = 0U;
        for (std::size_t index = 0U; index < statistics.tx_result_counts.size(); ++index)
        {
            rejected += index == queued ? 0U : statistics.tx_result_counts[index];
        }
        report_.tx_queued = statistics.tx_result_counts[queued];
        report_.tx_rejected = rejected;
        report_.register_replies = statistics.register_reply_count;
        report_.stale_replies = statistics.stale_reply_count;
        report_.request_timeouts = statistics.request_timeout_count;
        report_.enable_failures = statistics.enable_failure_count;
        report_.unknown_frames = statistics.unknown_frame_count;
        report_.rx_dropped = statistics.rx_dropped_count;
        report_.bus_off = statistics.bus_off_count;
    }
}

void DmCheck::enter(Stage stage, hal::os::Microseconds now) noexcept
{
    report_.stage = stage;
    stage_start_us_ = now;
}

void DmCheck::fail() noexcept
{
    report_.failed_stage = report_.stage;
    report_.stage = Stage::FAILED;
    (void)config_.motor->disable(MOTOR);
}

} // namespace bringup
