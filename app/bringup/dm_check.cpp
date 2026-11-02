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
constexpr float MINIMUM_SPIN_VELOCITY{0.05F};     // rad/s：转动时最大速度至少这么大才算动了
constexpr float MINIMUM_SPIN_TRAVEL{0.02F};       // rad：转动时位置至少前进这么多
constexpr hal::os::Microseconds PERIOD_US{1000U}; // 基准按 1 kHz 计抖动
constexpr hal::os::Microseconds LATE_PERIOD_US{1100U};

hal::os::Microseconds distance(hal::os::Microseconds a, hal::os::Microseconds b)
{
    return a > b ? a - b : b - a;
}

} // namespace

bool DmCheck::init(const Config &config) noexcept
{
    config_ = config;
    report_ = Report{};
    report_.spin_command_newton_meter = config.parameters.spin_torque_newton_meter;
    report_.spin_damping = config.parameters.damping;
    report_.bench_duration_us = config.parameters.benchmark ? config.parameters.benchmark_us : 0U;
    started_ = false;
    return config.bus != nullptr && config.motor != nullptr && config.joint != nullptr && config.motor->init();
}

void DmCheck::step(hal::os::Microseconds now) noexcept
{
    if (config_.bus == nullptr || config_.motor == nullptr || config_.joint == nullptr)
    {
        return;
    }
    if (!started_)
    {
        started_ = true;
        start_us_ = now;
        enter(Stage::VERIFY, now);
    }

    // 收 → 算 → 发在同一轮里（Decision-20261102-03）。通讯超时一步故意两头都不做：电机收不到帧，应按写进去的超时自己失能
    const bool silent = report_.stage == Stage::SILENCE;
    if (!silent)
    {
        config_.bus->receive();
    }
    bench_measured_ = false;
    Motor::Snapshot snapshot{};
    if (!config_.motor->get_snapshot(MOTOR, snapshot))
    {
        return;
    }
    record(snapshot);
    advance(snapshot, now);
    if (!silent && report_.stage != Stage::SILENCE)
    {
        config_.bus->transmit(now);
    }
    if (bench_measured_)
    {
        finish_measure(now, hal::os::now_us());
    }
}

void DmCheck::measure(hal::os::Microseconds now, hal::os::Microseconds computed_us) noexcept
{
    if (bench_previous_)
    {
        const hal::os::Microseconds jitter = distance(now - bench_previous_start_us_, PERIOD_US);
        report_.bench_jitter_max_us = jitter > report_.bench_jitter_max_us ? jitter : report_.bench_jitter_max_us;
        report_.bench_late_cycles += now - bench_previous_start_us_ > LATE_PERIOD_US ? 1U : 0U;
    }
    bench_feedback_us_ = config_.joint->state().stamp_us;
    const hal::os::Microseconds age = computed_us - bench_feedback_us_;
    report_.bench_feedback_age_max_us =
        age > report_.bench_feedback_age_max_us ? age : report_.bench_feedback_age_max_us;
    bench_previous_ = true;
    bench_previous_start_us_ = now;
    bench_measured_ = true;
    ++report_.bench_cycles;
}

void DmCheck::finish_measure(hal::os::Microseconds now, hal::os::Microseconds transmitted_us) noexcept
{
    // 单轮耗时：本轮开始 → 收帧、算 PD、指令放进发送队列
    const hal::os::Microseconds exec = transmitted_us - now;
    report_.bench_exec_max_us = exec > report_.bench_exec_max_us ? exec : report_.bench_exec_max_us;
    report_.bench_exec_sum_us += exec;
    // 延迟：本轮算指令所用反馈的接收时刻 → 这条指令在本轮放进发送队列
    const hal::os::Microseconds latency = transmitted_us - bench_feedback_us_;
    report_.bench_latency_max_us = latency > report_.bench_latency_max_us ? latency : report_.bench_latency_max_us;
    report_.bench_latency_sum_us += latency;
    ++report_.bench_latency_samples;
}

void DmCheck::advance(const device::DmMotor::Snapshot &snapshot, hal::os::Microseconds now) noexcept
{
    device::Joint &joint = *config_.joint;
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
        if (!snapshot.online || !joint.enable())
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
        if (!snapshot.output_active || !joint.set_torque(0.0F))
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
            !joint.set_impedance(0.0F, 0.0F, 0.0F, parameters.damping, parameters.spin_torque_newton_meter))
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
        if (!snapshot.output_active || !joint.set_impedance(0.0F, 0.0F, 0.0F, parameters.damping, 0.0F))
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
        // 停发后：驱动按帧的接收时刻判出离线，撤销出力并重新核对，核对时读到电机报"通讯丢失"。
        // 停发期间留在接收队列里的旧反馈不算数（Decision-20261102-04）；它也计入 feedback_count，所以要多等一帧
        if (snapshot.phase == Motor::Phase::READY && !snapshot.output_requested && snapshot.online &&
            snapshot.feedback_count >= stage_start_feedback_count_ + 2U)
        {
            report_.recover_after_us = in_stage;
            if (!joint.enable())
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
            joint.disable();
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
            if (!parameters.benchmark)
            {
                enter(Stage::DONE, now);
            }
            else if (!joint.enable())
            {
                fail();
            }
            else
            {
                enter(Stage::BENCH_ENABLE, now);
            }
        }
        else if (in_stage >= parameters.enable_timeout_us)
        {
            fail();
        }
        break;

    case Stage::BENCH_ENABLE:
        if (snapshot.output_active)
        {
            bench_target_rad_ = snapshot.position_rad;
            bench_previous_ = false;
            enter(Stage::BENCH, now);
        }
        else if (snapshot.enable_failed || !snapshot.output_requested || in_stage >= parameters.enable_timeout_us)
        {
            fail();
        }
        break;

    case Stage::BENCH:
        if (!snapshot.output_active)
        {
            fail();
            break;
        }
        measure(now, hal::os::now_us());
        {
            const float error = bench_target_rad_ - snapshot.position_rad;
            const float magnitude = std::fabs(error);
            report_.bench_hold_error_max_rad =
                magnitude > report_.bench_hold_error_max_rad ? magnitude : report_.bench_hold_error_max_rad;
            float torque =
                parameters.hold_stiffness * error - parameters.hold_damping * snapshot.velocity_rad_per_second;
            const float limit = parameters.hold_torque_limit_newton_meter;
            torque = torque > limit ? limit : (torque < -limit ? -limit : torque);
            if (!joint.set_torque(torque))
            {
                fail();
                break;
            }
        }
        if (in_stage >= parameters.benchmark_us)
        {
            joint.disable();
            enter(Stage::BENCH_DISABLE, now);
        }
        break;

    case Stage::BENCH_DISABLE:
        if (!snapshot.output_requested && snapshot.state == Dm::State::DISABLED)
        {
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
    }
    const hal::Can::Statistics port = config_.bus->statistics().port;
    report_.rx_dropped = port.rx_dropped_count;
    report_.bus_off = port.bus_off_count;
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
    config_.joint->disable();
}

} // namespace bringup
