#include "gimbal/gimbal.hpp"

#include "utils/math/basic_math.hpp"
#include "utils/math/quaternion.hpp"
#include "utils/math/vector3.hpp"

#include <cmath>
#include <cstdint>

namespace gimbal
{

namespace
{

constexpr float DEFAULT_DT_SECONDS{0.001F};
constexpr float MAXIMUM_DT_US{100000.0F}; // 相邻两份姿态相差超过 100 ms 就不当 dt 用
// pc_link.control.flags 的 bit1：云台子系统的字段有效
constexpr std::uint16_t GIMBAL_FLAG{0x0002U};
// flags 的 bit3：PC 请求急停。只当"PC 目标无效"处理，不当急停：急停走遥控 → safety.stop → 安全门，
// 安全通路要独立于上位机
constexpr std::uint16_t ESTOP_REQUEST_FLAG{0x0008U};
// 机械限位的总宽上限：再宽就绕不出一圈，回绕会把目标往回改
constexpr float MAXIMUM_LIMIT_WIDTH_RAD{2.0F * math::PI + 1.0e-3F};
// 目标有效期的上限：协议的 validity_hint_ms 不超过 50 ms，参数给大了也按这个上限
constexpr hal::os::Microseconds TARGET_AGE_LIMIT_US{200000U};
// pitch 有限位挡着，到不了 ±90°；姿态异常时 yaw_rate 的换算要防止除以 0
constexpr float MINIMUM_COS_PITCH{0.1F};

bool axis_valid(const Gimbal::AxisParameters &axis) noexcept
{
    if (!axis.limited)
    {
        return true;
    }
    return std::isfinite(axis.minimum_rad) && std::isfinite(axis.maximum_rad) && axis.minimum_rad <= axis.maximum_rad &&
           axis.maximum_rad - axis.minimum_rad <= MAXIMUM_LIMIT_WIDTH_RAD;
}

struct EulerAngles
{
    float yaw;
    float pitch;
    float roll;
    float yaw_rate;
    float pitch_rate;
    bool valid;
};

/**
 * @brief 从姿态算 yaw、pitch 与欧拉角速度。
 * @note 本板（IMU）跟着 pitch 一起转，云台系 → 世界系的 ZYX 欧拉角就是云台的世界系姿态。
 *       车上的板子若装在小 yaw 那一级、不跟 pitch 转，pitch 要用 IMU 的 pitch 加 pitch 关节角：只改这个函数。
 */
EulerAngles euler_from_attitude(const topics::imu::Attitude &attitude) noexcept
{
    const math::Quaternion orientation{
        attitude.quaternion[0], attitude.quaternion[1], attitude.quaternion[2], attitude.quaternion[3]};
    const math::Vector3 euler = math::to_euler_zyx_radians(orientation); // x = yaw，y = pitch，z = roll
    EulerAngles angles{};
    angles.yaw = euler.x;
    angles.pitch = euler.y;
    angles.roll = euler.z;
    // 角速度反馈用欧拉角的导数，不直接取陀螺的个别轴：角度与角速度要是同一种量
    //（pitch、roll 大时陀螺的 z、y 与欧拉角速度对不上）
    const float wy = attitude.gyro_rad_per_sec[1];
    const float wz = attitude.gyro_rad_per_sec[2];
    angles.pitch_rate = wy * std::cos(angles.roll) - wz * std::sin(angles.roll);
    const float cos_pitch = std::cos(angles.pitch);
    angles.yaw_rate = std::fabs(cos_pitch) < MINIMUM_COS_PITCH
                          ? 0.0F
                          : (wy * std::sin(angles.roll) + wz * std::cos(angles.roll)) / cos_pitch;
    angles.valid = std::isfinite(angles.yaw) && std::isfinite(angles.pitch) && std::isfinite(angles.roll) &&
                   std::isfinite(angles.yaw_rate) && std::isfinite(angles.pitch_rate);
    return angles;
}

} // namespace

bool Gimbal::init(const Config &config) noexcept
{
    initialized_ = false;
    if (config.yaw == nullptr || config.pitch == nullptr)
    {
        return false;
    }
    const Parameters &parameters = config.parameters;
    if (!axis_valid(parameters.yaw) || !axis_valid(parameters.pitch) ||
        !std::isfinite(parameters.pitch_gravity_newton_meter) || !std::isfinite(parameters.torque_limit_newton_meter) ||
        parameters.torque_limit_newton_meter <= 0.0F || parameters.attitude_timeout_us == 0U ||
        parameters.enable_timeout_us == 0U || parameters.target_timeout_us == 0U)
    {
        return false;
    }
    // 驱动在品牌驱动的 init() 里把关节接上：没接上时最大力矩为 0，组合根的初始化顺序错了在这里挡住
    const float yaw_maximum = config.yaw->maximum_torque_newton_meter();
    const float pitch_maximum = config.pitch->maximum_torque_newton_meter();
    if (yaw_maximum <= 0.0F || pitch_maximum <= 0.0F || parameters.torque_limit_newton_meter > yaw_maximum ||
        parameters.torque_limit_newton_meter > pitch_maximum)
    {
        return false;
    }
    if (!yaw_loop_.init(parameters.yaw.controller) || !pitch_loop_.init(parameters.pitch.controller))
    {
        return false;
    }
    config_ = config;
    joints_ = {config.yaw, config.pitch};
    report_ = Report{};
    attitude_message_ = {};
    control_message_ = {};
    yaw_rad_ = 0.0F;
    pitch_rad_ = 0.0F;
    roll_rad_ = 0.0F;
    yaw_rate_ = 0.0F;
    pitch_rate_ = 0.0F;
    attitude_timestamp_us_ = 0U;
    attitude_received_us_ = 0U;
    dt_seconds_ = DEFAULT_DT_SECONDS;
    has_attitude_ = false;
    has_control_ = false;
    mode_ = Mode::DISABLED;
    reason_ = DisableReason::NONE;
    enable_start_us_ = 0U;
    yaw_target_rad_ = 0.0F;
    pitch_target_rad_ = 0.0F;
    yaw_rate_feedforward_ = 0.0F;
    pitch_rate_feedforward_ = 0.0F;
    applied_yaw_target_rad_ = 0.0F;
    applied_pitch_target_rad_ = 0.0F;
    enable_failure_count_ = 0U;
    initialized_ = true;
    // 与 DISABLED 的状态一致：不请求出力
    disable(DisableReason::NONE);
    update_report();
    return true;
}

void Gimbal::step(hal::os::Microseconds now) noexcept
{
    if (!initialized_)
    {
        return;
    }
    const bool new_attitude = read_inputs(now);

    // 出力中任一关节掉了：只有一边出力的云台会乱甩，两个都撤销
    if ((mode_ == Mode::HOLD || mode_ == Mode::TRACK) && joint_dropped())
    {
        disable(DisableReason::JOINT_DROPPED);
        update_report();
        return;
    }

    const DisableReason blocked = blocked_reason(now);
    if (blocked != DisableReason::NONE)
    {
        disable(blocked);
        update_report();
        return;
    }

    if (mode_ == Mode::DISABLED)
    {
        if (!device::enable_all(joints_.data(), joints_.size()))
        {
            reason_ = DisableReason::JOINT_UNAVAILABLE;
            update_report();
            return; // 下一轮再试
        }
        mode_ = Mode::ENABLING;
        enable_start_us_ = now;
    }

    if (mode_ == Mode::ENABLING)
    {
        if (all_joints_enabled())
        {
            // 进入 HOLD：两个控制律清零，目标取当前姿态，这样一使能不会甩；连续 yaw 重新锚定到回绕后的值
            yaw_loop_.reset();
            pitch_loop_.reset();
            anchor_yaw_continuous();
            yaw_target_rad_ = yaw_unwrapper_.continuous_rad();
            pitch_target_rad_ = pitch_rad_;
            yaw_rate_feedforward_ = 0.0F;
            pitch_rate_feedforward_ = 0.0F;
            mode_ = Mode::HOLD;
        }
        else if (hal::os::has_elapsed(enable_start_us_, config_.parameters.enable_timeout_us, now))
        {
            ++enable_failure_count_;
            disable(DisableReason::ENABLE_TIMEOUT);
            update_report();
            return;
        }
        else
        {
            // 等确认期间每轮都给 0 力矩：超过指令超时驱动就按 0 力矩发，电机不出力
            (void)config_.yaw->set_torque(0.0F);
            (void)config_.pitch->set_torque(0.0F);
            update_report();
            return;
        }
    }

    // 这一轮没有新姿态：同一个测量值再积一次分等于把积分放大，重发上一拍的力矩就行
    if (!new_attitude)
    {
        (void)command_torque(report_.yaw_torque_newton_meter, report_.pitch_torque_newton_meter);
        update_report();
        return;
    }

    // 选目标：PC 有效就是 TRACK，否则 HOLD（保持最后一次的目标）
    const Setpoint setpoint = select_target(now);
    if (setpoint.valid)
    {
        mode_ = Mode::TRACK;
        yaw_target_rad_ = setpoint.yaw_rad;
        pitch_target_rad_ = setpoint.pitch_rad;
        yaw_rate_feedforward_ = setpoint.yaw_rate;
        pitch_rate_feedforward_ = setpoint.pitch_rate;
    }
    else
    {
        mode_ = Mode::HOLD;
        yaw_rate_feedforward_ = 0.0F;
        pitch_rate_feedforward_ = 0.0F;
    }

    // 目标换算。机械限位是关节角，世界系目标与它差一个底座倾斜（世界系 − 关节角）。
    // 有限位的轴一律在关节角里算：先把目标换到关节角、绕限位区间中点回绕、再钳位，最后换回世界系。
    // 不能先钳世界系目标再回绕：限位总宽超过半圈时，合法路线可能要绕远路，回绕会把它改走近路、穿过限位。
    // 无限位的轴把目标展开到测量值附近即可，误差不会绕远路（PID 的误差是 target − measure，不会自己回绕）。
    const device::JointState &yaw_state = config_.yaw->state();
    const device::JointState &pitch_state = config_.pitch->state();
    const float yaw_continuous = yaw_unwrapper_.continuous_rad();
    float yaw_target = 0.0F;
    float yaw_joint_target = 0.0F;
    float yaw_joint_minimum = 0.0F;
    float yaw_joint_maximum = 0.0F;
    if (config_.parameters.yaw.limited)
    {
        yaw_joint_minimum = config_.parameters.yaw.minimum_rad;
        yaw_joint_maximum = config_.parameters.yaw.maximum_rad;
        const float tilt = yaw_continuous - yaw_state.position_rad;
        const float center = 0.5F * (yaw_joint_minimum + yaw_joint_maximum);
        const float joint_target = center + math::wrap_pi(yaw_target_rad_ - tilt - center);
        yaw_joint_target = math::limit(joint_target, yaw_joint_minimum, yaw_joint_maximum);
        yaw_target = tilt + yaw_joint_target;
    }
    else
    {
        yaw_target = yaw_continuous + math::wrap_pi(yaw_target_rad_ - yaw_continuous);
    }
    float pitch_target = pitch_target_rad_;
    float pitch_joint_target = pitch_target_rad_;
    if (config_.parameters.pitch.limited)
    {
        const float tilt = pitch_rad_ - pitch_state.position_rad;
        pitch_joint_target = math::limit(
            pitch_target_rad_ - tilt, config_.parameters.pitch.minimum_rad, config_.parameters.pitch.maximum_rad);
        pitch_target = tilt + pitch_joint_target;
    }
    applied_yaw_target_rad_ = yaw_target;
    applied_pitch_target_rad_ = pitch_target;

    // 目标被限位钳住的轴不要再加角速度前馈：角度追不上目标，前馈会把轴顶过目标，
    // 可能顶到机械限位。被钳的只是这一拍的前馈，PC 的给定值留着，目标回到限位内就恢复。
    const bool yaw_clamped = config_.parameters.yaw.limited &&
                             (yaw_joint_target <= yaw_joint_minimum || yaw_joint_target >= yaw_joint_maximum);
    const bool pitch_clamped =
        config_.parameters.pitch.limited && (pitch_joint_target <= config_.parameters.pitch.minimum_rad ||
                                             pitch_joint_target >= config_.parameters.pitch.maximum_rad);
    const float yaw_feedforward = yaw_clamped ? 0.0F : yaw_rate_feedforward_;
    const float pitch_feedforward = pitch_clamped ? 0.0F : pitch_rate_feedforward_;

    // 两轴各算一拍，pitch 加 τg·cos(pitch) 的重力前馈，最后钳到总限幅
    AxisLoop::Measurement yaw_measurement{};
    yaw_measurement.angle = yaw_continuous;
    yaw_measurement.rate = yaw_rate_;
    yaw_measurement.joint_velocity = yaw_state.velocity_rad_per_second;
    AxisLoop::Reference yaw_reference{};
    yaw_reference.angle = yaw_target;
    yaw_reference.rate = yaw_feedforward;
    const AxisLoop::Output yaw_output = yaw_loop_.update(yaw_measurement, yaw_reference, dt_seconds_);

    AxisLoop::Measurement pitch_measurement{};
    pitch_measurement.angle = pitch_rad_;
    pitch_measurement.rate = pitch_rate_;
    pitch_measurement.joint_velocity = pitch_state.velocity_rad_per_second;
    AxisLoop::Reference pitch_reference{};
    pitch_reference.angle = pitch_target;
    pitch_reference.rate = pitch_feedforward;
    const AxisLoop::Output pitch_output = pitch_loop_.update(pitch_measurement, pitch_reference, dt_seconds_);
    const float gravity = config_.parameters.pitch_gravity_newton_meter * std::cos(pitch_rad_);
    const float limit = config_.parameters.torque_limit_newton_meter;
    const float yaw_torque = math::limit(yaw_output.torque, -limit, limit);
    const float pitch_torque = math::limit(pitch_output.torque + gravity, -limit, limit);
    // apply() 收到的是总限幅之后、去掉重力前馈的那一份
    yaw_loop_.apply(yaw_torque);
    pitch_loop_.apply(pitch_torque - gravity);

    if (!command_torque(yaw_torque, pitch_torque))
    {
        return;
    }
    report_.yaw_torque_newton_meter = yaw_torque;
    report_.pitch_torque_newton_meter = pitch_torque;
    report_.pitch_gravity_newton_meter = gravity;
    report_.yaw_diagnostics = yaw_output.diagnostics;
    report_.pitch_diagnostics = pitch_output.diagnostics;
    update_report();
}

bool Gimbal::read_inputs(hal::os::Microseconds now) noexcept
{
    bool new_attitude = false;
    topics::imu::Attitude attitude{};
    if (attitude_.update(attitude))
    {
        new_attitude = true;
        // 新鲜度按"这一轮收到"算：采样时刻可能晚于本轮开始时的 now（IMU 任务优先级更高，
        // 在本轮取好 now 之后插进来发布），拿它做无符号减法会下溢成一个巨大的数、误判成超时
        attitude_received_us_ = now;
        if (has_attitude_)
        {
            // dt 用相邻两份姿态的采样时刻之差；用有符号差，超出范围就用 1 ms
            const std::int32_t delta = static_cast<std::int32_t>(attitude.timestamp_us - attitude_timestamp_us_);
            dt_seconds_ = (static_cast<float>(delta) > 0.0F && static_cast<float>(delta) < MAXIMUM_DT_US)
                              ? hal::os::to_seconds(static_cast<hal::os::Microseconds>(delta))
                              : DEFAULT_DT_SECONDS;
        }
        attitude_message_ = attitude;
        attitude_timestamp_us_ = attitude.timestamp_us;
        has_attitude_ = true;
        if (attitude.valid)
        {
            const EulerAngles angles = euler_from_attitude(attitude);
            if (angles.valid)
            {
                (void)yaw_unwrapper_.update(angles.yaw); // 记圈数，连续值不靠逐次累加
                yaw_rad_ = angles.yaw;
                pitch_rad_ = angles.pitch;
                roll_rad_ = angles.roll;
                yaw_rate_ = angles.yaw_rate;
                pitch_rate_ = angles.pitch_rate;
            }
            else
            {
                attitude_message_.valid = false; // 姿态里有非有限值：按无效处理，不进控制
            }
        }
    }
    topics::pc_link::Control control{};
    if (control_.update(control))
    {
        control_message_ = control;
        has_control_ = true;
    }
    return new_attitude;
}

Gimbal::DisableReason Gimbal::blocked_reason(hal::os::Microseconds now) const noexcept
{
    if (!has_attitude_ || !attitude_message_.valid)
    {
        return DisableReason::IMU_INVALID;
    }
    if (attitude_message_.state == topics::imu::State::SETTLING)
    {
        return DisableReason::IMU_SETTLING;
    }
    if (hal::os::has_elapsed(attitude_received_us_, config_.parameters.attitude_timeout_us, now))
    {
        return DisableReason::IMU_STALE;
    }
    for (const device::Joint *joint : joints_)
    {
        if (joint == nullptr || !joint->available())
        {
            return DisableReason::JOINT_UNAVAILABLE;
        }
        if (joint->state().fault)
        {
            return DisableReason::JOINT_FAULT;
        }
    }
    return DisableReason::NONE;
}

bool Gimbal::joint_dropped() const noexcept
{
    for (const device::Joint *joint : joints_)
    {
        if (joint == nullptr || !joint->available() || joint->state().fault || !joint->state().enabled)
        {
            return true;
        }
    }
    return false;
}

bool Gimbal::all_joints_enabled() const noexcept
{
    for (const device::Joint *joint : joints_)
    {
        if (joint == nullptr || !joint->state().enabled)
        {
            return false;
        }
    }
    return true;
}

Gimbal::Setpoint Gimbal::select_target(hal::os::Microseconds now) const noexcept
{
    Setpoint setpoint{};
    setpoint.yaw_rad = yaw_target_rad_;
    setpoint.pitch_rad = pitch_target_rad_;
    setpoint.yaw_rate = 0.0F;
    setpoint.pitch_rate = 0.0F;
    setpoint.valid = false;
    if (!has_control_ || (control_message_.flags & GIMBAL_FLAG) == 0U)
    {
        return setpoint;
    }
    if ((control_message_.flags & ESTOP_REQUEST_FLAG) != 0U)
    {
        return setpoint; // PC 请求急停：目标无效，保持最后一个
    }
    hal::os::Microseconds validity = config_.parameters.target_timeout_us;
    if (control_message_.validity_hint_ms != 0U)
    {
        validity = static_cast<hal::os::Microseconds>(control_message_.validity_hint_ms) *
                   hal::os::MICROSECONDS_PER_MILLISECOND;
    }
    validity = validity < TARGET_AGE_LIMIT_US ? validity : TARGET_AGE_LIMIT_US;
    if (hal::os::has_elapsed(control_message_.received_us, validity, now))
    {
        return setpoint; // 目标过期：保持最后一个
    }
    setpoint.yaw_rad = control_message_.gimbal_yaw;
    setpoint.pitch_rad = control_message_.gimbal_pitch;
    setpoint.yaw_rate = control_message_.gimbal_yaw_rate;
    setpoint.pitch_rate = control_message_.gimbal_pitch_rate;
    setpoint.valid = true;
    return setpoint;
}

bool Gimbal::command_torque(float yaw_torque, float pitch_torque) noexcept
{
    const bool yaw_written = config_.yaw->set_torque(yaw_torque);
    const bool pitch_written = config_.pitch->set_torque(pitch_torque);
    if (!yaw_written || !pitch_written)
    {
        disable(DisableReason::COMMAND_REJECTED); // 关节拒绝了指令（掉线、故障、被撤销出力）
        update_report();
        return false;
    }
    return true;
}

void Gimbal::anchor_yaw_continuous() noexcept
{
    if (has_attitude_)
    {
        yaw_unwrapper_.reset(yaw_rad_);
    }
    else
    {
        yaw_unwrapper_ = math::AngleUnwrapper{}; // 还没有姿态：回到"没喂过"的状态，第一份姿态重新锚定
    }
}

void Gimbal::disable(DisableReason reason) noexcept
{
    device::disable_all(joints_.data(), joints_.size());
    yaw_loop_.reset();
    pitch_loop_.reset();
    anchor_yaw_continuous();
    mode_ = Mode::DISABLED;
    reason_ = reason;
    report_.yaw_torque_newton_meter = 0.0F;
    report_.pitch_torque_newton_meter = 0.0F;
    report_.pitch_gravity_newton_meter = 0.0F;
    report_.yaw_diagnostics = {};
    report_.pitch_diagnostics = {};
}

void Gimbal::update_report() noexcept
{
    report_.mode = mode_;
    report_.last_disable_reason = reason_;
    report_.imu_state = attitude_message_.state;
    report_.attitude_valid = has_attitude_ && attitude_message_.valid;
    report_.last_attitude_us = attitude_received_us_;
    report_.yaw_rad = yaw_rad_;
    report_.yaw_continuous_rad = yaw_unwrapper_.continuous_rad();
    report_.pitch_rad = pitch_rad_;
    report_.target_valid = mode_ == Mode::TRACK;
    report_.yaw_target_rad = applied_yaw_target_rad_;
    report_.pitch_target_rad = applied_pitch_target_rad_;

    const device::JointState &yaw_state = config_.yaw->state();
    const device::JointState &pitch_state = config_.pitch->state();
    report_.yaw_joint_rad = yaw_state.position_rad;
    report_.pitch_joint_rad = pitch_state.position_rad;
    report_.yaw_available = config_.yaw->available();
    report_.pitch_available = config_.pitch->available();
    report_.yaw_online = yaw_state.online;
    report_.pitch_online = pitch_state.online;
    report_.yaw_fault = yaw_state.fault;
    report_.pitch_fault = pitch_state.fault;
    report_.yaw_enabled = yaw_state.enabled;
    report_.pitch_enabled = pitch_state.enabled;
    report_.enable_failure_count = enable_failure_count_;
}

} // namespace gimbal
