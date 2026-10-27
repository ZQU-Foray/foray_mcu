#include "lib/alg_kinematics/kinematics.hpp"
#include <cmath>

namespace
{

constexpr float TWO_PI = 6.28318530717958647692f;
constexpr float QUARTER = 0.25f;

constexpr uint8_t WHEEL_M1 = static_cast<uint8_t>(alg_kinematics::WheelIndex::M1);
constexpr uint8_t WHEEL_M2 = static_cast<uint8_t>(alg_kinematics::WheelIndex::M2);
constexpr uint8_t WHEEL_M3 = static_cast<uint8_t>(alg_kinematics::WheelIndex::M3);
constexpr uint8_t WHEEL_M4 = static_cast<uint8_t>(alg_kinematics::WheelIndex::M4);

bool is_finite(float value)
{
    return std::isfinite(value);
}

bool is_finite(const alg_kinematics::ChassisVelocity &value)
{
    return is_finite(value.vx) && is_finite(value.vy) && is_finite(value.wz);
}

bool is_finite(const alg_kinematics::ChassisDelta &value)
{
    return is_finite(value.dx) && is_finite(value.dy) && is_finite(value.d_yaw);
}

bool is_finite(const alg_kinematics::WheelRpm &value)
{
    for (uint8_t i = 0; i < alg_kinematics::Kinematics::WHEEL_COUNT; i++)
    {
        if (!is_finite(value.value[i]))
        {
            return false;
        }
    }

    return true;
}

bool is_finite(const alg_kinematics::WheelDelta &value)
{
    for (uint8_t i = 0; i < alg_kinematics::Kinematics::WHEEL_COUNT; i++)
    {
        if (!is_finite(value.value[i]))
        {
            return false;
        }
    }

    return true;
}

bool is_finite(const alg_odometry::OdometryState &value)
{
    return is_finite(value.x) && is_finite(value.y) && is_finite(value.yaw) && is_finite(value.vx) &&
           is_finite(value.vy) && is_finite(value.wz);
}

void body_to_world(float body_x, float body_y, float yaw, float &world_x, float &world_y)
{
    const float cos_yaw = std::cos(yaw);
    const float sin_yaw = std::sin(yaw);

    world_x = body_x * cos_yaw - body_y * sin_yaw;
    world_y = body_x * sin_yaw + body_y * cos_yaw;
}

} // namespace

namespace alg_kinematics
{

bool Kinematics::init(const Config &config)
{
    cfg_ = Config{};
    velocity_to_rpm_ = 0.0f;
    rpm_to_velocity_ = 0.0f;
    rotation_radius_ = 0.0f;
    initialized_ = false;

    const bool mode_valid = config.mode == KinematicsMode::OMNI || config.mode == KinematicsMode::MECANUM;
    const bool param_finite = is_finite(config.wheel_radius) && is_finite(config.wheel_base_length) &&
                              is_finite(config.wheel_base_width) && is_finite(config.reduction_ratio) &&
                              is_finite(config.lateral_scale);

    if (!mode_valid || !param_finite || config.wheel_radius <= 0.0f || config.wheel_base_length < 0.0f ||
        config.wheel_base_width < 0.0f || config.reduction_ratio <= 0.0f || config.lateral_scale <= 0.0f)
    {
        return false;
    }

    const float rotation_radius = (config.wheel_base_length + config.wheel_base_width) * 0.5f;
    const float velocity_to_rpm = 60.0f * config.reduction_ratio / (TWO_PI * config.wheel_radius);
    const float rpm_to_velocity = TWO_PI * config.wheel_radius / (60.0f * config.reduction_ratio);

    if (!is_finite(rotation_radius) || rotation_radius <= 0.0f || !is_finite(velocity_to_rpm) ||
        velocity_to_rpm <= 0.0f || !is_finite(rpm_to_velocity) || rpm_to_velocity <= 0.0f)
    {
        return false;
    }

    cfg_ = config;
    velocity_to_rpm_ = velocity_to_rpm;
    rpm_to_velocity_ = rpm_to_velocity;
    rotation_radius_ = rotation_radius;
    initialized_ = true;

    return true;
}

bool Kinematics::inverse(const ChassisVelocity &input, WheelRpm &output) const
{
    output = WheelRpm{};

    if (!initialized_ || !is_finite(input))
    {
        return false;
    }

    const float rotation_velocity = input.wz * rotation_radius_;
    WheelRpm result{};

    switch (cfg_.mode)
    {
    case KinematicsMode::OMNI:
        result.value[WHEEL_M1] = (input.vx + input.vy - rotation_velocity) * velocity_to_rpm_;
        result.value[WHEEL_M2] = (-input.vx + input.vy - rotation_velocity) * velocity_to_rpm_;
        result.value[WHEEL_M3] = (-input.vx - input.vy - rotation_velocity) * velocity_to_rpm_;
        result.value[WHEEL_M4] = (input.vx - input.vy - rotation_velocity) * velocity_to_rpm_;
        break;

    case KinematicsMode::MECANUM:
        result.value[WHEEL_M1] = (input.vx - input.vy - rotation_velocity) * velocity_to_rpm_;
        result.value[WHEEL_M2] = (input.vx + input.vy - rotation_velocity) * velocity_to_rpm_;
        result.value[WHEEL_M3] = (input.vx - input.vy + rotation_velocity) * velocity_to_rpm_;
        result.value[WHEEL_M4] = (input.vx + input.vy + rotation_velocity) * velocity_to_rpm_;
        break;

    default:
        return false;
    }

    if (!is_finite(result))
    {
        return false;
    }

    output = result;
    return true;
}

bool Kinematics::forward(const WheelRpm &input, ChassisVelocity &output) const
{
    output = ChassisVelocity{};

    if (!initialized_ || !is_finite(input))
    {
        return false;
    }

    const float velocity_m1 = input.value[WHEEL_M1] * rpm_to_velocity_;
    const float velocity_m2 = input.value[WHEEL_M2] * rpm_to_velocity_;
    const float velocity_m3 = input.value[WHEEL_M3] * rpm_to_velocity_;
    const float velocity_m4 = input.value[WHEEL_M4] * rpm_to_velocity_;
    const float rotation_denominator = 4.0f * rotation_radius_;
    ChassisVelocity result{};

    switch (cfg_.mode)
    {
    case KinematicsMode::OMNI:
        result.vx = (velocity_m1 - velocity_m2 - velocity_m3 + velocity_m4) * QUARTER;
        result.vy = (velocity_m1 + velocity_m2 - velocity_m3 - velocity_m4) * QUARTER;
        result.wz = -(velocity_m1 + velocity_m2 + velocity_m3 + velocity_m4) / rotation_denominator;
        break;

    case KinematicsMode::MECANUM:
        result.vx = (velocity_m1 + velocity_m2 + velocity_m3 + velocity_m4) * QUARTER;
        result.vy = (-velocity_m1 + velocity_m2 - velocity_m3 + velocity_m4) * QUARTER * cfg_.lateral_scale;
        result.wz = (-velocity_m1 - velocity_m2 + velocity_m3 + velocity_m4) / rotation_denominator;
        break;

    default:
        return false;
    }

    if (!is_finite(result))
    {
        return false;
    }

    output = result;
    return true;
}

bool Kinematics::forward_delta(const WheelDelta &input, ChassisDelta &output) const
{
    output = ChassisDelta{};

    if (!initialized_ || !is_finite(input))
    {
        return false;
    }

    const float delta_m1 = input.value[WHEEL_M1];
    const float delta_m2 = input.value[WHEEL_M2];
    const float delta_m3 = input.value[WHEEL_M3];
    const float delta_m4 = input.value[WHEEL_M4];
    const float rotation_denominator = 4.0f * rotation_radius_;
    ChassisDelta result{};

    switch (cfg_.mode)
    {
    case KinematicsMode::OMNI:
        result.dx = (delta_m1 - delta_m2 - delta_m3 + delta_m4) * QUARTER;
        result.dy = (delta_m1 + delta_m2 - delta_m3 - delta_m4) * QUARTER;
        result.d_yaw = -(delta_m1 + delta_m2 + delta_m3 + delta_m4) / rotation_denominator;
        break;

    case KinematicsMode::MECANUM:
        result.dx = (delta_m1 + delta_m2 + delta_m3 + delta_m4) * QUARTER;
        result.dy = (-delta_m1 + delta_m2 - delta_m3 + delta_m4) * QUARTER * cfg_.lateral_scale;
        result.d_yaw = (-delta_m1 - delta_m2 + delta_m3 + delta_m4) / rotation_denominator;
        break;

    default:
        return false;
    }

    if (!is_finite(result))
    {
        return false;
    }

    output = result;
    return true;
}

bool Kinematics::limit_wheel_rpm(WheelRpm &wheel_rpm, float max_abs_rpm)
{
    if (!is_finite(max_abs_rpm) || max_abs_rpm < 0.0f || !is_finite(wheel_rpm))
    {
        wheel_rpm = WheelRpm{};
        return false;
    }

    float max_value = 0.0f;
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        const float abs_value = std::fabs(wheel_rpm.value[i]);
        if (abs_value > max_value)
        {
            max_value = abs_value;
        }
    }

    if (max_value <= max_abs_rpm)
    {
        return true;
    }

    const float scale = max_abs_rpm / max_value;
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        wheel_rpm.value[i] *= scale;
    }

    return true;
}

const Kinematics::Config &Kinematics::get_config() const
{
    return cfg_;
}

bool Kinematics::is_initialized() const
{
    return initialized_;
}

} // namespace alg_kinematics

namespace alg_odometry
{

void Odometry::reset()
{
    state_ = OdometryState{};
}

bool Odometry::reset(float x, float y, float yaw)
{
    if (!is_finite(x) || !is_finite(y) || !is_finite(yaw))
    {
        return false;
    }

    state_ = OdometryState{};
    state_.x = x;
    state_.y = y;
    state_.yaw = yaw;
    return true;
}

bool Odometry::update_velocity(const alg_kinematics::ChassisVelocity &velocity, float dt)
{
    if (!is_finite(velocity) || !is_finite(dt) || dt <= 0.0f)
    {
        return false;
    }

    const alg_kinematics::ChassisDelta delta = {
        velocity.vx * dt,
        velocity.vy * dt,
        velocity.wz * dt,
    };

    return update_delta(delta, dt);
}

bool Odometry::update_delta(const alg_kinematics::ChassisDelta &delta, float dt)
{
    if (!is_finite(state_) || !is_finite(delta) || !is_finite(dt) || dt <= 0.0f)
    {
        return false;
    }

    float world_dx = 0.0f;
    float world_dy = 0.0f;
    const float middle_yaw = state_.yaw + delta.d_yaw * 0.5f;
    body_to_world(delta.dx, delta.dy, middle_yaw, world_dx, world_dy);

    OdometryState result = state_;
    result.x += world_dx;
    result.y += world_dy;
    result.yaw += delta.d_yaw;
    result.vx = delta.dx / dt;
    result.vy = delta.dy / dt;
    result.wz = delta.d_yaw / dt;

    if (!is_finite(result))
    {
        return false;
    }

    state_ = result;
    return true;
}

bool Odometry::update_delta_with_yaw(const alg_kinematics::ChassisDelta &delta, float yaw, float dt)
{
    if (!is_finite(state_) || !is_finite(delta) || !is_finite(yaw) || !is_finite(dt) || dt <= 0.0f)
    {
        return false;
    }

    float world_dx = 0.0f;
    float world_dy = 0.0f;
    body_to_world(delta.dx, delta.dy, yaw, world_dx, world_dy);

    OdometryState result = state_;
    result.x += world_dx;
    result.y += world_dy;
    result.yaw = yaw;
    result.vx = delta.dx / dt;
    result.vy = delta.dy / dt;
    result.wz = delta.d_yaw / dt;

    if (!is_finite(result))
    {
        return false;
    }

    state_ = result;
    return true;
}

const OdometryState &Odometry::get_state() const
{
    return state_;
}

} // namespace alg_odometry
