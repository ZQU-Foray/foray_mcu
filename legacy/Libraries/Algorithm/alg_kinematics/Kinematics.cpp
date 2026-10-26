#include "Kinematics.h"
#include <cmath>

namespace
{

constexpr float TwoPi = 6.28318530717958647692f;
constexpr float Quarter = 0.25f;

constexpr uint8_t WheelM1 = static_cast<uint8_t>(alg_kinematics::WheelIndex::M1);
constexpr uint8_t WheelM2 = static_cast<uint8_t>(alg_kinematics::WheelIndex::M2);
constexpr uint8_t WheelM3 = static_cast<uint8_t>(alg_kinematics::WheelIndex::M3);
constexpr uint8_t WheelM4 = static_cast<uint8_t>(alg_kinematics::WheelIndex::M4);

bool IsFinite(float value)
{
    return std::isfinite(value);
}

bool IsFinite(const alg_kinematics::ChassisVelocity &value)
{
    return IsFinite(value.Vx) && IsFinite(value.Vy) && IsFinite(value.Wz);
}

bool IsFinite(const alg_kinematics::ChassisDelta &value)
{
    return IsFinite(value.Dx) && IsFinite(value.Dy) && IsFinite(value.DYaw);
}

bool IsFinite(const alg_kinematics::WheelRpm &value)
{
    for (uint8_t i = 0; i < alg_kinematics::Kinematics::WheelCount; i++)
    {
        if (!IsFinite(value.Value[i]))
        {
            return false;
        }
    }

    return true;
}

bool IsFinite(const alg_kinematics::WheelDelta &value)
{
    for (uint8_t i = 0; i < alg_kinematics::Kinematics::WheelCount; i++)
    {
        if (!IsFinite(value.Value[i]))
        {
            return false;
        }
    }

    return true;
}

bool IsFinite(const alg_odometry::OdometryState &value)
{
    return IsFinite(value.X) && IsFinite(value.Y) && IsFinite(value.Yaw) && IsFinite(value.Vx) && IsFinite(value.Vy) &&
           IsFinite(value.Wz);
}

void BodyToWorld(float bodyX, float bodyY, float yaw, float &worldX, float &worldY)
{
    const float cosYaw = std::cos(yaw);
    const float sinYaw = std::sin(yaw);

    worldX = bodyX * cosYaw - bodyY * sinYaw;
    worldY = bodyX * sinYaw + bodyY * cosYaw;
}

} // namespace

namespace alg_kinematics
{

bool Kinematics::Init(const Config &config)
{
    Cfg = Config{};
    VelocityToRpm = 0.0f;
    RpmToVelocity = 0.0f;
    RotationRadius = 0.0f;
    Initialized = false;

    const bool modeValid = config.Mode == KinematicsMode::Omni || config.Mode == KinematicsMode::Mecanum;
    const bool paramFinite = IsFinite(config.WheelRadius) && IsFinite(config.WheelBaseLength) &&
                             IsFinite(config.WheelBaseWidth) && IsFinite(config.ReductionRatio) &&
                             IsFinite(config.LateralScale);

    if (!modeValid || !paramFinite || config.WheelRadius <= 0.0f || config.WheelBaseLength < 0.0f ||
        config.WheelBaseWidth < 0.0f || config.ReductionRatio <= 0.0f || config.LateralScale <= 0.0f)
    {
        return false;
    }

    const float rotationRadius = (config.WheelBaseLength + config.WheelBaseWidth) * 0.5f;
    const float velocityToRpm = 60.0f * config.ReductionRatio / (TwoPi * config.WheelRadius);
    const float rpmToVelocity = TwoPi * config.WheelRadius / (60.0f * config.ReductionRatio);

    if (!IsFinite(rotationRadius) || rotationRadius <= 0.0f || !IsFinite(velocityToRpm) || velocityToRpm <= 0.0f ||
        !IsFinite(rpmToVelocity) || rpmToVelocity <= 0.0f)
    {
        return false;
    }

    Cfg = config;
    VelocityToRpm = velocityToRpm;
    RpmToVelocity = rpmToVelocity;
    RotationRadius = rotationRadius;
    Initialized = true;

    return true;
}

bool Kinematics::Inverse(const ChassisVelocity &input, WheelRpm &output) const
{
    output = WheelRpm{};

    if (!Initialized || !IsFinite(input))
    {
        return false;
    }

    const float rotationVelocity = input.Wz * RotationRadius;
    WheelRpm result{};

    switch (Cfg.Mode)
    {
    case KinematicsMode::Omni:
        result.Value[WheelM1] = (input.Vx + input.Vy - rotationVelocity) * VelocityToRpm;
        result.Value[WheelM2] = (-input.Vx + input.Vy - rotationVelocity) * VelocityToRpm;
        result.Value[WheelM3] = (-input.Vx - input.Vy - rotationVelocity) * VelocityToRpm;
        result.Value[WheelM4] = (input.Vx - input.Vy - rotationVelocity) * VelocityToRpm;
        break;

    case KinematicsMode::Mecanum:
        result.Value[WheelM1] = (input.Vx - input.Vy - rotationVelocity) * VelocityToRpm;
        result.Value[WheelM2] = (input.Vx + input.Vy - rotationVelocity) * VelocityToRpm;
        result.Value[WheelM3] = (input.Vx - input.Vy + rotationVelocity) * VelocityToRpm;
        result.Value[WheelM4] = (input.Vx + input.Vy + rotationVelocity) * VelocityToRpm;
        break;

    default:
        return false;
    }

    if (!IsFinite(result))
    {
        return false;
    }

    output = result;
    return true;
}

bool Kinematics::Forward(const WheelRpm &input, ChassisVelocity &output) const
{
    output = ChassisVelocity{};

    if (!Initialized || !IsFinite(input))
    {
        return false;
    }

    const float velocityM1 = input.Value[WheelM1] * RpmToVelocity;
    const float velocityM2 = input.Value[WheelM2] * RpmToVelocity;
    const float velocityM3 = input.Value[WheelM3] * RpmToVelocity;
    const float velocityM4 = input.Value[WheelM4] * RpmToVelocity;
    const float rotationDenominator = 4.0f * RotationRadius;
    ChassisVelocity result{};

    switch (Cfg.Mode)
    {
    case KinematicsMode::Omni:
        result.Vx = (velocityM1 - velocityM2 - velocityM3 + velocityM4) * Quarter;
        result.Vy = (velocityM1 + velocityM2 - velocityM3 - velocityM4) * Quarter;
        result.Wz = -(velocityM1 + velocityM2 + velocityM3 + velocityM4) / rotationDenominator;
        break;

    case KinematicsMode::Mecanum:
        result.Vx = (velocityM1 + velocityM2 + velocityM3 + velocityM4) * Quarter;
        result.Vy = (-velocityM1 + velocityM2 - velocityM3 + velocityM4) * Quarter * Cfg.LateralScale;
        result.Wz = (-velocityM1 - velocityM2 + velocityM3 + velocityM4) / rotationDenominator;
        break;

    default:
        return false;
    }

    if (!IsFinite(result))
    {
        return false;
    }

    output = result;
    return true;
}

bool Kinematics::ForwardDelta(const WheelDelta &input, ChassisDelta &output) const
{
    output = ChassisDelta{};

    if (!Initialized || !IsFinite(input))
    {
        return false;
    }

    const float deltaM1 = input.Value[WheelM1];
    const float deltaM2 = input.Value[WheelM2];
    const float deltaM3 = input.Value[WheelM3];
    const float deltaM4 = input.Value[WheelM4];
    const float rotationDenominator = 4.0f * RotationRadius;
    ChassisDelta result{};

    switch (Cfg.Mode)
    {
    case KinematicsMode::Omni:
        result.Dx = (deltaM1 - deltaM2 - deltaM3 + deltaM4) * Quarter;
        result.Dy = (deltaM1 + deltaM2 - deltaM3 - deltaM4) * Quarter;
        result.DYaw = -(deltaM1 + deltaM2 + deltaM3 + deltaM4) / rotationDenominator;
        break;

    case KinematicsMode::Mecanum:
        result.Dx = (deltaM1 + deltaM2 + deltaM3 + deltaM4) * Quarter;
        result.Dy = (-deltaM1 + deltaM2 - deltaM3 + deltaM4) * Quarter * Cfg.LateralScale;
        result.DYaw = (-deltaM1 - deltaM2 + deltaM3 + deltaM4) / rotationDenominator;
        break;

    default:
        return false;
    }

    if (!IsFinite(result))
    {
        return false;
    }

    output = result;
    return true;
}

bool Kinematics::LimitWheelRpm(WheelRpm &wheelRpm, float maxAbsRpm)
{
    if (!IsFinite(maxAbsRpm) || maxAbsRpm < 0.0f || !IsFinite(wheelRpm))
    {
        wheelRpm = WheelRpm{};
        return false;
    }

    float maxValue = 0.0f;
    for (uint8_t i = 0; i < WheelCount; i++)
    {
        const float absValue = std::fabs(wheelRpm.Value[i]);
        if (absValue > maxValue)
        {
            maxValue = absValue;
        }
    }

    if (maxValue <= maxAbsRpm)
    {
        return true;
    }

    const float scale = maxAbsRpm / maxValue;
    for (uint8_t i = 0; i < WheelCount; i++)
    {
        wheelRpm.Value[i] *= scale;
    }

    return true;
}

const Kinematics::Config &Kinematics::GetConfig() const
{
    return Cfg;
}

bool Kinematics::IsInitialized() const
{
    return Initialized;
}

} // namespace alg_kinematics

namespace alg_odometry
{

void Odometry::Reset()
{
    State = OdometryState{};
}

bool Odometry::Reset(float x, float y, float yaw)
{
    if (!IsFinite(x) || !IsFinite(y) || !IsFinite(yaw))
    {
        return false;
    }

    State = OdometryState{};
    State.X = x;
    State.Y = y;
    State.Yaw = yaw;
    return true;
}

bool Odometry::UpdateVelocity(const alg_kinematics::ChassisVelocity &velocity, float dt)
{
    if (!IsFinite(velocity) || !IsFinite(dt) || dt <= 0.0f)
    {
        return false;
    }

    const alg_kinematics::ChassisDelta delta = {
        velocity.Vx * dt,
        velocity.Vy * dt,
        velocity.Wz * dt,
    };

    return UpdateDelta(delta, dt);
}

bool Odometry::UpdateDelta(const alg_kinematics::ChassisDelta &delta, float dt)
{
    if (!IsFinite(State) || !IsFinite(delta) || !IsFinite(dt) || dt <= 0.0f)
    {
        return false;
    }

    float worldDx = 0.0f;
    float worldDy = 0.0f;
    const float middleYaw = State.Yaw + delta.DYaw * 0.5f;
    BodyToWorld(delta.Dx, delta.Dy, middleYaw, worldDx, worldDy);

    OdometryState result = State;
    result.X += worldDx;
    result.Y += worldDy;
    result.Yaw += delta.DYaw;
    result.Vx = delta.Dx / dt;
    result.Vy = delta.Dy / dt;
    result.Wz = delta.DYaw / dt;

    if (!IsFinite(result))
    {
        return false;
    }

    State = result;
    return true;
}

bool Odometry::UpdateDeltaWithYaw(const alg_kinematics::ChassisDelta &delta, float yaw, float dt)
{
    if (!IsFinite(State) || !IsFinite(delta) || !IsFinite(yaw) || !IsFinite(dt) || dt <= 0.0f)
    {
        return false;
    }

    float worldDx = 0.0f;
    float worldDy = 0.0f;
    BodyToWorld(delta.Dx, delta.Dy, yaw, worldDx, worldDy);

    OdometryState result = State;
    result.X += worldDx;
    result.Y += worldDy;
    result.Yaw = yaw;
    result.Vx = delta.Dx / dt;
    result.Vy = delta.Dy / dt;
    result.Wz = delta.DYaw / dt;

    if (!IsFinite(result))
    {
        return false;
    }

    State = result;
    return true;
}

const OdometryState &Odometry::GetState() const
{
    return State;
}

} // namespace alg_odometry
