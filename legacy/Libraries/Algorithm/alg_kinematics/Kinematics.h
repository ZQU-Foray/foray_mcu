#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cstdint>

namespace alg_kinematics
{

enum class KinematicsMode : uint8_t
{
    Omni = 0,
    Mecanum,
};

enum class WheelIndex : uint8_t
{
    M1 = 0,
    M2,
    M3,
    M4,
    Count,
};

// 底盘坐标系速度m/s、rad/s
struct ChassisVelocity
{
    float Vx = 0.0f;
    float Vy = 0.0f;
    float Wz = 0.0f;
};

// 底盘坐标系增量m、rad
struct ChassisDelta
{
    float Dx = 0.0f;
    float Dy = 0.0f;
    float DYaw = 0.0f;
};

// 电机转速：rpm，车轮顺序由底盘坐标系统一定义
struct WheelRpm
{
    float Value[static_cast<uint8_t>(WheelIndex::Count)] = {};
};

// 车轮路程增量，m，车轮顺序与 WheelRpm 一致
struct WheelDelta
{
    float Value[static_cast<uint8_t>(WheelIndex::Count)] = {};
};

class Kinematics
{
  public:
    static constexpr uint8_t WheelCount = static_cast<uint8_t>(WheelIndex::Count);

    struct Config
    {
        KinematicsMode Mode = KinematicsMode::Mecanum;

        // 几何参数，m
        float WheelRadius = 0.0f;
        float WheelBaseLength = 0.0f;
        float WheelBaseWidth = 0.0f;

        // 减速比
        float ReductionRatio = 1.0f;

        // 麦克纳姆轮正解和里程计的横向速度标定系数
        float LateralScale = 1.0f;
    };

    bool Init(const Config &config);

    bool Inverse(const ChassisVelocity &input, WheelRpm &output) const;

    bool Forward(const WheelRpm &input, ChassisVelocity &output) const;

    bool ForwardDelta(const WheelDelta &input, ChassisDelta &output) const;

    static bool LimitWheelRpm(WheelRpm &wheelRpm, float maxAbsRpm);

    const Config &GetConfig() const;

    bool IsInitialized() const;

  private:
    Config Cfg{};

    float VelocityToRpm = 0.0f;
    float RpmToVelocity = 0.0f;
    float RotationRadius = 0.0f;

    bool Initialized = false;
};

} // namespace alg_kinematics

namespace alg_odometry
{

struct OdometryState
{
    // 世界坐标系位姿m、rad
    float X = 0.0f;
    float Y = 0.0f;
    float Yaw = 0.0f;

    // 底盘坐标系速度m/s、rad/s
    float Vx = 0.0f;
    float Vy = 0.0f;
    float Wz = 0.0f;
};

class Odometry
{
  public:
    void Reset();

    bool Reset(float x, float y, float yaw);

    bool UpdateVelocity(const alg_kinematics::ChassisVelocity &velocity, float dt);

    bool UpdateDelta(const alg_kinematics::ChassisDelta &delta, float dt);

    bool UpdateDeltaWithYaw(const alg_kinematics::ChassisDelta &delta, float yaw, float dt);

    const OdometryState &GetState() const;

  private:
    OdometryState State{};
};

} // namespace alg_odometry

#endif
