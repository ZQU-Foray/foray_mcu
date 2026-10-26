#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cstdint>

namespace alg_kinematics
{

enum class KinematicsMode : uint8_t
{
    OMNI = 0,
    MECANUM,
};

enum class WheelIndex : uint8_t
{
    M1 = 0,
    M2,
    M3,
    M4,
    COUNT,
};

// 底盘坐标系速度m/s、rad/s
struct ChassisVelocity
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

// 底盘坐标系增量m、rad
struct ChassisDelta
{
    float dx = 0.0f;
    float dy = 0.0f;
    float d_yaw = 0.0f;
};

// 电机转速：rpm，车轮顺序由底盘坐标系统一定义
struct WheelRpm
{
    float value[static_cast<uint8_t>(WheelIndex::COUNT)] = {};
};

// 车轮路程增量，m，车轮顺序与 WheelRpm 一致
struct WheelDelta
{
    float value[static_cast<uint8_t>(WheelIndex::COUNT)] = {};
};

class Kinematics
{
  public:
    static constexpr uint8_t WHEEL_COUNT = static_cast<uint8_t>(WheelIndex::COUNT);

    struct Config
    {
        KinematicsMode mode = KinematicsMode::MECANUM;

        // 几何参数，m
        float wheel_radius = 0.0f;
        float wheel_base_length = 0.0f;
        float wheel_base_width = 0.0f;

        // 减速比
        float reduction_ratio = 1.0f;

        // 麦克纳姆轮正解和里程计的横向速度标定系数
        float lateral_scale = 1.0f;
    };

    bool init(const Config &config);

    bool inverse(const ChassisVelocity &input, WheelRpm &output) const;

    bool forward(const WheelRpm &input, ChassisVelocity &output) const;

    bool forward_delta(const WheelDelta &input, ChassisDelta &output) const;

    static bool limit_wheel_rpm(WheelRpm &wheel_rpm, float max_abs_rpm);

    const Config &get_config() const;

    bool is_initialized() const;

  private:
    Config cfg_{};

    float velocity_to_rpm_ = 0.0f;
    float rpm_to_velocity_ = 0.0f;
    float rotation_radius_ = 0.0f;

    bool initialized_ = false;
};

} // namespace alg_kinematics

namespace alg_odometry
{

struct OdometryState
{
    // 世界坐标系位姿m、rad
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;

    // 底盘坐标系速度m/s、rad/s
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

class Odometry
{
  public:
    void reset();

    bool reset(float x, float y, float yaw);

    bool update_velocity(const alg_kinematics::ChassisVelocity &velocity, float dt);

    bool update_delta(const alg_kinematics::ChassisDelta &delta, float dt);

    bool update_delta_with_yaw(const alg_kinematics::ChassisDelta &delta, float yaw, float dt);

    const OdometryState &get_state() const;

  private:
    OdometryState state_{};
};

} // namespace alg_odometry

#endif
