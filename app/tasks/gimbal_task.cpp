#include "tasks/gimbal_task.hpp"
#include "driver/motor/motor_group.hpp"
#include "driver/motor/dji/dji_motor_model_map.hpp"
#include "hal/time.hpp"
#include "cmsis_os2.h"

namespace application::task
{

namespace
{

constexpr std::size_t MOTOR_COUNT{module::GimbalModule::MOTOR_COUNT};
// 任务内选型：品牌映射包含 + 型号，更换品牌/型号仅修改这两行；
// 协议、原生减速比和 Kt 由品牌映射与设备层提供。
using GimbalMotors = device::MotorGroup<device::MotorModel::GM6020_CURRENT, MOTOR_COUNT>;
GimbalMotors g_motors{platform::Can::Channel::CHANNEL_2, {{{1U, 1}, {2U, 1}}}};

module::GimbalModule g_gimbal_module;
bool g_initialized{false};

} // namespace

module::GimbalModule::Config GimbalTask::controller_config() noexcept
{
    module::GimbalModule::Config config{};

    config.max_torque_newton_meter = 2.2F;
    config.max_gimbal_speed_rpm = 320.0F;

    module::GimbalModule::AxisConfig &yaw = config.axis[0];
    yaw.speed_loop.mode = alg_controller::PIDMode::POSITION;
    yaw.speed_loop.kp = 0.0027F;
    yaw.speed_loop.ki = 0.004F;
    yaw.speed_loop.kf = 0.0F;
    yaw.speed_loop.default_dt = 0.001F;
    yaw.speed_loop.maxout = 2.0F;
    yaw.speed_loop.integral_limit = 1.5F;
    yaw.angle_loop.mode = alg_controller::PIDMode::POSITION;
    yaw.angle_loop.kp = 5.0F;
    yaw.angle_loop.ki = 0.2F;
    yaw.angle_loop.improve.dead_band = 1U;
    yaw.angle_loop.dead_zone = 0.3F;
    yaw.angle_loop.default_dt = 0.001F;
    yaw.min_angle_deg = -100000.0F;
    yaw.max_angle_deg = 100000.0F;

    module::GimbalModule::AxisConfig &pitch = config.axis[1];
    pitch.speed_loop.mode = alg_controller::PIDMode::POSITION;
    pitch.speed_loop.kp = 0.1F;
    pitch.speed_loop.ki = 0.0F;
    pitch.speed_loop.default_dt = 0.001F;
    pitch.speed_loop.maxout = 2.2F;
    pitch.speed_loop.integral_limit = 1.5F;
    pitch.angle_loop.mode = alg_controller::PIDMode::POSITION;
    pitch.angle_loop.kp = 0.1F;
    pitch.angle_loop.ki = 0.0F;
    pitch.angle_loop.default_dt = 0.001F;
    pitch.min_angle_deg = -30.0F;
    pitch.max_angle_deg = 30.0F;

    return config;
}

bool GimbalTask::init() noexcept
{
    if (g_initialized)
    {
        return true;
    }

    if (!g_motors.init())
    {
        g_motors.clear_commands();
        return false;
    }

    // 执行器力矩能力由设备层查询后交给模块：模块保持零设备依赖，同时限幅越界在启动期被拦住。
    if (!g_gimbal_module.init(controller_config(), g_motors.maximum_torque_newton_meter()))
    {
        g_motors.clear_commands();
        return false;
    }

    g_initialized = true;
    return true;
}

bool GimbalTask::is_ready() noexcept
{
    return g_initialized;
}

void GimbalTask::set_mode(GimbalControlMode mode) noexcept
{
    g_gimbal_module.set_mode(mode);
}

void GimbalTask::set_gimbal_target(std::uint8_t gimbal, float value) noexcept
{
    g_gimbal_module.set_target(gimbal, value);
}

float GimbalTask::gimbal_target(std::uint8_t gimbal) noexcept
{
    return g_gimbal_module.target(gimbal);
}

void GimbalTask::set_enabled(bool enabled) noexcept
{
    g_gimbal_module.set_enabled(enabled);
}

module::GimbalModule &GimbalTask::module() noexcept
{
    return g_gimbal_module;
}

void GimbalTask::run(void *argument) noexcept
{
    (void)argument;
    if (!g_initialized)
    {
        osThreadExit();
    }

    platform::Time::Tick tick = platform::Time::now_ticks();
    device::MotorState snapshots[MOTOR_COUNT]{};
    component::AxisMeasurement measurements[MOTOR_COUNT]{};
    float outputs[MOTOR_COUNT]{};

    for (;;)
    {
        osDelay(PERIOD_MS);
        const float dt = platform::Time::delta_seconds(tick);

        g_motors.process();
        for (std::size_t gimbal = 0U; gimbal < MOTOR_COUNT; ++gimbal)
        {
            if (!g_motors.read_state(gimbal, snapshots[gimbal]))
            {
                snapshots[gimbal] = {};
            }
            // 设备快照 → 模块量测：一次 4 字段拷贝，换取业务模块不依赖设备层类型。
            measurements[gimbal].angle_degrees = snapshots[gimbal].angle_degrees;
            measurements[gimbal].speed_rpm = snapshots[gimbal].speed_rpm;
            measurements[gimbal].feedback_valid = snapshots[gimbal].feedback_valid;
            measurements[gimbal].online = snapshots[gimbal].online;
        }

        g_gimbal_module.update(measurements, dt, outputs);

        for (std::size_t gimbal = 0U; gimbal < MOTOR_COUNT; ++gimbal)
        {
            if (!g_motors.set_torque(gimbal, outputs[gimbal]))
            {
                g_motors.clear_commands();
                // D1：云台保持最后目标，只复位环并记一次失败。
                g_gimbal_module.on_command_failure();
                break;
            }
        }
    }
}

} // namespace application::task
