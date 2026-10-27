#include "Application/Task/1_chassis/ChassisTask.hpp"
#include "Application/Task/1_chassis/ChassisController.hpp"
#include "Libraries/Device/motor/MotorGroup.hpp"
#include "Libraries/Device/motor/dji/DjiMotorModelMap.hpp"
#include "Platform/Interface/Time.hpp"
#include "cmsis_os2.h"

namespace application::task
{

namespace
{

constexpr std::size_t MOTOR_COUNT{chassis::ChassisController::MOTOR_COUNT};
// 任务内选型：品牌映射包含 + 型号，更换品牌/型号仅修改这两行；
// 协议、原生减速比和 Kt 由品牌映射与设备层提供。
using ChassisMotors = device::MotorGroup<device::MotorModel::M2006, MOTOR_COUNT>;
ChassisMotors g_motors{platform::Can::Channel::CHANNEL_1, {{{1U, 1}, {2U, 1}, {3U, 1}, {4U, 1}}}};

chassis::ChassisController g_chassis_controller;
bool g_initialized{false};

} // namespace

chassis::ChassisController::Config ChassisTask::controller_config() noexcept
{
    chassis::ChassisController::Config config{};
    // 当前 M2006/C610 整定，全部采用输出轴单位。
    // 旧参数 ratio=36、输出轴 Kt=0.18：速度增益 ×36×0.18、
    // 速度输出/积分限幅 ×0.18、角度增益与限幅 ÷36。
    config.speed_loop.mode = alg_controller::PIDMode::POSITION;
    config.speed_loop.kp = 0.1296F;
    config.speed_loop.ki = 0.648F;
    config.speed_loop.default_dt = 0.001F;
    config.speed_loop.maxout = 0.54F;
    config.speed_loop.integral_limit = 0.27F;
    config.angle_loop.mode = alg_controller::PIDMode::POSITION;
    config.angle_loop.kp = 4.0F / 36.0F;
    config.angle_loop.default_dt = 0.001F;
    config.angle_loop.maxout = 500.0F;
    config.max_torque_newton_meter = 1.0F;
    config.max_wheel_speed_rpm = 500.0F;
    return config;
}

bool ChassisTask::init() noexcept
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

    // D4b：限幅不得超过执行器实际能力（M2006/C610 输出轴上限 10 A × 0.18 = 1.8 N·m）。
    const chassis::ChassisController::Config config = controller_config();
    if (!(config.max_torque_newton_meter <= g_motors.maximum_torque_newton_meter()))
    {
        g_motors.clear_commands();
        return false;
    }
    if (!g_chassis_controller.init(config))
    {
        g_motors.clear_commands();
        return false;
    }

    g_initialized = true;
    return true;
}

bool ChassisTask::is_ready() noexcept
{
    return g_initialized;
}

void ChassisTask::set_mode(WheelControlMode mode) noexcept
{
    g_chassis_controller.set_mode(mode);
}

void ChassisTask::set_wheel_target(std::uint8_t wheel, float value) noexcept
{
    g_chassis_controller.set_wheel_target(wheel, value);
}

float ChassisTask::wheel_target(std::uint8_t wheel) noexcept
{
    return g_chassis_controller.wheel_target(wheel);
}

void ChassisTask::set_enabled(bool enabled) noexcept
{
    g_chassis_controller.set_enabled(enabled);
}

void ChassisTask::run(void *argument) noexcept
{
    (void)argument;
    if (!g_initialized)
    {
        osThreadExit();
    }

    platform::Time::Tick tick = platform::Time::now_ticks();
    device::MotorState snapshots[MOTOR_COUNT]{};
    float outputs[MOTOR_COUNT]{};

    for (;;)
    {
        osDelay(PERIOD_MS);
        const float dt = platform::Time::delta_seconds(tick);

        g_motors.process();
        for (std::size_t wheel = 0U; wheel < MOTOR_COUNT; ++wheel)
        {
            if (!g_motors.read_state(wheel, snapshots[wheel]))
            {
                snapshots[wheel] = {};
            }
        }

        g_chassis_controller.update(snapshots, dt, outputs);

        for (std::size_t wheel = 0U; wheel < MOTOR_COUNT; ++wheel)
        {
            if (!g_motors.set_torque(wheel, outputs[wheel]))
            {
                g_motors.clear_commands();
                // D1：底盘目标立即归零，上层需重新下发。
                g_chassis_controller.on_command_failure();
                break;
            }
        }
    }
}

} // namespace application::task
