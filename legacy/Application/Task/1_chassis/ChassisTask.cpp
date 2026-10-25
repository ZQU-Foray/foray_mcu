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

constexpr std::size_t MotorCount{chassis::ChassisController::MotorCount};
// 任务内选型：品牌映射包含 + 型号，更换品牌/型号仅修改这两行；
// 协议、原生减速比和 Kt 由品牌映射与设备层提供。
using ChassisMotors = device::MotorGroup<device::MotorModel::M2006, MotorCount>;
ChassisMotors motors{platform::Can::Channel::Channel1, {{{1U, 1}, {2U, 1}, {3U, 1}, {4U, 1}}}};

chassis::ChassisController chassisController;
bool initialized{false};

} // namespace

chassis::ChassisController::Config ChassisTask::ControllerConfig() noexcept
{
  chassis::ChassisController::Config config{};
  // 当前 M2006/C610 整定，全部采用输出轴单位。
  // 旧参数 ratio=36、输出轴 Kt=0.18：速度增益 ×36×0.18、
  // 速度输出/积分限幅 ×0.18、角度增益与限幅 ÷36。
  config.speedLoop.Mode = alg_controller::PIDMode::Position;
  config.speedLoop.Kp = 0.1296F;
  config.speedLoop.Ki = 0.648F;
  config.speedLoop.DefaultDt = 0.001F;
  config.speedLoop.Maxout = 0.54F;
  config.speedLoop.IntegralLimit = 0.27F;
  config.angleLoop.Mode = alg_controller::PIDMode::Position;
  config.angleLoop.Kp = 4.0F / 36.0F;
  config.angleLoop.DefaultDt = 0.001F;
  config.angleLoop.Maxout = 500.0F;
  config.maxTorqueNewtonMeter = 1.0F;
  config.maxWheelSpeedRpm = 500.0F;
  return config;
}

bool ChassisTask::Init() noexcept
{
  if (initialized)
  {
    return true;
  }

  if (!motors.Init())
  {
    motors.ClearCommands();
    return false;
  }

  // D4b：限幅不得超过执行器实际能力（M2006/C610 输出轴上限 10 A × 0.18 = 1.8 N·m）。
  const chassis::ChassisController::Config config = ControllerConfig();
  if (!(config.maxTorqueNewtonMeter <= motors.MaximumTorqueNewtonMeter()))
  {
    motors.ClearCommands();
    return false;
  }
  if (!chassisController.Init(config))
  {
    motors.ClearCommands();
    return false;
  }

  initialized = true;
  return true;
}

bool ChassisTask::IsReady() noexcept
{
  return initialized;
}

void ChassisTask::SetMode(WheelControlMode mode) noexcept
{
  chassisController.SetMode(mode);
}

void ChassisTask::SetWheelTarget(std::uint8_t wheel, float value) noexcept
{
  chassisController.SetWheelTarget(wheel, value);
}

float ChassisTask::WheelTarget(std::uint8_t wheel) noexcept
{
  return chassisController.WheelTarget(wheel);
}

void ChassisTask::SetEnabled(bool enabled) noexcept
{
  chassisController.SetEnabled(enabled);
}

void ChassisTask::Run(void *argument) noexcept
{
  (void)argument;
  if (!initialized)
  {
    osThreadExit();
  }

  platform::Time::Tick tick = platform::Time::NowTicks();
  device::MotorState snapshots[MotorCount]{};
  float outputs[MotorCount]{};

  for (;;)
  {
    osDelay(PeriodMs);
    const float dt = platform::Time::DeltaSeconds(tick);

    motors.Process();
    for (std::size_t wheel = 0U; wheel < MotorCount; ++wheel)
    {
      if (!motors.ReadState(wheel, snapshots[wheel]))
      {
        snapshots[wheel] = {};
      }
    }

    chassisController.Update(snapshots, dt, outputs);

    for (std::size_t wheel = 0U; wheel < MotorCount; ++wheel)
    {
      if (!motors.SetTorque(wheel, outputs[wheel]))
      {
        motors.ClearCommands();
        // D1：底盘目标立即归零，上层需重新下发。
        chassisController.OnCommandFailure();
        break;
      }
    }
  }
}

} // namespace application::task
