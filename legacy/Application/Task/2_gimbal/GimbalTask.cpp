#include "Application/Task/2_gimbal/GimbalTask.hpp"
#include "Libraries/Device/motor/MotorGroup.hpp"
#include "Libraries/Device/motor/dji/DjiMotorModelMap.hpp"
#include "Platform/Interface/Time.hpp"
#include "cmsis_os2.h"

namespace application::task
{

namespace
{

constexpr std::size_t MotorCount{module::GimbalModule::MotorCount};
// 任务内选型：品牌映射包含 + 型号，更换品牌/型号仅修改这两行；
// 协议、原生减速比和 Kt 由品牌映射与设备层提供。
using GimbalMotors = device::MotorGroup<device::MotorModel::GM6020Current, MotorCount>;
GimbalMotors motors{platform::Can::Channel::Channel2, {{{1U, 1}, {2U, 1}}}};

module::GimbalModule gimbalModule;
bool initialized{false};

} // namespace

module::GimbalModule::Config GimbalTask::ControllerConfig() noexcept
{
  module::GimbalModule::Config config{};

  config.maxTorqueNewtonMeter = 2.2F;
  config.maxGimbalSpeedRpm = 320.0F;

  module::GimbalModule::AxisConfig &yaw = config.axis[0];
  yaw.speedLoop.Mode = alg_controller::PIDMode::Position;
  yaw.speedLoop.Kp = 0.0027F;
  yaw.speedLoop.Ki = 0.004F;
  yaw.speedLoop.Kf = 0.0F;
  yaw.speedLoop.DefaultDt = 0.001F;
  yaw.speedLoop.Maxout = 2.0F;
  yaw.speedLoop.IntegralLimit = 1.5F;
  yaw.angleLoop.Mode = alg_controller::PIDMode::Position;
  yaw.angleLoop.Kp = 5.0F;
  yaw.angleLoop.Ki = 0.2F;
  yaw.angleLoop.Improve.DeadBand = 1U;
  yaw.angleLoop.DeadZone = 0.3F;
  yaw.angleLoop.DefaultDt = 0.001F;
  yaw.minAngleDeg = -100000.0F;
  yaw.maxAngleDeg = 100000.0F;

  module::GimbalModule::AxisConfig &pitch = config.axis[1];
  pitch.speedLoop.Mode = alg_controller::PIDMode::Position;
  pitch.speedLoop.Kp = 0.1F;
  pitch.speedLoop.Ki = 0.0F;
  pitch.speedLoop.DefaultDt = 0.001F;
  pitch.speedLoop.Maxout = 2.2F;
  pitch.speedLoop.IntegralLimit = 1.5F;
  pitch.angleLoop.Mode = alg_controller::PIDMode::Position;
  pitch.angleLoop.Kp = 0.1F;
  pitch.angleLoop.Ki = 0.0F;
  pitch.angleLoop.DefaultDt = 0.001F;
  pitch.minAngleDeg = -30.0F;
  pitch.maxAngleDeg = 30.0F;

  return config;
}

bool GimbalTask::Init() noexcept
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

  // 执行器力矩能力由设备层查询后交给模块：模块保持零设备依赖，同时限幅越界在启动期被拦住。
  if (!gimbalModule.Init(ControllerConfig(), motors.MaximumTorqueNewtonMeter()))
  {
    motors.ClearCommands();
    return false;
  }

  initialized = true;
  return true;
}

bool GimbalTask::IsReady() noexcept
{
  return initialized;
}

void GimbalTask::SetMode(GimbalControlMode mode) noexcept
{
  gimbalModule.SetMode(mode);
}

void GimbalTask::SetGimbalTarget(std::uint8_t gimbal, float value) noexcept
{
  gimbalModule.SetTarget(gimbal, value);
}

float GimbalTask::GimbalTarget(std::uint8_t gimbal) noexcept
{
  return gimbalModule.Target(gimbal);
}

void GimbalTask::SetEnabled(bool enabled) noexcept
{
  gimbalModule.SetEnabled(enabled);
}

module::GimbalModule &GimbalTask::Module() noexcept
{
  return gimbalModule;
}

void GimbalTask::Run(void *argument) noexcept
{
  (void)argument;
  if (!initialized)
  {
    osThreadExit();
  }

  platform::Time::Tick tick = platform::Time::NowTicks();
  device::MotorState snapshots[MotorCount]{};
  component::AxisMeasurement measurements[MotorCount]{};
  float outputs[MotorCount]{};

  for (;;)
  {
    osDelay(PeriodMs);
    const float dt = platform::Time::DeltaSeconds(tick);

    motors.Process();
    for (std::size_t gimbal = 0U; gimbal < MotorCount; ++gimbal)
    {
      if (!motors.ReadState(gimbal, snapshots[gimbal]))
      {
        snapshots[gimbal] = {};
      }
      // 设备快照 → 模块量测：一次 4 字段拷贝，换取业务模块不依赖设备层类型。
      measurements[gimbal].angleDegrees = snapshots[gimbal].angleDegrees;
      measurements[gimbal].speedRpm = snapshots[gimbal].speedRpm;
      measurements[gimbal].feedbackValid = snapshots[gimbal].feedbackValid;
      measurements[gimbal].online = snapshots[gimbal].online;
    }

    gimbalModule.Update(measurements, dt, outputs);

    for (std::size_t gimbal = 0U; gimbal < MotorCount; ++gimbal)
    {
      if (!motors.SetTorque(gimbal, outputs[gimbal]))
      {
        motors.ClearCommands();
        // D1：云台保持最后目标，只复位环并记一次失败。
        gimbalModule.OnCommandFailure();
        break;
      }
    }
  }
}

} // namespace application::task
