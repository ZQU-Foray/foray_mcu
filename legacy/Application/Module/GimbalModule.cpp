#include "Application/Module/GimbalModule.hpp"
#include <cmath>

namespace application::module
{

bool GimbalModule::Init(const Config &config, float deviceMaximumTorqueNewtonMeter) noexcept
{
  axes_.Reset();
  initFailure_ = InitFailure::None;

  if (!std::isfinite(deviceMaximumTorqueNewtonMeter) || deviceMaximumTorqueNewtonMeter <= 0.0F)
  {
    // 能力不可查询时按"无法校验"处理：宁可拒绝启动，也不让限幅成为假的约束。
    initFailure_ = InitFailure::TorqueLimitExceedsDevice;
    return false;
  }
  deviceMaximumTorqueNewtonMeter_ = deviceMaximumTorqueNewtonMeter;

  if (!std::isfinite(config.maxTorqueNewtonMeter) || config.maxTorqueNewtonMeter <= 0.0F)
  {
    initFailure_ = InitFailure::TorqueLimitInvalid;
    return false;
  }
  if (!std::isfinite(config.maxGimbalSpeedRpm) || config.maxGimbalSpeedRpm <= 0.0F)
  {
    initFailure_ = InitFailure::SpeedLimitInvalid;
    return false;
  }
  // D4b：策略总限幅不得超过执行器能力（否则"总限幅"形同虚设）。
  if (config.maxTorqueNewtonMeter > deviceMaximumTorqueNewtonMeter_)
  {
    initFailure_ = InitFailure::TorqueLimitExceedsDevice;
    return false;
  }
  for (std::size_t gimbal = 0U; gimbal < MotorCount; ++gimbal)
  {
    const AxisConfig &axis = config.axis[gimbal];
    // 角度范围必须显式给出，否则忘配置会把所有目标静默钳到零。
    if (!std::isfinite(axis.minAngleDeg) || !std::isfinite(axis.maxAngleDeg) || axis.minAngleDeg >= axis.maxAngleDeg)
    {
      initFailure_ = InitFailure::AngleRangeInvalid;
      return false;
    }
    // 电调/热限幅同样不得超过执行器能力（两级限幅都是对执行器的约束）。
    if (std::isfinite(axis.speedLoop.Maxout) && axis.speedLoop.Maxout > deviceMaximumTorqueNewtonMeter_)
    {
      initFailure_ = InitFailure::SpeedLimitExceedsDevice;
      return false;
    }
  }

  // 映射到骨架配置：产品决策在此显式落地（D1 失败策略、D2 初始模式、逐轴目标钳位）。
  component::AxisController<MotorCount>::Config mapped{};
  mapped.maxTorqueNewtonMeter = config.maxTorqueNewtonMeter;
  mapped.maxSpeedRpm = config.maxGimbalSpeedRpm;
  mapped.initialMode = GimbalControlMode::Angle;
  mapped.failurePolicy = component::FailurePolicy::HoldTargets;
  for (std::size_t gimbal = 0U; gimbal < MotorCount; ++gimbal)
  {
    mapped.axis[gimbal].speedLoop = config.axis[gimbal].speedLoop;
    mapped.axis[gimbal].angleLoop = config.axis[gimbal].angleLoop;
    mapped.axis[gimbal].clamp = component::TargetClamp::AbsoluteRange;
    mapped.axis[gimbal].minTarget = config.axis[gimbal].minAngleDeg;
    mapped.axis[gimbal].maxTarget = config.axis[gimbal].maxAngleDeg;
  }
  if (!axes_.Init(mapped))
  {
    initFailure_ = InitFailure::ControllerRejected;
    return false;
  }

  commandFailureCount_ = 0U;
  return true;
}

void GimbalModule::Reset() noexcept
{
  axes_.Reset();
}

void GimbalModule::SetEnabled(bool enabled) noexcept
{
  axes_.SetEnabled(enabled);
}

void GimbalModule::SetMode(GimbalControlMode mode) noexcept
{
  axes_.SetMode(mode);
}

void GimbalModule::SetTarget(std::size_t gimbal, float value) noexcept
{
  axes_.SetTarget(gimbal, value);
}

void GimbalModule::OnCommandFailure() noexcept
{
  ++commandFailureCount_;
  axes_.OnCommandFailure();
}

bool GimbalModule::IsReady() const noexcept
{
  return axes_.IsReady();
}

bool GimbalModule::IsEnabled() const noexcept
{
  return axes_.IsEnabled();
}

GimbalModule::GimbalControlMode GimbalModule::Mode() const noexcept
{
  return axes_.GetMode();
}

float GimbalModule::Target(std::size_t gimbal) const noexcept
{
  return axes_.GetTarget(gimbal);
}

GimbalModule::InitFailure GimbalModule::GetInitFailure() const noexcept
{
  return initFailure_;
}

component::ControllerStatus GimbalModule::GetStatus() const noexcept
{
  return axes_.GetStatus();
}

bool GimbalModule::GetAxisStatus(std::size_t gimbal, component::AxisStatus &status) const noexcept
{
  return axes_.GetAxisStatus(gimbal, status);
}

std::uint32_t GimbalModule::GetLoopFaultCount() const noexcept
{
  return axes_.GetLoopFaultCount();
}

std::uint32_t GimbalModule::GetCommandFailureCount() const noexcept
{
  return commandFailureCount_;
}

void GimbalModule::Update(const component::AxisMeasurement (&measurements)[MotorCount],
                          float dt,
                          float (&outTorqueNewtonMeter)[MotorCount]) noexcept
{
  axes_.Update(measurements, dt, outTorqueNewtonMeter);
}

} // namespace application::module
