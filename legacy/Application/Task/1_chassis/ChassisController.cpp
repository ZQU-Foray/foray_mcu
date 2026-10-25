#include "Application/Task/1_chassis/ChassisController.hpp"

namespace application::chassis
{

bool ChassisController::Init(const Config &config) noexcept
{
  component::AxisController<MotorCount>::Config mapped{};
  mapped.maxTorqueNewtonMeter = config.maxTorqueNewtonMeter;
  mapped.maxSpeedRpm = config.maxWheelSpeedRpm;
  // D2：底盘默认转速模式（目标 0 即停转，比角度闭环更保守）。
  mapped.initialMode = WheelControlMode::Speed;
  // D1：要求底盘运动目标立即归零，失败后必须由上层重新下发。
  mapped.failurePolicy = component::FailurePolicy::ClearTargets;
  for (std::size_t wheel = 0U; wheel < MotorCount; ++wheel)
  {
    mapped.axis[wheel].speedLoop = config.speedLoop;
    mapped.axis[wheel].angleLoop = config.angleLoop;
    mapped.axis[wheel].clamp = component::TargetClamp::None;
  }
  return axes_.Init(mapped);
}

void ChassisController::Reset() noexcept
{
  axes_.Reset();
}

void ChassisController::SetEnabled(bool enabled) noexcept
{
  axes_.SetEnabled(enabled);
}

void ChassisController::SetMode(WheelControlMode mode) noexcept
{
  axes_.SetMode(mode);
}

void ChassisController::SetWheelTarget(std::size_t wheel, float value) noexcept
{
  axes_.SetTarget(wheel, value);
}

void ChassisController::OnCommandFailure() noexcept
{
  axes_.OnCommandFailure();
}

bool ChassisController::IsReady() const noexcept
{
  return axes_.IsReady();
}

bool ChassisController::IsEnabled() const noexcept
{
  return axes_.IsEnabled();
}

ChassisController::WheelControlMode ChassisController::Mode() const noexcept
{
  return axes_.GetMode();
}

float ChassisController::WheelTarget(std::size_t wheel) const noexcept
{
  return axes_.GetTarget(wheel);
}

component::ControllerStatus ChassisController::GetStatus() const noexcept
{
  return axes_.GetStatus();
}

bool ChassisController::GetAxisStatus(std::size_t wheel, component::AxisStatus &status) const noexcept
{
  return axes_.GetAxisStatus(wheel, status);
}

std::uint32_t ChassisController::GetLoopFaultCount() const noexcept
{
  return axes_.GetLoopFaultCount();
}

void ChassisController::Update(const device::MotorState (&snapshots)[MotorCount],
                               float dt,
                               float (&outTorqueNewtonMeter)[MotorCount]) noexcept
{
  // 设备快照 → 骨架量测：一次 4 字段拷贝，换取算法层不依赖设备层。
  component::AxisMeasurement measurements[MotorCount]{};
  for (std::size_t wheel = 0U; wheel < MotorCount; ++wheel)
  {
    measurements[wheel].angleDegrees = snapshots[wheel].angleDegrees;
    measurements[wheel].speedRpm = snapshots[wheel].speedRpm;
    measurements[wheel].feedbackValid = snapshots[wheel].feedbackValid;
    measurements[wheel].online = snapshots[wheel].online;
  }
  axes_.Update(measurements, dt, outTorqueNewtonMeter);
}

} // namespace application::chassis
