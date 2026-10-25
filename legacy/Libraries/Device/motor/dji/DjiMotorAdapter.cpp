#include "Libraries/Device/motor/dji/DjiMotorAdapter.hpp"
#include <cmath>

namespace device
{
DjiMotorAdapter::DjiMotorAdapter(const Config &config) noexcept : config_{config}, bus_{config.channel, config.profile}
{
}

bool DjiMotorAdapter::Init() noexcept
{
  if (ready_)
  {
    return true;
  }
  const auto &profile = config_.profile;
  const auto &current = profile.dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::Current)];
  if (static_cast<unsigned>(config_.channel) > static_cast<unsigned>(platform::Can::Channel::Channel3) ||
      config_.count == 0U || config_.count > config_.mappings.size() || current.groupOneControlIdentifier == 0U ||
      current.controlFullScaleCounts == 0U || current.controlFullScaleCounts > 32767U ||
      profile.dialect.maximumDeviceCount > DjiMotor::MaximumMotors || !std::isfinite(profile.currentFullScaleAmpere) ||
      profile.currentFullScaleAmpere <= 0.0F || !std::isfinite(profile.torqueConstantNewtonMeterPerAmpere) ||
      profile.torqueConstantNewtonMeterPerAmpere <= 0.0F)
  {
    return false;
  }
  for (std::size_t id = 0U; id < config_.count; ++id)
  {
    const auto &mapping = config_.mappings[id];
    if (mapping.deviceId == 0U || mapping.deviceId > profile.dialect.maximumDeviceCount ||
        !std::isfinite(mapping.gearRatio) || mapping.gearRatio < 1.0F ||
        (mapping.direction != 1 && mapping.direction != -1) ||
        (mapping.deviceId > protocol::DjiEsc::DevicesPerFrame && current.groupTwoControlIdentifier == 0U))
    {
      return false;
    }
    for (std::size_t previous = 0U; previous < id; ++previous)
    {
      if (config_.mappings[previous].deviceId == mapping.deviceId)
      {
        return false;
      }
    }
  }
  if (!bus_.Init())
  {
    return false;
  }
  for (std::size_t id = 0U; id < config_.count; ++id)
  {
    if (!bus_.EnableMotor(config_.mappings[id].deviceId))
    {
      bus_.ClearAllCommands();
      return false;
    }
  }
  ready_ = true;
  return true;
}

void DjiMotorAdapter::Process() noexcept
{
  if (ready_)
  {
    bus_.Process();
  }
}

bool DjiMotorAdapter::ReadState(std::size_t id, MotorState &state) const noexcept
{
  state = {};
  if (!ready_ || id >= config_.count)
  {
    return false;
  }
  DjiMotor::Snapshot snapshot{};
  const auto &mapping = config_.mappings[id];
  if (!bus_.GetSnapshot(mapping.deviceId, snapshot))
  {
    return false;
  }
  const float factor = static_cast<float>(mapping.direction) / mapping.gearRatio;
  state.angleDegrees = snapshot.totalAngleDegrees * factor;
  state.speedRpm = static_cast<float>(snapshot.rotorSpeedRpm) * factor;
  state.feedbackValid = snapshot.everReceived;
  state.online = snapshot.online;
  state.lastUpdateTick = snapshot.lastUpdateTick;
  return true;
}

bool DjiMotorAdapter::SetTorque(std::size_t id, float value) noexcept
{
  if (!ready_ || id >= config_.count)
  {
    return false;
  }
  const auto &mapping = config_.mappings[id];
  if (!std::isfinite(value))
  {
    const bool cleared = bus_.SetRawCommand(mapping.deviceId, 0);
    if (!cleared)
    {
      bus_.ClearAllCommands();
    }
    return false;
  }
  // 档案 Kt 已是输出轴力矩口径，切勿再乘减速比。
  return bus_.SetTorque(mapping.deviceId, value * mapping.direction);
}

void DjiMotorAdapter::ClearCommands() noexcept
{
  bus_.ClearAllCommands();
}
} // namespace device
