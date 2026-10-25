#include "Libraries/Device/motor/dji/DjiMotor.hpp"

namespace device
{

namespace
{

constexpr std::size_t SendResultCount{6U};

} // namespace

DjiMotor::DjiMotor(platform::Can::Channel channel, const Profile &profile) noexcept
    : channel_{channel}, esc_{profile.dialect}, profile_{profile},
      commandKind_{profile.dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::Current)]
                               .groupOneControlIdentifier != 0U
                       ? protocol::DjiEsc::CommandKind::Current
                       : protocol::DjiEsc::CommandKind::Voltage}
{
}

bool DjiMotor::Init() noexcept
{
  motors_ = {};
  txResultCounts_ = {};
  portStatistics_ = {};
  feedback_ = {};
  publishedSnapshots_ = {};
  publishedStatistics_ = {};
  snapshotSequence_.store(0U, std::memory_order_relaxed);
  dirty_ = false;

  const protocol::DjiEsc::CommandSpec &current =
      profile_.dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::Current)];
  const protocol::DjiEsc::CommandSpec &voltage =
      profile_.dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::Voltage)];
  if (current.groupOneControlIdentifier == 0U && voltage.groupOneControlIdentifier == 0U)
  {
    ready_ = false;
    return false;
  }

  if (!platform::Can::IsReady(channel_) || !platform::Time::IsReady())
  {
    ready_ = false;
    return false;
  }

  ready_ = true;
  dirty_ = true;
  Publish();
  return true;
}

void DjiMotor::Process() noexcept
{
  if (!ready_)
  {
    return;
  }

  ReceiveFrames();
  UpdateLinkAging();
  TransmitCommands();
  (void)platform::Can::GetStatistics(channel_, portStatistics_);
  if (dirty_)
  {
    Publish();
  }
}

bool DjiMotor::EnableMotor(std::uint8_t deviceId) noexcept
{
  if (!ready_ || !DeviceIdValid(deviceId))
  {
    return false;
  }
  motors_[deviceId - 1U].enabled = true;
  dirty_ = true;
  return true;
}

bool DjiMotor::SetRawCommand(std::uint8_t deviceId, std::int16_t rawCommand) noexcept
{
  if (!DeviceIdValid(deviceId))
  {
    return false;
  }
  motors_[deviceId - 1U].command = esc_.Saturate(rawCommand, commandKind_);
  return true;
}

bool DjiMotor::SetCurrentAmpere(std::uint8_t deviceId, float ampere) noexcept
{
  if (!DeviceIdValid(deviceId) || profile_.currentFullScaleAmpere <= 0.0F)
  {
    return false;
  }

  float ratio = ampere / profile_.currentFullScaleAmpere;
  if (ratio > 1.0F)
  {
    ratio = 1.0F;
  }
  else if (ratio < -1.0F)
  {
    ratio = -1.0F;
  }
  const std::int16_t fullScaleCounts =
      static_cast<std::int16_t>(esc_.GetCommandSpec(commandKind_).controlFullScaleCounts);
  return SetRawCommand(deviceId, static_cast<std::int16_t>(ratio * fullScaleCounts));
}

bool DjiMotor::SetTorque(std::uint8_t deviceId, float newtonMeter) noexcept
{
  if (profile_.torqueConstantNewtonMeterPerAmpere <= 0.0F)
  {
    return false;
  }
  return SetCurrentAmpere(deviceId, newtonMeter / profile_.torqueConstantNewtonMeterPerAmpere);
}

bool DjiMotor::SetTorqueRatio(std::uint8_t deviceId, float ratio) noexcept
{
  if (!DeviceIdValid(deviceId))
  {
    return false;
  }
  if (ratio > 1.0F)
  {
    ratio = 1.0F;
  }
  else if (ratio < -1.0F)
  {
    ratio = -1.0F;
  }
  const std::int16_t fullScaleCounts =
      static_cast<std::int16_t>(esc_.GetCommandSpec(commandKind_).controlFullScaleCounts);
  return SetRawCommand(deviceId, static_cast<std::int16_t>(ratio * fullScaleCounts));
}

void DjiMotor::ClearAllCommands() noexcept
{
  for (MotorState &motor : motors_)
  {
    motor.command = 0;
  }
}

bool DjiMotor::GetSnapshot(std::uint8_t deviceId, Snapshot &snapshot) const noexcept
{
  if (deviceId == 0U || deviceId > MaximumMotors)
  {
    return false;
  }

  Snapshot staged{};
  std::uint32_t sequenceBefore = 0U;
  std::uint32_t sequenceAfter = 0U;
  do
  {
    sequenceBefore = snapshotSequence_.load(std::memory_order_acquire);
    if ((sequenceBefore & 1U) != 0U)
    {
      continue;
    }
    staged = publishedSnapshots_[deviceId - 1U];
    sequenceAfter = snapshotSequence_.load(std::memory_order_acquire);
  } while (sequenceBefore != sequenceAfter || (sequenceAfter & 1U) != 0U);

  snapshot = staged;
  return true;
}

bool DjiMotor::GetStatistics(Statistics &statistics) const noexcept
{
  Statistics staged{};
  std::uint32_t sequenceBefore = 0U;
  std::uint32_t sequenceAfter = 0U;
  do
  {
    sequenceBefore = snapshotSequence_.load(std::memory_order_acquire);
    if ((sequenceBefore & 1U) != 0U)
    {
      continue;
    }
    staged = publishedStatistics_;
    sequenceAfter = snapshotSequence_.load(std::memory_order_acquire);
  } while (sequenceBefore != sequenceAfter || (sequenceAfter & 1U) != 0U);

  statistics = staged;
  return true;
}

void DjiMotor::ReceiveFrames() noexcept
{
  platform::Can::Frame frame{};
  for (;;)
  {
    const platform::Can::ReceiveResult result = platform::Can::TryReceive(channel_, frame);
    if (result != platform::Can::ReceiveResult::Received)
    {
      return;
    }

    if (esc_.DecodeFeedback(frame.identifier,
                            frame.identifierType == platform::Can::IdentifierType::Extended,
                            frame.length,
                            frame.data.data(),
                            feedback_) != protocol::DjiEsc::DecodeResult::Accepted)
    {
      continue;
    }

    const std::size_t index = feedback_.deviceId - 1U;
    if (index >= motors_.size())
    {
      continue;
    }

    MotorState &motor = motors_[index];
    if (motor.hasAngle)
    {
      std::int32_t delta =
          static_cast<std::int32_t>(feedback_.rotorAngleRaw) - static_cast<std::int32_t>(motor.lastAngleRaw);
      constexpr std::int32_t counts = protocol::DjiEsc::AngleCountsPerRevolution;
      // 恰好半圈时保留符号；该速率下方向本身有歧义。
      if (delta > counts / 2)
      {
        delta -= counts;
      }
      else if (delta < -counts / 2)
      {
        delta += counts;
      }
      motor.totalAngleCounts += delta;
    }
    else
    {
      motor.hasAngle = true;
    }
    motor.lastAngleRaw = feedback_.rotorAngleRaw;
    motor.feedback = feedback_;
    motor.lastUpdateTick = platform::Time::NowTicks();
    motor.everReceived = true;
    motor.stale = false;
    ++motor.acceptedFrameCount;
    dirty_ = true;
  }
}

void DjiMotor::UpdateLinkAging() noexcept
{
  for (MotorState &motor : motors_)
  {
    if (!motor.everReceived || motor.stale)
    {
      continue;
    }
    if (platform::Time::HasElapsedMs(motor.lastUpdateTick, LinkTimeoutMs))
    {
      motor.stale = true;
      dirty_ = true;
    }
  }
}

void DjiMotor::TransmitCommands() noexcept
{
  const protocol::DjiEsc::Group groups[]{protocol::DjiEsc::Group::First, protocol::DjiEsc::Group::Second};
  for (const protocol::DjiEsc::Group group : groups)
  {
    if (!GroupEnabled(group))
    {
      continue;
    }

    std::array<std::int16_t, protocol::DjiEsc::DevicesPerFrame> targets{};
    for (std::size_t slot = 0U; slot < targets.size(); ++slot)
    {
      const std::size_t index =
          group == protocol::DjiEsc::Group::First ? slot : slot + protocol::DjiEsc::DevicesPerFrame;
      targets[slot] = motors_[index].command;
    }

    protocol::DjiEsc::ControlFrame command{};
    if (!esc_.Encode(commandKind_, group, targets, command))
    {
      continue;
    }

    platform::Can::Frame frame{};
    frame.identifier = command.identifier;
    frame.identifierType = platform::Can::IdentifierType::Standard;
    frame.length = static_cast<std::uint8_t>(command.data.size());
    frame.data = command.data;
    const platform::Can::SendResult result = platform::Can::TrySend(channel_, frame);
    const std::size_t index =
        static_cast<std::size_t>(result) < SendResultCount ? static_cast<std::size_t>(result) : SendResultCount - 1U;
    ++txResultCounts_[index];
    dirty_ = true;
  }
}

void DjiMotor::Publish() noexcept
{
  snapshotSequence_.fetch_add(1U, std::memory_order_acq_rel);

  for (std::size_t index = 0U; index < motors_.size(); ++index)
  {
    FillSnapshot(index, publishedSnapshots_[index]);
  }

  const protocol::DjiEsc::Statistics &escStatistics = esc_.GetStatistics();
  publishedStatistics_.txResultCounts = txResultCounts_;
  publishedStatistics_.acceptedFrameCount = escStatistics.acceptedFrameCount;
  publishedStatistics_.rejectedFrameCount = escStatistics.rejectedFrameCount;
  publishedStatistics_.unknownIdentifierCount = escStatistics.unknownIdentifierCount;
  publishedStatistics_.rxDroppedCount = portStatistics_.rxDroppedCount;
  publishedStatistics_.rxHardwareLossEventCount = portStatistics_.rxHardwareLossEventCount;
  publishedStatistics_.busOffCount = portStatistics_.busOffCount;

  snapshotSequence_.fetch_add(1U, std::memory_order_release);
  dirty_ = false;
}

void DjiMotor::FillSnapshot(std::size_t index, Snapshot &snapshot) const noexcept
{
  const MotorState &motor = motors_[index];

  snapshot = Snapshot{};
  snapshot.deviceId = static_cast<std::uint8_t>(index + 1U);
  snapshot.enabled = motor.enabled;
  snapshot.everReceived = motor.everReceived;
  snapshot.online = motor.everReceived && !motor.stale;
  snapshot.lastUpdateTick = motor.lastUpdateTick;
  snapshot.acceptedFrameCount = motor.acceptedFrameCount;
  snapshot.rotorAngleRaw = motor.feedback.rotorAngleRaw;
  snapshot.rotorAngleDegrees = protocol::DjiEsc::RotorAngleDegrees(motor.feedback.rotorAngleRaw);
  snapshot.totalAngleCounts = motor.totalAngleCounts;
  snapshot.totalTurns = motor.totalAngleCounts / static_cast<std::int32_t>(protocol::DjiEsc::AngleCountsPerRevolution);
  snapshot.totalAngleDegrees = static_cast<float>(motor.totalAngleCounts) * 360.0F /
                               static_cast<float>(protocol::DjiEsc::AngleCountsPerRevolution);
  snapshot.rotorSpeedRpm = motor.feedback.rotorSpeedRaw;
  snapshot.torqueCurrentRaw = motor.feedback.torqueCurrentRaw;
  snapshot.torqueCurrentRatio =
      protocol::DjiEsc::ControlRatio(motor.feedback.torqueCurrentRaw, esc_.GetCommandSpec(commandKind_));
  snapshot.motorTemperatureCelsius = profile_.hasTemperatureFeedback ? motor.feedback.motorTemperatureCelsius : 0U;
}

bool DjiMotor::DeviceIdValid(std::uint8_t deviceId) const noexcept
{
  return deviceId >= 1U && deviceId <= MaximumDeviceCount();
}

bool DjiMotor::GroupEnabled(protocol::DjiEsc::Group group) const noexcept
{
  const std::size_t first = group == protocol::DjiEsc::Group::First ? 0U : protocol::DjiEsc::DevicesPerFrame;
  for (std::size_t index = first; index < first + protocol::DjiEsc::DevicesPerFrame && index < motors_.size(); ++index)
  {
    if (motors_[index].enabled)
    {
      return true;
    }
  }
  return false;
}

} // namespace device
