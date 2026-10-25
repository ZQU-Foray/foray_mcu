#include "Libraries/Device/buzzer/Buzzer.hpp"

namespace device
{

bool Buzzer::IsReady() const noexcept
{
  return platform::Pwm::IsReady(platform::Pwm::Channel::Buzzer);
}

Buzzer::Result Buzzer::Play(std::uint32_t frequencyHz, std::uint8_t volume) noexcept
{
  if (volume == 0U)
  {
    return Stop();
  }

  if (frequencyHz < MinFrequencyHz || frequencyHz > MaxFrequencyHz)
  {
    lastResult_ = Result::InvalidArgument;
    return lastResult_;
  }

  lastResult_ = platform::Pwm::Set(platform::Pwm::Channel::Buzzer, frequencyHz, VolumeToDutyPermille(volume));
  if (lastResult_ == Result::Completed)
  {
    state_ = State::Playing;
    frequencyHz_ = frequencyHz;
    volume_ = volume;
  }

  return lastResult_;
}

Buzzer::Result Buzzer::Stop() noexcept
{
  lastResult_ = platform::Pwm::Silence(platform::Pwm::Channel::Buzzer);
  if (lastResult_ == Result::Completed)
  {
    state_ = State::Stopped;
    frequencyHz_ = 0U;
    volume_ = 0U;
  }

  return lastResult_;
}

std::uint16_t Buzzer::VolumeToDutyPermille(std::uint8_t volume) noexcept
{
  return static_cast<std::uint16_t>((static_cast<std::uint32_t>(volume) * MaxDutyPermille + MaxVolume / 2U) /
                                    MaxVolume);
}

} // namespace device
