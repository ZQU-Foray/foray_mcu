#include "Libraries/Device/buzzer/Buzzer.hpp"

namespace device
{

bool Buzzer::is_ready() const noexcept
{
    return platform::Pwm::is_ready(platform::Pwm::Channel::BUZZER);
}

Buzzer::Result Buzzer::play(std::uint32_t frequency_hz, std::uint8_t volume) noexcept
{
    if (volume == 0U)
    {
        return stop();
    }

    if (frequency_hz < MIN_FREQUENCY_HZ || frequency_hz > MAX_FREQUENCY_HZ)
    {
        last_result_ = Result::INVALID_ARGUMENT;
        return last_result_;
    }

    last_result_ = platform::Pwm::set(platform::Pwm::Channel::BUZZER, frequency_hz, volume_to_duty_permille(volume));
    if (last_result_ == Result::COMPLETED)
    {
        state_ = State::PLAYING;
        frequency_hz_ = frequency_hz;
        volume_ = volume;
    }

    return last_result_;
}

Buzzer::Result Buzzer::stop() noexcept
{
    last_result_ = platform::Pwm::silence(platform::Pwm::Channel::BUZZER);
    if (last_result_ == Result::COMPLETED)
    {
        state_ = State::STOPPED;
        frequency_hz_ = 0U;
        volume_ = 0U;
    }

    return last_result_;
}

std::uint16_t Buzzer::volume_to_duty_permille(std::uint8_t volume) noexcept
{
    return static_cast<std::uint16_t>((static_cast<std::uint32_t>(volume) * MAX_DUTY_PERMILLE + MAX_VOLUME / 2U) /
                                      MAX_VOLUME);
}

} // namespace device
