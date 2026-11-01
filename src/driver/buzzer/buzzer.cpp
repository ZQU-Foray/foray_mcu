#include "driver/buzzer/buzzer.hpp"

namespace device
{

bool Buzzer::is_ready() const noexcept
{
    return pwm_.is_ready();
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

    last_result_ = pwm_.set(frequency_hz, volume_to_duty_permille(volume));
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
    last_result_ = pwm_.off();
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
