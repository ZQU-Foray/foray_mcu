#include "driver/ws2812/ws2812.hpp"

namespace device
{

void Ws2812::set_color(RgbColor color) noexcept
{
    if (color_ == color)
    {
        return;
    }

    color_ = color;
    refresh_pending_ = true;
}

void Ws2812::set_brightness(std::uint8_t brightness) noexcept
{
    if (brightness_ == brightness)
    {
        return;
    }

    brightness_ = brightness;
    refresh_pending_ = true;
}

void Ws2812::off() noexcept
{
    set_color(ws2812_color::BLACK);
}

void Ws2812::process() noexcept
{
    if (state_ == State::TRANSMITTING)
    {
        handle_completed_transfer();
        if (state_ == State::TRANSMITTING || state_ == State::NOT_READY || state_ == State::ERROR)
        {
            return;
        }
    }

    if (!refresh_pending_)
    {
        return;
    }

    build_frame();
    handle_start_result(
        platform::Spi::start_transmit_async(platform::Spi::Device::ADDRESSABLE_LED, frame_, nullptr, nullptr));
}

std::uint8_t Ws2812::scale(std::uint8_t value, std::uint8_t brightness) noexcept
{
    const std::uint16_t product = static_cast<std::uint16_t>(value) * static_cast<std::uint16_t>(brightness);
    return static_cast<std::uint8_t>((product + 127U) / 255U);
}

void Ws2812::encode_byte(std::uint8_t value, Frame &frame, std::size_t offset) noexcept
{
    for (std::size_t bit = 0U; bit < 8U; ++bit)
    {
        const std::uint8_t mask = static_cast<std::uint8_t>(0x80U >> bit);
        frame[offset + bit] = (value & mask) != 0U ? LEVEL_1 : LEVEL_0;
    }
}

void Ws2812::build_frame() noexcept
{
    frame_.fill(0U);
    encode_byte(scale(color_.green, brightness_), frame_, 0U);
    encode_byte(scale(color_.red, brightness_), frame_, 8U);
    encode_byte(scale(color_.blue, brightness_), frame_, 16U);
}

void Ws2812::handle_completed_transfer() noexcept
{
    switch (platform::Spi::get_async_result(platform::Spi::Device::ADDRESSABLE_LED))
    {
    case platform::Spi::Result::BUSY:
    case platform::Spi::Result::STARTED:
        return;

    case platform::Spi::Result::COMPLETED:
        state_ = State::IDLE;
        return;

    case platform::Spi::Result::NOT_READY:
        state_ = State::NOT_READY;
        refresh_pending_ = true;
        return;

    default:
        state_ = State::ERROR;
        refresh_pending_ = true;
        return;
    }
}

void Ws2812::handle_start_result(platform::Spi::Result result) noexcept
{
    switch (result)
    {
    case platform::Spi::Result::STARTED:
        state_ = State::TRANSMITTING;
        refresh_pending_ = false;
        return;

    case platform::Spi::Result::COMPLETED:
        state_ = State::IDLE;
        refresh_pending_ = false;
        return;

    case platform::Spi::Result::BUSY:
        state_ = State::BUSY;
        return;

    case platform::Spi::Result::NOT_READY:
        state_ = State::NOT_READY;
        return;

    default:
        state_ = State::ERROR;
        return;
    }
}

} // namespace device
