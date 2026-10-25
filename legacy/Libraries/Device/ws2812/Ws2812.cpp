#include "Libraries/Device/ws2812/Ws2812.hpp"

namespace device
{

void Ws2812::SetColor(RgbColor color) noexcept
{
  if (color_ == color)
  {
    return;
  }

  color_ = color;
  refreshPending_ = true;
}

void Ws2812::SetBrightness(std::uint8_t brightness) noexcept
{
  if (brightness_ == brightness)
  {
    return;
  }

  brightness_ = brightness;
  refreshPending_ = true;
}

void Ws2812::Off() noexcept
{
  SetColor(ws2812_color::Black);
}

void Ws2812::Process() noexcept
{
  if (state_ == State::Transmitting)
  {
    HandleCompletedTransfer();
    if (state_ == State::Transmitting || state_ == State::NotReady || state_ == State::Error)
    {
      return;
    }
  }

  if (!refreshPending_)
  {
    return;
  }

  BuildFrame();
  HandleStartResult(platform::Spi::StartTransmitAsync(platform::Spi::Device::AddressableLed, frame_, nullptr, nullptr));
}

std::uint8_t Ws2812::Scale(std::uint8_t value, std::uint8_t brightness) noexcept
{
  const std::uint16_t product = static_cast<std::uint16_t>(value) * static_cast<std::uint16_t>(brightness);
  return static_cast<std::uint8_t>((product + 127U) / 255U);
}

void Ws2812::EncodeByte(std::uint8_t value, Frame &frame, std::size_t offset) noexcept
{
  for (std::size_t bit = 0U; bit < 8U; ++bit)
  {
    const std::uint8_t mask = static_cast<std::uint8_t>(0x80U >> bit);
    frame[offset + bit] = (value & mask) != 0U ? Level1 : Level0;
  }
}

void Ws2812::BuildFrame() noexcept
{
  frame_.fill(0U);
  EncodeByte(Scale(color_.green, brightness_), frame_, 0U);
  EncodeByte(Scale(color_.red, brightness_), frame_, 8U);
  EncodeByte(Scale(color_.blue, brightness_), frame_, 16U);
}

void Ws2812::HandleCompletedTransfer() noexcept
{
  switch (platform::Spi::GetAsyncResult(platform::Spi::Device::AddressableLed))
  {
  case platform::Spi::Result::Busy:
  case platform::Spi::Result::Started:
    return;

  case platform::Spi::Result::Completed:
    state_ = State::Idle;
    return;

  case platform::Spi::Result::NotReady:
    state_ = State::NotReady;
    refreshPending_ = true;
    return;

  default:
    state_ = State::Error;
    refreshPending_ = true;
    return;
  }
}

void Ws2812::HandleStartResult(platform::Spi::Result result) noexcept
{
  switch (result)
  {
  case platform::Spi::Result::Started:
    state_ = State::Transmitting;
    refreshPending_ = false;
    return;

  case platform::Spi::Result::Completed:
    state_ = State::Idle;
    refreshPending_ = false;
    return;

  case platform::Spi::Result::Busy:
    state_ = State::Busy;
    return;

  case platform::Spi::Result::NotReady:
    state_ = State::NotReady;
    return;

  default:
    state_ = State::Error;
    return;
  }
}

} // namespace device
