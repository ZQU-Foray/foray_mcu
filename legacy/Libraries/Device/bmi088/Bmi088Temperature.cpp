#include "Libraries/Device/bmi088/Bmi088Temperature.hpp"

namespace device
{

namespace
{

constexpr std::uint8_t ReadFlag{0x80U};

constexpr std::uint8_t DataRegister{0x22U};

} // namespace

void Bmi088Temperature::Init() noexcept
{
  state_ = State::Disabled;
  transferActive_ = false;
  sampleValid_ = false;
  sample_ = Sample{};
  readCount_ = 0U;
  errorCount_ = 0U;
  transmitBuffer_.fill(0U);
}

void Bmi088Temperature::Enable() noexcept
{
  if (state_ != State::Disabled)
  {
    return;
  }

  state_ = State::Running;
}

void Bmi088Temperature::Harvest() noexcept
{
  if (state_ != State::Running)
  {
    return;
  }

  if (!transferActive_)
  {
    return;
  }

  switch (platform::Spi::GetAsyncResult(platform::Spi::Device::ImuAccelerometer))
  {
  case platform::Spi::Result::Busy:
  case platform::Spi::Result::Started:
    return;

  case platform::Spi::Result::Completed:
    transferActive_ = false;
    {
      // 接收布局：[0]地址 [1]保留字节 [2]TEMP_MSB(0x22) [3]TEMP_LSB(0x23)，
      // 原始值为 11 位 (MSB<<3 | LSB>>5)，按符号位扩展。
      const std::uint32_t packed = (static_cast<std::uint32_t>(receiveBuffer_[2]) << 3U) | (receiveBuffer_[3] >> 5U);
      sample_.raw = static_cast<std::int16_t>(static_cast<std::int32_t>((packed ^ 0x400U) - 0x400U));
    }
    sampleValid_ = true;
    ++readCount_;
    return;

  default:
    transferActive_ = false;
    Fail();
    return;
  }
}

bool Bmi088Temperature::TryStart() noexcept
{
  if (state_ != State::Running || transferActive_)
  {
    return false;
  }

  transmitBuffer_.fill(0U);
  transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | DataRegister);
  const platform::Spi::Result result = platform::Spi::StartTransferAsync(platform::Spi::Device::ImuAccelerometer,
                                                                         transmitBuffer_.data(),
                                                                         receiveBuffer_.data(),
                                                                         transmitBuffer_.size(),
                                                                         Bmi088Transfer::Complete,
                                                                         &completion_);
  switch (result)
  {
  case platform::Spi::Result::Started:
    transferActive_ = true;
    return true;

  case platform::Spi::Result::Busy:
    return false;

  default:
    Fail();
    return false;
  }
}

bool Bmi088Temperature::TryGetSample(Sample &sample) const noexcept
{
  if (!sampleValid_)
  {
    return false;
  }

  sample = sample_;
  return true;
}

void Bmi088Temperature::Fail() noexcept
{
  ++errorCount_;
  state_ = State::Error;
  transferActive_ = false;
}

} // namespace device
