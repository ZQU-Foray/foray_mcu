#include "Libraries/Device/bmi088/Bmi088Accel.hpp"

namespace device
{

namespace
{

constexpr std::uint8_t ReadFlag{0x80U};

constexpr std::uint8_t ChipIdRegister{0x00U};
constexpr std::uint8_t DataRegister{0x12U};
constexpr std::uint8_t ConfRegister{0x40U};
constexpr std::uint8_t RangeRegister{0x41U};
constexpr std::uint8_t Interrupt1IoControlRegister{0x53U};
constexpr std::uint8_t InterruptMapDataRegister{0x58U};
constexpr std::uint8_t PowerConfRegister{0x7CU};
constexpr std::uint8_t PowerCtrlRegister{0x7DU};

constexpr std::uint8_t PowerActiveMode{0x00U};
constexpr std::uint8_t PowerOnAccel{0x04U};
// INT1_IO_CTRL(0x53)：bit4 int1_in、bit3 int1_out（输出使能）、bit2 int1_od、
// bit1 int1_lvl（1＝高有效）。MCU 侧 EXTI 是上升沿触发，所以必须同时置位
// int1_out 与 int1_lvl：0x0A。旧值 0x08 只置了输出使能而把引脚配成低有效，
// DRDY 只在脉冲回到高电平时才被上升沿采到，DRDY→读取的延迟被拉长。
constexpr std::uint8_t Interrupt1OutputActiveHigh{0x0AU};
constexpr std::uint8_t MapDataReadyToInterrupt1{0x04U};

constexpr std::uint32_t PowerConfSettleMs{1U};
constexpr std::uint32_t PowerCtrlSettleMs{50U};
constexpr std::uint32_t ConfigSettleMs{1U};
constexpr std::uint32_t ConfVerifySettleMs{5U};

} // namespace

void Bmi088Accel::Init() noexcept
{
  state_ = State::Initializing;
  step_ = Step::CheckChipId;
  stepTick_ = platform::Time::NowTicks();
  stepWaitMs_ = PowerUpSettleMs;
  transferActive_ = false;
  sampleValid_ = false;
  sample_ = Sample{};
  readCount_ = 0U;
  errorCount_ = 0U;
  chipIdAttempts_ = 0U;
  configurationVerifyAttempts_ = 0U;
  transmitBuffer_.fill(0U);
}

void Bmi088Accel::Harvest() noexcept
{
  if (state_ != State::Initializing && state_ != State::Ready)
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
    CompleteStep();
    return;

  default:
    transferActive_ = false;
    Fail();
    return;
  }
}

bool Bmi088Accel::TryStart() noexcept
{
  if (state_ != State::Initializing && state_ != State::Ready)
  {
    return false;
  }

  if (transferActive_ || !WaitElapsed())
  {
    return false;
  }

  BeginStep();
  return transferActive_;
}

bool Bmi088Accel::TryGetSample(Sample &sample) const noexcept
{
  if (!sampleValid_)
  {
    return false;
  }

  sample = sample_;
  return true;
}

bool Bmi088Accel::WaitElapsed() const noexcept
{
  return stepWaitMs_ == 0U || platform::Time::HasElapsedMs(stepTick_, stepWaitMs_);
}

void Bmi088Accel::BeginStep() noexcept
{
  std::size_t length = 0U;

  transmitBuffer_.fill(0U);
  switch (step_)
  {
  case Step::CheckChipId:
    transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | ChipIdRegister);
    length = 3U;
    break;

  case Step::WritePowerConf:
    transmitBuffer_[0] = PowerConfRegister;
    transmitBuffer_[1] = PowerActiveMode;
    length = 2U;
    break;

  case Step::WritePowerCtrl:
    transmitBuffer_[0] = PowerCtrlRegister;
    transmitBuffer_[1] = PowerOnAccel;
    length = 2U;
    break;

  case Step::WriteRange:
    transmitBuffer_[0] = RangeRegister;
    transmitBuffer_[1] = RangeRegisterValue;
    length = 2U;
    break;

  case Step::WriteConf:
    transmitBuffer_[0] = ConfRegister;
    transmitBuffer_[1] = ConfRegisterValue;
    length = 2U;
    break;

  case Step::WriteInterruptIo:
    transmitBuffer_[0] = Interrupt1IoControlRegister;
    transmitBuffer_[1] = Interrupt1OutputActiveHigh;
    length = 2U;
    break;

  case Step::WriteInterruptMap:
    transmitBuffer_[0] = InterruptMapDataRegister;
    transmitBuffer_[1] = MapDataReadyToInterrupt1;
    length = 2U;
    break;

  case Step::VerifyCoreConfiguration:
    transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | ConfRegister);
    length = 4U;
    break;

  case Step::VerifyInterruptIo:
    transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | Interrupt1IoControlRegister);
    length = 3U;
    break;

  case Step::VerifyInterruptMap:
    transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | InterruptMapDataRegister);
    length = 3U;
    break;

  case Step::ReadData:
    transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | DataRegister);
    length = 8U;
    break;
  }

  const platform::Spi::Result result = platform::Spi::StartTransferAsync(platform::Spi::Device::ImuAccelerometer,
                                                                         transmitBuffer_.data(),
                                                                         receiveBuffer_.data(),
                                                                         length,
                                                                         Bmi088Transfer::Complete,
                                                                         &completion_);
  switch (result)
  {
  case platform::Spi::Result::Started:
    transferActive_ = true;
    return;

  case platform::Spi::Result::Busy:
    return;

  default:
    Fail();
    return;
  }
}

void Bmi088Accel::CompleteStep() noexcept
{
  switch (step_)
  {
  case Step::CheckChipId:
    if (receiveBuffer_[2] != ExpectedChipId)
    {
      ++chipIdAttempts_;
      if (chipIdAttempts_ < ChipIdMaxAttempts)
      {
        Advance(Step::CheckChipId, ChipIdRetryWaitMs);
        return;
      }
      Fail();
      return;
    }
    chipIdAttempts_ = 0U;
    Advance(Step::WritePowerConf, 0U);
    return;

  case Step::WritePowerConf:
    Advance(Step::WritePowerCtrl, PowerConfSettleMs);
    return;

  case Step::WritePowerCtrl:
    Advance(Step::WriteRange, PowerCtrlSettleMs);
    return;

  case Step::WriteRange:
    Advance(Step::WriteConf, ConfigSettleMs);
    return;

  case Step::WriteConf:
    Advance(Step::WriteInterruptIo, ConfigSettleMs);
    return;

  case Step::WriteInterruptIo:
    Advance(Step::WriteInterruptMap, ConfigSettleMs);
    return;

  case Step::WriteInterruptMap:
    Advance(Step::VerifyCoreConfiguration, ConfVerifySettleMs);
    return;

  case Step::VerifyCoreConfiguration:
    if (receiveBuffer_[2] != ConfRegisterValue || receiveBuffer_[3] != RangeRegisterValue)
    {
      ++configurationVerifyAttempts_;
      if (configurationVerifyAttempts_ < ConfigurationVerifyMaxAttempts)
      {
        Advance(Step::VerifyCoreConfiguration, ConfigurationVerifyRetryWaitMs);
        return;
      }
      Fail();
      return;
    }

    Advance(Step::VerifyInterruptIo, 0U);
    return;

  case Step::VerifyInterruptIo:
    if (receiveBuffer_[2] != Interrupt1OutputActiveHigh)
    {
      ++configurationVerifyAttempts_;
      if (configurationVerifyAttempts_ < ConfigurationVerifyMaxAttempts)
      {
        Advance(Step::VerifyInterruptIo, ConfigurationVerifyRetryWaitMs);
        return;
      }
      Fail();
      return;
    }
    Advance(Step::VerifyInterruptMap, 0U);
    return;

  case Step::VerifyInterruptMap:
    if (receiveBuffer_[2] != MapDataReadyToInterrupt1)
    {
      ++configurationVerifyAttempts_;
      if (configurationVerifyAttempts_ < ConfigurationVerifyMaxAttempts)
      {
        Advance(Step::VerifyInterruptMap, ConfigurationVerifyRetryWaitMs);
        return;
      }
      Fail();
      return;
    }

    configurationVerifyAttempts_ = 0U;
    state_ = State::Ready;
    sampleValid_ = false;
    Advance(Step::ReadData, 0U);
    return;

  case Step::ReadData:
    sample_.xAxis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receiveBuffer_[2]) |
                                              (static_cast<std::uint16_t>(receiveBuffer_[3]) << 8U));
    sample_.yAxis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receiveBuffer_[4]) |
                                              (static_cast<std::uint16_t>(receiveBuffer_[5]) << 8U));
    sample_.zAxis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receiveBuffer_[6]) |
                                              (static_cast<std::uint16_t>(receiveBuffer_[7]) << 8U));
    sampleValid_ = true;
    ++readCount_;
    return;
  }
}

void Bmi088Accel::Advance(Step nextStep, std::uint32_t waitMs) noexcept
{
  step_ = nextStep;
  stepWaitMs_ = waitMs;
  stepTick_ = platform::Time::NowTicks();
}

void Bmi088Accel::Fail() noexcept
{
  ++errorCount_;
  state_ = State::Error;
  transferActive_ = false;
}

} // namespace device
