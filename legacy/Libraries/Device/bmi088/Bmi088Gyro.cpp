#include "Libraries/Device/bmi088/Bmi088Gyro.hpp"

namespace device
{

namespace
{

constexpr std::uint8_t ReadFlag{0x80U};

constexpr std::uint8_t ChipIdRegister{0x00U};
constexpr std::uint8_t DataRegister{0x02U};
constexpr std::uint8_t RangeRegister{0x0FU};
constexpr std::uint8_t BandwidthRegister{0x10U};
constexpr std::uint8_t InterruptControlRegister{0x15U};
constexpr std::uint8_t InterruptIoConfigurationRegister{0x16U};
constexpr std::uint8_t InterruptMapRegister{0x18U};

constexpr std::uint8_t EnableDataReadyInterrupt{0x80U};
// INT3_INT4_IO_CONF(0x16)：bit0 int3_lvl（1＝高有效）、bit1 int3_od、
// bit2 int4_lvl、bit3 int4_od。MCU 侧 EXTI 是上升沿触发，INT3 必须配成
// 高有效推挽：0x01。旧值 0x0C 实际把 INT3 配成低有效推挽，并把未使用的
// INT4 配成高有效开漏（板级无上拉，引脚永远上不去）；两者都会让 DRDY 边沿
// 对振动与漏电敏感，进而影响采集时序与合并计数。
constexpr std::uint8_t Interrupt3ActiveHighPushPull{0x01U};
constexpr std::uint8_t MapDataReadyToInterrupt3{0x01U};

constexpr std::uint32_t ConfigSettleMs{1U};
constexpr std::uint32_t ConfigVerifySettleMs{1U};

} // namespace

void Bmi088Gyro::Init() noexcept
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

void Bmi088Gyro::Harvest() noexcept
{
  if (state_ != State::Initializing && state_ != State::Ready)
  {
    return;
  }

  if (!transferActive_)
  {
    return;
  }

  switch (platform::Spi::GetAsyncResult(platform::Spi::Device::ImuGyroscope))
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

bool Bmi088Gyro::TryStart() noexcept
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

bool Bmi088Gyro::TryGetSample(Sample &sample) const noexcept
{
  if (!sampleValid_)
  {
    return false;
  }

  sample = sample_;
  return true;
}

bool Bmi088Gyro::WaitElapsed() const noexcept
{
  return stepWaitMs_ == 0U || platform::Time::HasElapsedMs(stepTick_, stepWaitMs_);
}

void Bmi088Gyro::BeginStep() noexcept
{
  std::size_t length = 0U;

  transmitBuffer_.fill(0U);
  switch (step_)
  {
  case Step::CheckChipId:
    transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | ChipIdRegister);
    length = 2U;
    break;

  case Step::WriteRange:
    transmitBuffer_[0] = RangeRegister;
    transmitBuffer_[1] = RangeRegisterValue;
    length = 2U;
    break;

  case Step::WriteBandwidth:
    transmitBuffer_[0] = BandwidthRegister;
    transmitBuffer_[1] = BandwidthRegisterValue;
    length = 2U;
    break;

  case Step::WriteInterruptControl:
    transmitBuffer_[0] = InterruptControlRegister;
    transmitBuffer_[1] = EnableDataReadyInterrupt;
    length = 2U;
    break;

  case Step::WriteInterruptIo:
    transmitBuffer_[0] = InterruptIoConfigurationRegister;
    transmitBuffer_[1] = Interrupt3ActiveHighPushPull;
    length = 2U;
    break;

  case Step::WriteInterruptMap:
    transmitBuffer_[0] = InterruptMapRegister;
    transmitBuffer_[1] = MapDataReadyToInterrupt3;
    length = 2U;
    break;

  case Step::VerifyCoreConfiguration:
    transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | RangeRegister);
    length = 3U;
    break;

  case Step::VerifyInterruptControlAndIo:
    transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | InterruptControlRegister);
    length = 3U;
    break;

  case Step::VerifyInterruptMap:
    transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | InterruptMapRegister);
    length = 2U;
    break;

  case Step::ReadData:
    transmitBuffer_[0] = static_cast<std::uint8_t>(ReadFlag | DataRegister);
    length = 7U;
    break;
  }

  const platform::Spi::Result result = platform::Spi::StartTransferAsync(platform::Spi::Device::ImuGyroscope,
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

void Bmi088Gyro::CompleteStep() noexcept
{
  switch (step_)
  {
  case Step::CheckChipId:
    if (receiveBuffer_[1] != ExpectedChipId)
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
    Advance(Step::WriteRange, 0U);
    return;

  case Step::WriteRange:
    Advance(Step::WriteBandwidth, ConfigSettleMs);
    return;

  case Step::WriteBandwidth:
    Advance(Step::WriteInterruptControl, ConfigSettleMs);
    return;

  case Step::WriteInterruptControl:
    Advance(Step::WriteInterruptIo, ConfigSettleMs);
    return;

  case Step::WriteInterruptIo:
    Advance(Step::WriteInterruptMap, ConfigSettleMs);
    return;

  case Step::WriteInterruptMap:
    Advance(Step::VerifyCoreConfiguration, ConfigVerifySettleMs);
    return;

  case Step::VerifyCoreConfiguration:
    if ((receiveBuffer_[1] & RangeRegisterReadMask) != (RangeRegisterValue & RangeRegisterReadMask) ||
        (receiveBuffer_[2] & BandwidthRegisterReadMask) != (BandwidthRegisterValue & BandwidthRegisterReadMask))
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

    Advance(Step::VerifyInterruptControlAndIo, 0U);
    return;

  case Step::VerifyInterruptControlAndIo:
    if (receiveBuffer_[1] != EnableDataReadyInterrupt || receiveBuffer_[2] != Interrupt3ActiveHighPushPull)
    {
      ++configurationVerifyAttempts_;
      if (configurationVerifyAttempts_ < ConfigurationVerifyMaxAttempts)
      {
        Advance(Step::VerifyInterruptControlAndIo, ConfigurationVerifyRetryWaitMs);
        return;
      }
      Fail();
      return;
    }
    Advance(Step::VerifyInterruptMap, 0U);
    return;

  case Step::VerifyInterruptMap:
    if (receiveBuffer_[1] != MapDataReadyToInterrupt3)
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
    sample_.xAxis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receiveBuffer_[1]) |
                                              (static_cast<std::uint16_t>(receiveBuffer_[2]) << 8U));
    sample_.yAxis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receiveBuffer_[3]) |
                                              (static_cast<std::uint16_t>(receiveBuffer_[4]) << 8U));
    sample_.zAxis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receiveBuffer_[5]) |
                                              (static_cast<std::uint16_t>(receiveBuffer_[6]) << 8U));
    sampleValid_ = true;
    ++readCount_;
    return;
  }
}

void Bmi088Gyro::Advance(Step nextStep, std::uint32_t waitMs) noexcept
{
  step_ = nextStep;
  stepWaitMs_ = waitMs;
  stepTick_ = platform::Time::NowTicks();
}

void Bmi088Gyro::Fail() noexcept
{
  ++errorCount_;
  state_ = State::Error;
  transferActive_ = false;
}

} // namespace device
