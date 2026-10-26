#include "Libraries/Device/bmi088/Bmi088Temperature.hpp"

namespace device
{

namespace
{

constexpr std::uint8_t READ_FLAG{0x80U};

constexpr std::uint8_t DATA_REGISTER{0x22U};

} // namespace

void Bmi088Temperature::init() noexcept
{
    state_ = State::DISABLED;
    transfer_active_ = false;
    sample_valid_ = false;
    sample_ = Sample{};
    read_count_ = 0U;
    error_count_ = 0U;
    transmit_buffer_.fill(0U);
}

void Bmi088Temperature::enable() noexcept
{
    if (state_ != State::DISABLED)
    {
        return;
    }

    state_ = State::RUNNING;
}

void Bmi088Temperature::harvest() noexcept
{
    if (state_ != State::RUNNING)
    {
        return;
    }

    if (!transfer_active_)
    {
        return;
    }

    switch (platform::Spi::get_async_result(platform::Spi::Device::IMU_ACCELEROMETER))
    {
    case platform::Spi::Result::BUSY:
    case platform::Spi::Result::STARTED:
        return;

    case platform::Spi::Result::COMPLETED:
        transfer_active_ = false;
        {
            // 接收布局：[0]地址 [1]保留字节 [2]TEMP_MSB(0x22) [3]TEMP_LSB(0x23)，
            // 原始值为 11 位 (MSB<<3 | LSB>>5)，按符号位扩展。
            const std::uint32_t packed =
                (static_cast<std::uint32_t>(receive_buffer_[2]) << 3U) | (receive_buffer_[3] >> 5U);
            sample_.raw = static_cast<std::int16_t>(static_cast<std::int32_t>((packed ^ 0x400U) - 0x400U));
        }
        sample_valid_ = true;
        ++read_count_;
        return;

    default:
        transfer_active_ = false;
        fail();
        return;
    }
}

bool Bmi088Temperature::try_start() noexcept
{
    if (state_ != State::RUNNING || transfer_active_)
    {
        return false;
    }

    transmit_buffer_.fill(0U);
    transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | DATA_REGISTER);
    const platform::Spi::Result result = platform::Spi::start_transfer_async(platform::Spi::Device::IMU_ACCELEROMETER,
                                                                           transmit_buffer_.data(),
                                                                           receive_buffer_.data(),
                                                                           transmit_buffer_.size(),
                                                                           Bmi088Transfer::complete,
                                                                           &completion_);
    switch (result)
    {
    case platform::Spi::Result::STARTED:
        transfer_active_ = true;
        return true;

    case platform::Spi::Result::BUSY:
        return false;

    default:
        fail();
        return false;
    }
}

bool Bmi088Temperature::try_get_sample(Sample &sample) const noexcept
{
    if (!sample_valid_)
    {
        return false;
    }

    sample = sample_;
    return true;
}

void Bmi088Temperature::fail() noexcept
{
    ++error_count_;
    state_ = State::ERROR;
    transfer_active_ = false;
}

} // namespace device
