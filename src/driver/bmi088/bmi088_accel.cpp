#include "Libraries/Device/bmi088/Bmi088Accel.hpp"

namespace device
{

namespace
{

constexpr std::uint8_t READ_FLAG{0x80U};

constexpr std::uint8_t CHIP_ID_REGISTER{0x00U};
constexpr std::uint8_t DATA_REGISTER{0x12U};
constexpr std::uint8_t CONF_REGISTER{0x40U};
constexpr std::uint8_t RANGE_REGISTER{0x41U};
constexpr std::uint8_t INTERRUPT1_IO_CONTROL_REGISTER{0x53U};
constexpr std::uint8_t INTERRUPT_MAP_DATA_REGISTER{0x58U};
constexpr std::uint8_t POWER_CONF_REGISTER{0x7CU};
constexpr std::uint8_t POWER_CTRL_REGISTER{0x7DU};

constexpr std::uint8_t POWER_ACTIVE_MODE{0x00U};
constexpr std::uint8_t POWER_ON_ACCEL{0x04U};
// INT1_IO_CTRL(0x53)：bit4 int1_in、bit3 int1_out（输出使能）、bit2 int1_od、
// bit1 int1_lvl（1＝高有效）。MCU 侧 EXTI 是上升沿触发，所以必须同时置位
// int1_out 与 int1_lvl：0x0A。旧值 0x08 只置了输出使能而把引脚配成低有效，
// DRDY 只在脉冲回到高电平时才被上升沿采到，DRDY→读取的延迟被拉长。
constexpr std::uint8_t INTERRUPT1_OUTPUT_ACTIVE_HIGH{0x0AU};
constexpr std::uint8_t MAP_DATA_READY_TO_INTERRUPT1{0x04U};

constexpr std::uint32_t POWER_CONF_SETTLE_MS{1U};
constexpr std::uint32_t POWER_CTRL_SETTLE_MS{50U};
constexpr std::uint32_t CONFIG_SETTLE_MS{1U};
constexpr std::uint32_t CONF_VERIFY_SETTLE_MS{5U};

} // namespace

void Bmi088Accel::init() noexcept
{
    state_ = State::INITIALIZING;
    step_ = Step::CHECK_CHIP_ID;
    step_tick_ = platform::Time::now_ticks();
    step_wait_ms_ = POWER_UP_SETTLE_MS;
    transfer_active_ = false;
    sample_valid_ = false;
    sample_ = Sample{};
    read_count_ = 0U;
    error_count_ = 0U;
    chip_id_attempts_ = 0U;
    configuration_verify_attempts_ = 0U;
    transmit_buffer_.fill(0U);
}

void Bmi088Accel::harvest() noexcept
{
    if (state_ != State::INITIALIZING && state_ != State::READY)
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
        complete_step();
        return;

    default:
        transfer_active_ = false;
        fail();
        return;
    }
}

bool Bmi088Accel::try_start() noexcept
{
    if (state_ != State::INITIALIZING && state_ != State::READY)
    {
        return false;
    }

    if (transfer_active_ || !wait_elapsed())
    {
        return false;
    }

    begin_step();
    return transfer_active_;
}

bool Bmi088Accel::try_get_sample(Sample &sample) const noexcept
{
    if (!sample_valid_)
    {
        return false;
    }

    sample = sample_;
    return true;
}

bool Bmi088Accel::wait_elapsed() const noexcept
{
    return step_wait_ms_ == 0U || platform::Time::has_elapsed_ms(step_tick_, step_wait_ms_);
}

void Bmi088Accel::begin_step() noexcept
{
    std::size_t length = 0U;

    transmit_buffer_.fill(0U);
    switch (step_)
    {
    case Step::CHECK_CHIP_ID:
        transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | CHIP_ID_REGISTER);
        length = 3U;
        break;

    case Step::WRITE_POWER_CONF:
        transmit_buffer_[0] = POWER_CONF_REGISTER;
        transmit_buffer_[1] = POWER_ACTIVE_MODE;
        length = 2U;
        break;

    case Step::WRITE_POWER_CTRL:
        transmit_buffer_[0] = POWER_CTRL_REGISTER;
        transmit_buffer_[1] = POWER_ON_ACCEL;
        length = 2U;
        break;

    case Step::WRITE_RANGE:
        transmit_buffer_[0] = RANGE_REGISTER;
        transmit_buffer_[1] = RANGE_REGISTER_VALUE;
        length = 2U;
        break;

    case Step::WRITE_CONF:
        transmit_buffer_[0] = CONF_REGISTER;
        transmit_buffer_[1] = CONF_REGISTER_VALUE;
        length = 2U;
        break;

    case Step::WRITE_INTERRUPT_IO:
        transmit_buffer_[0] = INTERRUPT1_IO_CONTROL_REGISTER;
        transmit_buffer_[1] = INTERRUPT1_OUTPUT_ACTIVE_HIGH;
        length = 2U;
        break;

    case Step::WRITE_INTERRUPT_MAP:
        transmit_buffer_[0] = INTERRUPT_MAP_DATA_REGISTER;
        transmit_buffer_[1] = MAP_DATA_READY_TO_INTERRUPT1;
        length = 2U;
        break;

    case Step::VERIFY_CORE_CONFIGURATION:
        transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | CONF_REGISTER);
        length = 4U;
        break;

    case Step::VERIFY_INTERRUPT_IO:
        transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | INTERRUPT1_IO_CONTROL_REGISTER);
        length = 3U;
        break;

    case Step::VERIFY_INTERRUPT_MAP:
        transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | INTERRUPT_MAP_DATA_REGISTER);
        length = 3U;
        break;

    case Step::READ_DATA:
        transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | DATA_REGISTER);
        length = 8U;
        break;
    }

    const platform::Spi::Result result = platform::Spi::start_transfer_async(platform::Spi::Device::IMU_ACCELEROMETER,
                                                                             transmit_buffer_.data(),
                                                                             receive_buffer_.data(),
                                                                             length,
                                                                             Bmi088Transfer::complete,
                                                                             &completion_);
    switch (result)
    {
    case platform::Spi::Result::STARTED:
        transfer_active_ = true;
        return;

    case platform::Spi::Result::BUSY:
        return;

    default:
        fail();
        return;
    }
}

void Bmi088Accel::complete_step() noexcept
{
    switch (step_)
    {
    case Step::CHECK_CHIP_ID:
        if (receive_buffer_[2] != EXPECTED_CHIP_ID)
        {
            ++chip_id_attempts_;
            if (chip_id_attempts_ < CHIP_ID_MAX_ATTEMPTS)
            {
                advance(Step::CHECK_CHIP_ID, CHIP_ID_RETRY_WAIT_MS);
                return;
            }
            fail();
            return;
        }
        chip_id_attempts_ = 0U;
        advance(Step::WRITE_POWER_CONF, 0U);
        return;

    case Step::WRITE_POWER_CONF:
        advance(Step::WRITE_POWER_CTRL, POWER_CONF_SETTLE_MS);
        return;

    case Step::WRITE_POWER_CTRL:
        advance(Step::WRITE_RANGE, POWER_CTRL_SETTLE_MS);
        return;

    case Step::WRITE_RANGE:
        advance(Step::WRITE_CONF, CONFIG_SETTLE_MS);
        return;

    case Step::WRITE_CONF:
        advance(Step::WRITE_INTERRUPT_IO, CONFIG_SETTLE_MS);
        return;

    case Step::WRITE_INTERRUPT_IO:
        advance(Step::WRITE_INTERRUPT_MAP, CONFIG_SETTLE_MS);
        return;

    case Step::WRITE_INTERRUPT_MAP:
        advance(Step::VERIFY_CORE_CONFIGURATION, CONF_VERIFY_SETTLE_MS);
        return;

    case Step::VERIFY_CORE_CONFIGURATION:
        if (receive_buffer_[2] != CONF_REGISTER_VALUE || receive_buffer_[3] != RANGE_REGISTER_VALUE)
        {
            ++configuration_verify_attempts_;
            if (configuration_verify_attempts_ < CONFIGURATION_VERIFY_MAX_ATTEMPTS)
            {
                advance(Step::VERIFY_CORE_CONFIGURATION, CONFIGURATION_VERIFY_RETRY_WAIT_MS);
                return;
            }
            fail();
            return;
        }

        advance(Step::VERIFY_INTERRUPT_IO, 0U);
        return;

    case Step::VERIFY_INTERRUPT_IO:
        if (receive_buffer_[2] != INTERRUPT1_OUTPUT_ACTIVE_HIGH)
        {
            ++configuration_verify_attempts_;
            if (configuration_verify_attempts_ < CONFIGURATION_VERIFY_MAX_ATTEMPTS)
            {
                advance(Step::VERIFY_INTERRUPT_IO, CONFIGURATION_VERIFY_RETRY_WAIT_MS);
                return;
            }
            fail();
            return;
        }
        advance(Step::VERIFY_INTERRUPT_MAP, 0U);
        return;

    case Step::VERIFY_INTERRUPT_MAP:
        if (receive_buffer_[2] != MAP_DATA_READY_TO_INTERRUPT1)
        {
            ++configuration_verify_attempts_;
            if (configuration_verify_attempts_ < CONFIGURATION_VERIFY_MAX_ATTEMPTS)
            {
                advance(Step::VERIFY_INTERRUPT_MAP, CONFIGURATION_VERIFY_RETRY_WAIT_MS);
                return;
            }
            fail();
            return;
        }

        configuration_verify_attempts_ = 0U;
        state_ = State::READY;
        sample_valid_ = false;
        advance(Step::READ_DATA, 0U);
        return;

    case Step::READ_DATA:
        sample_.x_axis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receive_buffer_[2]) |
                                                   (static_cast<std::uint16_t>(receive_buffer_[3]) << 8U));
        sample_.y_axis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receive_buffer_[4]) |
                                                   (static_cast<std::uint16_t>(receive_buffer_[5]) << 8U));
        sample_.z_axis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receive_buffer_[6]) |
                                                   (static_cast<std::uint16_t>(receive_buffer_[7]) << 8U));
        sample_valid_ = true;
        ++read_count_;
        return;
    }
}

void Bmi088Accel::advance(Step next_step, std::uint32_t wait_ms) noexcept
{
    step_ = next_step;
    step_wait_ms_ = wait_ms;
    step_tick_ = platform::Time::now_ticks();
}

void Bmi088Accel::fail() noexcept
{
    ++error_count_;
    state_ = State::ERROR;
    transfer_active_ = false;
}

} // namespace device
