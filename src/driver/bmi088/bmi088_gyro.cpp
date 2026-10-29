#include "driver/bmi088/bmi088_gyro.hpp"

namespace device
{

namespace
{

constexpr std::uint8_t READ_FLAG{0x80U};

constexpr std::uint8_t CHIP_ID_REGISTER{0x00U};
constexpr std::uint8_t DATA_REGISTER{0x02U};
constexpr std::uint8_t RANGE_REGISTER{0x0FU};
constexpr std::uint8_t BANDWIDTH_REGISTER{0x10U};
constexpr std::uint8_t INTERRUPT_CONTROL_REGISTER{0x15U};
constexpr std::uint8_t INTERRUPT_IO_CONFIGURATION_REGISTER{0x16U};
constexpr std::uint8_t INTERRUPT_MAP_REGISTER{0x18U};

constexpr std::uint8_t ENABLE_DATA_READY_INTERRUPT{0x80U};
// INT3_INT4_IO_CONF(0x16)：bit0 int3_lvl（1＝高有效）、bit1 int3_od、
// bit2 int4_lvl、bit3 int4_od。MCU 侧 EXTI 是上升沿触发，INT3 必须配成
// 高有效推挽：0x01。旧值 0x0C 实际把 INT3 配成低有效推挽，并把未使用的
// INT4 配成高有效开漏（板级无上拉，引脚永远上不去）；两者都会让 DRDY 边沿
// 对振动与漏电敏感，进而影响采集时序与合并计数。
constexpr std::uint8_t INTERRUPT3_ACTIVE_HIGH_PUSH_PULL{0x01U};
constexpr std::uint8_t MAP_DATA_READY_TO_INTERRUPT3{0x01U};

constexpr std::uint32_t CONFIG_SETTLE_MS{1U};
constexpr std::uint32_t CONFIG_VERIFY_SETTLE_MS{1U};

} // namespace

void Bmi088Gyro::init() noexcept
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

void Bmi088Gyro::harvest() noexcept
{
    if (state_ != State::INITIALIZING && state_ != State::READY)
    {
        return;
    }

    if (!transfer_active_)
    {
        return;
    }

    switch (platform::Spi::get_async_result(platform::Spi::Device::IMU_GYROSCOPE))
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

bool Bmi088Gyro::try_start() noexcept
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

bool Bmi088Gyro::try_get_sample(Sample &sample) const noexcept
{
    if (!sample_valid_)
    {
        return false;
    }

    sample = sample_;
    return true;
}

bool Bmi088Gyro::wait_elapsed() const noexcept
{
    return step_wait_ms_ == 0U || platform::Time::has_elapsed_ms(step_tick_, step_wait_ms_);
}

void Bmi088Gyro::begin_step() noexcept
{
    std::size_t length = 0U;

    transmit_buffer_.fill(0U);
    switch (step_)
    {
    case Step::CHECK_CHIP_ID:
        transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | CHIP_ID_REGISTER);
        length = 2U;
        break;

    case Step::WRITE_RANGE:
        transmit_buffer_[0] = RANGE_REGISTER;
        transmit_buffer_[1] = RANGE_REGISTER_VALUE;
        length = 2U;
        break;

    case Step::WRITE_BANDWIDTH:
        transmit_buffer_[0] = BANDWIDTH_REGISTER;
        transmit_buffer_[1] = BANDWIDTH_REGISTER_VALUE;
        length = 2U;
        break;

    case Step::WRITE_INTERRUPT_CONTROL:
        transmit_buffer_[0] = INTERRUPT_CONTROL_REGISTER;
        transmit_buffer_[1] = ENABLE_DATA_READY_INTERRUPT;
        length = 2U;
        break;

    case Step::WRITE_INTERRUPT_IO:
        transmit_buffer_[0] = INTERRUPT_IO_CONFIGURATION_REGISTER;
        transmit_buffer_[1] = INTERRUPT3_ACTIVE_HIGH_PUSH_PULL;
        length = 2U;
        break;

    case Step::WRITE_INTERRUPT_MAP:
        transmit_buffer_[0] = INTERRUPT_MAP_REGISTER;
        transmit_buffer_[1] = MAP_DATA_READY_TO_INTERRUPT3;
        length = 2U;
        break;

    case Step::VERIFY_CORE_CONFIGURATION:
        transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | RANGE_REGISTER);
        length = 3U;
        break;

    case Step::VERIFY_INTERRUPT_CONTROL_AND_IO:
        transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | INTERRUPT_CONTROL_REGISTER);
        length = 3U;
        break;

    case Step::VERIFY_INTERRUPT_MAP:
        transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | INTERRUPT_MAP_REGISTER);
        length = 2U;
        break;

    case Step::READ_DATA:
        transmit_buffer_[0] = static_cast<std::uint8_t>(READ_FLAG | DATA_REGISTER);
        length = 7U;
        break;
    }

    const platform::Spi::Result result = platform::Spi::start_transfer_async(platform::Spi::Device::IMU_GYROSCOPE,
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

void Bmi088Gyro::complete_step() noexcept
{
    switch (step_)
    {
    case Step::CHECK_CHIP_ID:
        if (receive_buffer_[1] != EXPECTED_CHIP_ID)
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
        advance(Step::WRITE_RANGE, 0U);
        return;

    case Step::WRITE_RANGE:
        advance(Step::WRITE_BANDWIDTH, CONFIG_SETTLE_MS);
        return;

    case Step::WRITE_BANDWIDTH:
        advance(Step::WRITE_INTERRUPT_CONTROL, CONFIG_SETTLE_MS);
        return;

    case Step::WRITE_INTERRUPT_CONTROL:
        advance(Step::WRITE_INTERRUPT_IO, CONFIG_SETTLE_MS);
        return;

    case Step::WRITE_INTERRUPT_IO:
        advance(Step::WRITE_INTERRUPT_MAP, CONFIG_SETTLE_MS);
        return;

    case Step::WRITE_INTERRUPT_MAP:
        advance(Step::VERIFY_CORE_CONFIGURATION, CONFIG_VERIFY_SETTLE_MS);
        return;

    case Step::VERIFY_CORE_CONFIGURATION:
        if ((receive_buffer_[1] & RANGE_REGISTER_READ_MASK) != (RANGE_REGISTER_VALUE & RANGE_REGISTER_READ_MASK) ||
            (receive_buffer_[2] & BANDWIDTH_REGISTER_READ_MASK) !=
                (BANDWIDTH_REGISTER_VALUE & BANDWIDTH_REGISTER_READ_MASK))
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

        advance(Step::VERIFY_INTERRUPT_CONTROL_AND_IO, 0U);
        return;

    case Step::VERIFY_INTERRUPT_CONTROL_AND_IO:
        if (receive_buffer_[1] != ENABLE_DATA_READY_INTERRUPT || receive_buffer_[2] != INTERRUPT3_ACTIVE_HIGH_PUSH_PULL)
        {
            ++configuration_verify_attempts_;
            if (configuration_verify_attempts_ < CONFIGURATION_VERIFY_MAX_ATTEMPTS)
            {
                advance(Step::VERIFY_INTERRUPT_CONTROL_AND_IO, CONFIGURATION_VERIFY_RETRY_WAIT_MS);
                return;
            }
            fail();
            return;
        }
        advance(Step::VERIFY_INTERRUPT_MAP, 0U);
        return;

    case Step::VERIFY_INTERRUPT_MAP:
        if (receive_buffer_[1] != MAP_DATA_READY_TO_INTERRUPT3)
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
        sample_.x_axis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receive_buffer_[1]) |
                                                   (static_cast<std::uint16_t>(receive_buffer_[2]) << 8U));
        sample_.y_axis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receive_buffer_[3]) |
                                                   (static_cast<std::uint16_t>(receive_buffer_[4]) << 8U));
        sample_.z_axis = static_cast<std::int16_t>(static_cast<std::uint16_t>(receive_buffer_[5]) |
                                                   (static_cast<std::uint16_t>(receive_buffer_[6]) << 8U));
        sample_valid_ = true;
        ++read_count_;
        return;
    }
}

void Bmi088Gyro::advance(Step next_step, std::uint32_t wait_ms) noexcept
{
    step_ = next_step;
    step_wait_ms_ = wait_ms;
    step_tick_ = platform::Time::now_ticks();
}

void Bmi088Gyro::fail() noexcept
{
    ++error_count_;
    state_ = State::ERROR;
    transfer_active_ = false;
}

} // namespace device
