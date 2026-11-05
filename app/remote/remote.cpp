#include "remote/remote.hpp"

#include "utils/math/basic_math.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace remote
{

namespace
{

/** 摇杆原始值 → [−1, 1]：扣中位、按两端量程、按方向、按死区。 */
float normalize(const Remote::Stick &stick, std::uint16_t raw) noexcept
{
    if (stick.channel == NO_CHANNEL || !(stick.maximum_raw > stick.minimum_raw) ||
        !(stick.center_raw >= stick.minimum_raw) || !(stick.center_raw <= stick.maximum_raw))
    {
        return 0.0F;
    }
    const float center = static_cast<float>(stick.center_raw);
    const float value = static_cast<float>(raw);
    float normalized = 0.0F;
    if (value >= center)
    {
        const float span = static_cast<float>(stick.maximum_raw) - center;
        normalized = span > 0.0F ? (value - center) / span : 0.0F;
    }
    else
    {
        const float span = center - static_cast<float>(stick.minimum_raw);
        normalized = span > 0.0F ? (value - center) / span : 0.0F; // 负数
    }
    normalized = math::limit(normalized, -1.0F, 1.0F);
    if (stick.inverted)
    {
        normalized = -normalized;
    }
    const float deadband = math::limit(stick.deadband, 0.0F, 0.99F);
    if (std::fabs(normalized) <= deadband)
    {
        return 0.0F;
    }
    // 死区之外重新拉伸到 [−1, 1]，满量程仍然是 ±1
    const float magnitude = (std::fabs(normalized) - deadband) / (1.0F - deadband);
    return normalized > 0.0F ? magnitude : -magnitude;
}

/** 拨杆原始值 → 档位：低于下门限、高于上门限，之间是 MIDDLE。 */
topics::SwitchPosition switch_position(const Remote::Switch &sw, std::uint16_t raw) noexcept
{
    if (sw.channel == NO_CHANNEL)
    {
        return topics::SwitchPosition::NONE;
    }
    if (raw < sw.lower_raw)
    {
        return sw.low_position;
    }
    if (raw > sw.upper_raw)
    {
        return sw.high_position;
    }
    return topics::SwitchPosition::MIDDLE;
}

bool parameters_valid(const Remote::Parameters &parameters) noexcept
{
    for (const Remote::Stick &stick : parameters.sticks)
    {
        if (stick.channel != NO_CHANNEL &&
            (stick.channel >= protocol::SbusParser::CHANNEL_COUNT || stick.maximum_raw < stick.minimum_raw ||
             stick.center_raw < stick.minimum_raw || stick.center_raw > stick.maximum_raw ||
             !std::isfinite(stick.deadband) || stick.deadband < 0.0F || stick.deadband >= 1.0F))
        {
            return false;
        }
    }
    for (const Remote::Switch &sw : parameters.switches)
    {
        if (sw.channel != NO_CHANNEL &&
            (sw.channel >= protocol::SbusParser::CHANNEL_COUNT || sw.lower_raw > sw.upper_raw ||
             sw.low_position == topics::SwitchPosition::NONE || sw.high_position == topics::SwitchPosition::NONE))
        {
            return false;
        }
    }
    return parameters.frame_timeout_us != 0U && parameters.channel < protocol::SbusParser::CHANNEL_COUNT;
}

} // namespace

bool Remote::init(const Parameters &parameters) noexcept
{
    if (!parameters_valid(parameters))
    {
        return false;
    }
    parameters_ = parameters;
    parser_.reset();
    report_ = Report{};
    last_frame_us_ = 0U;
    last_normal_frame_us_ = 0U;
    has_frame_ = false;
    has_previous_frame_ = false;
    last_run_position_ = false;
    failsafe_ = false;
    input_publisher_ = topics::remote::input.claim_publisher();
    publisher_ = topics::safety::stop.claim_publisher();
    return static_cast<bool>(input_publisher_) && static_cast<bool>(publisher_);
}

void Remote::step(hal::os::Microseconds now) noexcept
{
    std::array<std::uint8_t, 64> buffer{};
    protocol::SbusParser::Frame frame{};
    bool received = false;
    for (std::size_t length = port_.read(buffer.data(), buffer.size()); length > 0U;
         length = port_.read(buffer.data(), buffer.size()))
    {
        report_.bytes += static_cast<std::uint32_t>(length);
        const std::size_t frames = parser_.input(buffer.data(), length, frame);
        if (frames > 0U)
        {
            report_.frames += static_cast<std::uint32_t>(frames);
            received = true;
            if (frame.failsafe)
            {
                ++report_.failsafe;
            }
        }
    }
    report_.rejected = parser_.get_statistics().rejected_frame_count;
    report_.port_ready = port_.is_ready();

    if (received)
    {
        has_frame_ = true;
        last_frame_us_ = now;
        report_.channel_raw = frame.channels[parameters_.channel];
        failsafe_ = frame.failsafe;
        if (!frame.failsafe)
        {
            // 只有正常帧才更新摇杆与拨杆：失控保护帧里拨杆的位置与摇杆的值不可信
            last_normal_frame_us_ = now;
            for (std::size_t index = 0U; index < topics::STICK_COUNT; ++index)
            {
                const std::uint8_t channel = parameters_.sticks[index].channel;
                report_.sticks[index] =
                    channel == NO_CHANNEL ? 0.0F : normalize(parameters_.sticks[index], frame.channels[channel]);
            }
            for (std::size_t index = 0U; index < topics::SWITCH_COUNT; ++index)
            {
                const std::uint8_t channel = parameters_.switches[index].channel;
                report_.switches[index] = channel == NO_CHANNEL
                                              ? topics::SwitchPosition::NONE
                                              : switch_position(parameters_.switches[index], frame.channels[channel]);
            }

            // 安全门的拨杆（第 2 步搬到仲裁后删掉）
            const bool run_position = report_.channel_raw >= parameters_.run_minimum_raw &&
                                      report_.channel_raw <= parameters_.run_maximum_raw;
            if (has_previous_frame_ && run_position && !last_run_position_)
            {
                ++report_.arm_sequence;
            }
            last_run_position_ = run_position;
            has_previous_frame_ = true;
        }
    }

    report_.fresh = has_frame_ && !hal::os::has_elapsed(last_frame_us_, parameters_.frame_timeout_us, now);
    report_.connected = report_.fresh && !failsafe_;
    report_.running = report_.connected && last_run_position_;
    report_.stop = !report_.running;

    topics::OperatorInput input{};
    input.sticks = report_.sticks;
    input.switches = report_.switches;
    input.has_remote = true;
    input.has_keyboard_mouse = false;
    input.connected = report_.connected;
    input.timestamp_us = last_normal_frame_us_;
    input_publisher_.publish(input);

    topics::safety::Stop message{};
    message.stop = report_.stop;
    message.arm_sequence = report_.arm_sequence;
    message.timestamp_us = now;
    publisher_.publish(message);
}

} // namespace remote
