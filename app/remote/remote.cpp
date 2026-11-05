#include "remote/remote.hpp"

#include <array>
#include <cstddef>

namespace remote
{

bool Remote::init(const Parameters &parameters) noexcept
{
    if (parameters.channel >= protocol::SbusParser::CHANNEL_COUNT ||
        parameters.run_minimum_raw > parameters.run_maximum_raw || parameters.frame_timeout_us == 0U)
    {
        return false;
    }
    parameters_ = parameters;
    parser_.reset();
    report_ = Report{};
    last_frame_us_ = 0U;
    has_frame_ = false;
    has_previous_frame_ = false;
    last_run_position_ = false;
    failsafe_ = false;
    publisher_ = topics::safety::stop.claim_publisher();
    return static_cast<bool>(publisher_);
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
            // 只有正常帧才算一次拨杆的观测：失控保护帧里拨杆的位置不可信，不更新档位，
            // 否则信号恢复时会被当成一次"拨上"，不经过拨杆就让安全门放开
            const bool run_position = report_.channel_raw >= parameters_.run_minimum_raw &&
                                      report_.channel_raw <= parameters_.run_maximum_raw;
            if (has_previous_frame_ && run_position && !last_run_position_)
            {
                ++report_.arm_sequence; // 操作者拨上来了
            }
            last_run_position_ = run_position;
            has_previous_frame_ = true;
        }
    }

    report_.fresh = has_frame_ && !hal::os::has_elapsed(last_frame_us_, parameters_.frame_timeout_us, now);
    report_.running = report_.fresh && !failsafe_ && last_run_position_;
    report_.stop = !report_.running;

    topics::safety::Stop message{};
    message.stop = report_.stop;
    message.arm_sequence = report_.arm_sequence;
    message.timestamp_us = now;
    publisher_.publish(message);
}

} // namespace remote
