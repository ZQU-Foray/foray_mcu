#include "bringup/sbus_check.hpp"

#include <array>
#include <cstddef>

namespace bringup
{

bool SbusCheck::init() noexcept
{
    parser_.reset();
    report_ = Report{};
    publisher_ = topics::bringup::sbus.claim_publisher();
    return static_cast<bool>(publisher_);
}

void SbusCheck::step(hal::os::Microseconds now) noexcept
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
    report_.statistics = port_.statistics();
    report_.port_ready = port_.is_ready();

    if (received)
    {
        topics::bringup::Sbus message{};
        message.channels = frame.channels;
        message.frame_count = report_.frames;
        message.timestamp_us = now;
        message.frame_lost = frame.frame_lost;
        message.failsafe = frame.failsafe;
        publisher_.publish(message);
    }
}

} // namespace bringup
