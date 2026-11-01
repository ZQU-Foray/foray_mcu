#include "bringup/can_check.hpp"

namespace bringup
{

namespace
{

constexpr std::uint32_t DJI_FEEDBACK_FIRST{0x201U};
constexpr std::uint32_t DJI_FEEDBACK_LAST{0x20BU};

} // namespace

bool CanCheck::init() noexcept
{
    report_ = Report{};
    return bus_.claim_receiver();
}

void CanCheck::step(hal::os::Microseconds now) noexcept
{
    hal::Can::Frame frame{};
    while (bus_.receive(frame) == hal::Can::ReceiveResult::RECEIVED)
    {
        ++report_.frames;
        report_.last_identifier = frame.identifier;
        report_.last_frame_us = now;
        if (frame.identifier_type == hal::Can::IdentifierType::STANDARD && frame.identifier >= DJI_FEEDBACK_FIRST &&
            frame.identifier <= DJI_FEEDBACK_LAST)
        {
            report_.dji_feedback_seen =
                static_cast<std::uint16_t>(report_.dji_feedback_seen | (1U << (frame.identifier - DJI_FEEDBACK_FIRST)));
        }
    }
    report_.statistics = bus_.statistics();
    report_.bus_ready = bus_.is_ready();
}

} // namespace bringup
