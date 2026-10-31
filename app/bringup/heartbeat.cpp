#include "bringup/heartbeat.hpp"

namespace bringup
{

bool Heartbeat::init() noexcept
{
    publisher_ = topics::bringup::heartbeat.claim_publisher();
    return static_cast<bool>(publisher_);
}

void Heartbeat::step(hal::os::Microseconds now) noexcept
{
    ++sequence_;
    publisher_.publish(topics::bringup::Heartbeat{sequence_, now});
}

} // namespace bringup
