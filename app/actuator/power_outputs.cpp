#include "actuator/power_outputs.hpp"

namespace actuator
{

bool PowerOutputs::init(const Config &config) noexcept
{
    if (config.outputs == nullptr && config.count != 0U)
    {
        return false;
    }
    const hal::Gpio::Level off_level =
        config.on_level == hal::Gpio::Level::HIGH ? hal::Gpio::Level::LOW : hal::Gpio::Level::HIGH;
    const hal::Gpio::Level level = config.parameters.on ? config.on_level : off_level;
    bool written = true;
    for (std::size_t index = 0U; index < config.count; ++index)
    {
        written = config.outputs[index].write(level) && written; // 一路失败也继续写其余各路
    }
    on_ = written && config.parameters.on;
    return written;
}

} // namespace actuator
