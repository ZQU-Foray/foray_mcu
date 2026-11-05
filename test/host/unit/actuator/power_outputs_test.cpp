// actuator::PowerOutputs：上电按参数把每一路写成打开或关闭；打开电平由组合根给出；有一路写失败也继续写其余各路。
#include "actuator/power_outputs.hpp"
#include "hal/gpio.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"

#include <array>
#include <cassert>

int main()
{
    mock::reset();
    mock::gpio::reset();
    const std::array outputs{hal::Gpio{10U}, hal::Gpio{11U}};
    actuator::PowerOutputs power;
    assert(power.init(actuator::PowerOutputs::Config{outputs.data(), outputs.size(), hal::Gpio::Level::HIGH, {true}}));
    assert(power.is_on());
    assert(mock::gpio::level(10U) == hal::Gpio::Level::HIGH && mock::gpio::level(11U) == hal::Gpio::Level::HIGH);
    // 参数为关：写成关闭电平
    assert(power.init(actuator::PowerOutputs::Config{outputs.data(), outputs.size(), hal::Gpio::Level::HIGH, {false}}));
    assert(!power.is_on() && mock::gpio::level(10U) == hal::Gpio::Level::LOW);
    // 低电平打开的板
    assert(power.init(actuator::PowerOutputs::Config{outputs.data(), outputs.size(), hal::Gpio::Level::LOW, {true}}));
    assert(mock::gpio::level(11U) == hal::Gpio::Level::LOW);
    // 有一路不是输出引脚：返回 false，其余各路照写
    mock::gpio::set_capabilities(10U, false, false);
    assert(!power.init(actuator::PowerOutputs::Config{outputs.data(), outputs.size(), hal::Gpio::Level::HIGH, {true}}));
    assert(!power.is_on() && mock::gpio::level(11U) == hal::Gpio::Level::HIGH);
    assert(!power.init(actuator::PowerOutputs::Config{nullptr, 2U, hal::Gpio::Level::HIGH, {true}}));
    return 0;
}
