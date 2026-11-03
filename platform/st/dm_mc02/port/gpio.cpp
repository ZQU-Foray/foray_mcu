// hal::Gpio 在 DM-MC02 上的实现。引脚表的下标就是 board.hpp 里句柄的编号。
// 覆盖 HAL 弱符号 HAL_GPIO_EXTI_Callback：main.h 带进 HAL 头，定义因此是 C 链接（规则 16）。
#include "hal/gpio.hpp"

#include "board.hpp"
#include "main.h" // CubeMX 的引脚名；带进 stm32h7xx_hal.h
#include "port.hpp"

#include <array>
#include <cstdint>

namespace
{

struct Pin
{
    GPIO_TypeDef *port;
    std::uint16_t mask; // GPIO_PIN_x，也是外部中断线的编号
    bool output;
    bool interrupt;
};

// 顺序与 board.hpp 一致。SPI 片选（BMI088_*_CS）归 SPI 的 port 管，不在这里。
const std::array PINS{
    Pin{DC24_0_OUTPUT_GPIO_Port, DC24_0_OUTPUT_Pin, true, false},
    Pin{DC24_1_OUTPUT_GPIO_Port, DC24_1_OUTPUT_Pin, true, false},
    Pin{DC5_OUTPUT_GPIO_Port, DC5_OUTPUT_Pin, true, false},
    Pin{BMI088_ACCEL_INTERRUPT_GPIO_Port, BMI088_ACCEL_INTERRUPT_Pin, false, true},
    Pin{BMI088_GYRO_INTERRUPT_GPIO_Port, BMI088_GYRO_INTERRUPT_Pin, false, true},
    Pin{KEY_INPUT_GPIO_Port, KEY_INPUT_Pin, false, false},
};
static_assert(std::tuple_size_v<decltype(PINS)> == board::GPIO_COUNT, "表长必须与 board.hpp 一致");

std::array<port::Notifier, board::GPIO_COUNT> g_interrupts{};

} // namespace

namespace hal
{

Gpio::Level Gpio::read() const noexcept
{
    if (index_ >= PINS.size())
    {
        return Level::LOW;
    }
    const Pin &pin = PINS[index_];
    return HAL_GPIO_ReadPin(pin.port, pin.mask) == GPIO_PIN_SET ? Level::HIGH : Level::LOW;
}

bool Gpio::write(Level level) const noexcept
{
    if (index_ >= PINS.size() || !PINS[index_].output)
    {
        return false;
    }
    const Pin &pin = PINS[index_];
    // 写 BSRR，单条存储，不需要临界区
    HAL_GPIO_WritePin(pin.port, pin.mask, level == Level::HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

bool Gpio::toggle() const noexcept
{
    if (index_ >= PINS.size() || !PINS[index_].output)
    {
        return false;
    }
    HAL_GPIO_TogglePin(PINS[index_].port, PINS[index_].mask);
    return true;
}

bool Gpio::set_interrupt(Callback callback, void *context) const noexcept
{
    if (index_ >= PINS.size() || !PINS[index_].interrupt)
    {
        return false;
    }
    g_interrupts[index_].set(callback, context);
    return true;
}

} // namespace hal

// 外部中断（EXTI15_10_IRQHandler → HAL_GPIO_EXTI_IRQHandler）：按中断线找到引脚，调用登记的回调。
void HAL_GPIO_EXTI_Callback(std::uint16_t gpio_pin)
{
    for (std::size_t index = 0U; index < PINS.size(); ++index)
    {
        if (PINS[index].interrupt && PINS[index].mask == gpio_pin)
        {
            g_interrupts[index].fire();
        }
    }
}
