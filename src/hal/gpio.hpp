#ifndef PLATFORM_INTERFACE_GPIO_HPP
#define PLATFORM_INTERFACE_GPIO_HPP

#include "Detail/Gpio.h"
#include <cstdint>

namespace platform
{

class Gpio final
{
  public:
    using InterruptCallback = GpioPort_InterruptCallback;

    enum class Pin : std::uint8_t
    {
        POWER_24V_1 = GPIO_PORT_PIN_POWER_24V_1,
        POWER_24V_0 = GPIO_PORT_PIN_POWER_24V_0,
        POWER_5V = GPIO_PORT_PIN_POWER_5V,
        IMU_ACCELEROMETER_CHIP_SELECT = GPIO_PORT_PIN_IMU_ACCELEROMETER_CHIP_SELECT,
        IMU_GYROSCOPE_CHIP_SELECT = GPIO_PORT_PIN_IMU_GYROSCOPE_CHIP_SELECT,
        IMU_ACCELEROMETER_INTERRUPT = GPIO_PORT_PIN_IMU_ACCELEROMETER_INTERRUPT,
        IMU_GYROSCOPE_INTERRUPT = GPIO_PORT_PIN_IMU_GYROSCOPE_INTERRUPT,
        USER_KEY = GPIO_PORT_PIN_USER_KEY
    };

    enum class Level : std::uint8_t
    {
        LOW = 0U,
        HIGH = 1U
    };

    /**
   * @brief 禁止创建 Gpio 实例，所有能力均通过静态方法访问。
   */
    Gpio() = delete;

    /**
   * @brief 读取指定逻辑 GPIO 引脚的当前电平。
   * @param pin 要读取的逻辑引脚。
   * @param level 用于接收高低电平的输出引用。
   * @return 引脚有效且读取成功时返回 true。
   */
    [[nodiscard]] static bool read(Pin pin, Level &level) noexcept
    {
        bool is_high = false;
        if (!GpioPort_Read(to_port_pin(pin), &is_high))
        {
            return false;
        }

        level = is_high ? Level::HIGH : Level::LOW;
        return true;
    }

    /**
   * @brief 设置指定逻辑 GPIO 输出引脚的电平。
   * @param pin 要写入的逻辑引脚。
   * @param level 需要输出的高低电平。
   * @return 引脚支持写入且操作成功时返回 true。
   */
    [[nodiscard]] static bool write(Pin pin, Level level) noexcept
    {
        return GpioPort_Write(to_port_pin(pin), level == Level::HIGH);
    }

    /**
   * @brief 将指定逻辑 GPIO 输出引脚设置为高电平。
   * @param pin 要写入的逻辑引脚。
   * @return 引脚支持写入且操作成功时返回 true。
   */
    [[nodiscard]] static bool set_high(Pin pin) noexcept
    {
        return write(pin, Level::HIGH);
    }

    /**
   * @brief 将指定逻辑 GPIO 输出引脚设置为低电平。
   * @param pin 要写入的逻辑引脚。
   * @return 引脚支持写入且操作成功时返回 true。
   */
    [[nodiscard]] static bool set_low(Pin pin) noexcept
    {
        return write(pin, Level::LOW);
    }

    /**
   * @brief 翻转指定逻辑 GPIO 输出引脚的当前电平。
   * @param pin 要翻转的逻辑引脚。
   * @return 引脚支持写入且操作成功时返回 true。
   */
    [[nodiscard]] static bool toggle(Pin pin) noexcept
    {
        return GpioPort_Toggle(to_port_pin(pin));
    }

    /**
   * @brief 注册逻辑 GPIO 的外部中断通知。
   * @param pin 支持外部中断的逻辑引脚。
   * @param callback ISR 上下文调用的短回调，传入 nullptr 表示取消注册。
   * @param context 透传给回调的上下文指针。
   * @return 引脚支持外部中断且注册完成时返回 true。
   */
    [[nodiscard]] static bool
    set_interrupt_callback(Pin pin, InterruptCallback callback, void *context = nullptr) noexcept
    {
        return GpioPort_SetInterruptCallback(to_port_pin(pin), callback, context);
    }

  private:
    /**
   * @brief 将 C++ 逻辑引脚转换为 Detail C ABI 使用的引脚值。
   * @param pin C++ 逻辑引脚。
   * @return 对应的 C ABI 引脚值。
   */
    [[nodiscard]] static constexpr GpioPort_Pin to_port_pin(Pin pin) noexcept
    {
        return static_cast<GpioPort_Pin>(pin);
    }
};

} // namespace platform

#endif
