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
    Power24V1 = GPIO_PORT_PIN_POWER_24V_1,
    Power24V0 = GPIO_PORT_PIN_POWER_24V_0,
    Power5V = GPIO_PORT_PIN_POWER_5V,
    ImuAccelerometerChipSelect = GPIO_PORT_PIN_IMU_ACCELEROMETER_CHIP_SELECT,
    ImuGyroscopeChipSelect = GPIO_PORT_PIN_IMU_GYROSCOPE_CHIP_SELECT,
    ImuAccelerometerInterrupt = GPIO_PORT_PIN_IMU_ACCELEROMETER_INTERRUPT,
    ImuGyroscopeInterrupt = GPIO_PORT_PIN_IMU_GYROSCOPE_INTERRUPT,
    UserKey = GPIO_PORT_PIN_USER_KEY
  };

  enum class Level : std::uint8_t
  {
    Low = 0U,
    High = 1U
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
  [[nodiscard]] static bool Read(Pin pin, Level &level) noexcept
  {
    bool isHigh = false;
    if (!GpioPort_Read(ToPortPin(pin), &isHigh))
    {
      return false;
    }

    level = isHigh ? Level::High : Level::Low;
    return true;
  }

  /**
   * @brief 设置指定逻辑 GPIO 输出引脚的电平。
   * @param pin 要写入的逻辑引脚。
   * @param level 需要输出的高低电平。
   * @return 引脚支持写入且操作成功时返回 true。
   */
  [[nodiscard]] static bool Write(Pin pin, Level level) noexcept
  {
    return GpioPort_Write(ToPortPin(pin), level == Level::High);
  }

  /**
   * @brief 将指定逻辑 GPIO 输出引脚设置为高电平。
   * @param pin 要写入的逻辑引脚。
   * @return 引脚支持写入且操作成功时返回 true。
   */
  [[nodiscard]] static bool SetHigh(Pin pin) noexcept
  {
    return Write(pin, Level::High);
  }

  /**
   * @brief 将指定逻辑 GPIO 输出引脚设置为低电平。
   * @param pin 要写入的逻辑引脚。
   * @return 引脚支持写入且操作成功时返回 true。
   */
  [[nodiscard]] static bool SetLow(Pin pin) noexcept
  {
    return Write(pin, Level::Low);
  }

  /**
   * @brief 翻转指定逻辑 GPIO 输出引脚的当前电平。
   * @param pin 要翻转的逻辑引脚。
   * @return 引脚支持写入且操作成功时返回 true。
   */
  [[nodiscard]] static bool Toggle(Pin pin) noexcept
  {
    return GpioPort_Toggle(ToPortPin(pin));
  }

  /**
   * @brief 注册逻辑 GPIO 的外部中断通知。
   * @param pin 支持外部中断的逻辑引脚。
   * @param callback ISR 上下文调用的短回调，传入 nullptr 表示取消注册。
   * @param context 透传给回调的上下文指针。
   * @return 引脚支持外部中断且注册完成时返回 true。
   */
  [[nodiscard]] static bool SetInterruptCallback(Pin pin, InterruptCallback callback, void *context = nullptr) noexcept
  {
    return GpioPort_SetInterruptCallback(ToPortPin(pin), callback, context);
  }

private:
  /**
   * @brief 将 C++ 逻辑引脚转换为 Detail C ABI 使用的引脚值。
   * @param pin C++ 逻辑引脚。
   * @return 对应的 C ABI 引脚值。
   */
  [[nodiscard]] static constexpr GpioPort_Pin ToPortPin(Pin pin) noexcept
  {
    return static_cast<GpioPort_Pin>(pin);
  }
};

} // namespace platform

#endif
