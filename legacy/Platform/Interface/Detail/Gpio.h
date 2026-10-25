#ifndef PLATFORM_INTERFACE_DETAIL_GPIO_H
#define PLATFORM_INTERFACE_DETAIL_GPIO_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Port 实现能力分级（约束 Port 移植；业务代码须按此兼容）：
 *   - MUST：任何 Port 实现都必须完整支持，业务代码可无条件依赖；
 *   - MAY ：Port 可以不实现。未实现时按该函数注释中的降级约定返回
 *           （返回 false，或返回 *_UNSUPPORTED），业务代码必须能处理该返回值。
 *   未标注能力分级的函数均为 MUST。
 */

#ifdef __cplusplus
extern "C"
{
#endif

  typedef uint8_t GpioPort_Pin;
  typedef void (*GpioPort_InterruptCallback)(void *context);

  enum
  {
    GPIO_PORT_PIN_POWER_24V_1 = 0U,
    GPIO_PORT_PIN_POWER_24V_0,
    GPIO_PORT_PIN_POWER_5V,
    GPIO_PORT_PIN_IMU_ACCELEROMETER_CHIP_SELECT,
    GPIO_PORT_PIN_IMU_GYROSCOPE_CHIP_SELECT,
    GPIO_PORT_PIN_IMU_ACCELEROMETER_INTERRUPT,
    GPIO_PORT_PIN_IMU_GYROSCOPE_INTERRUPT,
    GPIO_PORT_PIN_USER_KEY,
    GPIO_PORT_PIN_COUNT
  };

  /**
 * @brief 读取指定逻辑 GPIO 引脚的当前电平。
 * @param pin 要读取的逻辑引脚。
 * @param is_high 用于接收电平状态的输出指针，true 表示高电平。
 * @return 引脚和输出指针有效且读取成功时返回 true。
 */
  bool GpioPort_Read(GpioPort_Pin pin, bool *is_high);

  /**
 * @brief 设置指定逻辑 GPIO 输出引脚的电平。
 * @param pin 要写入的逻辑引脚。
 * @param is_high 为 true 时输出高电平，为 false 时输出低电平。
 * @return 引脚支持写入且操作成功时返回 true。
 */
  bool GpioPort_Write(GpioPort_Pin pin, bool is_high);

  /**
 * @brief 翻转指定逻辑 GPIO 输出引脚的当前电平。
 * @param pin 要翻转的逻辑引脚。
 * @return 引脚支持写入且操作成功时返回 true。
 */
  bool GpioPort_Toggle(GpioPort_Pin pin);

  /**
 * @brief 注册 GPIO 外部中断通知。
 * @param pin 支持外部中断的逻辑引脚。
 * @param callback ISR 上下文调用的短回调，传入 NULL 表示取消注册。
 * @param context 透传给回调的上下文指针。
 * @return 引脚支持外部中断且注册完成时返回 true。
 * @note 回调运行在 ISR 上下文，只允许执行中断安全操作。
 */
  bool GpioPort_SetInterruptCallback(GpioPort_Pin pin, GpioPort_InterruptCallback callback, void *context);

#ifdef __cplusplus
}
#endif

#endif
