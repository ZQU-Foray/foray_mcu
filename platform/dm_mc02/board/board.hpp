#ifndef PLATFORM_DM_MC02_BOARD_BOARD_HPP
#define PLATFORM_DM_MC02_BOARD_BOARD_HPP

#include "hal/can.hpp"
#include "hal/gpio.hpp"
#include "hal/pwm.hpp"

#include <cstdint>

// DM-MC02（STM32H723）的物理资源：hal 契约的句柄、各类外设的个数与能力常量（Decision-20261101-01）。
// 只给组合根（config/assembly）与本板的 port 用：驱动与能力只认 Config 里传进来的句柄，不知道这些名字（规则 1）。
// 句柄的编号是 port 里外设表的下标，port 用 static_assert 核对表长与这里的个数。

namespace board
{

// ---- CAN（port/can.cpp）：FDCAN1–3，Classic CAN 1 Mbps（CubeMX 配置）----
inline constexpr hal::Can CAN_1{0U};
inline constexpr hal::Can CAN_2{1U};
inline constexpr hal::Can CAN_3{2U};
inline constexpr std::uint8_t CAN_COUNT{3U};

// ---- GPIO（port/gpio.cpp）----
// 电源输出：高电平打开，上电为关。
inline constexpr hal::Gpio POWER_24V_0{0U};
inline constexpr hal::Gpio POWER_24V_1{1U};
inline constexpr hal::Gpio POWER_5V{2U};
// BMI088 的数据就绪：输入，上升沿外部中断（set_interrupt 可用）。
inline constexpr hal::Gpio IMU_ACCEL_INTERRUPT{3U};
inline constexpr hal::Gpio IMU_GYRO_INTERRUPT{4U};
// 板上的用户按键：输入，没有外部中断。
inline constexpr hal::Gpio USER_KEY{5U};
inline constexpr std::uint8_t GPIO_COUNT{6U};

// ---- PWM（port/pwm.cpp）----
// 计数频率 1 MHz，周期 2–65536 个计数：可设频率约 16 Hz–500 kHz。
inline constexpr hal::Pwm PWM_BUZZER{0U};     // 板载蜂鸣器（TIM12 CH2）
inline constexpr hal::Pwm PWM_IMU_HEATER{1U}; // BMI088 加热电阻（TIM3 CH4）
inline constexpr std::uint8_t PWM_COUNT{2U};
inline constexpr std::uint32_t PWM_MIN_FREQUENCY_HZ{16U};
inline constexpr std::uint32_t PWM_MAX_FREQUENCY_HZ{500000U};

} // namespace board

#endif // PLATFORM_DM_MC02_BOARD_BOARD_HPP
