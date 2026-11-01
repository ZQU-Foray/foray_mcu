#ifndef PLATFORM_MOCK_MOCK_HAL_HPP
#define PLATFORM_MOCK_MOCK_HAL_HPP

#include "hal/gpio.hpp"
#include "hal/pwm.hpp"

#include <cstdint>

// 宿主测试控制 hal 外设替身的接口（Decision-20261101-01）。
// 测试自己构造句柄（hal::Pwm{0U}），这里按同一个编号注入输入、检查输出。编号从 0 到各外设的 *_COUNT − 1，
// 超出范围的句柄在替身里和真实 port 一样当作无效编号处理。
// 状态是进程内全局的，每个测试开始时调用对应外设的 reset()。全部函数线程安全；
// 模拟中断的函数（trigger_interrupt 等）在调用线程里直接执行登记的回调，不持有替身内部的锁。

namespace mock::gpio
{

inline constexpr std::uint8_t PIN_COUNT{16U};

/** 全部引脚恢复为：电平 LOW、可输出、有外部中断、没有登记回调。 */
void reset() noexcept;

/** 设置引脚能力，用来模拟板上只能输入或没有中断线的引脚。 */
void set_capabilities(std::uint8_t pin, bool output, bool interrupt) noexcept;

/** 设置输入电平（read() 读到的值）。 */
void set_level(std::uint8_t pin, hal::Gpio::Level level) noexcept;

/** 当前电平：write()、toggle() 或 set_level() 最后设定的值。 */
[[nodiscard]] hal::Gpio::Level level(std::uint8_t pin) noexcept;

/** write() 与 toggle() 成功的次数。 */
[[nodiscard]] std::uint32_t write_count(std::uint8_t pin) noexcept;

/** 模拟一次外部中断：调用登记的回调。没有登记时返回 false。 */
bool trigger_interrupt(std::uint8_t pin) noexcept;

} // namespace mock::gpio

namespace mock::pwm
{

inline constexpr std::uint8_t CHANNEL_COUNT{8U};

struct Output
{
    std::uint32_t frequency_hz;  // 最后一次 set() 的频率，从没 set() 过为 0
    std::uint16_t duty_permille; // 当前占空比，off() 之后为 0
    std::uint32_t set_count;     // set() 与 off() 成功的次数
};

/** 全部通道恢复为：已启动（is_ready() 为 true）、频率与占空比为 0。 */
void reset() noexcept;

/** 设置通道是否已启动；未启动时 set()、off() 返回 NOT_READY。 */
void set_ready(std::uint8_t channel, bool ready) noexcept;

[[nodiscard]] Output output(std::uint8_t channel) noexcept;

} // namespace mock::pwm

#endif // PLATFORM_MOCK_MOCK_HAL_HPP
