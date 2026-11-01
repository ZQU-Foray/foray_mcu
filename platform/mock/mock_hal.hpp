#ifndef PLATFORM_MOCK_MOCK_HAL_HPP
#define PLATFORM_MOCK_MOCK_HAL_HPP

#include "hal/can.hpp"
#include "hal/gpio.hpp"
#include "hal/pwm.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

// 宿主测试控制 hal 外设替身的接口（Decision-20261101-01）。
// 测试自己构造句柄（hal::Pwm{0U}），这里按同一个编号注入输入、检查输出。编号从 0 到各外设的 *_COUNT − 1，
// 超出范围的句柄在替身里和真实 port 一样当作无效编号处理。
// 状态是进程内全局的，每个测试开始时调用对应外设的 reset()。全部函数线程安全；
// 模拟中断的函数（trigger_interrupt 等）在调用线程里直接执行登记的回调，不持有替身内部的锁。

namespace mock::can
{

inline constexpr std::uint8_t BUS_COUNT{8U};
/** 每条总线的接收队列与发送记录最多保存的帧数，超出时与真实 port 一样丢弃并计数。 */
inline constexpr std::size_t QUEUE_CAPACITY{64U};

/**
 * 全部总线恢复为：已启动、不在 Bus-Off、接收方未占用、队列与发送记录为空、统计为 0、send() 返回 QUEUED。
 */
void reset() noexcept;

/** 设置总线是否已启动；未启动时 is_ready() 为 false，send()、空队列的 receive() 返回 NOT_READY。 */
void set_ready(std::uint8_t bus, bool ready) noexcept;

/** 模拟 Bus-Off：is_ready() 为 false，send() 与空队列的 receive() 返回 BUS_OFF。 */
void set_bus_off(std::uint8_t bus, bool bus_off) noexcept;

/** 强制 send() 返回 result（如 QUEUE_FULL）。只有 QUEUED 会把帧记进发送记录。 */
void set_send_result(std::uint8_t bus, hal::Can::SendResult result) noexcept;

/** 模拟收到一帧：放进接收队列并调用登记的接收通知。队列满时丢弃、计入 rx_dropped_count，返回 false。 */
bool inject(std::uint8_t bus, const hal::Can::Frame &frame) noexcept;

/** 便利写法：注入一帧 8 字节的标准帧。 */
bool inject(std::uint8_t bus, std::uint32_t identifier, const std::array<std::uint8_t, 8> &data) noexcept;

/** 发送记录里的帧数（send() 返回 QUEUED 的帧，按发送顺序）。 */
[[nodiscard]] std::size_t sent_count(std::uint8_t bus) noexcept;

/** 取出最早的一条发送记录；没有时返回 false。 */
bool pop_sent(std::uint8_t bus, hal::Can::Frame &frame) noexcept;

/** 最后一次 QUEUED 的帧（不从记录里取出）；从没发过时返回 false。 */
bool last_sent(std::uint8_t bus, hal::Can::Frame &frame) noexcept;

/** 设置 statistics() 返回的计数（rx_dropped_count 在 inject() 丢帧时还会再加）。 */
void set_statistics(std::uint8_t bus, const hal::Can::Statistics &statistics) noexcept;

} // namespace mock::can

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
