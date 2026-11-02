#ifndef PLATFORM_MOCK_MOCK_HAL_HPP
#define PLATFORM_MOCK_MOCK_HAL_HPP

#include "hal/can.hpp"
#include "hal/gpio.hpp"
#include "hal/pwm.hpp"
#include "hal/spi.hpp"
#include "hal/uart.hpp"
#include "hal/usb_cdc.hpp"

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

/** 模拟收到一帧：received_us 写为当时的模拟时刻，放进接收队列并调用登记的接收通知。队列满时丢弃、计入 rx_dropped_count，返回 false。 */
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

namespace mock::spi
{

inline constexpr std::uint8_t DEVICE_COUNT{8U};

/**
 * 设备应答：替身在事务完成时调用它，由测试扮演设备——读 transmit、写 receive（各 length 字节）。
 * 阻塞事务在 transmit()/transfer() 里调用；异步事务在 complete() 里、回调之前调用。只发送的事务 receive 为空。
 */
using Responder = void (*)(
    void *context, std::uint8_t device, const std::uint8_t *transmit, std::uint8_t *receive, std::size_t length);

/**
 * 全部设备恢复为：已初始化、支持异步与全双工、各占一条总线（总线号 = 设备号）、没有应答函数（接收全 0）、
 * async_result() 为 NOT_READY、没有进行中的事务；关闭立即完成模式。
 */
void reset() noexcept;

/** 把设备放到某条总线上：同一条总线上同时只能有一个事务，其余设备得到 BUSY。 */
void set_bus(std::uint8_t device, std::uint8_t bus) noexcept;

/** 设置能力：未初始化时返回 NOT_READY；不支持异步或全双工的操作返回 UNSUPPORTED。 */
void set_capabilities(std::uint8_t device, bool ready, bool async, bool transfer) noexcept;

void set_responder(std::uint8_t device, Responder responder, void *context) noexcept;

/** 立即完成模式：异步事务在 start_* 返回之前就完成并调用回调（模拟 DMA 完成中断抢在调用方之前）。 */
void set_complete_immediately(bool enabled) noexcept;

/** 总线上有没有进行中的异步事务；有时把设备号写进 device（可以为空）。 */
bool busy(std::uint8_t bus, std::uint8_t *device = nullptr) noexcept;

/**
 * 结束总线上进行中的异步事务：成功时调用应答函数、把数据拷进调用方的接收缓冲区，async_result() 变为 COMPLETED；
 * error 为 true 时不调用应答函数，结果为 ERROR。然后在调用线程里调用登记的回调。没有进行中的事务时返回 false。
 */
bool complete(std::uint8_t bus, bool error = false) noexcept;

/** 设备开始过的事务数（阻塞与异步，不含被拒绝的）。 */
[[nodiscard]] std::uint32_t transaction_count(std::uint8_t device) noexcept;

/** 设备最后一次事务发出的字节（最多 hal::Spi::MAX_ASYNC_LENGTH 个）；返回字节数。 */
std::size_t last_transmit(std::uint8_t device, std::uint8_t *data, std::size_t capacity) noexcept;

} // namespace mock::spi

namespace mock::uart
{

inline constexpr std::uint8_t PORT_COUNT{8U};
/** 软件接收队列的容量（字节），超出时与真实 port 一样丢弃并计数。 */
inline constexpr std::size_t QUEUE_CAPACITY{256U};

/** 全部端口恢复为：已启动、可发送、队列与发送记录为空、没有进行中的发送、统计为 0、没有登记通知。 */
void reset() noexcept;

/** 设置能力：未启动时 is_ready() 为 false、write() 返回 NOT_READY；不可发送时 write() 返回 UNSUPPORTED。 */
void set_capabilities(std::uint8_t port, bool ready, bool writable) noexcept;

/** 模拟收到字节：放进接收队列（满了丢弃并计数），收下了至少一个字节就调用接收通知。返回收下的字节数。 */
std::size_t inject(std::uint8_t port, const std::uint8_t *data, std::size_t length) noexcept;

/** 有没有进行中的发送（write() 返回 STARTED 之后、complete_write() 之前）。 */
[[nodiscard]] bool write_pending(std::uint8_t port) noexcept;

/** 结束进行中的发送并调用发送通知：成功计入 transmitted_byte_count，error 为 true 时计入 transmit_failure_count。 */
bool complete_write(std::uint8_t port, bool error = false) noexcept;

/** 取出已开始发送的字节（按顺序），返回字节数。 */
std::size_t take_written(std::uint8_t port, std::uint8_t *data, std::size_t capacity) noexcept;

} // namespace mock::uart

namespace mock::usb_cdc
{

/** 替身只有一个接口，编号 0（与 DM-MC02 一致）。 */
inline constexpr std::size_t QUEUE_CAPACITY{4096U};

/** 恢复为：已初始化、未连接、队列与发送记录为空、没有进行中的发送、统计为 0、没有登记通知。 */
void reset() noexcept;

/**
 * 模拟主机打开或关闭端口。连上：清空接收队列、计入 connection_count；断开：清空接收队列、计入
 * disconnection_count，进行中的发送算失败并调用发送通知。
 */
void set_connected(bool connected) noexcept;

/** 模拟收到字节（只在连上时收下）：放进接收队列（满了丢弃并计数），收下了就调用接收通知。返回收下的字节数。 */
std::size_t inject(const std::uint8_t *data, std::size_t length) noexcept;

[[nodiscard]] bool write_pending() noexcept;

/** 结束进行中的发送并调用发送通知（同 mock::uart::complete_write）。 */
bool complete_write(bool error = false) noexcept;

/** 取出已开始发送的字节（按顺序），返回字节数。 */
std::size_t take_written(std::uint8_t *data, std::size_t capacity) noexcept;

} // namespace mock::usb_cdc

#endif // PLATFORM_MOCK_MOCK_HAL_HPP
