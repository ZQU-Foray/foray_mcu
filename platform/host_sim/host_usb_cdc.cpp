// platform_host_sim 的 hal::UsbCdc 实现（宿主下位机，I26）。
//
// 用宿主机上的字符设备（通常是 virtual_cdc.py 建出的 PTY 从端）顶替板上的 USB CDC 外设：
//   - 设备路径取环境变量 DIESEL_SIM_USB_CDC，缺省 /tmp/diesel_mcu；打开失败不算错误，
//     之后每次询问连接状态时按 100 ms 节流重试（对应"主机还没打开端口"）；
//   - 读：非阻塞 read()，一次取走调用方容量的数据；对端关闭（read 返回 0 或 EIO）算断开；
//   - 写：一次只允许一个进行中的发送，未写完的部分留在待发缓冲区，下一次调用继续；
//   - 统计：与板上语义一致，按 2^32 回绕。
// 这里允许用宿主标准库与 POSIX：本文件只在宿主构建里编入，不进入固件。

#include "hal/os/clock.hpp"
#include "hal/usb_cdc.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <mutex>
#include <termios.h>
#include <string>
#include <unistd.h>

namespace
{

constexpr hal::os::Microseconds REOPEN_PERIOD_US{100000U}; // 重开重试节流
constexpr std::size_t RECEIVE_QUEUE_CAPACITY{4096U};       // 与 platform/mock 一致

struct State
{
    std::mutex mutex;
    int fd{-1};
    std::string path{};
    hal::os::Microseconds last_open_attempt{0U};
    bool opened_once{false};
    bool open_failure_reported{false}; // 重试每 100 ms 一次，失败只报一次
    std::deque<std::uint8_t> receive_queue{};
    std::deque<std::uint8_t> transmit_queue{};
    hal::UsbCdc::Statistics statistics{};
    hal::Callback receive_callback{nullptr};
    void *receive_context{nullptr};
    hal::Callback write_callback{nullptr};
    void *write_context{nullptr};
};

State &state()
{
    static State instance;
    return instance;
}

std::string device_path()
{
    const char *from_env = std::getenv("DIESEL_SIM_USB_CDC");
    if (from_env != nullptr && from_env[0] != '\0')
    {
        return std::string{from_env};
    }
    return std::string{"/tmp/diesel_mcu"};
}

void close_locked(State &s) noexcept
{
    if (s.fd >= 0)
    {
        ::close(s.fd);
        s.fd = -1;
        ++s.statistics.disconnection_count;
        s.transmit_queue.clear();
        s.receive_queue.clear();
    }
}

// 尝试打开一次；成功返回 true。调用方持锁。
bool open_locked(State &s) noexcept
{
    if (s.fd >= 0)
    {
        return true;
    }
    const hal::os::Microseconds now = hal::os::now_us();
    if (s.opened_once && !hal::os::has_elapsed(s.last_open_attempt, REOPEN_PERIOD_US, now))
    {
        return false;
    }
    s.opened_once = true;
    s.last_open_attempt = now;
    s.path = device_path();
    const int fd = ::open(s.path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
    {
        // 设备还没出现（virtual_cdc.py 后起）是常态，只提示一次，之后静默重试
        if (!s.open_failure_reported)
        {
            s.open_failure_reported = true;
            std::fprintf(stderr, "host_sim: open(%s) 失败（将重试）: %s\n", s.path.c_str(), std::strerror(errno));
        }
        return false;
    }
    s.open_failure_reported = false;
    // PTY 默认是行缓冲 + 回显 + 换行转换，会把二进制帧吃掉或改坏，必须 raw
    termios attributes{};
    if (::tcgetattr(fd, &attributes) == 0)
    {
        // VMIN=1 / VTIME=0（cfmakeraw 的默认）：没数据时 read 返回 EAGAIN，
        // 只有对端关闭才返回 0；若设成 VMIN=0，"没数据"也会返回 0，与断开无法区分。
        ::cfmakeraw(&attributes);
        attributes.c_cc[VMIN] = 1;
        attributes.c_cc[VTIME] = 0;
        (void)::tcsetattr(fd, TCSANOW, &attributes);
    }
    s.fd = fd;
    ++s.statistics.connection_count;
    return true;
}

// 把接收队列尽量装满；对端关闭时断开。调用方持锁。
void receive_locked(State &s) noexcept
{
    if (s.fd < 0)
    {
        return;
    }
    for (;;)
    {
        std::uint8_t buffer[256];
        const ssize_t count = ::read(s.fd, buffer, sizeof(buffer));
        if (count > 0)
        {
            s.statistics.received_byte_count += static_cast<std::uint32_t>(count);
            for (ssize_t index = 0; index < count; ++index)
            {
                if (s.receive_queue.size() >= RECEIVE_QUEUE_CAPACITY)
                {
                    ++s.statistics.dropped_byte_count;
                    continue;
                }
                s.receive_queue.push_back(buffer[index]);
            }
            continue;
        }
        if (count == 0)
        {
            close_locked(s); // 对端关闭（VMIN=1 下只有断开才返回 0）
            return;
        }
        if (errno == EINTR)
        {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return;
        }
        ++s.statistics.receive_error_count;
        close_locked(s);
        return;
    }
}

// 尽量把待发缓冲区写出去；写完时调用发送通知。调用方持锁。
void flush_locked(State &s) noexcept
{
    if (s.fd < 0)
    {
        return;
    }
    while (!s.transmit_queue.empty())
    {
        std::uint8_t buffer[512];
        const std::size_t count = std::min(s.transmit_queue.size(), sizeof(buffer));
        for (std::size_t index = 0; index < count; ++index)
        {
            buffer[index] = s.transmit_queue[index];
        }
        const ssize_t written = ::write(s.fd, buffer, count);
        if (written > 0)
        {
            s.transmit_queue.erase(s.transmit_queue.begin(),
                                   s.transmit_queue.begin() + static_cast<std::ptrdiff_t>(written));
            s.statistics.transmitted_byte_count += static_cast<std::uint32_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR)
        {
            continue;
        }
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            return; // 留到下次
        }
        ++s.statistics.transmit_failure_count;
        close_locked(s);
        return;
    }
    if (s.write_callback != nullptr)
    {
        s.write_callback(s.write_context);
    }
}

} // namespace

namespace hal
{

bool UsbCdc::is_ready() const noexcept
{
    return index_ == 0U;
}

bool UsbCdc::is_connected() const noexcept
{
    if (index_ != 0U)
    {
        return false;
    }
    State &s = state();
    const std::lock_guard<std::mutex> lock{s.mutex};
    open_locked(s);
    receive_locked(s);
    return s.fd >= 0;
}

std::size_t UsbCdc::read(std::uint8_t *data, std::size_t capacity) const noexcept
{
    if (index_ != 0U || data == nullptr || capacity == 0U)
    {
        return 0U;
    }
    State &s = state();
    const std::lock_guard<std::mutex> lock{s.mutex};
    open_locked(s);
    receive_locked(s);
    std::size_t taken = 0U;
    while (taken < capacity && !s.receive_queue.empty())
    {
        data[taken] = s.receive_queue.front();
        s.receive_queue.pop_front();
        ++taken;
    }
    return taken;
}

UsbCdc::WriteResult UsbCdc::write(const std::uint8_t *data, std::size_t length) const noexcept
{
    if (index_ != 0U || data == nullptr || length == 0U || length > MAX_WRITE_LENGTH)
    {
        return WriteResult::INVALID_ARGUMENT;
    }
    State &s = state();
    const std::lock_guard<std::mutex> lock{s.mutex};
    if (!open_locked(s) || s.fd < 0)
    {
        return WriteResult::NOT_READY;
    }
    if (!s.transmit_queue.empty())
    {
        return WriteResult::BUSY; // 上一次发送还没结束
    }
    s.transmit_queue.assign(data, data + length);
    flush_locked(s);
    if (s.transmit_queue.empty())
    {
        return WriteResult::STARTED;
    }
    return WriteResult::STARTED; // 已拷走并开始发送；剩余部分下次继续
}

bool UsbCdc::set_receive_notification(Callback callback, void *context) const noexcept
{
    if (index_ != 0U)
    {
        return false;
    }
    State &s = state();
    const std::lock_guard<std::mutex> lock{s.mutex};
    s.receive_callback = callback;
    s.receive_context = context;
    return true;
}

bool UsbCdc::set_write_notification(Callback callback, void *context) const noexcept
{
    if (index_ != 0U)
    {
        return false;
    }
    State &s = state();
    const std::lock_guard<std::mutex> lock{s.mutex};
    s.write_callback = callback;
    s.write_context = context;
    return true;
}

UsbCdc::Statistics UsbCdc::statistics() const noexcept
{
    if (index_ != 0U)
    {
        return Statistics{};
    }
    State &s = state();
    const std::lock_guard<std::mutex> lock{s.mutex};
    return s.statistics;
}

} // namespace hal
