// platform_host_sim 的 hal_os 实现（宿主下位机，I26）。
//
// 与 platform/mock 的区别：这里是**真实**时基与真实线程同步，不给测试推进时间。
//   - now_us()：steady_clock 自首次调用起的单调微秒数，按契约截断到 32 位回绕；
//   - CriticalSection：一把递归锁，保存并恢复进入前的嵌套深度（与真实板"保存并屏蔽"同语义）；
//   - Signal：条件变量；timeout 是真实时间，WAIT_FOREVER 一直等。
// 误用（在临界区内 notify/wait、两个线程等同一个 Signal）当场 abort：这类错误在宿主上必须暴露。

#include "hal/os/clock.hpp"
#include "hal/os/critical_section.hpp"
#include "hal/os/signal.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>

namespace
{

std::recursive_mutex g_mutex;
std::condition_variable_any g_changed;
const std::chrono::steady_clock::time_point g_start{std::chrono::steady_clock::now()};

thread_local std::uint32_t g_critical_depth{0U}; // 调用线程的临界区嵌套深度
thread_local char g_waiter_token{0};             // 地址用作等待者标识，对应 RTOS 的任务句柄

[[noreturn]] void fail(const char *message) noexcept
{
    std::fprintf(stderr, "platform_host_sim: %s\n", message);
    std::abort();
}

} // namespace

namespace hal::os
{

Microseconds now_us() noexcept
{
    const auto elapsed = std::chrono::steady_clock::now() - g_start;
    const auto count = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    return static_cast<Microseconds>(static_cast<std::uint64_t>(count)); // 低 32 位，按 2^32 回绕
}

// 与真实平台的"保存并屏蔽、析构时恢复"一致：保存的是进入前的嵌套深度，析构时原样恢复。
CriticalSection::CriticalSection() noexcept : saved_state_{0U}
{
    g_mutex.lock();
    saved_state_ = g_critical_depth;
    ++g_critical_depth;
}

CriticalSection::~CriticalSection()
{
    g_critical_depth = saved_state_;
    g_mutex.unlock();
}

void Signal::notify() noexcept
{
    if (g_critical_depth != 0U)
    {
        fail("Signal::notify() 不能在 CriticalSection 内调用");
    }
    const std::lock_guard<std::recursive_mutex> lock{g_mutex};
    pending_ = true;
    g_changed.notify_all();
}

bool Signal::wait(Microseconds timeout) noexcept
{
    // 在临界区内上锁会成功（递归锁），条件变量等待时却放不开外层的锁，必须挡在入口
    if (g_critical_depth != 0U)
    {
        fail("Signal::wait() 不能在 CriticalSection 内调用");
    }
    if (waiter_ != nullptr)
    {
        fail("两个线程同时等同一个 Signal");
    }

    std::unique_lock<std::recursive_mutex> lock{g_mutex};
    waiter_ = &g_waiter_token;
    bool notified = false;
    if (timeout == NO_WAIT)
    {
        notified = pending_;
        pending_ = false;
    }
    else if (timeout == WAIT_FOREVER)
    {
        g_changed.wait(lock, [this] { return pending_; });
        pending_ = false;
        notified = true;
    }
    else
    {
        const bool signalled =
            g_changed.wait_for(lock, std::chrono::microseconds{timeout}, [this] { return pending_; });
        pending_ = false;
        notified = signalled;
    }
    waiter_ = nullptr;
    return notified;
}

} // namespace hal::os
