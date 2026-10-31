#include "mock_os.hpp"

#include "hal/os/critical_section.hpp"
#include "hal/os/signal.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>

namespace
{

// 等待超过这么久的真实时间仍没被叫醒，认为测试漏了 notify() 或推进时间
constexpr std::chrono::seconds WAIT_GUARD{10};

// 临界区就是这把锁；推进时间、读写 Signal 的状态也在锁内进行
std::recursive_mutex g_mutex;
// notify() 或推进时间时广播，等待中的线程各自重新检查自己的 Signal 与超时
std::condition_variable_any g_changed;

std::atomic<hal::os::Microseconds> g_now_us{0U}; // 锁内写；now_us() 不进临界区，所以用原子读
std::uint32_t g_critical_entry_count{0U};        // 锁内读写
std::uint32_t g_waiting_count{0U};               // 锁内读写

thread_local std::uint32_t g_critical_depth{0U}; // 调用线程的临界区嵌套深度
thread_local char g_waiter_token{0};             // 地址用作等待者标识，对应 FreeRTOS 的任务句柄

[[noreturn]] void fail(const char *message) noexcept
{
    std::fprintf(stderr, "platform_mock: %s\n", message);
    std::abort();
}

void set_now_locked(hal::os::Microseconds now) noexcept
{
    g_now_us.store(now);
    g_changed.notify_all();
}

} // namespace

namespace hal::os
{

Microseconds now_us() noexcept
{
    return g_now_us.load();
}

// 与真实平台的"保存并屏蔽、析构时恢复"一致：保存的是进入前的嵌套深度，析构时原样恢复。
CriticalSection::CriticalSection() noexcept : saved_state_{0U}
{
    g_mutex.lock();
    saved_state_ = g_critical_depth;
    ++g_critical_depth;
    ++g_critical_entry_count;
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
    // 先查嵌套深度再上锁：在临界区内上锁会成功（递归锁），条件变量等待时却放不开外层的锁
    if (g_critical_depth != 0U)
    {
        fail("Signal::wait() 不能在 CriticalSection 内调用");
    }
    std::unique_lock<std::recursive_mutex> lock{g_mutex};
    if (waiter_ != nullptr)
    {
        fail("两个线程同时等同一个 Signal");
    }
    waiter_ = &g_waiter_token;
    ++g_waiting_count;

    const Microseconds start = g_now_us.load();
    const auto guard_deadline = std::chrono::steady_clock::now() + WAIT_GUARD;
    bool notified = false;
    for (;;)
    {
        if (pending_)
        {
            pending_ = false;
            notified = true;
            break;
        }
        if (timeout != WAIT_FOREVER && has_elapsed(start, timeout, g_now_us.load()))
        {
            break;
        }
        if (g_changed.wait_until(lock, guard_deadline) == std::cv_status::timeout)
        {
            fail("Signal::wait() 等了 10 s 真实时间仍未被叫醒：测试漏了 notify() 或推进时间");
        }
    }

    --g_waiting_count;
    waiter_ = nullptr;
    return notified;
}

} // namespace hal::os

namespace mock
{

void reset() noexcept
{
    const std::lock_guard<std::recursive_mutex> lock{g_mutex};
    if (g_critical_depth != 0U || g_waiting_count != 0U)
    {
        fail("reset() 时还有线程在临界区内或在等待");
    }
    set_now_locked(0U);
    g_critical_entry_count = 0U;
}

void set_now_us(hal::os::Microseconds now) noexcept
{
    const std::lock_guard<std::recursive_mutex> lock{g_mutex};
    set_now_locked(now);
}

void advance_us(hal::os::Microseconds delta) noexcept
{
    const std::lock_guard<std::recursive_mutex> lock{g_mutex};
    set_now_locked(g_now_us.load() + delta);
}

std::uint32_t critical_depth() noexcept
{
    return g_critical_depth;
}

std::uint32_t critical_entry_count() noexcept
{
    const std::lock_guard<std::recursive_mutex> lock{g_mutex};
    return g_critical_entry_count;
}

std::uint32_t waiting_count() noexcept
{
    const std::lock_guard<std::recursive_mutex> lock{g_mutex};
    return g_waiting_count;
}

} // namespace mock
