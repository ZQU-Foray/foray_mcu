// hal::os::Signal 的语义（Decision-20261031-02），用 platform_mock 的实现验证：
// 通知不丢、多次合并、超时按测试推进的时间、跨线程唤醒、一个线程先后等多个 Signal、多个线程轮流等，
// 以及"先检查条件再等待"的写法在高频通知下不丢唤醒。
#include "hal/os/clock.hpp"
#include "hal/os/signal.hpp"
#include "mock_os.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <thread>

namespace
{

using hal::os::NO_WAIT;
using hal::os::Signal;
using hal::os::WAIT_FOREVER;

// 构造函数是 constexpr：能定义 constexpr 对象，作全局对象时就是常量初始化
[[maybe_unused]] constexpr Signal CONSTANT_INITIALIZED{};
Signal g_global_signal;

// 等到有 count 个线程在 wait() 里睡下；5 s 真实时间内没等到就失败
void wait_for_sleepers(std::uint32_t count)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
    while (mock::waiting_count() != count)
    {
        assert(std::chrono::steady_clock::now() < deadline);
        std::this_thread::yield();
    }
}

// 给等待者一段真实时间，让它有机会（错误地）返回
void let_waiter_run()
{
    std::this_thread::sleep_for(std::chrono::milliseconds{20});
}

void notification_before_wait_is_kept()
{
    mock::reset();
    Signal signal;
    assert(!signal.wait(NO_WAIT));

    signal.notify();
    assert(signal.wait(NO_WAIT));
    assert(!signal.wait(NO_WAIT)); // 已经取走

    // 早到的通知让一直等的 wait() 立即返回，不用推进时间
    g_global_signal.notify();
    assert(g_global_signal.wait(WAIT_FOREVER));
    assert(mock::waiting_count() == 0U);
}

void notifications_merge()
{
    mock::reset();
    Signal signal;
    signal.notify();
    signal.notify();
    signal.notify();
    assert(signal.wait(NO_WAIT));
    assert(!signal.wait(NO_WAIT));
}

void another_thread_wakes_waiter()
{
    mock::reset();
    Signal signal;
    std::atomic<bool> result{false};
    std::thread waiter{[&]() { result.store(signal.wait(WAIT_FOREVER)); }};
    wait_for_sleepers(1U);
    signal.notify();
    waiter.join();
    assert(result.load());
    assert(mock::waiting_count() == 0U);
}

void timeout_follows_test_time()
{
    mock::reset();
    // 起点放在回绕前，超时时刻落在回绕之后
    mock::set_now_us(0xFFFFFF00U);
    Signal signal;
    std::atomic<bool> done{false};
    std::atomic<bool> result{true};
    std::thread waiter{[&]()
                       {
                           result.store(signal.wait(0x200U));
                           done.store(true);
                       }};
    wait_for_sleepers(1U);

    mock::advance_us(0x1FFU); // 差 1 µs
    let_waiter_run();
    assert(!done.load());

    mock::advance_us(1U);
    waiter.join();
    assert(!result.load());
    assert(hal::os::now_us() == 0x100U);
}

void notify_ends_timed_wait_early()
{
    mock::reset();
    Signal signal;
    std::atomic<bool> result{false};
    std::thread waiter{[&]() { result.store(signal.wait(1000U)); }};
    wait_for_sleepers(1U);
    signal.notify();
    waiter.join();
    assert(result.load());
    assert(hal::os::now_us() == 0U); // 没有推进时间
}

void one_thread_waits_on_several_signals()
{
    mock::reset();
    Signal first;
    Signal second;
    std::atomic<bool> done{false};
    std::atomic<bool> result{false};
    std::thread waiter{[&]()
                       {
                           result.store(first.wait(WAIT_FOREVER));
                           done.store(true);
                       }};
    wait_for_sleepers(1U);

    // 通知另一个 Signal 不会让 first 的 wait() 返回，second 的通知也留着
    second.notify();
    let_waiter_run();
    assert(!done.load());

    first.notify();
    waiter.join();
    assert(result.load());
    assert(second.wait(NO_WAIT));
}

void waiters_take_turns()
{
    mock::reset();
    Signal signal;
    for (int round = 0; round < 3; ++round)
    {
        std::atomic<bool> result{false};
        std::thread waiter{[&]() { result.store(signal.wait(WAIT_FOREVER)); }};
        wait_for_sleepers(1U);
        signal.notify();
        waiter.join();
        assert(result.load());
    }
}

// topic 的用法：发布者先写数据再 notify()；订阅者先检查数据，没有新的才 wait()。
// 检查与 wait() 之间来的通知留在 Signal 里，wait() 立即返回，所以不会漏掉最后一次发布。
// 漏了就会一直睡，mock 在 10 s 后 abort()。
void check_then_wait_never_misses_last_publish()
{
    mock::reset();
    constexpr std::uint32_t PUBLISH_COUNT = 200000U;
    Signal signal;
    std::atomic<std::uint32_t> published{0U};
    std::uint32_t wakeups = 0U;

    std::thread subscriber{[&]()
                           {
                               while (published.load() != PUBLISH_COUNT)
                               {
                                   if (signal.wait(WAIT_FOREVER))
                                   {
                                       ++wakeups;
                                   }
                               }
                           }};
    for (std::uint32_t i = 1U; i <= PUBLISH_COUNT; ++i)
    {
        published.store(i);
        signal.notify();
    }
    subscriber.join();

    // 多次通知会合并，醒来的次数不会超过发布次数
    assert(wakeups >= 1U);
    assert(wakeups <= PUBLISH_COUNT);
    // 最后一次通知可能在订阅者看到最终值之后才被取走，也可能还留着
    static_cast<void>(signal.wait(NO_WAIT));
    assert(!signal.wait(NO_WAIT));
}

} // namespace

int main()
{
    notification_before_wait_is_kept();
    notifications_merge();
    another_thread_wakes_waiter();
    timeout_follows_test_time();
    notify_ends_timed_wait_early();
    one_thread_waits_on_several_signals();
    waiters_take_turns();
    check_then_wait_never_misses_last_publish();
    return 0;
}
