// platform_mock 对 Signal 误用的检查：每个场景都应该打印原因并 abort()。
// ctest 把被信号终止的进程一律算失败，所以这里接住 SIGABRT 正常退出；
// ctest 按场景各登记一次，用 PASS_REGULAR_EXPRESSION 检查打印的原因（见 test/host/CMakeLists.txt）。
//   notify_in_critical：在 CriticalSection 内调用 notify()
//   wait_in_critical：  在 CriticalSection 内调用 wait()
//   two_waiters：       两个线程同时等同一个 Signal
#include "hal/os/critical_section.hpp"
#include "hal/os/signal.hpp"
#include "mock_os.hpp"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace
{

hal::os::Signal g_signal;

void exit_on_abort(int /*signal_number*/)
{
    std::_Exit(EXIT_FAILURE);
}

void notify_in_critical()
{
    const hal::os::CriticalSection section;
    g_signal.notify();
}

void wait_in_critical()
{
    const hal::os::CriticalSection section;
    static_cast<void>(g_signal.wait(hal::os::NO_WAIT));
}

void two_waiters()
{
    std::thread first{[]() { static_cast<void>(g_signal.wait(hal::os::WAIT_FOREVER)); }};
    while (mock::waiting_count() != 1U)
    {
        std::this_thread::yield();
    }
    static_cast<void>(g_signal.wait(hal::os::NO_WAIT)); // 在这里 abort()
    first.join();
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "用法：%s notify_in_critical|wait_in_critical|two_waiters\n", argv[0]);
        return 2;
    }
    static_cast<void>(std::signal(SIGABRT, exit_on_abort));
    mock::reset();
    const char *scenario = argv[1];
    if (std::strcmp(scenario, "notify_in_critical") == 0)
    {
        notify_in_critical();
    }
    else if (std::strcmp(scenario, "wait_in_critical") == 0)
    {
        wait_in_critical();
    }
    else if (std::strcmp(scenario, "two_waiters") == 0)
    {
        two_waiters();
    }
    else
    {
        std::fprintf(stderr, "未知场景：%s\n", scenario);
        return 2;
    }
    // 走到这里说明误用没有被拦住
    std::fprintf(stderr, "场景 %s 没有 abort()\n", scenario);
    return 0;
}
