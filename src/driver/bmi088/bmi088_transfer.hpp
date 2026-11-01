#ifndef DRIVER_BMI088_BMI088_TRANSFER_HPP
#define DRIVER_BMI088_BMI088_TRANSFER_HPP

#include "hal/callback.hpp"
#include "hal/os/clock.hpp"
#include <atomic>
#include <cstdint>

namespace device
{
// 由属主在启动传输前配置。完成回调（框架中断）只做时间戳与通知；
// 解析、队列与调度始终归属任务。
class Bmi088Transfer final
{
  public:
    void set_notification(hal::Callback notification, void *context) noexcept
    {
        notification_ = notification;
        context_ = context;
    }
    // hal::Spi 异步事务结束时在框架中断里调用
    static void complete(void *context) noexcept
    {
        auto &self = *static_cast<Bmi088Transfer *>(context);
        self.completed_us_.store(hal::os::now_us(), std::memory_order_release);
        if (self.notification_ != nullptr)
            self.notification_(self.context_);
    }
    [[nodiscard]] hal::os::Microseconds completed_us() const noexcept
    {
        return completed_us_.load(std::memory_order_acquire);
    }

  private:
    // 只用原子读、原子写（中断写、任务读），不用读改写：Cortex-M0+ 上也是单条 LDR/STR 加屏障（规则 13）。
    // 不检查 is_always_lock_free：它要求读改写也无锁，Cortex-M0+ 上为 false（同 utils/concurrency）。
    std::atomic<hal::os::Microseconds> completed_us_{0U};
    hal::Callback notification_{nullptr};
    void *context_{nullptr};
};
} // namespace device
#endif
