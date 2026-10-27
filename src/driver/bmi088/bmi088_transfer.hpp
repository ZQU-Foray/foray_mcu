#ifndef DRIVER_BMI088_BMI088_TRANSFER_HPP
#define DRIVER_BMI088_BMI088_TRANSFER_HPP

#include "hal/spi.hpp"
#include "hal/time.hpp"
#include <atomic>

namespace device
{
// 由属主在启动传输前配置。ISR 只做时间戳与通知；
// 解析、队列与调度始终归属任务。
class Bmi088Transfer final
{
  public:
    void set_notification(platform::Spi::CompletionNotification notification, void *context) noexcept
    {
        notification_ = notification;
        context_ = context;
    }
    static void complete(void *context) noexcept
    {
        auto &self = *static_cast<Bmi088Transfer *>(context);
        self.completed_tick_.store(platform::Time::now_ticks(), std::memory_order_release);
        if (self.notification_ != nullptr)
            self.notification_(self.context_);
    }
    [[nodiscard]] std::uint32_t completed_tick() const noexcept
    {
        return completed_tick_.load(std::memory_order_acquire);
    }

  private:
    static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
    std::atomic<std::uint32_t> completed_tick_{0U};
    platform::Spi::CompletionNotification notification_{nullptr};
    void *context_{nullptr};
};
} // namespace device
#endif
