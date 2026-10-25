#ifndef LIBRARIES_DEVICE_BMI088_TRANSFER_HPP
#define LIBRARIES_DEVICE_BMI088_TRANSFER_HPP

#include "Platform/Interface/Spi.hpp"
#include "Platform/Interface/Time.hpp"
#include <atomic>

namespace device
{
// 由属主在启动传输前配置。ISR 只做时间戳与通知；
// 解析、队列与调度始终归属任务。
class Bmi088Transfer final
{
public:
  void SetNotification(platform::Spi::CompletionNotification notification, void *context) noexcept
  {
    notification_ = notification;
    context_ = context;
  }
  static void Complete(void *context) noexcept
  {
    auto &self = *static_cast<Bmi088Transfer *>(context);
    self.completedTick_.store(platform::Time::NowTicks(), std::memory_order_release);
    if (self.notification_ != nullptr)
      self.notification_(self.context_);
  }
  [[nodiscard]] std::uint32_t CompletedTick() const noexcept
  {
    return completedTick_.load(std::memory_order_acquire);
  }

private:
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
  std::atomic<std::uint32_t> completedTick_{0U};
  platform::Spi::CompletionNotification notification_{nullptr};
  void *context_{nullptr};
};
} // namespace device
#endif
