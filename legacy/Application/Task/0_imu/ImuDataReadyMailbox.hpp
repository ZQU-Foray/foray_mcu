#ifndef APPLICATION_TASK_IMU_DATA_READY_MAILBOX_HPP
#define APPLICATION_TASK_IMU_DATA_READY_MAILBOX_HPP

#include "Libraries/Device/bmi088/Bmi088SampleQueue.hpp"
#include <atomic>

namespace application::task
{
// 单 ISR 写入、单任务读取。所有字段均为原子量：普通字段上的 seqlock
// 会构成 C++ 数据竞争。最新值邮箱故意不假装旧传感器寄存器还能被读回。
class ImuDataReadyMailbox final
{
public:
  void Publish(std::uint32_t tick) noexcept
  {
    version_.fetch_add(1U);
    tick_.store(tick);
    sequence_.fetch_add(1U);
    version_.fetch_add(1U);
  }
  device::Bmi088DataReady Snapshot() const noexcept
  {
    for (;;)
    {
      const auto before = version_.load();
      if ((before & 1U) != 0U)
        continue;
      const device::Bmi088DataReady result{sequence_.load(), tick_.load()};
      if (before == version_.load())
        return result;
    }
  }

private:
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
  std::atomic<std::uint32_t> version_{0U}, sequence_{0U}, tick_{0U};
};
} // namespace application::task
#endif
