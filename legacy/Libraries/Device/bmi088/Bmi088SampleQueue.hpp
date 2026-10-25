#ifndef LIBRARIES_DEVICE_BMI088_SAMPLE_QUEUE_HPP
#define LIBRARIES_DEVICE_BMI088_SAMPLE_QUEUE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{
// DRDY tick 是 MCU 中断观测时刻，不是传感器内部
// 转换时刻。所有时间戳差值均用无符号回绕运算。
struct Bmi088DataReady final
{
  std::uint32_t sequence{0U};
  std::uint32_t tick{0U};
};

struct Bmi088SampleRecord final
{
  enum : std::uint16_t
  {
    LateStart = 1U,
    NewerEventBeforeCompletion = 2U
  };
  std::uint32_t sequence{0U}; // 驱动成功读取计数
  std::uint32_t drdySequence{0U};
  std::uint32_t drdyTick{0U};
  std::uint32_t startTick{0U};
  std::uint32_t completedTick{0U};
  std::uint32_t harvestedTick{0U};
  std::int16_t xyz[3]{};
  std::uint16_t flags{0U};
};
static_assert(sizeof(Bmi088SampleRecord) == 32U);

// 单任务属主，非 ISR/RTOS 并发队列。溢出丢弃最新，
// 使慢消费者能察觉丢失，而不是悄悄覆盖旧数据。
template <std::size_t Capacity = 32U> class Bmi088SampleQueue final
{
public:
  static_assert(Capacity > 0U);
  bool Push(const Bmi088SampleRecord &record) noexcept
  {
    if (size_ == Capacity)
    {
      ++overflow_;
      return false;
    }
    records_[(head_ + size_) % Capacity] = record;
    ++size_;
    if (size_ > highWater_)
      highWater_ = static_cast<std::uint32_t>(size_);
    return true;
  }
  bool Pop(Bmi088SampleRecord &record) noexcept
  {
    if (size_ == 0U)
      return false;
    record = records_[head_];
    head_ = (head_ + 1U) % Capacity;
    --size_;
    return true;
  }
  bool Peek(Bmi088SampleRecord &record) const noexcept
  {
    if (size_ == 0U)
      return false;
    record = records_[head_];
    return true;
  }
  void Reset() noexcept
  {
    head_ = size_ = 0U;
    overflow_ = highWater_ = 0U;
  }
  [[nodiscard]] std::uint32_t OverflowCount() const noexcept
  {
    return overflow_;
  }
  [[nodiscard]] std::uint32_t HighWater() const noexcept
  {
    return highWater_;
  }

private:
  std::array<Bmi088SampleRecord, Capacity> records_{};
  std::size_t head_{0U}, size_{0U};
  std::uint32_t overflow_{0U}, highWater_{0U};
};
} // namespace device
#endif
