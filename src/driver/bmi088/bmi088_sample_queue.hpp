#ifndef DRIVER_BMI088_BMI088_SAMPLE_QUEUE_HPP
#define DRIVER_BMI088_BMI088_SAMPLE_QUEUE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{
// 时间戳都是 hal::os::now_us() 的读数（µs，按 2^32 回绕）。DRDY 时刻是 MCU 中断观测时刻，
// 不是传感器内部转换时刻。所有时间戳差值均用无符号回绕运算。
struct Bmi088DataReady final
{
    std::uint32_t sequence{0U};     // 累计的数据就绪中断次数
    std::uint32_t timestamp_us{0U}; // 最近一次中断的时刻
};

struct Bmi088SampleRecord final
{
    enum : std::uint16_t
    {
        LATE_START = 1U,
        NEWER_EVENT_BEFORE_COMPLETION = 2U
    };
    std::uint32_t sequence{0U}; // 驱动成功读取计数
    std::uint32_t drdy_sequence{0U};
    std::uint32_t drdy_us{0U};      // 触发这次读取的数据就绪中断时刻
    std::uint32_t start_us{0U};     // SPI 事务开始时刻
    std::uint32_t completed_us{0U}; // SPI 事务完成（中断）时刻
    std::uint32_t harvested_us{0U}; // 任务收割时刻
    std::int16_t xyz[3]{};
    std::uint16_t flags{0U};
};
static_assert(sizeof(Bmi088SampleRecord) == 32U);

// 单任务属主，非 ISR/RTOS 并发队列。溢出丢弃最新，
// 使慢消费者能察觉丢失，而不是悄悄覆盖旧数据。
template <std::size_t Capacity = 32U>
class Bmi088SampleQueue final
{
  public:
    static_assert(Capacity > 0U);
    bool push(const Bmi088SampleRecord &record) noexcept
    {
        if (size_ == Capacity)
        {
            ++overflow_;
            return false;
        }
        records_[(head_ + size_) % Capacity] = record;
        ++size_;
        if (size_ > high_water_)
            high_water_ = static_cast<std::uint32_t>(size_);
        return true;
    }
    bool pop(Bmi088SampleRecord &record) noexcept
    {
        if (size_ == 0U)
            return false;
        record = records_[head_];
        head_ = (head_ + 1U) % Capacity;
        --size_;
        return true;
    }
    bool peek(Bmi088SampleRecord &record) const noexcept
    {
        if (size_ == 0U)
            return false;
        record = records_[head_];
        return true;
    }
    void reset() noexcept
    {
        head_ = size_ = 0U;
        overflow_ = high_water_ = 0U;
    }
    [[nodiscard]] std::uint32_t overflow_count() const noexcept
    {
        return overflow_;
    }
    [[nodiscard]] std::uint32_t high_water() const noexcept
    {
        return high_water_;
    }

  private:
    std::array<Bmi088SampleRecord, Capacity> records_{};
    std::size_t head_{0U}, size_{0U};
    std::uint32_t overflow_{0U}, high_water_{0U};
};
} // namespace device
#endif
