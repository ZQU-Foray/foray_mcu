#ifndef UTILS_CONCURRENCY_SPSC_QUEUE_HPP
#define UTILS_CONCURRENCY_SPSC_QUEUE_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace concurrency
{

template <typename T, std::size_t Capacity>
class SpscQueue
{
    // 下标用 head & (Capacity - 1U) 计算，只有 Capacity 是 2 的幂时才正确
    static_assert(Capacity > 0U && (Capacity & (Capacity - 1U)) == 0U, "Capacity 必须是大于 0 的 2 的幂");
    // 计数器是 32 位，head - tail 最大为 Capacity，超过 2^31 时满与空无法区分
    static_assert(Capacity <= (std::size_t{1U} << 31U), "Capacity 不能超过 2^31");
    // 批量读写用 memcpy
    static_assert(std::is_trivially_copyable_v<T>, "T 必须可平凡拷贝");
    // 不检查 std::atomic<std::uint32_t>::is_always_lock_free：它要求全部原子操作（含读改写）都无锁，
    // Cortex-M0+ 没有 LDREX/STREX，GCC 在那里报 false。本队列只用原子读、原子写，在 Cortex-M0+、
    // M4、M7 上都编译成单条 LDR/STR 加 DMB（GCC 15 实测），不会退化成加锁实现。

  public:
    // 生产者端：只能有一个调用方（一个任务或一个中断），可以在中断里调用
    bool push(const T &item) noexcept;
    std::size_t push(const T *items, std::size_t count) noexcept; // 空间不足时截断，返回实际写入个数
    // 消费者端
    bool pop(T &item) noexcept;
    std::size_t pop(T *items, std::size_t max_count) noexcept;
    void clear() noexcept; // 丢弃已有数据，只能由消费者调用
    // 任何一方都可调用，返回某一时刻的快照
    std::size_t size() const noexcept;
    static constexpr std::size_t capacity() noexcept
    {
        return Capacity;
    }

  private:
    std::array<T, Capacity> buffer_{};
    std::atomic<std::uint32_t> head_{0U}; // 只由生产者写
    std::atomic<std::uint32_t> tail_{0U}; // 只由消费者写
};

template <typename T, std::size_t Capacity>
bool SpscQueue<T, Capacity>::push(const T &item) noexcept
{
    const std::uint32_t head = head_.load(std::memory_order_relaxed);
    const std::uint32_t tail = tail_.load(std::memory_order_acquire);

    if (head - tail >= Capacity)
    {
        return false;
    }

    buffer_[head & (Capacity - 1U)] = item; // Capacity 是 2 的幂，用掩码代替取模
    head_.store(head + 1U, std::memory_order_release);
    return true;
}

template <typename T, std::size_t Capacity>
std::size_t SpscQueue<T, Capacity>::push(const T *items, std::size_t count) noexcept
{
    if (count == 0 || items == nullptr)
    {
        return 0;
    }

    const std::uint32_t head = head_.load(std::memory_order_relaxed);
    const std::uint32_t tail = tail_.load(std::memory_order_acquire);

    const std::uint32_t used = head - tail;
    const std::size_t free_space = Capacity - used;

    const std::size_t n = (count < free_space) ? count : free_space;
    if (n == 0)
    {
        return 0;
    }

    const std::uint32_t idx = head & static_cast<std::uint32_t>(Capacity - 1U);
    std::size_t first = Capacity - idx;
    if (first > n)
    {
        first = n;
    }

    std::memcpy(&buffer_[idx], items, first * sizeof(T));

    if (n > first)
    {
        std::memcpy(&buffer_[0], items + first, (n - first) * sizeof(T));
    }

    head_.store(head + static_cast<std::uint32_t>(n), std::memory_order_release);

    return n;
}

template <typename T, std::size_t Capacity>
bool SpscQueue<T, Capacity>::pop(T &item) noexcept
{
    const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
    const std::uint32_t head = head_.load(std::memory_order_acquire);

    if (head == tail)
    {
        return false;
    }

    item = buffer_[tail & static_cast<std::uint32_t>(Capacity - 1U)];
    tail_.store(tail + 1U, std::memory_order_release);

    return true;
}

template <typename T, std::size_t Capacity>
std::size_t SpscQueue<T, Capacity>::pop(T *items, std::size_t max_count) noexcept
{
    if (max_count == 0 || items == nullptr)
    {
        return 0;
    }

    const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
    const std::uint32_t head = head_.load(std::memory_order_acquire);

    const std::size_t available = head - tail;
    const std::size_t n = (max_count < available) ? max_count : available;
    if (n == 0)
    {
        return 0;
    }

    const std::uint32_t idx = tail & static_cast<std::uint32_t>(Capacity - 1U);
    std::size_t first = Capacity - idx;
    if (first > n)
    {
        first = n;
    }

    std::memcpy(items, &buffer_[idx], first * sizeof(T));

    if (n > first)
    {
        std::memcpy(items + first, &buffer_[0], (n - first) * sizeof(T));
    }

    tail_.store(tail + static_cast<std::uint32_t>(n), std::memory_order_release);

    return n;
}

template <typename T, std::size_t Capacity>
void SpscQueue<T, Capacity>::clear() noexcept
{
    // tail 追平 head：调用时已经写入的数据全部丢弃，之后生产者写入的数据保留
    tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
}

template <typename T, std::size_t Capacity>
std::size_t SpscQueue<T, Capacity>::size() const noexcept
{
    // 先读 tail 再读 head：head 只增不减，后读到的 head 不会小于 tail，差值不会下溢。
    // 生产者或消费者调用时结果不超过 Capacity；第三方（如调试任务）调用时，两次读取之间
    // 双方可能都前进了，差值可能偏大，所以截到 Capacity。结果只是某一时刻的快照。
    const std::uint32_t tail = tail_.load(std::memory_order_acquire);
    const std::uint32_t head = head_.load(std::memory_order_acquire);
    const std::size_t used = head - tail;
    return (used < Capacity) ? used : Capacity;
}

} // namespace concurrency

#endif
