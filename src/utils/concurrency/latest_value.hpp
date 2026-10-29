#ifndef UTILS_CONCURRENCY_LATEST_VALUE_HPP
#define UTILS_CONCURRENCY_LATEST_VALUE_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace concurrency
{

// 最新值邮箱：一个写者、多个读者；写入覆盖旧值，读者读到最近一次写完的数据。
// 实现是 seqlock（序号锁）：只用原子读、原子写与内存屏障，不用原子读改写（规则 13），Cortex-M0+ 也能用。
//
// 使用限制：读者遇到写入中途会一直重试，直到写者写完，所以读者不能在写入中途抢占写者。
//   - 写者在中断里、读者在任务里：可以（写者打断读者，读者事后发现序号变了就重读）；
//   - 都在任务里：读者的优先级不能高于写者，否则写者被读者抢占后再也轮不到运行，读者无限重试（单核上即死锁）；
//   - 读者在中断里、写者在任务里：不可以，理由同上。
//   上层在写入期间关中断（如 topic 用 hal::os::CriticalSection 包住 write）时，写入不会被打断，以上限制都不存在。
//
// 版本号：还没写过时为 0，每写一次加 1。序号的最低位用来标记"正在写"，所以版本号只有 31 位，按 2^31 回绕；
// 判断有没有新数据只比较是否相等；求两个版本号之差要先相减再与 VERSION_MASK 按位与。
template <typename T>
class LatestValue
{
    // write、read 用 memcpy 在 T 与 32 位字之间转换
    static_assert(std::is_trivially_copyable_v<T>, "T 必须可平凡拷贝");
    // 与 SpscQueue 相同，不检查 std::atomic<std::uint32_t>::is_always_lock_free（Cortex-M0+ 上为 false）：
    // 本类只用原子读、原子写与屏障，在 Cortex-M0+、M4、M7 上都编译成 LDR/STR 加 DMB，不会退化成加锁实现。

  public:
    static constexpr std::uint32_t VERSION_MASK = 0x7FFFFFFFU;

    // 写者：只能有一个（一个任务或一个中断），永不阻塞，覆盖旧值
    void write(const T &value) noexcept;
    // 读者：可以有多个；读到一份完整的数据（不会读到写了一半的），返回版本号（每写一次加 1）
    // 还没写过时读到全零字节，返回 0
    std::uint32_t read(T &out) const noexcept;
    // 只看版本号不拷贝，topic 的 update() 用它判断有没有新数据；写入中途调用时返回上一次写完的版本
    std::uint32_t version() const noexcept;

  private:
    static constexpr std::size_t WORD_COUNT = (sizeof(T) + 3U) / 4U;

    std::atomic<std::uint32_t> seq_{0U};                         // 奇数 = 正在写；版本号 = seq_ / 2
    std::array<std::atomic<std::uint32_t>, WORD_COUNT> words_{}; // 数据按 32 位字存
};

// 为什么数据按原子字存：读者拷贝时写者可能正在改同一块内存。用普通的 T 加 memcpy，
// 在 C++ 里就是数据竞争（未定义行为，ThreadSanitizer 也会报）；按 32 位字做 relaxed 原子读写，
// 再用屏障排好"序号"与"数据"的先后，读到写了一半的数据也只是重试，不是未定义行为。
// 屏障的配对（Boehm, "Can Seqlocks Get Along with Programming Language Memory Models?", 2012）：
//   写者：seq_ 写奇数 → release 屏障 → 写数据 → seq_ 写偶数（release）
//   读者：seq_ 读（acquire）→ 读数据 → acquire 屏障 → seq_ 再读
//   读者只要读到一个新写的字，两道屏障就保证它第二次读 seq_ 时看得到奇数或更新的值，于是两次不等、重试；
//   第一次读到偶数时，acquire 保证这次写入的全部数据都已可见。

template <typename T>
void LatestValue<T>::write(const T &value) noexcept
{
    // 先拷到本地的字数组。sizeof(T) 不是 4 的倍数时，最后一个字有空余字节，先补 0，
    // 避免把未初始化的值写进去
    std::array<std::uint32_t, WORD_COUNT> buffer;
    buffer[WORD_COUNT - 1U] = 0U;
    std::memcpy(buffer.data(), &value, sizeof(T));

    // 只有写者改 seq_，读到的就是自己上次写的值，"读一次、写一次"代替原子读改写
    const std::uint32_t seq = seq_.load(std::memory_order_relaxed);
    seq_.store(seq + 1U, std::memory_order_relaxed); // 变成奇数：开始写
    std::atomic_thread_fence(std::memory_order_release);
    for (std::size_t i = 0U; i < WORD_COUNT; ++i)
    {
        words_[i].store(buffer[i], std::memory_order_relaxed);
    }
    seq_.store(seq + 2U, std::memory_order_release); // 变回偶数：写完
}

template <typename T>
std::uint32_t LatestValue<T>::read(T &out) const noexcept
{
    std::array<std::uint32_t, WORD_COUNT> buffer;
    std::uint32_t seq = 0U;
    for (;;)
    {
        seq = seq_.load(std::memory_order_acquire);
        if ((seq & 1U) != 0U)
        {
            continue; // 写者正在写，等它写完
        }
        for (std::size_t i = 0U; i < WORD_COUNT; ++i)
        {
            buffer[i] = words_[i].load(std::memory_order_relaxed);
        }
        std::atomic_thread_fence(std::memory_order_acquire); // 上面读数据不能挪到下面再读 seq_ 之后
        if (seq_.load(std::memory_order_relaxed) == seq)
        {
            break; // 拷贝期间没有新的写入，数据完整
        }
    }
    std::memcpy(&out, buffer.data(), sizeof(T));
    return seq >> 1U;
}

template <typename T>
std::uint32_t LatestValue<T>::version() const noexcept
{
    // 写入中途 seq_ 是奇数，右移一位正好是上一次写完的版本
    return seq_.load(std::memory_order_acquire) >> 1U;
}

} // namespace concurrency

#endif
