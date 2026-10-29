// utils_concurrency 的 SpscQueue（单生产者、单消费者环形队列）：空与满、先进先出、下标绕回、
// 批量读写的截断与绕回、空指针与零长度、clear、结构体元素、容量 1、计数器跨过 2^32，
// 以及生产者、消费者、旁观者三个线程的压力测试（顺序不乱、不丢不重、size 不超过容量）。
#include "utils/concurrency/spsc_queue.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <thread>

namespace
{

void starts_empty()
{
    concurrency::SpscQueue<std::uint32_t, 8U> queue;
    static_assert(concurrency::SpscQueue<std::uint32_t, 8U>::capacity() == 8U);
    assert(queue.size() == 0U);
    std::uint32_t value = 0U;
    assert(!queue.pop(value));
    std::uint32_t items[4] = {};
    assert(queue.pop(items, 4U) == 0U);
}

void fifo_until_full()
{
    concurrency::SpscQueue<std::uint32_t, 8U> queue;
    for (std::uint32_t i = 0U; i < 8U; ++i)
    {
        assert(queue.push(i));
    }
    assert(!queue.push(8U)); // 满了
    assert(queue.size() == 8U);
    for (std::uint32_t i = 0U; i < 8U; ++i)
    {
        std::uint32_t value = 0U;
        assert(queue.pop(value) && value == i);
    }
    std::uint32_t value = 0U;
    assert(!queue.pop(value));
    assert(queue.size() == 0U);
}

void index_wraps_around()
{
    concurrency::SpscQueue<std::uint32_t, 8U> queue;
    std::uint32_t next_in = 0U;
    std::uint32_t next_out = 0U;
    for (int round = 0; round < 100; ++round) // 每轮 3 个，下标反复绕回
    {
        for (int i = 0; i < 3; ++i)
        {
            assert(queue.push(next_in++));
        }
        for (int i = 0; i < 3; ++i)
        {
            std::uint32_t value = 0U;
            assert(queue.pop(value) && value == next_out++);
        }
    }
    assert(queue.size() == 0U);
}

void batch_truncates_and_wraps()
{
    concurrency::SpscQueue<std::uint32_t, 8U> queue;
    std::uint32_t value = 0U;
    for (std::uint32_t i = 0U; i < 5U; ++i) // 把下标推到 5
    {
        assert(queue.push(i));
        assert(queue.pop(value));
    }

    const std::uint32_t first[6] = {10U, 11U, 12U, 13U, 14U, 15U};
    assert(queue.push(first, 6U) == 6U); // 下标 5、6、7、0、1、2：分两段拷贝
    const std::uint32_t second[4] = {16U, 17U, 18U, 19U};
    assert(queue.push(second, 4U) == 2U); // 只剩 2 个空位，截断
    assert(queue.size() == 8U);
    assert(queue.push(second, 4U) == 0U);

    std::uint32_t out[10] = {};
    assert(queue.pop(out, 3U) == 3U); // 下标 5、6、7
    assert(out[0] == 10U && out[1] == 11U && out[2] == 12U);
    assert(queue.pop(out, 10U) == 5U); // 只有 5 个，从下标 0 开始
    const std::uint32_t expected[5] = {13U, 14U, 15U, 16U, 17U};
    assert(std::memcmp(out, expected, sizeof(expected)) == 0);
    assert(queue.size() == 0U);

    // 跨过末尾的批量读：下标 6、7、0、1
    for (std::uint32_t i = 0U; i < 4U; ++i)
    {
        assert(queue.push(i));
        assert(queue.pop(value));
    }
    const std::uint32_t third[4] = {20U, 21U, 22U, 23U};
    assert(queue.push(third, 4U) == 4U);
    assert(queue.pop(out, 4U) == 4U);
    assert(std::memcmp(out, third, sizeof(third)) == 0);
}

void ignores_null_and_zero_length()
{
    concurrency::SpscQueue<std::uint32_t, 8U> queue;
    const std::uint32_t items[3] = {1U, 2U, 3U};
    assert(queue.push(nullptr, 3U) == 0U);
    assert(queue.push(items, 0U) == 0U);
    assert(queue.size() == 0U);
    assert(queue.push(items, 3U) == 3U);
    std::uint32_t out[3] = {};
    assert(queue.pop(nullptr, 3U) == 0U);
    assert(queue.pop(out, 0U) == 0U);
    assert(queue.size() == 3U);
}

void clear_discards_existing_items()
{
    concurrency::SpscQueue<std::uint32_t, 8U> queue;
    for (std::uint32_t i = 0U; i < 3U; ++i)
    {
        assert(queue.push(i));
    }
    queue.clear();
    assert(queue.size() == 0U);
    std::uint32_t value = 0U;
    assert(!queue.pop(value));
    assert(queue.push(42U)); // clear 之后写入的保留
    assert(queue.pop(value) && value == 42U);
}

struct Sample
{
    float x;
    std::uint16_t id;
};

void holds_struct_elements()
{
    concurrency::SpscQueue<Sample, 4U> queue;
    assert(queue.push(Sample{1.5F, 7U}));
    const Sample batch[2] = {{2.5F, 8U}, {3.5F, 9U}};
    assert(queue.push(batch, 2U) == 2U);
    Sample out{};
    assert(queue.pop(out) && out.x == 1.5F && out.id == 7U);
    Sample outs[2] = {};
    assert(queue.pop(outs, 2U) == 2U);
    assert(outs[0].id == 8U && outs[1].x == 3.5F);
}

void capacity_of_one()
{
    concurrency::SpscQueue<std::uint32_t, 1U> queue;
    std::uint32_t value = 0U;
    for (std::uint32_t i = 0U; i < 5U; ++i)
    {
        assert(queue.push(i));
        assert(!queue.push(i));
        assert(queue.size() == 1U);
        assert(queue.pop(value) && value == i);
        assert(!queue.pop(value));
    }
}

// 计数器跨过 2^32：head、tail 回绕时，差值、下标与数据仍然正确。
// 队列始终保持半满，所以回绕时 head 与 tail 分处 2^32 两侧。每轮先写半个容量、再读半个容量，
// 约 8200 轮，共读写 2^32 个字节（-O2 下约 1 秒）。
constexpr std::size_t BIG_CAPACITY = std::size_t{1U} << 20U;
constexpr std::size_t HALF = BIG_CAPACITY / 2U;
concurrency::SpscQueue<std::uint8_t, BIG_CAPACITY> g_big_queue;
std::array<std::uint8_t, HALF> g_chunk;
std::array<std::uint8_t, HALF> g_received;

void counters_cross_two_to_the_32()
{
    // 第 n 个元素的值为 n & 0xFF；每次写入的起点都是 HALF 的倍数，所以每块的内容相同
    for (std::size_t i = 0U; i < HALF; ++i)
    {
        g_chunk[i] = static_cast<std::uint8_t>(i);
    }
    assert(g_big_queue.push(g_chunk.data(), HALF) == HALF);

    const std::uint64_t rounds = ((std::uint64_t{1U} << 32U) / HALF) + 8U;
    for (std::uint64_t round = 0U; round < rounds; ++round)
    {
        assert(g_big_queue.push(g_chunk.data(), HALF) == HALF);
        assert(g_big_queue.size() == BIG_CAPACITY);
        assert(g_big_queue.pop(g_received.data(), HALF) == HALF);
        assert(g_big_queue.size() == HALF);
        assert(std::memcmp(g_received.data(), g_chunk.data(), HALF) == 0);
    }
    // 结束时半满：再写 HALF 个正好写满
    assert(g_big_queue.push(g_chunk.data(), HALF) == HALF);
    assert(g_big_queue.size() == BIG_CAPACITY);
    assert(!g_big_queue.push(g_chunk[0]));
    std::uint8_t value = 1U;
    assert(g_big_queue.pop(value) && value == 0U);
}

// 生产者、消费者各一个线程，单个与批量读写混用。另有一个旁观线程不停调用 size()：
// 它的结果本身截到容量，没什么可断言的，这个线程用来检查第三方读取不构成数据竞争（用 ThreadSanitizer 运行时）。
void stress_producer_consumer()
{
    constexpr std::uint32_t TOTAL = 400000U;
    constexpr std::size_t CAPACITY = 64U; // 容量小，满与空频繁出现
    concurrency::SpscQueue<std::uint32_t, CAPACITY> queue;
    std::atomic<bool> done{false};
    std::uint64_t order_errors = 0U;

    std::thread producer(
        [&queue]
        {
            std::uint32_t next = 0U;
            std::uint32_t batch[17] = {};
            std::uint32_t step = 0U;
            while (next < TOTAL)
            {
                ++step;
                if ((step % 3U) == 0U)
                {
                    if (queue.push(next))
                    {
                        ++next;
                    }
                    else
                    {
                        std::this_thread::yield();
                    }
                    continue;
                }
                std::uint32_t count = 1U + (step % 17U);
                if (count > TOTAL - next)
                {
                    count = TOTAL - next;
                }
                for (std::uint32_t i = 0U; i < count; ++i)
                {
                    batch[i] = next + i;
                }
                const std::size_t pushed = queue.push(batch, count);
                next += static_cast<std::uint32_t>(pushed);
                if (pushed == 0U)
                {
                    std::this_thread::yield();
                }
            }
        });

    std::thread consumer(
        [&queue, &order_errors]
        {
            std::uint32_t expected = 0U;
            std::uint32_t batch[13] = {};
            std::uint32_t step = 0U;
            while (expected < TOTAL)
            {
                ++step;
                if ((step % 2U) == 0U)
                {
                    std::uint32_t value = 0U;
                    if (queue.pop(value))
                    {
                        order_errors += (value != expected) ? 1U : 0U;
                        expected = value + 1U;
                    }
                    else
                    {
                        std::this_thread::yield();
                    }
                    continue;
                }
                const std::size_t popped = queue.pop(batch, 1U + (step % 13U));
                for (std::size_t i = 0U; i < popped; ++i)
                {
                    order_errors += (batch[i] != expected) ? 1U : 0U;
                    expected = batch[i] + 1U;
                }
                if (popped == 0U)
                {
                    std::this_thread::yield();
                }
            }
        });

    std::thread observer(
        [&queue, &done]
        {
            while (!done.load(std::memory_order_relaxed))
            {
                static_cast<void>(queue.size());
            }
        });

    producer.join();
    consumer.join();
    done.store(true, std::memory_order_relaxed);
    observer.join();

    assert(order_errors == 0U);
    assert(queue.size() == 0U);
}

} // namespace

int main()
{
    starts_empty();
    fifo_until_full();
    index_wraps_around();
    batch_truncates_and_wraps();
    ignores_null_and_zero_length();
    clear_discards_existing_items();
    holds_struct_elements();
    capacity_of_one();
    counters_cross_two_to_the_32();
    stress_producer_consumer();
    return 0;
}
