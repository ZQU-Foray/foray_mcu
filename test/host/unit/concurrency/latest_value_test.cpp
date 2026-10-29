// utils_concurrency 的 LatestValue（单写者 seqlock）：初值、覆盖、大小不是 4 的倍数的类型、版本号回绕，
// 以及一个写线程、多个读线程的一致性压力测试（读到的数据自洽、与版本号一致、版本号不倒退）。
// 宿主是多核，读写真正并行；固件是单核，读者被写者打断的情况由同一套重试逻辑处理。
#include "utils/concurrency/latest_value.hpp"

#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

namespace
{

struct Pose
{
    float x;
    float y;
    std::int16_t heading;
    bool valid;
}; // 含填充，sizeof 为 12

struct Odd7
{
    std::uint8_t bytes[7]; // 不是 4 的倍数：最后一个字只用 3 个字节
};

struct Big
{
    std::uint32_t seq;
    std::uint32_t payload[31]; // 共 128 字节，拷贝时间长，容易被写者打断
};

constexpr std::uint32_t STRESS_WRITES = 2000000U;
constexpr int STRESS_READERS = 3;

void starts_empty()
{
    concurrency::LatestValue<Pose> box;
    assert(box.version() == 0U);
    Pose pose{1.0F, 2.0F, 3, true};
    assert(box.read(pose) == 0U); // 还没写过：全零字节，版本 0
    assert(pose.x == 0.0F && pose.y == 0.0F && pose.heading == 0 && !pose.valid);
}

void write_overwrites_and_read_does_not_consume()
{
    concurrency::LatestValue<Pose> box;
    box.write(Pose{1.5F, -2.5F, 90, true});
    assert(box.version() == 1U);
    Pose pose{};
    assert(box.read(pose) == 1U);
    assert(pose.x == 1.5F && pose.y == -2.5F && pose.heading == 90 && pose.valid);

    box.write(Pose{3.0F, 4.0F, -1, false});
    box.write(Pose{5.0F, 6.0F, 7, true}); // 覆盖，不排队
    assert(box.version() == 3U);
    assert(box.read(pose) == 3U);
    assert(pose.x == 5.0F && pose.y == 6.0F && pose.heading == 7 && pose.valid);
    assert(box.read(pose) == 3U); // 读不消费
}

void handles_sizes_not_multiple_of_four()
{
    concurrency::LatestValue<Odd7> box7;
    const Odd7 in{{1, 2, 3, 4, 5, 6, 7}};
    box7.write(in);
    Odd7 out{};
    assert(box7.read(out) == 1U);
    assert(std::memcmp(&in, &out, sizeof(in)) == 0);

    concurrency::LatestValue<std::uint8_t> box1;
    box1.write(0xA5U);
    std::uint8_t byte = 0U;
    assert(box1.read(byte) == 1U && byte == 0xA5U);

    concurrency::LatestValue<double> box_double; // 8 字节、对齐要求比 32 位字高
    box_double.write(3.25);
    double value = 0.0;
    assert(box_double.read(value) == 1U && value == 3.25);
}

void version_difference_uses_mask()
{
    constexpr std::uint32_t MASK = concurrency::LatestValue<int>::VERSION_MASK;
    const std::uint32_t older = MASK - 1U; // 回绕前
    const std::uint32_t newer = 2U;        // 回绕后，中间隔了 4 次写入
    assert(((newer - older) & MASK) == 4U);
}

// 版本号按 2^31 回绕：实写 2^31 + 3 次（-O2 下约 2 秒）
void version_wraps_at_two_to_the_31()
{
    concurrency::LatestValue<std::uint32_t> box;
    const std::uint32_t total = 0x80000003U;
    for (std::uint32_t k = 1U; k <= total; ++k)
    {
        box.write(k);
        if (k == concurrency::LatestValue<std::uint32_t>::VERSION_MASK)
        {
            std::uint32_t value = 0U;
            assert(box.read(value) == k && value == k);
        }
    }
    std::uint32_t value = 0U;
    assert(box.read(value) == 3U);
    assert(value == total);
}

struct ReaderResult
{
    std::uint64_t reads;
    std::uint64_t torn;      // 数据不自洽，或与版本号对不上
    std::uint64_t backwards; // 版本号倒退
};

// 写第 k 次时全部字段都由 k 生成；读者用 verify 检查读到的数据确实是第 version 次写入的完整内容
template <typename Msg, typename Fill, typename Verify>
void one_writer_many_readers(Fill fill, Verify verify)
{
    concurrency::LatestValue<Msg> box;
    std::atomic<bool> done{false};
    std::vector<ReaderResult> results(STRESS_READERS, ReaderResult{0U, 0U, 0U});

    std::vector<std::thread> readers;
    for (int r = 0; r < STRESS_READERS; ++r)
    {
        readers.emplace_back(
            [&box, &done, &verify, &result = results[static_cast<std::size_t>(r)]]
            {
                const Msg zero{};
                std::uint32_t last = 0U;
                Msg msg{};
                while (!done.load(std::memory_order_relaxed))
                {
                    const std::uint32_t version = box.read(msg);
                    ++result.reads;
                    const bool consistent =
                        (version == 0U) ? (std::memcmp(&msg, &zero, sizeof(Msg)) == 0) : verify(msg, version);
                    if (!consistent)
                    {
                        ++result.torn;
                    }
                    if (version < last || box.version() < version)
                    {
                        ++result.backwards;
                    }
                    last = version;
                }
            });
    }

    for (std::uint32_t k = 1U; k <= STRESS_WRITES; ++k)
    {
        Msg msg{};
        fill(msg, k);
        box.write(msg);
    }
    done.store(true, std::memory_order_relaxed);
    for (std::thread &reader : readers)
    {
        reader.join();
    }

    for (const ReaderResult &result : results)
    {
        assert(result.reads > 0U);
        assert(result.torn == 0U);
        assert(result.backwards == 0U);
    }
    Msg msg{};
    assert(box.read(msg) == STRESS_WRITES);
    assert(verify(msg, STRESS_WRITES));
}

std::uint32_t big_payload(std::uint32_t k, std::uint32_t i)
{
    return k * 2654435761U + i; // 每个字都不同，错位或混入别的版本都能发现
}

void stress_big_message()
{
    one_writer_many_readers<Big>(
        [](Big &msg, std::uint32_t k)
        {
            msg.seq = k;
            for (std::uint32_t i = 0U; i < 31U; ++i)
            {
                msg.payload[i] = big_payload(k, i);
            }
        },
        [](const Big &msg, std::uint32_t version)
        {
            if (msg.seq != version)
            {
                return false;
            }
            for (std::uint32_t i = 0U; i < 31U; ++i)
            {
                if (msg.payload[i] != big_payload(version, i))
                {
                    return false;
                }
            }
            return true;
        });
}

void stress_odd_sized_message()
{
    one_writer_many_readers<Odd7>(
        [](Odd7 &msg, std::uint32_t k)
        {
            for (std::uint8_t &byte : msg.bytes)
            {
                byte = static_cast<std::uint8_t>(k);
            }
        },
        [](const Odd7 &msg, std::uint32_t version)
        {
            for (const std::uint8_t byte : msg.bytes)
            {
                if (byte != static_cast<std::uint8_t>(version))
                {
                    return false;
                }
            }
            return true;
        });
}

} // namespace

int main()
{
    starts_empty();
    write_overwrites_and_read_does_not_consume();
    handles_sizes_not_multiple_of_four();
    version_difference_uses_mask();
    version_wraps_at_two_to_the_31();
    stress_big_message();
    stress_odd_sized_message();
    return 0;
}
