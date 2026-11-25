// estimate::BiasStore：日志式记录（追加、CRC、坏记录、区满压缩）、温度模型（常数 / 直线 / 外推夹紧 /
// 权重 / 斜率护栏）、参数检查、设备不可用与写失败。存储后端用一个进程内的假块设备（带擦除与对齐语义）。
#include "lib/estimate/bias_store.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace
{

constexpr std::size_t REGION_SIZE{4096U};
constexpr std::size_t GRANULARITY{32U};

struct FakeStorage
{
    std::array<std::uint8_t, REGION_SIZE> bytes{};
    std::size_t write_count{0U};
    std::size_t erase_count{0U};
    bool fail_write{false};
    bool fail_erase{false};
    bool available{true};
};

std::size_t device_size(void *context) noexcept
{
    return static_cast<FakeStorage *>(context)->available ? REGION_SIZE : 0U;
}

std::size_t device_granularity(void *context) noexcept
{
    return static_cast<FakeStorage *>(context)->available ? GRANULARITY : 0U;
}

bool device_read(void *context, std::size_t offset, std::uint8_t *data, std::size_t length) noexcept
{
    auto &device = *static_cast<FakeStorage *>(context);
    if (data == nullptr || length == 0U || offset > REGION_SIZE || length > REGION_SIZE - offset)
    {
        return false;
    }
    std::memcpy(data, device.bytes.data() + offset, length);
    return true;
}

bool device_write(void *context, std::size_t offset, const std::uint8_t *data, std::size_t length) noexcept
{
    auto &device = *static_cast<FakeStorage *>(context);
    if (device.fail_write || data == nullptr || length == 0U || offset > REGION_SIZE || length > REGION_SIZE - offset ||
        (offset % GRANULARITY) != 0U || (length % GRANULARITY) != 0U)
    {
        return false;
    }
    for (std::size_t index = 0U; index < length; ++index)
    {
        if (device.bytes[offset + index] != 0xFFU)
        {
            return false;
        }
    }
    std::memcpy(device.bytes.data() + offset, data, length);
    ++device.write_count;
    return true;
}

bool device_erase(void *context) noexcept
{
    auto &device = *static_cast<FakeStorage *>(context);
    if (device.fail_erase)
    {
        return false;
    }
    device.bytes.fill(0xFFU);
    ++device.erase_count;
    return true;
}

estimate::BiasStore::Storage storage_of(FakeStorage &device) noexcept
{
    return estimate::BiasStore::Storage{
        &device, device_size, device_granularity, device_read, device_write, device_erase};
}

void fill_erased(FakeStorage &device) noexcept
{
    device.bytes.fill(0xFFU);
}

estimate::BiasStore::Config fast_config() noexcept
{
    estimate::BiasStore::Config config{};
    config.maximum_records = 8U;
    config.temperature_span_celsius = 0.5F;
    config.extrapolation_celsius = 3.0F;
    config.maximum_slope_rad_per_sec_per_celsius = 0.02F;
    return config;
}

bool close(float first, float second) noexcept
{
    return std::fabs(first - second) < 1e-5F;
}

bool close(const math::Vector3 &first, const math::Vector3 &second) noexcept
{
    return close(first.x, second.x) && close(first.y, second.y) && close(first.z, second.z);
}

const math::Vector3 BIAS_A{0.010F, -0.009F, 0.003F};

void empty_region_has_no_bias()
{
    FakeStorage device;
    fill_erased(device);
    estimate::BiasStore store;
    assert(store.init(storage_of(device), fast_config()));
    assert(store.available() && !store.has_bias());
    assert(close(store.bias_at(25.0F), math::Vector3{}));
    assert(store.snapshot().capacity == REGION_SIZE / GRANULARITY);
    assert(store.snapshot().append_slot == 0U);
    assert(device.write_count == 0U && device.erase_count == 0U);
}

void one_record_gives_a_constant()
{
    FakeStorage device;
    fill_erased(device);
    estimate::BiasStore store;
    assert(store.init(storage_of(device), fast_config()));
    assert(store.record(42.0F, BIAS_A, 500000U));
    assert(store.has_bias() && !store.snapshot().fit_is_line);
    assert(close(store.newest_temperature_celsius(), 42.0F));
    assert(close(store.bias_at(42.0F), BIAS_A));
    assert(close(store.bias_at(10.0F), BIAS_A)); // 一个点：哪里都是常数
    assert(close(store.bias_at(std::numeric_limits<float>::quiet_NaN()), BIAS_A));
    assert(store.snapshot().records == 1U && store.snapshot().written == 1U);
    assert(device.write_count == 1U);
}

void two_temperatures_give_a_line_and_clamp()
{
    FakeStorage device;
    fill_erased(device);
    estimate::BiasStore store;
    assert(store.init(storage_of(device), fast_config()));
    const math::Vector3 low{0.010F, -0.010F, 0.020F};
    const math::Vector3 high{0.020F, -0.020F, 0.030F};
    assert(store.record(40.0F, low, 500000U));
    assert(store.record(50.0F, high, 5000000U));
    assert(store.snapshot().fit_is_line);
    const math::Vector3 middle = store.bias_at(45.0F);
    assert(close(middle, math::Vector3{0.015F, -0.015F, 0.025F}));
    // 外推夹在观测范围外 3 ℃：40 → 37、50 → 53
    assert(close(store.bias_at(0.0F), math::Vector3{0.007F, -0.007F, 0.017F}));
    assert(close(store.bias_at(100.0F), math::Vector3{0.023F, -0.023F, 0.033F}));
    assert(close(store.snapshot().slope_rad_per_sec_per_celsius, math::Vector3{0.001F, -0.001F, 0.001F}));
}

void weights_favour_the_longer_window()
{
    FakeStorage device;
    fill_erased(device);
    estimate::BiasStore store;
    assert(store.init(storage_of(device), fast_config()));
    assert(store.record(42.0F, math::Vector3{0.010F, 0.0F, 0.0F}, 500000U));  // 快窗口：权重 500
    assert(store.record(42.0F, math::Vector3{0.020F, 0.0F, 0.0F}, 5000000U)); // 精确：权重 5000
    assert(!store.snapshot().fit_is_line);
    assert(std::fabs(store.bias_at(42.0F).x - 0.0190909F) < 1e-5F);
    // 窗口为 0 按 1 ms 计，不让权重变成 0
    assert(store.record(43.0F, math::Vector3{0.5F, 0.0F, 0.0F}, 0U));
    assert(store.snapshot().records == 3U);
}

void steep_slope_falls_back_to_constant()
{
    FakeStorage device;
    fill_erased(device);
    estimate::BiasStore store;
    assert(store.init(storage_of(device), fast_config()));
    assert(store.record(40.0F, math::Vector3{0.0F, 0.0F, 0.0F}, 1000000U));
    assert(store.record(50.0F, math::Vector3{1.0F, 0.0F, 0.0F}, 1000000U)); // 0.1 rad/s/℃ > 上限 0.02
    assert(!store.snapshot().fit_is_line);
    assert(close(store.bias_at(45.0F), math::Vector3{0.5F, 0.0F, 0.0F}));
}

void corrupt_record_stops_the_scan_and_compacts()
{
    FakeStorage device;
    fill_erased(device);
    estimate::BiasStore store;
    assert(store.init(storage_of(device), fast_config()));
    assert(store.record(40.0F, BIAS_A, 1000000U));
    assert(store.record(41.0F, BIAS_A, 1000000U));
    assert(store.record(42.0F, BIAS_A, 1000000U));
    // 第 1 条（槽 1）的 CRC 位置翻一位
    device.bytes[GRANULARITY * 1U + 30U] ^= 0xFFU;

    estimate::BiasStore reopened;
    assert(reopened.init(storage_of(device), fast_config()));
    assert(reopened.snapshot().compactions == 1U && device.erase_count == 1U);
    assert(reopened.snapshot().records == 1U); // 只留着槽 0 那条
    assert(close(reopened.newest_temperature_celsius(), 40.0F));
    assert(reopened.snapshot().append_slot == 1U);
    assert(reopened.record(41.0F, BIAS_A, 1000000U)); // 压缩之后还能写
}

void full_region_compacts_at_open()
{
    FakeStorage device;
    fill_erased(device);
    estimate::BiasStore store;
    assert(store.init(storage_of(device), fast_config()));
    const std::size_t capacity = store.snapshot().capacity;
    for (std::size_t index = 0U; index < capacity; ++index)
    {
        const float temperature = 40.0F + static_cast<float>(index % 10U);
        assert(store.record(temperature, math::Vector3{0.01F, 0.0F, 0.0F}, 1000000U));
    }
    assert(store.snapshot().append_slot == capacity);
    assert(!store.record(45.0F, math::Vector3{0.02F, 0.0F, 0.0F}, 1000000U)); // 区满：运行期不擦
    const float newest = store.newest_temperature_celsius();

    estimate::BiasStore reopened;
    assert(reopened.init(storage_of(device), fast_config()));
    assert(reopened.snapshot().compactions == 1U && device.erase_count == 1U);
    assert(reopened.snapshot().records == fast_config().maximum_records);
    assert(close(reopened.newest_temperature_celsius(), newest));
    assert(reopened.snapshot().append_slot == fast_config().maximum_records);
    assert(reopened.record(45.0F, math::Vector3{0.02F, 0.0F, 0.0F}, 1000000U));
}

void interrupted_erase_is_recovered()
{
    FakeStorage device;
    fill_erased(device);
    device.bytes[0] = 0x12U; // 擦到一半：既不是有效记录，也不是擦除态
    device.bytes[1] = 0x34U;
    estimate::BiasStore store;
    assert(store.init(storage_of(device), fast_config()));
    assert(store.snapshot().compactions == 1U && device.erase_count == 1U);
    assert(!store.has_bias() && store.snapshot().append_slot == 0U);
    assert(store.record(42.0F, BIAS_A, 1000000U));
}

void persistence_across_instances()
{
    FakeStorage device;
    fill_erased(device);
    {
        estimate::BiasStore first;
        assert(first.init(storage_of(device), fast_config()));
        assert(first.record(43.5F, BIAS_A, 5000000U));
        assert(first.record(48.0F, math::Vector3{0.012F, -0.011F, 0.004F}, 5000000U));
    }
    estimate::BiasStore second;
    assert(second.init(storage_of(device), fast_config()));
    assert(second.has_bias() && second.snapshot().records == 2U);
    assert(close(second.newest_temperature_celsius(), 48.0F));
    assert(close(second.bias_at(45.75F), math::Vector3{0.011F, -0.010F, 0.0035F}));
    assert(second.snapshot().written == 0U);
}

void rejects_bad_config_and_bad_input()
{
    FakeStorage device;
    fill_erased(device);
    estimate::BiasStore store;
    estimate::BiasStore::Config config = fast_config();
    config.maximum_records = 0U;
    assert(!store.init(storage_of(device), config));
    config = fast_config();
    config.temperature_span_celsius = 0.0F;
    assert(!store.init(storage_of(device), config));
    config = fast_config();
    config.maximum_slope_rad_per_sec_per_celsius = 0.0F;
    assert(!store.init(storage_of(device), config));
    config = fast_config();
    config.minimum_temperature_celsius = 90.0F;
    assert(!store.init(storage_of(device), config));

    assert(store.init(storage_of(device), fast_config()));
    assert(!store.record(std::numeric_limits<float>::quiet_NaN(), BIAS_A, 1000000U));
    assert(!store.record(200.0F, BIAS_A, 1000000U));
    assert(!store.record(40.0F, math::Vector3{std::numeric_limits<float>::infinity(), 0.0F, 0.0F}, 1000000U));
    assert(!store.has_bias());
}

void unavailable_or_failing_device_is_tolerated()
{
    FakeStorage device;
    fill_erased(device);
    device.available = false;
    estimate::BiasStore store;
    assert(store.init(storage_of(device), fast_config()));
    assert(!store.available() && !store.has_bias());
    assert(!store.record(42.0F, BIAS_A, 1000000U));

    device.available = true;
    assert(store.init(storage_of(device), fast_config()));
    assert(store.available());
    device.fail_write = true;
    assert(!store.record(42.0F, BIAS_A, 1000000U));
    assert(!store.available()); // 写失败之后不再尝试
    device.fail_write = false;
    device.fail_erase = true;
    device.bytes[0] = 0x00U; // 逼出一次压缩，压缩的擦除失败
    estimate::BiasStore failing;
    assert(failing.init(storage_of(device), fast_config()));
    assert(!failing.available());
}

} // namespace

int main()
{
    empty_region_has_no_bias();
    one_record_gives_a_constant();
    two_temperatures_give_a_line_and_clamp();
    weights_favour_the_longer_window();
    steep_slope_falls_back_to_constant();
    corrupt_record_stops_the_scan_and_compacts();
    full_region_compacts_at_open();
    interrupted_erase_is_recovered();
    persistence_across_instances();
    rejects_bad_config_and_bad_input();
    unavailable_or_failing_device_is_tolerated();
    return 0;
}
