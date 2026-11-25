// hal::Nvs 的宿主替身：两个 4 KB 的区（各 128 条 32 B 记录，测试里能很快写满），
// 参数检查与 DM-MC02 的 port 一致：只能写在擦除态上、按 32 B 对齐、擦除是整区。
#include "hal/nvs.hpp"
#include "mock_hal.hpp"

#include <array>
#include <cstring>
#include <mutex>

namespace
{

constexpr std::uint8_t ERASED_BYTE{0xFFU};

struct Region
{
    bool ready{true};
    hal::Nvs::Result write_result{hal::Nvs::Result::OK}; // OK：正常写；其他：直接返回它
    hal::Nvs::Result erase_result{hal::Nvs::Result::OK};
    std::size_t write_count{0U};
    std::size_t erase_count{0U};
    std::array<std::uint8_t, mock::nvs::REGION_SIZE> bytes{};
};

std::mutex g_mutex;
std::array<Region, mock::nvs::REGION_COUNT> g_regions{};

Region *region_of(std::uint8_t index) noexcept
{
    return index < g_regions.size() ? &g_regions[index] : nullptr;
}

} // namespace

namespace hal
{

std::size_t Nvs::size() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return region_of(index_) != nullptr ? mock::nvs::REGION_SIZE : 0U;
}

std::size_t Nvs::write_granularity() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    return region_of(index_) != nullptr ? mock::nvs::WRITE_GRANULARITY : 0U;
}

Nvs::Result Nvs::read(std::size_t offset, std::uint8_t *data, std::size_t length) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    const Region *region = region_of(index_);
    if (region == nullptr)
    {
        return Result::UNSUPPORTED;
    }
    if (!region->ready)
    {
        return Result::NOT_READY;
    }
    if (data == nullptr || length == 0U || offset > mock::nvs::REGION_SIZE || length > mock::nvs::REGION_SIZE - offset)
    {
        return Result::INVALID_ARGUMENT;
    }
    std::memcpy(data, region->bytes.data() + offset, length);
    return Result::OK;
}

Nvs::Result Nvs::write(std::size_t offset, const std::uint8_t *data, std::size_t length) const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    Region *region = region_of(index_);
    if (region == nullptr)
    {
        return Result::UNSUPPORTED;
    }
    if (!region->ready)
    {
        return Result::NOT_READY;
    }
    if (data == nullptr || length == 0U || offset > mock::nvs::REGION_SIZE || length > mock::nvs::REGION_SIZE - offset)
    {
        return Result::INVALID_ARGUMENT;
    }
    if ((offset % mock::nvs::WRITE_GRANULARITY) != 0U || (length % mock::nvs::WRITE_GRANULARITY) != 0U)
    {
        return Result::NOT_ALIGNED;
    }
    const hal::Nvs::Result forced = region->write_result;
    if (forced != Result::OK && forced != Result::NOT_ERASED)
    {
        return forced;
    }
    for (std::size_t index = 0U; index < length; ++index)
    {
        if (region->bytes[offset + index] != ERASED_BYTE)
        {
            return Result::NOT_ERASED;
        }
    }
    if (forced == Result::NOT_ERASED)
    {
        return forced;
    }
    std::memcpy(region->bytes.data() + offset, data, length);
    ++region->write_count;
    return Result::OK;
}

Nvs::Result Nvs::erase() const noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    Region *region = region_of(index_);
    if (region == nullptr)
    {
        return Result::UNSUPPORTED;
    }
    if (!region->ready)
    {
        return Result::NOT_READY;
    }
    if (region->erase_result != Result::OK)
    {
        return region->erase_result;
    }
    region->bytes.fill(ERASED_BYTE);
    ++region->erase_count;
    return Result::OK;
}

} // namespace hal

namespace mock::nvs
{

void reset() noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    for (Region &region : g_regions)
    {
        region = Region{};
        region.bytes.fill(ERASED_BYTE);
    }
}

void reset(std::uint8_t index) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    Region *region = region_of(index);
    if (region != nullptr)
    {
        *region = Region{};
        region->bytes.fill(ERASED_BYTE);
    }
}

void set_ready(std::uint8_t index, bool ready) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (Region *region = region_of(index); region != nullptr)
    {
        region->ready = ready;
    }
}

void set_write_result(std::uint8_t index, hal::Nvs::Result result) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (Region *region = region_of(index); region != nullptr)
    {
        region->write_result = result;
    }
}

void set_erase_result(std::uint8_t index, hal::Nvs::Result result) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (Region *region = region_of(index); region != nullptr)
    {
        region->erase_result = result;
    }
}

std::size_t write_count(std::uint8_t index) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    const Region *region = region_of(index);
    return region != nullptr ? region->write_count : 0U;
}

std::size_t erase_count(std::uint8_t index) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    const Region *region = region_of(index);
    return region != nullptr ? region->erase_count : 0U;
}

bool is_erased(std::uint8_t index, std::size_t offset, std::size_t length) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    const Region *region = region_of(index);
    if (region == nullptr || offset > REGION_SIZE || length > REGION_SIZE - offset)
    {
        return false;
    }
    for (std::size_t position = 0U; position < length; ++position)
    {
        if (region->bytes[offset + position] != ERASED_BYTE)
        {
            return false;
        }
    }
    return true;
}

void poke(std::uint8_t index, std::size_t offset, std::uint8_t value) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    if (Region *region = region_of(index); region != nullptr && offset < REGION_SIZE)
    {
        region->bytes[offset] = value;
    }
}

std::uint8_t peek(std::uint8_t index, std::size_t offset) noexcept
{
    const std::lock_guard<std::mutex> lock{g_mutex};
    const Region *region = region_of(index);
    if (region == nullptr || offset >= REGION_SIZE)
    {
        return ERASED_BYTE;
    }
    return region->bytes[offset];
}

} // namespace mock::nvs
