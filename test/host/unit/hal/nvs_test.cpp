// hal::Nvs 在 platform/mock 上的契约语义：大小与粒度、读写、只能写在擦除态上、
// 对齐、越界、可用性与强制失败。
#include "hal/nvs.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"

#include <cassert>
#include <cstdint>

namespace
{

constexpr std::uint8_t REGION{0U};
constexpr std::size_t GRANULARITY{mock::nvs::WRITE_GRANULARITY};

void erased_region_reads_0xff()
{
    mock::nvs::reset();
    const hal::Nvs nvs{REGION};
    assert(nvs.size() == mock::nvs::REGION_SIZE);
    assert(nvs.write_granularity() == GRANULARITY);
    std::uint8_t bytes[GRANULARITY]{};
    assert(nvs.read(0U, bytes, sizeof(bytes)) == hal::Nvs::Result::OK);
    for (const std::uint8_t byte : bytes)
    {
        assert(byte == 0xFFU);
    }
}

void write_requires_alignment_and_erased_area()
{
    mock::nvs::reset();
    const hal::Nvs nvs{REGION};
    std::uint8_t data[2U * GRANULARITY]{};
    for (std::size_t index = 0U; index < sizeof(data); ++index)
    {
        data[index] = static_cast<std::uint8_t>(index + 1U);
    }
    // 对齐不符
    assert(nvs.write(1U, data, GRANULARITY) == hal::Nvs::Result::NOT_ALIGNED);
    assert(nvs.write(0U, data, GRANULARITY - 1U) == hal::Nvs::Result::NOT_ALIGNED);
    // 正常写
    assert(nvs.write(0U, data, GRANULARITY) == hal::Nvs::Result::OK);
    std::uint8_t readback[GRANULARITY]{};
    assert(nvs.read(0U, readback, sizeof(readback)) == hal::Nvs::Result::OK);
    for (std::size_t index = 0U; index < sizeof(readback); ++index)
    {
        assert(readback[index] == data[index]);
    }
    // 同一位置再写：不是擦除态
    assert(nvs.write(0U, data, GRANULARITY) == hal::Nvs::Result::NOT_ERASED);
    // 第二块还是擦除态：能写
    assert(nvs.write(GRANULARITY, data + GRANULARITY, GRANULARITY) == hal::Nvs::Result::OK);
    assert(mock::nvs::write_count(REGION) == 2U);
}

void bounds_and_arguments_are_checked()
{
    mock::nvs::reset();
    const hal::Nvs nvs{REGION};
    std::uint8_t data[GRANULARITY]{};
    assert(nvs.read(mock::nvs::REGION_SIZE, data, 1U) == hal::Nvs::Result::INVALID_ARGUMENT);
    assert(nvs.read(mock::nvs::REGION_SIZE - 1U, data, 2U) == hal::Nvs::Result::INVALID_ARGUMENT);
    assert(nvs.read(0U, nullptr, 1U) == hal::Nvs::Result::INVALID_ARGUMENT);
    assert(nvs.read(0U, data, 0U) == hal::Nvs::Result::INVALID_ARGUMENT);
    assert(nvs.write(mock::nvs::REGION_SIZE - GRANULARITY, data, 2U * GRANULARITY) ==
           hal::Nvs::Result::INVALID_ARGUMENT);
    assert(nvs.write(0U, nullptr, GRANULARITY) == hal::Nvs::Result::INVALID_ARGUMENT);
    // 编号越界
    const hal::Nvs bad{mock::nvs::REGION_COUNT};
    assert(bad.size() == 0U && bad.write_granularity() == 0U);
    assert(bad.read(0U, data, 1U) == hal::Nvs::Result::UNSUPPORTED);
    assert(bad.write(0U, data, GRANULARITY) == hal::Nvs::Result::UNSUPPORTED);
    assert(bad.erase() == hal::Nvs::Result::UNSUPPORTED);
}

void erase_clears_the_region_and_failures_are_reported()
{
    mock::nvs::reset();
    const hal::Nvs nvs{REGION};
    std::uint8_t data[GRANULARITY]{};
    assert(nvs.write(0U, data, GRANULARITY) == hal::Nvs::Result::OK);
    assert(!mock::nvs::is_erased(REGION, 0U, GRANULARITY));
    assert(nvs.erase() == hal::Nvs::Result::OK);
    assert(mock::nvs::is_erased(REGION, 0U, mock::nvs::REGION_SIZE));
    // 擦完之后能重新写
    assert(nvs.write(0U, data, GRANULARITY) == hal::Nvs::Result::OK);
    // 强制失败
    mock::nvs::set_write_result(REGION, hal::Nvs::Result::ERROR);
    assert(nvs.write(GRANULARITY, data, GRANULARITY) == hal::Nvs::Result::ERROR);
    mock::nvs::set_erase_result(REGION, hal::Nvs::Result::ERROR);
    assert(nvs.erase() == hal::Nvs::Result::ERROR);
    // 不可用
    mock::nvs::set_ready(REGION, false);
    assert(nvs.read(0U, data, 1U) == hal::Nvs::Result::NOT_READY);
    assert(nvs.write(GRANULARITY, data, GRANULARITY) == hal::Nvs::Result::NOT_READY);
    assert(nvs.erase() == hal::Nvs::Result::NOT_READY);
}

} // namespace

int main()
{
    mock::reset();
    erased_region_reads_0xff();
    write_requires_alignment_and_erased_area();
    bounds_and_arguments_are_checked();
    erase_clears_the_region_and_failures_are_reported();
    return 0;
}
