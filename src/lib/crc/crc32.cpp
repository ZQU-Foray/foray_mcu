#include "lib/alg_crc/crc32.hpp"
#include <array>

namespace alg_crc
{
namespace
{

constexpr uint32_t table_entry(uint32_t value)
{
    for (uint8_t bit = 0; bit < 8; bit++)
    {
        value = (value & 1U) ? static_cast<uint32_t>((value >> 1) ^ 0xEDB88320U) : static_cast<uint32_t>(value >> 1);
    }
    return value;
}

constexpr auto TABLE = []
{
    std::array<uint32_t, 256> table{};
    for (uint16_t i = 0; i < 256; i++)
    {
        table[i] = table_entry(static_cast<uint32_t>(i));
    }
    return table;
}();

} // namespace

uint32_t crc32_calc(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU; // init = 0xFFFFFFFF
    if (data == nullptr && len != 0U)
    {
        return 0U;
    }

    while (len--)
    {
        crc = TABLE[(crc ^ *data++) & 0xFFU] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFU; // xorout = 0xFFFFFFFF
}

} // namespace alg_crc
