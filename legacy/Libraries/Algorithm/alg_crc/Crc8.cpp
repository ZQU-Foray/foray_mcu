#include "Crc8.h"
#include <array>

namespace alg_crc
{
namespace
{

constexpr uint8_t table_entry(uint8_t value)
{
    for (uint8_t bit = 0; bit < 8; bit++)
    {
        value = (value & 0x80U) ? static_cast<uint8_t>((value << 1) ^ 0x07U) : static_cast<uint8_t>(value << 1);
    }
    return value;
}

constexpr auto TABLE = []
{
    std::array<uint8_t, 256> table{};
    for (uint16_t i = 0; i < 256; i++)
    {
        table[i] = table_entry(static_cast<uint8_t>(i));
    }
    return table;
}();

} // namespace

uint8_t crc8_calc(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00; // init = 0x00
    if (data == nullptr && len != 0U)
    {
        return crc;
    }

    while (len--)
    {
        crc = TABLE[crc ^ *data++];
    }
    return crc; // xorout = 0x00，无需再异或
}

} // namespace alg_crc
