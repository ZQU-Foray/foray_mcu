#include "utils/crc/crc16.hpp"
#include <array>

namespace crc
{
namespace
{

constexpr uint16_t table_entry(uint16_t value)
{
    for (uint8_t bit = 0; bit < 8; bit++)
    {
        value = (value & 1U) ? static_cast<uint16_t>((value >> 1) ^ 0x8408U) : static_cast<uint16_t>(value >> 1);
    }
    return value;
}

constexpr auto TABLE = []
{
    std::array<uint16_t, 256> table{};
    for (uint16_t i = 0; i < 256; i++)
    {
        table[i] = table_entry(i);
    }
    return table;
}();

// 与裁判系统协议附录中的 wCRC_Table 对照前几项
static_assert(TABLE[1] == 0x1189U && TABLE[2] == 0x2312U && TABLE[255] == 0x0F78U, "CRC-16 表与协议不一致");

} // namespace

uint16_t crc16_calc(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU; // init = 0xFFFF
    if (data == nullptr && len != 0U)
    {
        return 0U;
    }

    while (len--)
    {
        crc = static_cast<uint16_t>(TABLE[(crc ^ *data++) & 0xFFU] ^ (crc >> 8));
    }
    return crc; // xorout = 0x0000，无需再异或
}

} // namespace crc
