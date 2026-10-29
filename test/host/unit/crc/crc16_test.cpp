// utils_crc 的 CRC-16/MCRF4XX：标准校验值、边界输入、与逐位实现对照、裁判系统帧尾的小端约定。
#include "utils/crc/crc16.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>

namespace
{

// 逐位实现，与查表法互相独立，用来对照
std::uint16_t crc16_bitwise(const std::uint8_t *data, std::size_t len)
{
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t i = 0; i < len; ++i)
    {
        crc = static_cast<std::uint16_t>(crc ^ data[i]);
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1U) != 0U ? static_cast<std::uint16_t>((crc >> 1) ^ 0x8408U)
                                   : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

void matches_standard_check_value()
{
    // CRC 参数目录中 CRC-16/MCRF4XX 的 check 值：对 ASCII "123456789" 计算
    const std::uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    assert(crc::crc16_calc(text, sizeof(text)) == 0x6F91U);
}

void handles_empty_and_null_input()
{
    const std::uint8_t byte = 0x00U;
    assert(crc::crc16_calc(&byte, 0U) == 0xFFFFU); // 不处理任何字节，返回 init
    assert(crc::crc16_calc(nullptr, 0U) == 0xFFFFU);
    assert(crc::crc16_calc(nullptr, 4U) == 0U);
}

void matches_bitwise_reference()
{
    std::uint8_t data[300] = {};
    std::uint32_t seed = 0x12345678U;
    for (std::uint8_t &value : data)
    {
        seed = seed * 1664525U + 1013904223U; // 线性同余，结果可复现
        value = static_cast<std::uint8_t>(seed >> 24);
    }
    for (std::size_t len = 0; len <= sizeof(data); ++len)
    {
        assert(crc::crc16_calc(data, len) == crc16_bitwise(data, len));
    }
}

void frame_tail_is_little_endian()
{
    // 裁判系统帧：SOF 0xA5、data_length 2 字节、seq、CRC8、cmd_id 2 字节、数据，最后 2 字节是 CRC16。
    // 这里只关心帧尾：校验值低字节在前，连同帧尾一起计算结果为 0。
    std::uint8_t frame[] = {0xA5U, 0x03U, 0x00U, 0x07U, 0x00U, 0x01U, 0x02U, 0x11U, 0x22U, 0x33U, 0x00U, 0x00U};
    constexpr std::size_t BODY_LEN = sizeof(frame) - 2U;
    const std::uint16_t crc = crc::crc16_calc(frame, BODY_LEN);
    frame[BODY_LEN] = static_cast<std::uint8_t>(crc & 0xFFU);
    frame[BODY_LEN + 1U] = static_cast<std::uint8_t>(crc >> 8);
    assert(crc::crc16_calc(frame, sizeof(frame)) == 0U);

    // 改动一位，校验不再为 0
    frame[5] = static_cast<std::uint8_t>(frame[5] ^ 0x10U);
    assert(crc::crc16_calc(frame, sizeof(frame)) != 0U);
}

} // namespace

int main()
{
    matches_standard_check_value();
    handles_empty_and_null_input();
    matches_bitwise_reference();
    frame_tail_is_little_endian();
    return 0;
}
