#ifndef LIB_CRC_CRC8_HPP
#define LIB_CRC_CRC8_HPP

#include <cstddef>
#include <cstdint>

namespace crc
{

/**
 * @brief 计算 CRC-8 校验值（查表法）
 * @param data 数据缓冲区
 * @param len  数据长度（字节）
 * @return 8 位 CRC 校验值；data 为空且 len 非零时返回 0
 */
uint8_t crc8_calc(const uint8_t *data, size_t len);

} // namespace crc

#endif
