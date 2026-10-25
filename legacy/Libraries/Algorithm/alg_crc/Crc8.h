#ifndef CRC8_H
#define CRC8_H

#include <cstddef>
#include <cstdint>

namespace alg_crc
{

/**
 * @brief 计算 CRC-8 校验值（查表法）
 * @param data 数据缓冲区
 * @param len  数据长度（字节）
 * @return 8 位 CRC 校验值；data 为空且 len 非零时返回 0
 */
uint8_t Crc8Calc(const uint8_t *data, size_t len);

} // namespace alg_crc

#endif
