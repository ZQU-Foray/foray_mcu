#ifndef CRC32_H
#define CRC32_H

#include <cstddef>
#include <cstdint>

namespace alg_crc
{

/**
 * @brief 计算 CRC-32/ISO-HDLC 校验值（查表法，反射多项式 0xEDB88320）
 * @param data 数据缓冲区
 * @param len  数据长度（字节）
 * @return 32 位 CRC 校验值；data 为空且 len 非零时返回 0
 */
uint32_t Crc32Calc(const uint8_t *data, size_t len);

} // namespace alg_crc

#endif
