#ifndef UTILS_CRC_CRC16_HPP
#define UTILS_CRC_CRC16_HPP

#include <cstddef>
#include <cstdint>

namespace crc
{

/**
 * @brief 计算 CRC-16/MCRF4XX 校验值（查表法，反射多项式 0x8408，init 0xFFFF，xorout 0x0000）
 *
 * RoboMaster 裁判系统串口协议的帧尾校验用这一种。帧尾按小端存放（低字节在前）；
 * 把帧尾两个字节一起算进去，结果为 0。
 *
 * @param data 数据缓冲区
 * @param len  数据长度（字节）
 * @return 16 位 CRC 校验值；len 为 0 时返回 0xFFFF（init）；data 为空且 len 非零时返回 0
 */
uint16_t crc16_calc(const uint8_t *data, size_t len);

} // namespace crc

#endif
