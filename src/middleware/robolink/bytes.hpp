#ifndef MIDDLEWARE_ROBOLINK_BYTES_HPP
#define MIDDLEWARE_ROBOLINK_BYTES_HPP

#include <cstdint>
#include <cstring>
#include <limits>

// RoboLink 的小端读写（§0：全部小端，禁止结构体直传）。只给 middleware/robolink 内部用。

namespace robolink::bytes
{

static_assert(sizeof(float) == 4U && std::numeric_limits<float>::is_iec559, "RoboLink 需要 IEEE 754 单精度浮点");

inline void put_u16(std::uint8_t *out, std::uint16_t value) noexcept
{
    out[0] = static_cast<std::uint8_t>(value);
    out[1] = static_cast<std::uint8_t>(value >> 8U);
}

inline void put_u32(std::uint8_t *out, std::uint32_t value) noexcept
{
    out[0] = static_cast<std::uint8_t>(value);
    out[1] = static_cast<std::uint8_t>(value >> 8U);
    out[2] = static_cast<std::uint8_t>(value >> 16U);
    out[3] = static_cast<std::uint8_t>(value >> 24U);
}

inline void put_u64(std::uint8_t *out, std::uint64_t value) noexcept
{
    put_u32(out, static_cast<std::uint32_t>(value));
    put_u32(out + 4, static_cast<std::uint32_t>(value >> 32U));
}

inline void put_f32(std::uint8_t *out, float value) noexcept
{
    std::uint32_t bits{0U};
    std::memcpy(&bits, &value, sizeof(bits));
    put_u32(out, bits);
}

[[nodiscard]] inline std::uint16_t get_u16(const std::uint8_t *in) noexcept
{
    return static_cast<std::uint16_t>(in[0] | (in[1] << 8U));
}

[[nodiscard]] inline std::uint32_t get_u32(const std::uint8_t *in) noexcept
{
    return static_cast<std::uint32_t>(in[0]) | (static_cast<std::uint32_t>(in[1]) << 8U) |
           (static_cast<std::uint32_t>(in[2]) << 16U) | (static_cast<std::uint32_t>(in[3]) << 24U);
}

[[nodiscard]] inline std::uint64_t get_u64(const std::uint8_t *in) noexcept
{
    return static_cast<std::uint64_t>(get_u32(in)) | (static_cast<std::uint64_t>(get_u32(in + 4)) << 32U);
}

[[nodiscard]] inline float get_f32(const std::uint8_t *in) noexcept
{
    const std::uint32_t bits = get_u32(in);
    float value{0.0F};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

} // namespace robolink::bytes

#endif // MIDDLEWARE_ROBOLINK_BYTES_HPP
