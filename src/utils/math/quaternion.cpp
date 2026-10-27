#include "lib/math/quaternion.hpp"
#include <cmath>

namespace math
{

float norm(const Quaternion &value) noexcept
{
    return std::sqrt(value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
}

bool normalize(Quaternion &value) noexcept
{
    const float norm = math::norm(value);
    if (!std::isfinite(norm) || !(norm > 0.0f))
    {
        value = Quaternion{};
        return false;
    }
    const float inverse_norm = 1.0f / norm;
    value.w *= inverse_norm;
    value.x *= inverse_norm;
    value.y *= inverse_norm;
    value.z *= inverse_norm;
    return true;
}

Quaternion conjugate(const Quaternion &value) noexcept
{
    return Quaternion{value.w, -value.x, -value.y, -value.z};
}

Quaternion multiply(const Quaternion &first, const Quaternion &second) noexcept
{
    Quaternion result{};
    result.w = first.w * second.w - first.x * second.x - first.y * second.y - first.z * second.z;
    result.x = first.w * second.x + first.x * second.w + first.y * second.z - first.z * second.y;
    result.y = first.w * second.y - first.x * second.z + first.y * second.w + first.z * second.x;
    result.z = first.w * second.z + first.x * second.y - first.y * second.x + first.z * second.w;
    return result;
}

Vector3 rotate(const Quaternion &rotation, const Vector3 &value) noexcept
{
    // 由单位四元数展开的旋转矩阵，与 q ⊗ v ⊗ q* 等价。
    const float xx = rotation.x * rotation.x;
    const float yy = rotation.y * rotation.y;
    const float zz = rotation.z * rotation.z;
    const float xy = rotation.x * rotation.y;
    const float xz = rotation.x * rotation.z;
    const float yz = rotation.y * rotation.z;
    const float wx = rotation.w * rotation.x;
    const float wy = rotation.w * rotation.y;
    const float wz = rotation.w * rotation.z;

    Vector3 result{};
    result.x = (1.0f - 2.0f * (yy + zz)) * value.x + 2.0f * (xy - wz) * value.y + 2.0f * (xz + wy) * value.z;
    result.y = 2.0f * (xy + wz) * value.x + (1.0f - 2.0f * (xx + zz)) * value.y + 2.0f * (yz - wx) * value.z;
    result.z = 2.0f * (xz - wy) * value.x + 2.0f * (yz + wx) * value.y + (1.0f - 2.0f * (xx + yy)) * value.z;
    return result;
}

Vector3 rotate_inverse(const Quaternion &rotation, const Vector3 &value) noexcept
{
    // 上式的转置：把交叉项的符号取反。
    const float xx = rotation.x * rotation.x;
    const float yy = rotation.y * rotation.y;
    const float zz = rotation.z * rotation.z;
    const float xy = rotation.x * rotation.y;
    const float xz = rotation.x * rotation.z;
    const float yz = rotation.y * rotation.z;
    const float wx = rotation.w * rotation.x;
    const float wy = rotation.w * rotation.y;
    const float wz = rotation.w * rotation.z;

    Vector3 result{};
    result.x = (1.0f - 2.0f * (yy + zz)) * value.x + 2.0f * (xy + wz) * value.y + 2.0f * (xz - wy) * value.z;
    result.y = 2.0f * (xy - wz) * value.x + (1.0f - 2.0f * (xx + zz)) * value.y + 2.0f * (yz + wx) * value.z;
    result.z = 2.0f * (xz + wy) * value.x + 2.0f * (yz - wx) * value.y + (1.0f - 2.0f * (xx + yy)) * value.z;
    return result;
}

Vector3 to_euler_zyx_radians(const Quaternion &value) noexcept
{
    // R = Rz(yaw)·Ry(pitch)·Rx(roll) 下的标准提取：
    //   yaw   = atan2(R21, R11)
    //   pitch = asin(R31)，用 −2(xz − wy) 表达，即绕 y 轴的转角
    //   roll  = atan2(R32, R33)，即绕 x 轴的转角
    const float yaw = std::atan2(2.0f * (value.w * value.z + value.x * value.y),
                                 2.0f * (value.w * value.w + value.x * value.x) - 1.0f);
    float pitch_sine = 2.0f * (value.w * value.y - value.x * value.z);
    if (pitch_sine > 1.0f)
    {
        pitch_sine = 1.0f;
    }
    else if (pitch_sine < -1.0f)
    {
        pitch_sine = -1.0f;
    }
    const float roll = std::atan2(2.0f * (value.w * value.x + value.y * value.z),
                                  2.0f * (value.w * value.w + value.z * value.z) - 1.0f);
    return Vector3{yaw, std::asin(pitch_sine), roll};
}

Quaternion from_euler_zyx_radians(const Vector3 &euler) noexcept
{
    const float half_yaw = 0.5f * euler.x;
    const float half_pitch = 0.5f * euler.y;
    const float half_roll = 0.5f * euler.z;
    const float cos_yaw = std::cos(half_yaw);
    const float sin_yaw = std::sin(half_yaw);
    const float cos_pitch = std::cos(half_pitch);
    const float sin_pitch = std::sin(half_pitch);
    const float cos_roll = std::cos(half_roll);
    const float sin_roll = std::sin(half_roll);

    // q = qz ⊗ qy ⊗ qx，与 R = Rz(yaw)·Ry(pitch)·Rx(roll) 一致。
    Quaternion result{};
    result.w = cos_yaw * cos_pitch * cos_roll + sin_yaw * sin_pitch * sin_roll;
    result.x = cos_yaw * cos_pitch * sin_roll - sin_yaw * sin_pitch * cos_roll;
    result.y = cos_yaw * sin_pitch * cos_roll + sin_yaw * cos_pitch * sin_roll;
    result.z = sin_yaw * cos_pitch * cos_roll - cos_yaw * sin_pitch * sin_roll;
    return result;
}

bool is_finite(const Quaternion &value) noexcept
{
    return std::isfinite(value.w) && std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

} // namespace math
