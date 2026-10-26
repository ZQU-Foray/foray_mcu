#include "Libraries/Algorithm/alg_math/Quaternion.h"
#include <cmath>

namespace alg_math
{

float Norm(const Quaternion &value) noexcept
{
    return std::sqrt(value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
}

bool Normalize(Quaternion &value) noexcept
{
    const float norm = Norm(value);
    if (!std::isfinite(norm) || !(norm > 0.0f))
    {
        value = Quaternion{};
        return false;
    }
    const float inverseNorm = 1.0f / norm;
    value.w *= inverseNorm;
    value.x *= inverseNorm;
    value.y *= inverseNorm;
    value.z *= inverseNorm;
    return true;
}

Quaternion Conjugate(const Quaternion &value) noexcept
{
    return Quaternion{value.w, -value.x, -value.y, -value.z};
}

Quaternion Multiply(const Quaternion &first, const Quaternion &second) noexcept
{
    Quaternion result{};
    result.w = first.w * second.w - first.x * second.x - first.y * second.y - first.z * second.z;
    result.x = first.w * second.x + first.x * second.w + first.y * second.z - first.z * second.y;
    result.y = first.w * second.y - first.x * second.z + first.y * second.w + first.z * second.x;
    result.z = first.w * second.z + first.x * second.y - first.y * second.x + first.z * second.w;
    return result;
}

Vector3 Rotate(const Quaternion &rotation, const Vector3 &value) noexcept
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

Vector3 RotateInverse(const Quaternion &rotation, const Vector3 &value) noexcept
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

Vector3 ToEulerZyxRadians(const Quaternion &value) noexcept
{
    // R = Rz(yaw)·Ry(pitch)·Rx(roll) 下的标准提取：
    //   yaw   = atan2(R21, R11)
    //   pitch = asin(R31)，用 −2(xz − wy) 表达，即绕 y 轴的转角
    //   roll  = atan2(R32, R33)，即绕 x 轴的转角
    const float yaw = std::atan2(2.0f * (value.w * value.z + value.x * value.y),
                                 2.0f * (value.w * value.w + value.x * value.x) - 1.0f);
    float pitchSine = 2.0f * (value.w * value.y - value.x * value.z);
    if (pitchSine > 1.0f)
    {
        pitchSine = 1.0f;
    }
    else if (pitchSine < -1.0f)
    {
        pitchSine = -1.0f;
    }
    const float roll = std::atan2(2.0f * (value.w * value.x + value.y * value.z),
                                  2.0f * (value.w * value.w + value.z * value.z) - 1.0f);
    return Vector3{yaw, std::asin(pitchSine), roll};
}

Quaternion FromEulerZyxRadians(const Vector3 &euler) noexcept
{
    const float halfYaw = 0.5f * euler.x;
    const float halfPitch = 0.5f * euler.y;
    const float halfRoll = 0.5f * euler.z;
    const float cosYaw = std::cos(halfYaw);
    const float sinYaw = std::sin(halfYaw);
    const float cosPitch = std::cos(halfPitch);
    const float sinPitch = std::sin(halfPitch);
    const float cosRoll = std::cos(halfRoll);
    const float sinRoll = std::sin(halfRoll);

    // q = qz ⊗ qy ⊗ qx，与 R = Rz(yaw)·Ry(pitch)·Rx(roll) 一致。
    Quaternion result{};
    result.w = cosYaw * cosPitch * cosRoll + sinYaw * sinPitch * sinRoll;
    result.x = cosYaw * cosPitch * sinRoll - sinYaw * sinPitch * cosRoll;
    result.y = cosYaw * sinPitch * cosRoll + sinYaw * cosPitch * sinRoll;
    result.z = sinYaw * cosPitch * cosRoll - cosYaw * sinPitch * sinRoll;
    return result;
}

bool IsFinite(const Quaternion &value) noexcept
{
    return std::isfinite(value.w) && std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

} // namespace alg_math
