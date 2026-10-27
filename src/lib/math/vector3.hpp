#ifndef LIB_MATH_VECTOR3_HPP
#define LIB_MATH_VECTOR3_HPP

#include <cmath>

namespace math
{

/**
 * @brief 三轴浮点向量。
 * @note 本类型不约定单位与坐标系，由调用方决定（例如机体系 m/s^2、rad/s、m）。
 *       所有函数都是纯函数：不修改输入以外的任何状态，不做隐藏的归一化或限幅。
 */
struct Vector3 final
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/** @brief 向量点积。 */
[[nodiscard]] inline float dot(const Vector3 &first, const Vector3 &second) noexcept
{
    return first.x * second.x + first.y * second.y + first.z * second.z;
}

/** @brief 向量叉积，结果方向满足右手定则。 */
[[nodiscard]] inline Vector3 cross(const Vector3 &first, const Vector3 &second) noexcept
{
    return Vector3{first.y * second.z - first.z * second.y,
                   first.z * second.x - first.x * second.z,
                   first.x * second.y - first.y * second.x};
}

/** @brief 向量欧几里得范数。 */
[[nodiscard]] inline float norm(const Vector3 &value) noexcept
{
    return std::sqrt(dot(value, value));
}

/**
 * @brief 就地单位化。
 * @param value 输入输出向量。
 * @return 输入各分量有限且范数大于零时返回 true；否则把向量清零并返回 false。
 * @note 失败时清零而不是保留原值，避免调用方在非有限数据上继续计算。
 */
[[nodiscard]] inline bool normalize(Vector3 &value) noexcept
{
    const float norm = math::norm(value);
    if (!std::isfinite(norm) || !(norm > 0.0f))
    {
        value = Vector3{};
        return false;
    }
    const float inverse_norm = 1.0f / norm;
    value.x *= inverse_norm;
    value.y *= inverse_norm;
    value.z *= inverse_norm;
    return true;
}

/** @brief 三个分量都是有限值时返回 true。 */
[[nodiscard]] inline bool is_finite(const Vector3 &value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

} // namespace math

#endif
