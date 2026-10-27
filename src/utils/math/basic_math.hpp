#ifndef UTILS_MATH_BASIC_MATH_HPP
#define UTILS_MATH_BASIC_MATH_HPP

#include <cmath>
#include <cstdint>
#include <cstring>

namespace math
{

constexpr float PI = 3.14159265358979323846f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float DEG_TO_RAD = PI / 180.0f;

constexpr float rad_to_deg(float rad)
{
    return rad * RAD_TO_DEG;
}

constexpr float deg_to_rad(float deg)
{
    return deg * DEG_TO_RAD;
}

/**
 * @brief 限幅函数（值语义）
 * @tparam Type 类型
 * @param x 传入数据
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的输出值
 */
template <typename Type> Type limit(Type x, Type min, Type max)
{
    if (x < min)
    {
        x = min;
    }
    else if (x > max)
    {
        x = max;
    }
    return (x);
}

/**
 * @brief 平方（模板保留入参类型，避免每处手写 x*x 与隐式提升不一致）
 * @tparam Type 类型
 * @param x 传入数据
 * @return x 的平方
 */
template <typename Type> constexpr Type square(Type x)
{
    return x * x;
}

/**
 * @brief 限幅函数（引用语义，直接修改传入变量）
 * @tparam Type 类型
 * @param x 传入数据引用
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的输出值
 */
template <typename Type> Type limit_in_place(Type &x, Type min, Type max)
{
    if (x < min)
    {
        x = min;
    }
    else if (x > max)
    {
        x = max;
    }
    return x;
}

/**
 * @brief 求绝对值
 * @tparam Type 类型
 * @param x 传入数据
 * @return Type x的绝对值
 */
template <typename Type> Type abs(Type x)
{
    return ((x > 0) ? x : -x);
}

/**
 * @brief 快速反平方根（Quake III 魔数算法）
 * @param[in] x 输入值，必须为有限正数
 * @return 1/sqrt(x) 的近似值；输入无效时返回 0
 * @note 用 memcpy 做位转换，规避 C++ 严格别名规则的未定义行为；
 *       相对误差约 1e-4 量级，精度低于 1/sqrtf 但速度更快
 */
inline float inv_sqrt(float x)
{
    if (!std::isfinite(x) || x <= 0.0f)
    {
        return 0.0f;
    }

    float halfx = 0.5f * x;
    float y = x;
    uint32_t i;
    std::memcpy(&i, &y, sizeof(i));
    i = 0x5f3759dfu - (i >> 1);
    std::memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

} // namespace math

#endif
