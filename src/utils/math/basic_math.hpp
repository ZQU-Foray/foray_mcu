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
template <typename Type>
Type limit(Type x, Type min, Type max)
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
template <typename Type>
constexpr Type square(Type x)
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
template <typename Type>
Type limit_in_place(Type &x, Type min, Type max)
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
template <typename Type>
Type abs(Type x)
{
    return ((x > 0) ? x : -x);
}

/**
 * @brief 把回绕在 (−π, π] 的角度累计成连续角度：记圈数，连续值 = 回绕角度 + 圈数 × 2π。
 * @note 与"每拍累加回绕后的增量"不同：那种写法每加一次都舍入一次，一直朝一个方向转会越积越大
 *       （1 rad/s 转 7 分钟能差约 0.19 rad）；这里连续值只由当前的回绕角度与整数圈数算出，
 *       误差不随转动时间增长。
 * @note 前提是相邻两份回绕角度的真实变化不超过半圈（1 kHz 下相当于 3141 rad/s），否则圈数会记反。
 */
class AngleUnwrapper final
{
  public:
    /** 从这份回绕角度重新开始（圈数清零）。复位、进入 HOLD 时用。 */
    void reset(float wrapped_rad) noexcept;

    /** 喂一份新的回绕角度，返回连续角度；第一次调用等价于 reset()；非有限值不改变状态。 */
    [[nodiscard]] float update(float wrapped_rad) noexcept;

    /** 当前的连续角度；还没喂过时为 0。 */
    [[nodiscard]] float continuous_rad() const noexcept
    {
        return continuous_rad_;
    }

  private:
    float previous_rad_{0.0F};
    float continuous_rad_{0.0F};
    std::int32_t turns_{0};
    bool has_value_{false};
};

/**
 * @brief 把角度回绕到 [−π, π)。
 * @param angle 任意角度，单位 rad。
 * @return 回绕后的角度；恰好等于 π 时返回 −π（区间右开）；非有限值返回 0。
 * @note 用于角度误差：目标与测量各自在 (−π, π] 时，误差要回绕，否则会绕远路转一整圈。
 */
[[nodiscard]] float wrap_pi(float angle) noexcept;

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
