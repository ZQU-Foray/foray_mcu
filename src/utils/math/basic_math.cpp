#include "utils/math/basic_math.hpp"

namespace math
{

float wrap_pi(float angle) noexcept
{
    if (!std::isfinite(angle))
    {
        return 0.0F;
    }
    // 先减 π 再用 fmod 取到 [0, 2π)，避免负数取模的分支
    float wrapped = std::fmod(angle + PI, 2.0F * PI);
    if (wrapped < 0.0F)
    {
        wrapped += 2.0F * PI;
    }
    // fmod 在输入已经是 2π 的整数倍时返回 0：结果落在 [−π, π)
    return wrapped - PI;
}

void AngleUnwrapper::reset(float wrapped_rad) noexcept
{
    previous_rad_ = std::isfinite(wrapped_rad) ? wrapped_rad : 0.0F;
    continuous_rad_ = previous_rad_;
    turns_ = 0;
    has_value_ = true;
}

float AngleUnwrapper::update(float wrapped_rad) noexcept
{
    if (!std::isfinite(wrapped_rad))
    {
        return continuous_rad_; // 坏值不参与累计
    }
    if (!has_value_)
    {
        reset(wrapped_rad);
        return continuous_rad_;
    }
    const float delta = wrapped_rad - previous_rad_;
    if (delta > PI)
    {
        --turns_; // 往正方向转过了 +π，回绕值跳到负的一侧
    }
    else if (delta < -PI)
    {
        ++turns_;
    }
    previous_rad_ = wrapped_rad;
    continuous_rad_ = wrapped_rad + 2.0F * PI * static_cast<float>(turns_);
    return continuous_rad_;
}

} // namespace math
