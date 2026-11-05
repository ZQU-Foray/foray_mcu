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

} // namespace math
