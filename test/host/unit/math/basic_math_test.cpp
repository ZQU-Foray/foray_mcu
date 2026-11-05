// math::wrap_pi：回绕到 [−π, π)。覆盖 ±π 边界、几圈以外的角度、扫描区间与非有限值。
#include "utils/math/basic_math.hpp"

#include <cassert>
#include <cmath>
#include <limits>

namespace
{

constexpr float TOLERANCE{1.0e-6F};

bool near(float actual, float expected, float tolerance = TOLERANCE)
{
    return std::fabs(actual - expected) <= tolerance;
}

void test_basic()
{
    assert(math::wrap_pi(0.0F) == 0.0F);
    assert(near(math::wrap_pi(0.5F), 0.5F));
    assert(near(math::wrap_pi(-0.5F), -0.5F));
    assert(near(math::wrap_pi(3.0F), 3.0F));
    assert(near(math::wrap_pi(-3.0F), -3.0F));
}

// 区间是 [−π, π)：恰为 π 回绕成 −π，−π 保持不变
void test_boundary()
{
    assert(near(math::wrap_pi(math::PI), -math::PI));
    assert(near(math::wrap_pi(-math::PI), -math::PI));
    assert(near(math::wrap_pi(math::PI - 0.001F), math::PI - 0.001F));
    assert(near(math::wrap_pi(-math::PI + 0.001F), -math::PI + 0.001F));
    // 越过边界一点点就翻到另一侧
    assert(near(math::wrap_pi(math::PI + 0.001F), -math::PI + 0.001F));
    assert(near(math::wrap_pi(-math::PI - 0.001F), math::PI - 0.001F));
}

void test_turns()
{
    constexpr float TWO_PI{2.0F * math::PI};
    assert(near(math::wrap_pi(TWO_PI + 0.3F), 0.3F, 1.0e-4F));
    assert(near(math::wrap_pi(-TWO_PI - 0.3F), -0.3F, 1.0e-4F));
    assert(near(math::wrap_pi(10.0F * math::PI + 0.1F), 0.1F, 1.0e-3F));
    assert(near(math::wrap_pi(-10.0F * math::PI - 0.1F), -0.1F, 1.0e-3F));
    // 2 圈正好回绕到 0
    assert(near(math::wrap_pi(TWO_PI), 0.0F, 1.0e-4F));
    // 1.25 圈：2.5π 与 0.5π 等价
    assert(near(math::wrap_pi(1.25F * TWO_PI), 0.5F * math::PI, 1.0e-3F));
}

// 扫描整个区间：结果都在 [−π, π)，且与输入相差 2π 的整数倍
void test_sweep()
{
    constexpr float STEP{0.01F};
    for (float angle = -20.0F; angle <= 20.0F; angle += STEP)
    {
        const float wrapped = math::wrap_pi(angle);
        assert(wrapped >= -math::PI && wrapped < math::PI);
        const float turns = (angle - wrapped) / (2.0F * math::PI);
        assert(near(turns, std::round(turns), 1.0e-3F));
    }
}

void test_invalid()
{
    assert(math::wrap_pi(std::numeric_limits<float>::quiet_NaN()) == 0.0F);
    assert(math::wrap_pi(std::numeric_limits<float>::infinity()) == 0.0F);
    assert(math::wrap_pi(-std::numeric_limits<float>::infinity()) == 0.0F);
}

} // namespace

int main()
{
    test_basic();
    test_boundary();
    test_turns();
    test_sweep();
    test_invalid();
    return 0;
}
