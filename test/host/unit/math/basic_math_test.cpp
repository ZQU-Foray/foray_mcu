// utils/math：wrap_pi（回绕到 [−π, π)）与 AngleUnwrapper（记圈数的连续角度）。
// 连续角度覆盖 ±π 边界、正反两个方向、复位与非法值，以及"一直朝一个方向转几十万拍"的精度。
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

// 连续角度：正、反两个方向跨过 ±π，回到原处时连续值也回到原处
void test_unwrap_crossings()
{
    math::AngleUnwrapper unwrapper;
    assert(near(unwrapper.update(3.0F), 3.0F));
    // 继续往正方向转：回绕值跳到负的一侧，连续值接着走
    assert(near(unwrapper.update(math::wrap_pi(3.5F)), 3.5F, 1.0e-5F));
    assert(near(unwrapper.update(math::wrap_pi(4.0F)), 4.0F, 1.0e-5F));
    // 转回来
    assert(near(unwrapper.update(math::wrap_pi(3.5F)), 3.5F, 1.0e-5F));
    assert(near(unwrapper.update(3.0F), 3.0F, 1.0e-5F));

    // 反方向跨过 −π
    math::AngleUnwrapper negative;
    assert(near(negative.update(-3.0F), -3.0F));
    assert(near(negative.update(math::wrap_pi(-3.5F)), -3.5F, 1.0e-5F));
    assert(near(negative.update(math::wrap_pi(-4.0F)), -4.0F, 1.0e-5F));

    // reset 之后圈数清零
    negative.reset(0.5F);
    assert(near(negative.update(0.6F), 0.6F, 1.0e-5F));
    assert(near(negative.continuous_rad(), 0.6F, 1.0e-5F));
}

/** 一直转 CYCLES 拍，返回连续角度与 double 真值的偏差。rate_of 给第几拍的角速度。 */
template <typename RateOf>
float unwrap_long_run_error(std::size_t cycles, RateOf rate_of)
{
    constexpr float DT{0.001F};
    math::AngleUnwrapper unwrapper;
    double truth = 0.0;
    float continuous = 0.0F;
    for (std::size_t cycle = 0U; cycle < cycles; ++cycle)
    {
        truth += static_cast<double>(rate_of(cycle)) * static_cast<double>(DT);
        continuous = unwrapper.update(math::wrap_pi(static_cast<float>(truth)));
    }
    // 连续值确实走过了很多圈，不是回绕后的值
    assert(std::fabs(continuous) > 3.5F);
    return std::fabs(static_cast<double>(continuous) - truth);
}

// 连续角度：几十万拍、一直朝一个方向转与操作手随便转，和 double 算的真值比。
// 记圈数的写法误差不随转动时间增长（离散累加增量的写法每拍都舍入一次，会越积越大）。
void test_unwrap_long_run()
{
    constexpr std::size_t CYCLES{600000U}; // 600 s，约 95 圈
    // 一直朝一个方向转：1 rad/s
    assert(unwrap_long_run_error(CYCLES, [](std::size_t) { return 1.0F; }) < 1.0e-3F);
    // 操作手随便转：确定性伪随机，−2.5 到 3.5 rad/s（整体往一个方向走，叠加抖动）
    std::uint32_t seed = 12345U;
    auto operator_like = [&seed](std::size_t)
    {
        seed = seed * 1664525U + 1013904223U;
        return 0.5F + 6.0F * (static_cast<float>((seed >> 8U) & 0xFFFFU) / 65535.0F) - 3.0F;
    };
    assert(unwrap_long_run_error(CYCLES, operator_like) < 1.0e-3F);
}

// 连续角度：非有限值不改变状态
void test_unwrap_invalid()
{
    math::AngleUnwrapper unwrapper;
    assert(unwrapper.update(1.0F) == 1.0F);
    assert(unwrapper.update(std::numeric_limits<float>::quiet_NaN()) == 1.0F);
    assert(unwrapper.update(std::numeric_limits<float>::infinity()) == 1.0F);
    assert(near(unwrapper.update(1.1F), 1.1F, 1.0e-6F));
}

} // namespace

int main()
{
    test_basic();
    test_boundary();
    test_turns();
    test_sweep();
    test_invalid();
    test_unwrap_crossings();
    test_unwrap_long_run();
    test_unwrap_invalid();
    return 0;
}
