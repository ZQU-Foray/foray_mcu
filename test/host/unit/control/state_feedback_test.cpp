// control::StateFeedback：手算的 2×1、与朴素矩阵乘法的 6×2 对账、符号、无效 K 与输入、离散双积分器的闭环。
#include "lib/control/state_feedback.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace
{

using control::StateFeedback;

/** 手算：K = [2 3]，x − x_ref = [0.5, −0.25]，u = −(2·0.5 + 3·(−0.25)) = −0.25。 */
void two_by_one()
{
    StateFeedback<2U, 1U> feedback;
    assert(feedback.set_gain(StateFeedback<2U, 1U>::Gain{{{2.0F, 3.0F}}}));
    StateFeedback<2U, 1U>::Input output{};
    assert(
        feedback.compute(StateFeedback<2U, 1U>::State{1.5F, 0.75F}, StateFeedback<2U, 1U>::State{1.0F, 1.0F}, output));
    assert(std::fabs(output[0U] + 0.25F) < 1.0e-6F);
}

/** 6 个状态、2 个输入：与测试里的朴素矩阵乘法对账。 */
void six_by_two()
{
    constexpr std::size_t STATES{6U};
    constexpr std::size_t INPUTS{2U};
    StateFeedback<STATES, INPUTS> feedback;
    StateFeedback<STATES, INPUTS>::Gain gain{};
    // 线性同余的伪随机数，结果可复现
    std::uint32_t seed{2026U};
    const auto next = [&seed]()
    {
        seed = seed * 1664525U + 1013904223U;
        return static_cast<float>(seed >> 8U) / 16777216.0F * 4.0F - 2.0F; // [−2, 2)
    };
    for (std::size_t input = 0U; input < INPUTS; ++input)
    {
        for (std::size_t state = 0U; state < STATES; ++state)
        {
            gain[input][state] = next();
        }
    }
    assert(feedback.set_gain(gain));
    StateFeedback<STATES, INPUTS>::State state{};
    StateFeedback<STATES, INPUTS>::State reference{};
    for (std::size_t index = 0U; index < STATES; ++index)
    {
        state[index] = next();
        reference[index] = next();
    }
    StateFeedback<STATES, INPUTS>::Input output{};
    assert(feedback.compute(state, reference, output));
    for (std::size_t input = 0U; input < INPUTS; ++input)
    {
        float expected = 0.0F;
        for (std::size_t index = 0U; index < STATES; ++index)
        {
            expected += gain[input][index] * (state[index] - reference[index]);
        }
        assert(std::fabs(output[input] + expected) < 1.0e-5F);
    }
}

/** 符号：状态在期望的正方向、K 为正时，输出为负。 */
void sign()
{
    StateFeedback<2U, 1U> feedback;
    assert(feedback.set_gain(StateFeedback<2U, 1U>::Gain{{{2.0F, 3.0F}}}));
    StateFeedback<2U, 1U>::Input output{};
    const StateFeedback<2U, 1U>::State reference{0.0F, 0.0F};
    assert(feedback.compute(StateFeedback<2U, 1U>::State{0.1F, 0.0F}, reference, output));
    assert(output[0U] < 0.0F);
    assert(feedback.compute(StateFeedback<2U, 1U>::State{-0.1F, 0.0F}, reference, output));
    assert(output[0U] > 0.0F);
    assert(feedback.compute(StateFeedback<2U, 1U>::State{0.0F, 0.1F}, reference, output));
    assert(output[0U] < 0.0F);
}

/** 无效的 K：返回 false，原来的 K 不变。 */
void rejects_bad_gain()
{
    StateFeedback<2U, 1U> feedback;
    assert(feedback.set_gain(StateFeedback<2U, 1U>::Gain{{{2.0F, 3.0F}}}));
    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    assert(!feedback.set_gain(StateFeedback<2U, 1U>::Gain{{{nan_value, 3.0F}}}));
    assert(!feedback.set_gain(StateFeedback<2U, 1U>::Gain{{{2.0F, infinity}}}));
    assert(feedback.gain()[0U][0U] == 2.0F);
    assert(feedback.gain()[0U][1U] == 3.0F);
    StateFeedback<2U, 1U>::Input output{};
    assert(feedback.compute(StateFeedback<2U, 1U>::State{0.5F, 0.0F}, StateFeedback<2U, 1U>::State{}, output));
    assert(std::fabs(output[0U] + 1.0F) < 1.0e-6F);
}

/** 没设过 K、输入非有限：返回 false、输出全 0。 */
void rejects_missing_gain_and_bad_input()
{
    StateFeedback<2U, 1U> feedback;
    StateFeedback<2U, 1U>::Input output{};
    output[0U] = 5.0F; // 调用前的值要清掉
    assert(!feedback.compute(StateFeedback<2U, 1U>::State{1.0F, 1.0F}, StateFeedback<2U, 1U>::State{}, output));
    assert(output[0U] == 0.0F);

    assert(feedback.set_gain(StateFeedback<2U, 1U>::Gain{{{2.0F, 3.0F}}}));
    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    for (const float bad : {nan_value, infinity, -infinity})
    {
        output[0U] = 5.0F;
        assert(!feedback.compute(StateFeedback<2U, 1U>::State{bad, 0.0F}, StateFeedback<2U, 1U>::State{}, output));
        assert(output[0U] == 0.0F);
        output[0U] = 5.0F;
        assert(!feedback.compute(StateFeedback<2U, 1U>::State{}, StateFeedback<2U, 1U>::State{0.0F, bad}, output));
        assert(output[0U] == 0.0F);
    }
}

/** 离散双积分器 + 设计表里 ω_n = 40 的 K：从 0.3 rad 出发 2000 拍后回到 0；把 k₁ 改成负数则发散。 */
void closed_loop_double_integrator()
{
    constexpr float DT{0.001F};
    constexpr float ANGLE_GAIN{1555.4F}; // tool/control/lqr_design.py double-integrator --dt 0.001 --q 2.56e6,0 --r 1
    constexpr float RATE_GAIN{55.77F};
    StateFeedback<2U, 1U> feedback;
    assert(feedback.set_gain(StateFeedback<2U, 1U>::Gain{{{ANGLE_GAIN, RATE_GAIN}}}));
    float angle = 0.3F;
    float rate = 0.0F;
    for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
    {
        StateFeedback<2U, 1U>::Input acceleration{};
        assert(
            feedback.compute(StateFeedback<2U, 1U>::State{angle, rate}, StateFeedback<2U, 1U>::State{}, acceleration));
        angle += DT * rate + 0.5F * DT * DT * acceleration[0U];
        rate += DT * acceleration[0U];
    }
    assert(std::fabs(angle) < 1.0e-4F);
    assert(std::fabs(rate) < 1.0e-3F);

    // k₁ 变成负数：闭环不稳定，跑同样的拍数之后早已飞出
    assert(feedback.set_gain(StateFeedback<2U, 1U>::Gain{{{-ANGLE_GAIN, RATE_GAIN}}}));
    angle = 0.3F;
    rate = 0.0F;
    for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
    {
        StateFeedback<2U, 1U>::Input acceleration{};
        assert(
            feedback.compute(StateFeedback<2U, 1U>::State{angle, rate}, StateFeedback<2U, 1U>::State{}, acceleration));
        angle += DT * rate + 0.5F * DT * DT * acceleration[0U];
        rate += DT * acceleration[0U];
    }
    assert(std::fabs(angle) > 1.0F);
}

} // namespace

int main()
{
    two_by_one();
    six_by_two();
    sign();
    rejects_bad_gain();
    rejects_missing_gain_and_bad_input();
    closed_loop_double_integrator();
    return 0;
}
