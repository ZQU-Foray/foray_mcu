// estimate::Eso：增益对应的极点、常值扰动收敛、斜坡扰动跟得上、b₀ 偏差被 f̂ 吸收、2 阶的速度估计、
// 复位与断档、运行中换参数、dt 抖动、非有限输入、参数检查。
#include "lib/estimate/eso.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace
{

using Eso1 = estimate::Eso<1U>;
using Eso2 = estimate::Eso<2U>;

constexpr float H{0.001F};          // 设计周期 1 ms
constexpr float INPUT_GAIN{200.0F}; // 云台量级：b₀ = 1/J，J = 5 g·m²
constexpr float PI{3.14159265358979F};

float beta_of(float bandwidth)
{
    return std::exp(-bandwidth * H);
}

/** 一阶：Φ = [1 h; 0 1]、C = [1 0]，特征多项式 z² − 迹·z + 行列式。 */
void poles_first_order()
{
    for (const float bandwidth : {50.0F, 150.0F, 400.0F})
    {
        Eso1 eso;
        assert(eso.init(Eso1::Parameters{bandwidth, INPUT_GAIN, H}));
        const float first = eso.gains()[0U];
        const float second = eso.gains()[1U];
        // (I − L·C)·Φ = [1 − l₁, (1 − l₁)h; −l₂, 1 − l₂h]
        const float trace = (1.0F - first) + (1.0F - second * H);
        const float determinant = 1.0F - first;
        const float beta = beta_of(bandwidth);
        assert(std::fabs(trace - 2.0F * beta) < 1.0e-5F);
        assert(std::fabs(determinant - beta * beta) < 1.0e-5F);
    }
}

/** 二阶：3×3 的特征多项式的三个系数等于 (z − β)³ = z³ − 3βz² + 3β²z − β³。 */
void poles_second_order()
{
    for (const float bandwidth : {50.0F, 150.0F, 400.0F})
    {
        Eso2 eso;
        assert(eso.init(Eso2::Parameters{bandwidth, INPUT_GAIN, H}));
        const float first = eso.gains()[0U];
        const float second = eso.gains()[1U];
        const float third = eso.gains()[2U];
        const float half_h_squared = 0.5F * H * H;
        // (I − L·C)·Φ：第一列减掉 L，其余列来自 Φ。
        const float m00 = 1.0F - first;
        const float m01 = m00 * H;
        const float m02 = m00 * half_h_squared;
        const float m10 = -second;
        const float m11 = 1.0F - second * H;
        const float m12 = H - second * half_h_squared;
        const float m20 = -third;
        const float m21 = -third * H;
        const float m22 = 1.0F - third * half_h_squared;
        const float trace = m00 + m11 + m22;
        const float minors = m00 * m11 - m01 * m10 + m00 * m22 - m02 * m20 + m11 * m22 - m12 * m21;
        const float determinant =
            m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20);
        const float beta = beta_of(bandwidth);
        assert(std::fabs(trace - 3.0F * beta) < 1.0e-4F);
        assert(std::fabs(minors - 3.0F * beta * beta) < 1.0e-4F);
        assert(std::fabs(determinant - beta * beta * beta) < 1.0e-4F);
    }
}

/** 常值扰动：被控对象 ẏ = b·u + f，u 是正弦，f = 3；几个时间常数后 f̂ 收敛到 f。 */
void constant_disturbance_converges()
{
    constexpr float BANDWIDTH{150.0F};
    constexpr float DISTURBANCE{3.0F};
    Eso1 eso;
    assert(eso.init(Eso1::Parameters{BANDWIDTH, INPUT_GAIN, H}));
    float rate = 0.0F;
    float input = 0.0F;
    // 两个极点重合，误差按 k·β^k 衰减：5 个时间常数时约 4%，6 个时间常数才进入 2%。
    const std::size_t cycles = static_cast<std::size_t>(6.0F / (BANDWIDTH * H));
    for (std::size_t cycle = 0U; cycle < cycles; ++cycle)
    {
        assert(eso.update(rate, input, H)); // 上一拍到这一拍之间施加的是 input
        input = 2.0F * std::sin(2.0F * PI * 5.0F * static_cast<float>(cycle) * H);
        rate += (INPUT_GAIN * input + DISTURBANCE) * H;
    }
    assert(std::fabs(eso.disturbance() - DISTURBANCE) < 0.02F * DISTURBANCE);
}

/** 斜坡扰动：f 线性增长，f̂ 跟得上、误差有界、不发散。 */
void ramp_disturbance_is_tracked()
{
    constexpr float BANDWIDTH{150.0F};
    constexpr float SLOPE{2.0F}; // rad/s³
    Eso1 eso;
    assert(eso.init(Eso1::Parameters{BANDWIDTH, INPUT_GAIN, H}));
    float rate = 0.0F;
    float input = 0.0F;
    float time = 0.0F;
    for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
    {
        assert(eso.update(rate, input, H));
        input = 0.5F * std::sin(2.0F * PI * 3.0F * time);
        rate += (INPUT_GAIN * input + SLOPE * time) * H;
        time += H;
        assert(std::isfinite(eso.disturbance()));
        // 斜坡的稳态误差约 2·SLOPE/ω₀（这里约 0.03），给一个宽裕的上限防止发散。
        assert(std::fabs(eso.disturbance() - SLOPE * time) < 0.5F);
    }
    assert(std::fabs(eso.disturbance() - SLOPE * time) < 0.1F);
}

/** b₀ 估偏：f̂ 把 (b − b₀)·u 吸收进去，对 y 的估计照样准。 */
void input_gain_error_is_absorbed()
{
    constexpr float PLANT_GAIN{200.0F};
    constexpr float ESTIMATED_GAIN{140.0F}; // 偏 30%
    constexpr float DISTURBANCE{0.5F};
    constexpr float INPUT{0.5F};
    Eso1 eso;
    assert(eso.init(Eso1::Parameters{150.0F, ESTIMATED_GAIN, H}));
    float rate = 0.0F;
    float measurement = 0.0F;
    for (std::size_t cycle = 0U; cycle < 200U; ++cycle)
    {
        measurement = rate;
        assert(eso.update(measurement, INPUT, H));
        rate += (PLANT_GAIN * INPUT + DISTURBANCE) * H;
    }
    // f̂ = f + (b − b₀)·u；y 的估计跟得上测量
    assert(std::fabs(eso.disturbance() - (DISTURBANCE + (PLANT_GAIN - ESTIMATED_GAIN) * INPUT)) < 0.5F);
    assert(std::fabs(eso.state(0U) - measurement) < 1.0e-3F);
}

/** 2 阶：测位置、u 恒定，速度估计收敛。 */
void second_order_estimates_rate()
{
    constexpr float INPUT{2.0F}; // 输入 u；乘上 b₀ 才是角加速度（200 × 2 = 400 rad/s²）
    Eso2 eso;
    assert(eso.init(Eso2::Parameters{150.0F, INPUT_GAIN, H}));
    float angle = 0.0F;
    float rate = 0.0F;
    for (std::size_t cycle = 0U; cycle < 300U; ++cycle)
    {
        assert(eso.update(angle, INPUT, H));
        rate += INPUT_GAIN * INPUT * H;
        angle += rate * H;
    }
    // 上面比的是上一拍的角速度（这一拍的测量已经推进过）
    assert(std::fabs(eso.state(1U) - (rate - INPUT_GAIN * INPUT * H)) < 0.5F);
    assert(std::fabs(eso.state(0U) - (angle - rate * H)) < 0.5F);
    assert(std::fabs(eso.disturbance()) < 0.05F);
}

/** reset() 之后第一次 update()：z₁ = 测量，其余为 0；之后的估计从测量长起来。 */
void reset_uses_measurement()
{
    Eso1 eso;
    assert(eso.init(Eso1::Parameters{150.0F, INPUT_GAIN, H}));
    for (std::size_t cycle = 0U; cycle < 100U; ++cycle)
    {
        assert(eso.update(1.0F, 0.5F, H));
    }
    assert(eso.disturbance() != 0.0F);
    eso.reset();
    assert(eso.state(0U) == 0.0F);
    assert(eso.update(2.5F, 7.0F, H)); // 输入、dt 都不影响第一拍：只用测量初始化
    assert(eso.state(0U) == 2.5F);
    assert(eso.state(1U) == 0.0F);
    assert(eso.disturbance() == 0.0F);

    Eso2 second;
    assert(second.init(Eso2::Parameters{150.0F, INPUT_GAIN, H}));
    assert(second.update(-0.25F, 3.0F, H));
    assert(second.state(0U) == -0.25F);
    assert(second.state(1U) == 0.0F);
    assert(second.disturbance() == 0.0F);
}

/** 断档：dt 超过 10 倍设计周期时对象状态用测量重新初始化，扰动估计保留。 */
void gap_keeps_disturbance()
{
    Eso1 eso;
    assert(eso.init(Eso1::Parameters{150.0F, INPUT_GAIN, H}));
    float rate = 0.0F;
    for (std::size_t cycle = 0U; cycle < 300U; ++cycle)
    {
        assert(eso.update(rate, 0.0F, H));
        rate += 3.0F * H;
    }
    const float disturbance = eso.disturbance();
    assert(std::fabs(disturbance - 3.0F) < 0.1F);
    assert(eso.update(1.5F, 0.0F, 50.0F * H));
    assert(eso.disturbance() == disturbance);
    assert(eso.state(0U) == 1.5F);
}

/** 运行中换参数：f̂ 不跳，新参数接着用。 */
void retune_keeps_state()
{
    Eso1 eso;
    assert(eso.init(Eso1::Parameters{100.0F, INPUT_GAIN, H}));
    float rate = 0.0F;
    for (std::size_t cycle = 0U; cycle < 300U; ++cycle)
    {
        assert(eso.update(rate, 0.0F, H));
        rate += 3.0F * H;
    }
    const float disturbance = eso.disturbance();
    const float gain_before = eso.gains()[0U];
    assert(eso.init(Eso1::Parameters{250.0F, INPUT_GAIN, H}));
    assert(eso.disturbance() == disturbance);
    assert(eso.gains()[0U] != gain_before); // 增益确实换了
    for (std::size_t cycle = 0U; cycle < 300U; ++cycle)
    {
        assert(eso.update(rate, 0.0F, H));
        rate += 3.0F * H;
    }
    assert(std::fabs(eso.disturbance() - 3.0F) < 0.05F);
}

/** dt 在设计周期附近抖动（0.8–1.2 倍），常值扰动照样收敛。 */
void jittered_step_time_converges()
{
    Eso1 eso;
    assert(eso.init(Eso1::Parameters{150.0F, INPUT_GAIN, H}));
    // 线性同余的伪随机数，结果可复现
    std::uint32_t seed{12345U};
    float rate = 0.0F;
    float input = 0.0F;
    float time = 0.0F;
    float dt = H;
    for (std::size_t cycle = 0U; cycle < 1000U; ++cycle)
    {
        assert(eso.update(rate, input, dt)); // 上一段的时长是 dt
        seed = seed * 1664525U + 1013904223U;
        dt = H * (0.8F + 0.4F * static_cast<float>(seed >> 8U) / 16777216.0F);
        input = std::sin(2.0F * PI * 5.0F * time);
        rate += (INPUT_GAIN * input + 3.0F) * dt;
        time += dt;
    }
    assert(std::fabs(eso.disturbance() - 3.0F) < 0.05F);
}

/** 非有限输入：返回 false，状态不动。 */
void rejects_non_finite_inputs()
{
    Eso1 eso;
    assert(eso.init(Eso1::Parameters{150.0F, INPUT_GAIN, H}));
    for (std::size_t cycle = 0U; cycle < 100U; ++cycle)
    {
        assert(eso.update(1.0F, 0.5F, H));
    }
    const float state = eso.state(0U);
    const float disturbance = eso.disturbance();
    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    assert(!eso.update(nan_value, 0.0F, H));
    assert(!eso.update(infinity, 0.0F, H));
    assert(!eso.update(1.0F, nan_value, H));
    assert(!eso.update(1.0F, -infinity, H));
    assert(!eso.update(1.0F, 0.0F, nan_value));
    assert(!eso.update(1.0F, 0.0F, infinity));
    assert(!eso.update(1.0F, 0.0F, 0.0F));
    assert(!eso.update(1.0F, 0.0F, -H));
    assert(eso.state(0U) == state);
    assert(eso.disturbance() == disturbance);
    assert(eso.update(1.0F, 0.0F, H)); // 之后照常工作
}

/** 参数检查：无效参数返回 false，原来的配置不变。 */
void rejects_bad_parameters()
{
    Eso1 eso;
    assert(!eso.update(0.0F, 0.0F, H)); // 没配置
    assert(eso.init(Eso1::Parameters{150.0F, INPUT_GAIN, H}));
    const float gain = eso.gains()[0U];
    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    const Eso1::Parameters bad[] = {
        {0.0F, INPUT_GAIN, H},           // ω₀ 为 0
        {-100.0F, INPUT_GAIN, H},        // ω₀ 为负
        {nan_value, INPUT_GAIN, H},      // ω₀ 非有限
        {infinity, INPUT_GAIN, H},       // ω₀ 无穷
        {150.0F, 0.0F, H},               // b₀ 为 0
        {150.0F, -INPUT_GAIN, H},        // b₀ 为负
        {150.0F, nan_value, H},          // b₀ 非有限
        {150.0F, INPUT_GAIN, 0.0F},      // h 为 0
        {150.0F, INPUT_GAIN, -H},        // h 为负
        {150.0F, INPUT_GAIN, nan_value}, // h 非有限
        {150.0F, INPUT_GAIN, 0.101F},    // h 超过 0.1 s
        {1500.0F, INPUT_GAIN, H},        // ω₀·h = 1.5
    };
    for (const Eso1::Parameters &parameters : bad)
    {
        assert(!eso.init(parameters));
        assert(eso.gains()[0U] == gain);
    }
    assert(eso.update(0.0F, 0.0F, H)); // 配置还在
    // 边界值可以：ω₀·h 正好 1、h 正好 0.1 秒
    assert(eso.init(Eso1::Parameters{1000.0F, INPUT_GAIN, H}));
    assert(eso.init(Eso1::Parameters{10.0F, INPUT_GAIN, 0.1F}));
    assert(eso.state(5U) == 0.0F); // 越界返回 0
}

} // namespace

int main()
{
    poles_first_order();
    poles_second_order();
    constant_disturbance_converges();
    ramp_disturbance_is_tracked();
    input_gain_error_is_absorbed();
    second_order_estimates_rate();
    reset_uses_measurement();
    gap_keeps_disturbance();
    retune_keeps_state();
    jittered_step_time_converges();
    rejects_non_finite_inputs();
    rejects_bad_parameters();
    return 0;
}
