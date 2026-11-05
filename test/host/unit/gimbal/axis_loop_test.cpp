// gimbal::AxisLoop：单轴串级控制律，在宿主机上拿一个简单的被控对象闭环跑。
// 被控对象：一个轴 J·ω̇ = τ − b·ω − τc·tanh(ω/ωs) − τg·cos(θ)，力矩晚一个周期生效（模拟 CAN 往返）。
// 摩擦用 tanh 在零速附近平滑，避免离散仿真在零速抖动（真实减速箱是静摩擦 + 库仑摩擦）。
// 验证：符号、阶跃收敛、重力积分与重力前馈、饱和与积分限幅、总限幅时的积分抗饱和、reset、非有限输入。
#include "gimbal/axis_loop.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>

namespace
{

constexpr float DT{0.001F};
constexpr float FRICTION_SPEED{0.01F}; // tanh 的平滑尺度，rad/s

// 被控对象：小云台 4310（J、阻尼取估计值，摩擦与峰值力矩按说明书）
struct Plant
{
    float inertia{0.005F}; // kg·m²
    float damping{0.02F};  // N·m·s/rad
    float friction{0.2F};  // N·m，4310 减速箱实测约 0.2
    float gravity{0.5F};   // N·m
    float angle{0.0F};
    float rate{0.0F};
    float pending_torque{0.0F};

    void advance(float torque, float dt)
    {
        const float applied = pending_torque; // 上一拍的力矩这一拍才生效
        pending_torque = torque;
        const float net =
            applied - damping * rate - friction * std::tanh(rate / FRICTION_SPEED) - gravity * std::cos(angle);
        rate += net / inertia * dt;
        angle += rate * dt;
    }
};

gimbal::AxisLoop::Gains angle_gains()
{
    return gimbal::AxisLoop::Gains{30.0F, 0.0F, 0.0F, 6.0F};
}

gimbal::AxisLoop::Gains rate_gains()
{
    return gimbal::AxisLoop::Gains{0.6F, 3.0F, 1.0F, 1.0F};
}

gimbal::AxisLoop make_loop()
{
    gimbal::AxisLoop loop;
    assert(loop.init(angle_gains(), rate_gains()));
    return loop;
}

bool near(float actual, float expected, float tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

/** 跑一拍：算力矩、加重力前馈、总限幅、告诉控制器实际值、推进被控对象。 */
gimbal::AxisLoop::Output
step(gimbal::AxisLoop &loop, Plant &plant, float target, float torque_limit, float gravity_ff_newton_meter)
{
    const gimbal::AxisLoop::Output output = loop.update(plant.angle, target, plant.rate, 0.0F, DT);
    const float feedforward = gravity_ff_newton_meter * std::cos(plant.angle);
    const float total = std::fmax(std::fmin(output.torque + feedforward, torque_limit), -torque_limit);
    loop.apply(total - feedforward);
    plant.advance(total, DT);
    return output;
}

void test_sign()
{
    gimbal::AxisLoop loop = make_loop();
    // 目标在测量值正方向：力矩为正；反之为负
    assert(loop.update(0.0F, 0.1F, 0.0F, 0.0F, DT).torque > 0.0F);
    loop.reset();
    assert(loop.update(0.0F, -0.1F, 0.0F, 0.0F, DT).torque < 0.0F);
    // 角速度环也在起作用：角度一致、角速度为正时输出为负（阻尼）
    loop.reset();
    assert(loop.update(0.0F, 0.0F, 1.0F, 0.0F, DT).torque < 0.0F);
    // 角速度前馈：目标角速度为正时输出为正
    loop.reset();
    assert(loop.update(0.0F, 0.0F, 0.0F, 1.0F, DT).torque > 0.0F);
}

void test_step_response()
{
    gimbal::AxisLoop loop = make_loop();
    Plant plant;
    plant.gravity = 0.0F;
    constexpr float TARGET{0.3F};
    float peak = -10.0F;
    float angle_at_one_second = 0.0F;
    for (std::size_t cycle = 0U; cycle < 1000U; ++cycle)
    {
        (void)step(loop, plant, TARGET, 1.0F, 0.0F);
        peak = std::fmax(peak, plant.angle);
        if (cycle == 999U)
        {
            angle_at_one_second = plant.angle;
        }
    }
    // 1 s 内收敛，稳态误差小于 0.005 rad，超调有上限
    assert(near(angle_at_one_second, TARGET, 0.005F));
    assert(peak < TARGET + 0.05F);
    assert(peak >= TARGET);
}

void test_gravity()
{
    // 重力前馈给 0：积分把稳态误差消掉，补偿力矩收敛到 τg·cos(θ)
    {
        gimbal::AxisLoop loop = make_loop();
        Plant plant;
        gimbal::AxisLoop::Output output{};
        for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
        {
            output = step(loop, plant, 0.0F, 1.0F, 0.0F);
        }
        assert(near(plant.angle, 0.0F, 0.005F));
        assert(near(output.diagnostics.compensation_newton_meter, plant.gravity, 0.05F));
    }
    // 前馈给对了：补偿力矩回到 0 附近
    {
        gimbal::AxisLoop loop = make_loop();
        Plant plant;
        gimbal::AxisLoop::Output output{};
        for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
        {
            output = step(loop, plant, 0.0F, 1.0F, plant.gravity);
        }
        assert(near(plant.angle, 0.0F, 0.005F));
        assert(near(output.diagnostics.compensation_newton_meter, 0.0F, 0.02F));
    }
}

void test_saturation()
{
    gimbal::AxisLoop loop = make_loop();
    Plant plant;
    plant.gravity = 0.0F;
    constexpr float LIMIT{0.1F};
    for (std::size_t cycle = 0U; cycle < 1000U; ++cycle)
    {
        const gimbal::AxisLoop::Output output = loop.update(plant.angle, 0.3F, plant.rate, 0.0F, DT);
        const float limited = std::fmax(std::fmin(output.torque, LIMIT), -LIMIT);
        loop.apply(limited);
        plant.advance(limited, DT);
        // 下发的力矩不超过总限幅；积分不超过积分限幅
        assert(std::fabs(limited) <= LIMIT + 1.0e-6F);
        assert(std::fabs(output.diagnostics.compensation_newton_meter) <= 1.0F + 1.0e-6F);
    }
}

// 总限幅把力矩削掉时，apply() 让积分不再往饱和方向长：放开限幅后没有积分饱和带来的大超调
void test_total_limit()
{
    gimbal::AxisLoop loop = make_loop();
    Plant plant;
    plant.gravity = 0.0F;
    constexpr float TARGET{0.02F};
    constexpr float LIMITED{0.1F};
    float peak = -10.0F;
    for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
    {
        const float limit = cycle < 1000U ? LIMITED : 1.0F;
        (void)step(loop, plant, TARGET, limit, 0.0F);
        peak = std::fmax(peak, plant.angle);
    }
    assert(peak < TARGET + 0.01F);
    assert(near(plant.angle, TARGET, 0.005F));
}

// 有重力前馈、力矩没被总限幅削时，积分照常增长。前馈是 Gimbal 在外面加的、再减掉，
// 浮点上"输出 + 前馈 − 前馈"不保证逐位等于输出，不能因此把这一拍的积分退回去。
void test_feedforward_does_not_block_integral()
{
    // 恒定误差、不接被控对象：前馈 0 与 0.3 两种情况下，积分应该长得一模一样
    const std::array<float, 2> feedforwards{0.0F, 0.3F};
    std::array<float, 2> compensation{0.0F, 0.0F};
    for (std::size_t index = 0U; index < feedforwards.size(); ++index)
    {
        gimbal::AxisLoop loop = make_loop();
        for (std::size_t cycle = 0U; cycle < 1000U; ++cycle)
        {
            const gimbal::AxisLoop::Output output = loop.update(0.0F, 0.03F, 0.0F, 0.0F, DT);
            // 总限幅给得足够大：力矩没有被削，apply() 收到的是"输出 + 前馈 − 前馈"
            const float total = std::fmax(std::fmin(output.torque + feedforwards[index], 5.0F), -5.0F);
            loop.apply(total - feedforwards[index]);
            compensation[index] = output.diagnostics.compensation_newton_meter;
        }
    }
    assert(compensation[0] > 0.4F);
    assert(near(compensation[1], compensation[0], 0.01F));
}

// 没被削时 apply() 什么都不改
void test_apply_without_clipping()
{
    gimbal::AxisLoop loop = make_loop();
    Plant plant;
    plant.gravity = 0.0F;
    const gimbal::AxisLoop::Output output = loop.update(plant.angle, 0.01F, plant.rate, 0.0F, DT);
    const float compensation = output.diagnostics.compensation_newton_meter;
    loop.apply(output.torque); // 与实际下发的一致
    const gimbal::AxisLoop::Output next = loop.update(plant.angle, 0.01F, plant.rate, 0.0F, DT);
    // 第二拍的积分只增加了这一拍的增量，没有被 apply() 倒退
    assert(next.diagnostics.compensation_newton_meter > compensation);
}

void test_reset()
{
    gimbal::AxisLoop loop = make_loop();
    Plant plant;
    plant.gravity = 0.0F;
    for (std::size_t cycle = 0U; cycle < 100U; ++cycle)
    {
        (void)step(loop, plant, 0.3F, 1.0F, 0.0F);
    }
    loop.reset();
    // reset 之后第一拍：目标与测量一致、角速度为 0，没有残留输出
    const gimbal::AxisLoop::Output output = loop.update(plant.angle, plant.angle, 0.0F, 0.0F, DT);
    assert(output.torque == 0.0F);
    assert(output.diagnostics.compensation_newton_meter == 0.0F);
    assert(output.diagnostics.rate_target == 0.0F);
}

void test_non_finite()
{
    gimbal::AxisLoop loop = make_loop();
    constexpr float NAN_VALUE{std::numeric_limits<float>::quiet_NaN()};
    const std::initializer_list<float> bad_values{NAN_VALUE, std::numeric_limits<float>::infinity()};
    for (const float bad : bad_values)
    {
        assert(loop.update(bad, 0.1F, 0.0F, 0.0F, DT).torque == 0.0F);
        assert(loop.update(0.0F, bad, 0.0F, 0.0F, DT).torque == 0.0F);
        assert(loop.update(0.0F, 0.1F, bad, 0.0F, DT).torque == 0.0F);
        assert(loop.update(0.0F, 0.1F, 0.0F, bad, DT).torque == 0.0F);
        loop.apply(bad); // 不崩、不污染
    }
    // 非有限输入之后照常工作
    const gimbal::AxisLoop::Output output = loop.update(0.0F, 0.1F, 0.0F, 0.0F, DT);
    assert(std::isfinite(output.torque) && output.torque > 0.0F);
    // dt 无效时按 1 ms 算，不产生 NaN
    assert(std::isfinite(loop.update(0.0F, 0.1F, 0.0F, 0.0F, 0.0F).torque));
    assert(std::isfinite(loop.update(0.0F, 0.1F, 0.0F, 0.0F, 5.0F).torque));
}

void test_invalid_gains()
{
    gimbal::AxisLoop loop;
    assert(!loop.init(gimbal::AxisLoop::Gains{-1.0F, 0.0F, 0.0F, 1.0F}, rate_gains()));
    assert(
        !loop.init(angle_gains(), gimbal::AxisLoop::Gains{0.0F, std::numeric_limits<float>::quiet_NaN(), 0.0F, 1.0F}));
    assert(!loop.init(angle_gains(), gimbal::AxisLoop::Gains{0.0F, 0.0F, -1.0F, 1.0F}));
}

} // namespace

int main()
{
    test_sign();
    test_step_response();
    test_gravity();
    test_feedforward_does_not_block_integral();
    test_saturation();
    test_total_limit();
    test_apply_without_clipping();
    test_reset();
    test_non_finite();
    test_invalid_gains();
    return 0;
}
