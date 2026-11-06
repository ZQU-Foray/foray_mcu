// gimbal::AxisLoop（PID 版单轴串级）：先跑一遍控制律无关的闭环场景，再跑 PID 特有的场景。
// 通用场景在 axis_law_scenarios.hpp（被控对象、阶跃、重力、总限幅、apply() 的中性、斜坡、reset、
// 非有限输入、跨 ±π 连续）；换控制律时只改本文件的 make_controller() 与下面的特有场景。
#include "gimbal/axis_loop.hpp"

#include "unit/gimbal/axis_law_scenarios.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using axis_law_test::DT;
using axis_law_test::near;
using axis_law_test::Plant;
using axis_law_test::step;

gimbal::AxisLoop::Gains angle_gains()
{
    return gimbal::AxisLoop::Gains{30.0F, 0.0F, 0.0F, 6.0F};
}

gimbal::AxisLoop::Gains rate_gains()
{
    return gimbal::AxisLoop::Gains{0.6F, 3.0F, 1.0F, 1.0F};
}

/** 造一个 PID 版单轴控制律（generic 场景要求造好即 init 过）。 */
gimbal::AxisLoop make_controller()
{
    gimbal::AxisLoop loop;
    assert(loop.init(gimbal::AxisLoop::Parameters{angle_gains(), rate_gains()}));
    return loop;
}

// PID 特有：参数检查
void test_gains()
{
    gimbal::AxisLoop loop;
    assert(!loop.init(gimbal::AxisLoop::Parameters{gimbal::AxisLoop::Gains{-1.0F, 0.0F, 0.0F, 1.0F}, rate_gains()}));
    assert(!loop.init(gimbal::AxisLoop::Parameters{
        angle_gains(), gimbal::AxisLoop::Gains{0.0F, std::numeric_limits<float>::quiet_NaN(), 0.0F, 1.0F}}));
    assert(!loop.init(gimbal::AxisLoop::Parameters{angle_gains(), gimbal::AxisLoop::Gains{0.0F, 0.0F, -1.0F, 1.0F}}));
    assert(loop.init(gimbal::AxisLoop::Parameters{angle_gains(), rate_gains()}));
}

// PID 特有：积分不超过积分限幅；饱和时力矩不超过角度环的输出上限
void test_integral_limit()
{
    gimbal::AxisLoop loop = make_controller();
    Plant plant;
    plant.gravity = 0.0F;
    constexpr float LIMIT{0.1F};
    for (std::size_t cycle = 0U; cycle < 1000U; ++cycle)
    {
        const gimbal::AxisLoop::Output output = loop.update(
            gimbal::AxisLoop::Measurement{plant.angle, plant.rate, 0.0F}, gimbal::AxisLoop::Reference{0.3F, 0.0F}, DT);
        const float limited = std::fmax(std::fmin(output.torque, LIMIT), -LIMIT);
        loop.apply(limited);
        plant.advance(limited, DT);
        assert(std::fabs(limited) <= LIMIT + 1.0e-6F);
        assert(std::fabs(output.diagnostics.compensation_newton_meter) <= 1.0F + 1.0e-6F);
    }
}

// PID 特有：rate_target 诊断（角度环输出加前馈，再按角度环的输出上限钳位）
void test_rate_target_diagnostic()
{
    gimbal::AxisLoop loop = make_controller();
    const gimbal::AxisLoop::Measurement measurement{0.0F, 0.0F, 0.0F};
    // 角度误差 0.1 rad、角度环 kp = 30：给定 3 rad/s，前馈再加 1 rad/s
    const gimbal::AxisLoop::Output output = loop.update(measurement, {0.1F, 1.0F}, DT);
    assert(near(output.diagnostics.rate_target, 4.0F, 1.0e-3F));
    // 超过角度环的输出上限（6 rad/s）时钳住
    loop.reset();
    const gimbal::AxisLoop::Output clamped = loop.update(measurement, {0.5F, 1.0F}, DT);
    assert(near(clamped.diagnostics.rate_target, 6.0F, 1.0e-3F));
}

// PID 特有：retune() 不清积分；无效参数返回 false 且行为不变；没 init() 时返回 false
void test_retune()
{
    gimbal::AxisLoop loop = make_controller();
    Plant plant; // 重力 0.5 N·m、不加前馈：积分托着它
    for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
    {
        (void)step(loop, plant, 0.0F, 1.0F, 0.0F);
    }
    // 零误差调用一次：积分不再变化，记下来
    gimbal::AxisLoop::Output output = loop.update(gimbal::AxisLoop::Measurement{plant.angle, plant.rate, 0.0F},
                                                  gimbal::AxisLoop::Reference{plant.angle, plant.rate},
                                                  DT);
    const float compensation = output.diagnostics.compensation_newton_meter;
    assert(std::fabs(compensation) > 0.1F);

    // 换一组参数（角速度环 kp 0.6 → 0.8），积分不清零
    gimbal::AxisLoop::Gains rate = rate_gains();
    rate.kp = 0.8F;
    assert(loop.retune(gimbal::AxisLoop::Parameters{angle_gains(), rate}));
    output = loop.update(gimbal::AxisLoop::Measurement{plant.angle, plant.rate, 0.0F},
                         gimbal::AxisLoop::Reference{plant.angle, plant.rate},
                         DT);
    assert(near(output.diagnostics.compensation_newton_meter, compensation, 1.0e-6F));

    // 无效参数：返回 false，行为不变（与没调过的副本逐拍相同）
    gimbal::AxisLoop reference = loop;
    assert(!loop.retune(gimbal::AxisLoop::Parameters{
        angle_gains(), gimbal::AxisLoop::Gains{0.0F, std::numeric_limits<float>::quiet_NaN(), 0.0F, 1.0F}}));
    assert(!loop.retune(gimbal::AxisLoop::Parameters{gimbal::AxisLoop::Gains{-1.0F, 0.0F, 0.0F, 1.0F}, rate_gains()}));
    for (std::size_t cycle = 0U; cycle < 100U; ++cycle)
    {
        const gimbal::AxisLoop::Measurement measurement{plant.angle, plant.rate, 0.0F};
        const gimbal::AxisLoop::Reference target{0.2F, 0.0F};
        const gimbal::AxisLoop::Output actual = loop.update(measurement, target, DT);
        const gimbal::AxisLoop::Output expected = reference.update(measurement, target, DT);
        assert(near(actual.torque, expected.torque, 1.0e-6F));
    }

    // 还没 init()：返回 false
    gimbal::AxisLoop uninitialized;
    assert(!uninitialized.retune(gimbal::AxisLoop::Parameters{angle_gains(), rate_gains()}));
}

} // namespace

int main()
{
    axis_law_test::run_all(make_controller);
    test_gains();
    test_integral_limit();
    test_rate_target_diagnostic();
    test_retune();
    return 0;
}
