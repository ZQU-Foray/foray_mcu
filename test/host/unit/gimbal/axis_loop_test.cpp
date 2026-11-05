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

// ---- PID 特有：参数检查 ----
void test_gains()
{
    gimbal::AxisLoop loop;
    assert(!loop.init(gimbal::AxisLoop::Parameters{gimbal::AxisLoop::Gains{-1.0F, 0.0F, 0.0F, 1.0F}, rate_gains()}));
    assert(!loop.init(gimbal::AxisLoop::Parameters{
        angle_gains(), gimbal::AxisLoop::Gains{0.0F, std::numeric_limits<float>::quiet_NaN(), 0.0F, 1.0F}}));
    assert(!loop.init(gimbal::AxisLoop::Parameters{angle_gains(), gimbal::AxisLoop::Gains{0.0F, 0.0F, -1.0F, 1.0F}}));
    assert(loop.init(gimbal::AxisLoop::Parameters{angle_gains(), rate_gains()}));
}

// ---- PID 特有：积分不超过积分限幅；饱和时力矩不超过环的输出上限 ----
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

// ---- PID 特有：rate_target 诊断（角度环输出加前馈、再按角度环的输出上限钳位）----
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

} // namespace

int main()
{
    axis_law_test::run_all(make_controller);
    test_gains();
    test_integral_limit();
    test_rate_target_diagnostic();
    return 0;
}
