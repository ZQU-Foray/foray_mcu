#ifndef TEST_HOST_UNIT_GIMBAL_AXIS_LAW_SCENARIOS_HPP
#define TEST_HOST_UNIT_GIMBAL_AXIS_LAW_SCENARIOS_HPP

// 控制律无关的闭环场景：一个轴、一个被控对象、一组断言。被测控制器由 make_controller() 造出来
// （造好就已经 init 过），只要求它有 update/reset/apply 与 Measurement/Reference/Output 这几个名字。
// 换控制律时不用改这个文件：新写一个 make_controller() 与它自己的特有场景即可。
// 这里只用行为判断，不看控制律的内部量（积分值、内部目标之类）。

#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>

namespace axis_law_test
{

constexpr float DT{0.001F};
constexpr float FRICTION_SPEED{0.01F}; // tanh 的平滑尺度，rad/s

// 一个轴：J·ω̇ = τ − b·ω − τc·tanh(ω/ωs) − τg·cos(θ)，力矩晚一个周期生效（模拟 CAN 往返）。
// 摩擦在零速附近用 tanh 平滑，避免离散仿真在零速抖动（真实减速箱是静摩擦 + 库仑摩擦）。
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

inline bool near(float actual, float expected, float tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

/** apply() 传什么：总限幅之后的力矩减去前馈（Gimbal 的写法），还是控制律自己算出的力矩。 */
enum class ApplyTorque
{
    ACTUAL,  // 实际下发的力矩 − 前馈
    COMPUTED // 控制律自己算的力矩
};

/**
 * @brief 跑一拍：算力矩、加重力前馈、总限幅、告诉控制器实际值、推进被控对象。
 * @param target 角度目标。
 * @param target_rate 角速度前馈。
 * @param apply_torque apply() 收到的是"实际下发的力矩 − 前馈"还是"控制律自己算的力矩"。
 */
template <typename Controller>
typename Controller::Output step(Controller &controller,
                                 Plant &plant,
                                 float target,
                                 float torque_limit,
                                 float gravity_feedforward_newton_meter,
                                 float target_rate = 0.0F,
                                 ApplyTorque apply_torque = ApplyTorque::ACTUAL)
{
    typename Controller::Measurement measurement{};
    measurement.angle = plant.angle;
    measurement.rate = plant.rate;
    measurement.joint_velocity = plant.rate; // 仿真里关节与被控轴一起动
    typename Controller::Reference reference{};
    reference.angle = target;
    reference.rate = target_rate;
    const typename Controller::Output output = controller.update(measurement, reference, DT);
    const float feedforward = gravity_feedforward_newton_meter * std::cos(plant.angle);
    const float total = std::fmax(std::fmin(output.torque + feedforward, torque_limit), -torque_limit);
    controller.apply(apply_torque == ApplyTorque::ACTUAL ? total - feedforward : output.torque);
    plant.advance(total, DT);
    return output;
}

// 符号：目标在测量值正方向时力矩为正，反之为负；角速度反馈与角速度前馈的方向也对
template <typename MakeController>
void sign(MakeController make_controller)
{
    auto controller = make_controller();
    typename decltype(controller)::Measurement measurement{};
    typename decltype(controller)::Reference reference{};
    reference.angle = 0.1F;
    assert(controller.update(measurement, reference, DT).torque > 0.0F);
    controller.reset();
    reference.angle = -0.1F;
    assert(controller.update(measurement, reference, DT).torque < 0.0F);
    // 角度一致、角速度为正：输出为负（阻尼）
    controller.reset();
    reference.angle = 0.0F;
    measurement.rate = 1.0F;
    assert(controller.update(measurement, reference, DT).torque < 0.0F);
    // 角速度前馈：目标角速度为正时输出为正
    controller.reset();
    measurement.rate = 0.0F;
    reference.rate = 1.0F;
    assert(controller.update(measurement, reference, DT).torque > 0.0F);
}

// 阶跃 0.3 rad：1 s 内收敛，稳态误差小于 0.005 rad，超调有上限
template <typename MakeController>
void step_response(MakeController make_controller)
{
    auto controller = make_controller();
    Plant plant;
    plant.gravity = 0.0F;
    constexpr float TARGET{0.3F};
    float peak = -10.0F;
    float angle_at_one_second = 0.0F;
    for (std::size_t cycle = 0U; cycle < 1000U; ++cycle)
    {
        (void)step(controller, plant, TARGET, 1.0F, 0.0F);
        peak = std::fmax(peak, plant.angle);
        if (cycle == 999U)
        {
            angle_at_one_second = plant.angle;
        }
    }
    assert(near(angle_at_one_second, TARGET, 0.005F));
    assert(peak < TARGET + 0.05F);
    assert(peak >= TARGET);
}

// 有重力时补偿力矩趋近 τg·cos(θ)；前馈给对了时趋近 0
template <typename MakeController>
void gravity_compensation(MakeController make_controller)
{
    {
        auto controller = make_controller();
        Plant plant;
        typename decltype(controller)::Output output{};
        for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
        {
            output = step(controller, plant, 0.0F, 1.0F, 0.0F);
        }
        assert(near(plant.angle, 0.0F, 0.005F));
        assert(near(output.diagnostics.compensation_newton_meter, plant.gravity, 0.05F));
    }
    {
        auto controller = make_controller();
        Plant plant;
        typename decltype(controller)::Output output{};
        for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
        {
            output = step(controller, plant, 0.0F, 1.0F, plant.gravity);
        }
        assert(near(plant.angle, 0.0F, 0.005F));
        assert(near(output.diagnostics.compensation_newton_meter, 0.0F, 0.02F));
    }
}

// 总限幅把力矩削一段时间再放开：放开后没有积分饱和带来的大超调
template <typename MakeController>
void total_limit(MakeController make_controller)
{
    auto controller = make_controller();
    Plant plant;
    plant.gravity = 0.0F;
    constexpr float TARGET{0.02F};
    constexpr float LIMITED{0.1F};
    float peak = -10.0F;
    for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
    {
        const float limit = cycle < 1000U ? LIMITED : 1.0F;
        (void)step(controller, plant, TARGET, limit, 0.0F);
        peak = std::fmax(peak, plant.angle);
    }
    assert(peak < TARGET + 0.01F);
    assert(near(plant.angle, TARGET, 0.005F));
}

// 没被总限幅削的时候，apply() 传"实际下发的力矩 − 前馈"与传"控制律自己算的力矩"要给出同一条轨迹：
// 前者是先加前馈、再减前馈算出来的，浮点上与后者不一定逐位相等，不能因此当成"被削了"。
// 两边都每拍调用 apply()，符合接口约定（观测器也要每拍拿到实际力矩）。
template <typename MakeController>
void apply_neutral(MakeController make_controller)
{
    constexpr float TARGET{0.02F};
    constexpr float FEEDFORWARD{0.3F}; // 只补一部分，剩下的靠控制器
    Plant from_actual{};
    Plant from_computed{};
    auto actual_controller = make_controller();
    auto computed_controller = make_controller();
    for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
    {
        (void)step(actual_controller, from_actual, TARGET, 1.0F, FEEDFORWARD, 0.0F, ApplyTorque::ACTUAL);
        (void)step(computed_controller, from_computed, TARGET, 1.0F, FEEDFORWARD, 0.0F, ApplyTorque::COMPUTED);
    }
    assert(near(from_actual.angle, from_computed.angle, 1.0e-3F));
    assert(near(from_actual.rate, from_computed.rate, 1.0e-3F));
}

// 带角速度前馈跟斜坡：目标按 1 rad/s 走，1 s 后跟上
template <typename MakeController>
void ramp_feedforward(MakeController make_controller)
{
    auto controller = make_controller();
    Plant plant;
    plant.gravity = 0.0F;
    constexpr float RATE{1.0F};
    float target = 0.0F;
    for (std::size_t cycle = 0U; cycle < 2000U; ++cycle)
    {
        target += RATE * DT;
        (void)step(controller, plant, target, 1.0F, 0.0F, RATE);
    }
    assert(near(plant.rate, RATE, 0.05F));
    assert(near(plant.angle, target, 0.05F));
}

// reset() 后第一拍没有残留：目标等于测量、角速度为 0 时输出为 0
template <typename MakeController>
void reset_first_cycle(MakeController make_controller)
{
    auto controller = make_controller();
    Plant plant;
    plant.gravity = 0.0F;
    for (std::size_t cycle = 0U; cycle < 500U; ++cycle)
    {
        (void)step(controller, plant, 0.3F, 1.0F, 0.0F);
    }
    controller.reset();
    typename decltype(controller)::Measurement measurement{};
    measurement.angle = plant.angle;
    measurement.rate = 0.0F;
    measurement.joint_velocity = 0.0F;
    typename decltype(controller)::Reference reference{};
    reference.angle = plant.angle;
    reference.rate = 0.0F;
    const typename decltype(controller)::Output output = controller.update(measurement, reference, DT);
    assert(output.torque == 0.0F);
}

// 非有限输入：输出 0，不崩、不污染，之后照常工作
template <typename MakeController>
void non_finite_inputs(MakeController make_controller)
{
    auto controller = make_controller();
    constexpr float NAN_VALUE{std::numeric_limits<float>::quiet_NaN()};
    const std::initializer_list<float> bad_values{NAN_VALUE, std::numeric_limits<float>::infinity()};
    for (const float bad : bad_values)
    {
        typename decltype(controller)::Measurement measurement{};
        typename decltype(controller)::Reference reference{};
        measurement.angle = bad;
        assert(controller.update(measurement, reference, DT).torque == 0.0F);
        measurement.angle = 0.0F;
        reference.angle = bad;
        assert(controller.update(measurement, reference, DT).torque == 0.0F);
        reference.angle = 0.0F;
        measurement.rate = bad;
        assert(controller.update(measurement, reference, DT).torque == 0.0F);
        measurement.rate = 0.0F;
        measurement.joint_velocity = bad;
        assert(controller.update(measurement, reference, DT).torque == 0.0F);
        measurement.joint_velocity = 0.0F;
        reference.rate = bad;
        assert(controller.update(measurement, reference, DT).torque == 0.0F);
        reference.rate = 0.0F;
        controller.apply(bad); // 不崩、不污染
    }
    typename decltype(controller)::Measurement measurement{};
    typename decltype(controller)::Reference reference{};
    reference.angle = 0.1F;
    const typename decltype(controller)::Output output = controller.update(measurement, reference, DT);
    assert(std::isfinite(output.torque) && output.torque > 0.0F);
    assert(std::isfinite(controller.update(measurement, reference, 0.0F).torque));
    assert(std::isfinite(controller.update(measurement, reference, 5.0F).torque));
}

// 角度连续：测量值连续地扫过 ±π 时输出不跳变（控制律内部不处理回绕，也不该在 ±π 处做特殊处理）
template <typename MakeController>
void angle_sweep(MakeController make_controller)
{
    auto controller = make_controller();
    constexpr float REFERENCE_ANGLE{3.15F}; // 固定在 π 附近，扫过的角度会跨过它
    constexpr float STEP{0.001F};
    float angle = 3.0F;
    float previous_torque = 0.0F;
    bool has_previous = false;
    for (std::size_t cycle = 0U; cycle < 300U; ++cycle)
    {
        typename decltype(controller)::Measurement measurement{};
        measurement.angle = angle;
        typename decltype(controller)::Reference reference{};
        reference.angle = REFERENCE_ANGLE;
        const typename decltype(controller)::Output output = controller.update(measurement, reference, DT);
        if (has_previous)
        {
            assert(std::fabs(output.torque - previous_torque) < 0.2F);
        }
        previous_torque = output.torque;
        has_previous = true;
        angle += STEP;
    }
}

/** 跑一遍全部通用场景。 */
template <typename MakeController>
void run_all(MakeController make_controller)
{
    sign(make_controller);
    step_response(make_controller);
    gravity_compensation(make_controller);
    total_limit(make_controller);
    apply_neutral(make_controller);
    ramp_feedforward(make_controller);
    reset_first_cycle(make_controller);
    non_finite_inputs(make_controller);
    angle_sweep(make_controller);
}

} // namespace axis_law_test

#endif
