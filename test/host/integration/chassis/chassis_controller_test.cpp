#include "chassis/chassis_controller.hpp"
#include "tasks/chassis_task.hpp"
#include <limits>

#include <cassert>
#include <cmath>
#include <cstddef>

using Controller = application::chassis::ChassisController;
using Mode = Controller::WheelControlMode;
using Snapshot = device::MotorState;

namespace
{

// M2006(P36) 手册参数：Kt=0.18 N·m/A（输出轴）、减速比 36:1。
// 被控对象：稳态转子转速/电流 ≈ 转速常数 32.96rpm/V × 36 × 相电阻
// 0.461Ω ≈ 547 rpm/A；机械时间常数取 0.1s（小电机轻负载）。
constexpr float KT = 0.18F;
constexpr float RATIO = 36.0F;
constexpr float SPEED_PER_AMP = 547.0F;
constexpr float TAU = 0.1F;
constexpr float DT = 0.001F;

Snapshot online_snapshot(float rotor_speed_rpm, float total_angle_degrees)
{
    Snapshot snapshot{};
    snapshot.feedback_valid = true;
    snapshot.online = true;
    snapshot.speed_rpm = static_cast<float>(static_cast<std::int16_t>(rotor_speed_rpm)) / RATIO;
    snapshot.angle_degrees = total_angle_degrees / RATIO;
    return snapshot;
}

} // namespace

int main()
{
    // ---- 配置校验 ----
    Controller controller;
    assert(!controller.is_ready());
    assert(controller.init(application::task::ChassisTask::controller_config()));
    assert(controller.is_ready());
    assert(controller.mode() == Mode::SPEED);
    {
        Controller bad;
        Controller::Config config = application::task::ChassisTask::controller_config();
        config.max_wheel_speed_rpm = 0.0F;
        assert(!bad.init(config));
        config = application::task::ChassisTask::controller_config();
        config.max_torque_newton_meter = std::numeric_limits<float>::quiet_NaN();
        assert(!bad.init(config));
    }

    Snapshot snapshots[Controller::MOTOR_COUNT]{};
    float out[Controller::MOTOR_COUNT]{};

    // ---- 禁用 / 离线 / 非法 dt 恒 0 ----
    snapshots[0] = online_snapshot(0.0F, 0.0F);
    controller.set_wheel_target(0U, 0.5F);
    controller.update(snapshots, DT, out);
    assert(out[0] == 0.0F); // 未使能

    controller.set_enabled(true);
    snapshots[1] = online_snapshot(0.0F, 0.0F);
    snapshots[1].online = false;
    controller.update(snapshots, DT, out);
    assert(out[1] == 0.0F); // 离线恒 0

    controller.update(snapshots, 0.0F, out);
    assert(out[0] == 0.0F); // 非法 dt

    // ---- Torque 模式：开环直通与钳位（M2006 maxTorque=1.0 N·m）----
    controller.set_mode(Mode::TORQUE);
    assert(controller.mode() == Mode::TORQUE);
    controller.set_wheel_target(0U, 0.5F);
    controller.update(snapshots, DT, out);
    assert(out[0] == 0.5F); // 直通
    controller.set_wheel_target(0U, 99.0F);
    controller.update(snapshots, DT, out);
    assert(out[0] == 1.0F); // maxTorque 钳位
    controller.set_wheel_target(0U, -99.0F);
    controller.update(snapshots, DT, out);
    assert(out[0] == -1.0F);

    // ---- 模式切换复位：积分不跨模式泄漏 ----
    controller.set_mode(Mode::SPEED);
    float rpm = 0.0F;
    for (int step = 0; step < 200; ++step)
    { // 误差累积建立积分
        Snapshot sim[Controller::MOTOR_COUNT]{};
        sim[0] = online_snapshot(rpm, 0.0F);
        controller.set_wheel_target(0U, 100.0F); // 输出轴 100rpm → 转子 3600
        controller.update(sim, DT, out);
    }
    {
        Snapshot sim[Controller::MOTOR_COUNT]{};
        sim[0] = online_snapshot(3600.0F, 0.0F);
        controller.set_wheel_target(0U, 100.0F);
        controller.update(sim, DT, out);
        assert(out[0] > 0.0F); // 误差已 0，积分仍在维持输出
    }
    controller.set_mode(Mode::SPEED); // 同模式重设即复位
    controller.set_wheel_target(0U, 100.0F);
    {
        Snapshot sim[Controller::MOTOR_COUNT]{};
        sim[0] = online_snapshot(3600.0F, 0.0F);
        controller.update(sim, DT, out);
        // 积分已清：残差仅为 100×36f 与 3600 的浮点尾数差
        assert(std::fabs(out[0]) < 1.0e-4F);
    }

    // ---- Speed 模式闭环仿真（输出为输出轴力矩）----
    controller.set_mode(Mode::SPEED);
    controller.set_wheel_target(0U, 40.0F); // 输出轴 40rpm → 转子 1440
    rpm = 0.0F;
    float peak_abs_torque = 0.0F;
    for (int step = 0; step < 4000; ++step)
    {
        Snapshot sim[Controller::MOTOR_COUNT]{};
        sim[0] = online_snapshot(rpm, 0.0F);
        controller.update(sim, DT, out);
        if (std::fabs(out[0]) > peak_abs_torque)
        {
            peak_abs_torque = std::fabs(out[0]);
        }
        assert(std::fabs(out[0]) <= 1.0F + 1.0e-3F); // maxTorque 总限幅
        assert(std::fabs(out[0]) <= 0.55F);          // 3A 安培限幅 × Kt=0.18
        rpm += ((out[0] / KT * SPEED_PER_AMP) - rpm) * DT / TAU;
    }
    assert(std::fabs(rpm - 1440.0F) < 80.0F); // 收敛到转子目标
    assert(peak_abs_torque >= 0.5F);          // 启动阶段曾接近安培限幅

    // 转速目标钳位：超大目标 → 转子目标被钳 18000 → 输出为安培限幅
    controller.set_wheel_target(0U, 100000.0F);
    {
        Snapshot sim[Controller::MOTOR_COUNT]{};
        sim[0] = online_snapshot(0.0F, 0.0F);
        controller.update(sim, DT, out);
        assert(out[0] > 0.5F);
    }

    // ---- Angle 模式串级仿真：目标输出轴 90° ----
    controller.set_mode(Mode::ANGLE);
    controller.set_wheel_target(0U, 90.0F);
    rpm = 0.0F;
    float angle_out = 0.0F;
    for (int step = 0; step < 6000; ++step)
    { // 6 秒
        Snapshot sim[Controller::MOTOR_COUNT]{};
        sim[0] = online_snapshot(rpm, angle_out * RATIO);
        controller.update(sim, DT, out);
        assert(std::fabs(out[0]) <= 1.0F + 1.0e-3F);
        rpm += ((out[0] / KT * SPEED_PER_AMP) - rpm) * DT / TAU;
        angle_out += (rpm / RATIO) * DT * 6.0F; // 输出轴 rpm → deg/s
    }
    assert(std::fabs(angle_out - 90.0F) < 5.0F);

    // ---- 轮间独立 ----
    controller.set_mode(Mode::SPEED);
    controller.set_wheel_target(1U, -100.0F);
    {
        Snapshot sim[Controller::MOTOR_COUNT]{};
        sim[0] = online_snapshot(0.0F, 0.0F);
        sim[1] = online_snapshot(0.0F, 0.0F);
        controller.update(sim, DT, out);
        assert(out[0] == 0.0F);
        assert(out[1] < 0.0F);
    }
    // Invalid inputs reset accumulated state instead of replaying an old output.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (const float bad_value : {nan, inf, -inf})
    {
        auto valid = online_snapshot(0.0F, 0.0F);
        Snapshot states[Controller::MOTOR_COUNT]{};
        states[0] = valid;
        controller.set_mode(Mode::SPEED);
        controller.set_wheel_target(0U, 100.0F);
        controller.update(states, DT, out);
        assert(out[0] > 0.0F);
        controller.update(states, bad_value, out);
        assert(out[0] == 0.0F);
        controller.set_wheel_target(0U, bad_value);
        controller.update(states, DT, out);
        assert(out[0] == 0.0F);
        controller.set_wheel_target(0U, 100.0F);
        states[0].speed_rpm = bad_value;
        controller.update(states, DT, out);
        assert(out[0] == 0.0F);
        states[0] = valid;
        states[0].angle_degrees = bad_value;
        controller.update(states, DT, out);
        assert(out[0] == 0.0F);
        states[0] = valid;
        states[0].feedback_valid = false;
        controller.update(states, DT, out);
        assert(out[0] == 0.0F);
        states[0] = valid;
        controller.set_wheel_target(0U, 0.0F);
        controller.update(states, DT, out);
        assert(out[0] == 0.0F);
    }
    // Finite but overflowing PID arithmetic must also reset and output zero.
    auto extreme_config = application::task::ChassisTask::controller_config();
    extreme_config.speed_loop.kp = std::numeric_limits<float>::max();
    assert(controller.init(extreme_config));
    controller.set_enabled(true);
    controller.set_wheel_target(0U, 2.0F);
    snapshots[0] = online_snapshot(0.0F, 0.0F);
    controller.update(snapshots, DT, out);
    assert(out[0] == 0.0F);
    controller.set_wheel_target(0U, 0.0F);
    controller.update(snapshots, DT, out);
    assert(out[0] == 0.0F);
    return 0;
}
