#include "tasks/chassis_task.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

using Controller = application::chassis::ChassisController;
using Mode = Controller::WheelControlMode;

// Frozen pre-migration equations: rotor rpm -> amperes -> output-shaft torque.
// Kept independent of the new config to detect erroneous unit conversions.
struct Legacy
{
    alg_controller::PID speed, angle;
    Legacy()
    {
        alg_controller::PID::Config s{};
        s.kp = 0.02F;
        s.ki = 0.1F;
        s.default_dt = 0.001F;
        s.maxout = 3;
        s.integral_limit = 1.5F;
        alg_controller::PID::Config a{};
        a.kp = 4;
        a.default_dt = 0.001F;
        a.maxout = 18000;
        assert(speed.init(s) && angle.init(a));
    }
    void reset()
    {
        speed.reset();
        angle.reset();
    }
    float update(Mode mode, float target, float rotor_rpm, float rotor_angle, float dt)
    {
        if (mode == Mode::TORQUE)
        {
            reset();
            return std::clamp(target, -1.0F, 1.0F);
        }
        float rpm;
        if (mode == Mode::SPEED)
        {
            angle.reset();
            rpm = std::clamp(target * 36.0F, -18000.0F, 18000.0F);
        }
        else
        {
            assert(angle.calculate_loop(rotor_angle / 36.0F, target, dt));
            rpm = std::clamp(angle.get_out(), -18000.0F, 18000.0F);
        }
        assert(speed.calculate_loop(rotor_rpm, rpm, dt));
        return std::clamp(speed.get_out() * 0.18F, -1.0F, 1.0F);
    }
};
int main()
{
    Controller controller;
    assert(controller.init(application::task::ChassisTask::controller_config()));
    controller.set_enabled(true);
    Legacy old[Controller::MOTOR_COUNT];
    float max_difference = 0;
    for (Mode mode : {Mode::TORQUE, Mode::SPEED, Mode::ANGLE, Mode::SPEED, Mode::SPEED})
    {
        controller.set_mode(mode);
        for (auto &item : old)
            item.reset();
        for (int step = 0; step < 6000; ++step)
        {
            device::MotorState states[Controller::MOTOR_COUNT]{};
            float expected[Controller::MOTOR_COUNT]{}, result[Controller::MOTOR_COUNT]{};
            float dt = 0.001F + static_cast<float>(step % 5) * 0.0001F;
            for (std::size_t wheel = 0; wheel < Controller::MOTOR_COUNT; ++wheel)
            {
                const float phase = static_cast<float>(step) * 0.017F + static_cast<float>(wheel);
                const float rotor_rpm = static_cast<float>(static_cast<int>(7000.0F * std::sin(phase)));
                const float rotor_angle = 5000.0F * std::cos(phase * 0.3F);
                float target = mode == Mode::TORQUE ? 1.8F * std::sin(phase) : 120.0F * std::sin(phase * 0.6F);
                if (step % 200 > 180)
                    target = step % 2 == 0 ? 100000.0F : -100000.0F;
                states[wheel] = {rotor_angle / 36.0F, rotor_rpm / 36.0F, true, true, 0};
                controller.set_wheel_target(wheel, target);
                expected[wheel] = old[wheel].update(mode, target, rotor_rpm, rotor_angle, dt);
            }
            controller.update(states, dt, result);
            for (std::size_t wheel = 0; wheel < Controller::MOTOR_COUNT; ++wheel)
            {
                const float difference = std::fabs(expected[wheel] - result[wheel]);
                max_difference = std::max(max_difference, difference);
                assert(difference <= 1e-4F);
            }
        }
    }
    std::printf("Maximum torque difference: %.9g N.m\n", static_cast<double>(max_difference));
}
