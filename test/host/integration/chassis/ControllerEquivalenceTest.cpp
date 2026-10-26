#include "Application/Task/1_chassis/ChassisTask.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

using Controller = application::chassis::ChassisController;
using Mode = Controller::WheelControlMode;

// Frozen pre-migration equations: rotor rpm -> amperes -> output-shaft torque.
// Kept independent of the new config to detect erroneous unit conversions.
struct Legacy {
  alg_controller::PID speed, angle;
  Legacy() {
    alg_controller::PID::Config s{};
    s.Kp=0.02F; s.Ki=0.1F; s.DefaultDt=0.001F; s.Maxout=3; s.IntegralLimit=1.5F;
    alg_controller::PID::Config a{};
    a.Kp=4; a.DefaultDt=0.001F; a.Maxout=18000;
    assert(speed.Init(s) && angle.Init(a));
  }
  void Reset() { speed.Reset(); angle.Reset(); }
  float Update(Mode mode, float target, float rotorRpm, float rotorAngle, float dt) {
    if (mode == Mode::Torque) { Reset(); return std::clamp(target,-1.0F,1.0F); }
    float rpm;
    if (mode == Mode::Speed) {
      angle.Reset(); rpm=std::clamp(target*36.0F,-18000.0F,18000.0F);
    } else {
      assert(angle.CalculateLoop(rotorAngle/36.0F,target,dt));
      rpm=std::clamp(angle.GetOut(),-18000.0F,18000.0F);
    }
    assert(speed.CalculateLoop(rotorRpm,rpm,dt));
    return std::clamp(speed.GetOut()*0.18F,-1.0F,1.0F);
  }
};
int main() {
  Controller controller;
  assert(controller.Init(application::task::ChassisTask::ControllerConfig()));
  controller.SetEnabled(true);
  Legacy old[Controller::MotorCount];
  float maxDifference=0;
  for (Mode mode : {Mode::Torque,Mode::Speed,Mode::Angle,Mode::Speed,Mode::Speed}) {
    controller.SetMode(mode);
    for (auto &item : old) item.Reset();
    for (int step=0; step<6000; ++step) {
      device::MotorState states[Controller::MotorCount]{};
      float expected[Controller::MotorCount]{}, result[Controller::MotorCount]{};
      float dt=0.001F + static_cast<float>(step%5)*0.0001F;
      for (std::size_t wheel=0; wheel<Controller::MotorCount; ++wheel) {
        const float phase=static_cast<float>(step)*0.017F + static_cast<float>(wheel);
        const float rotorRpm=static_cast<float>(static_cast<int>(7000.0F*std::sin(phase)));
        const float rotorAngle=5000.0F*std::cos(phase*0.3F);
        float target=mode==Mode::Torque ? 1.8F*std::sin(phase) : 120.0F*std::sin(phase*0.6F);
        if (step%200>180) target=step%2==0 ? 100000.0F : -100000.0F;
        states[wheel]={rotorAngle/36.0F,rotorRpm/36.0F,true,true,0};
        controller.SetWheelTarget(wheel,target);
        expected[wheel]=old[wheel].Update(mode,target,rotorRpm,rotorAngle,dt);
      }
      controller.Update(states,dt,result);
      for (std::size_t wheel=0; wheel<Controller::MotorCount; ++wheel) {
        const float difference=std::fabs(expected[wheel]-result[wheel]);
        maxDifference=std::max(maxDifference,difference);
        assert(difference<=1e-4F);
      }
    }
  }
  std::printf("Maximum torque difference: %.9g N.m\n",static_cast<double>(maxDifference));
}
