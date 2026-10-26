#include "Application/Task/1_chassis/ChassisController.hpp"
#include "Application/Task/1_chassis/ChassisTask.hpp"
#include <limits>

#include <cassert>
#include <cmath>
#include <cstddef>

using Controller = application::chassis::ChassisController;
using Mode = Controller::WheelControlMode;
using Snapshot = device::MotorState;

namespace {

// M2006(P36) 手册参数：Kt=0.18 N·m/A（输出轴）、减速比 36:1。
// 被控对象：稳态转子转速/电流 ≈ 转速常数 32.96rpm/V × 36 × 相电阻
// 0.461Ω ≈ 547 rpm/A；机械时间常数取 0.1s（小电机轻负载）。
constexpr float Kt = 0.18F;
constexpr float Ratio = 36.0F;
constexpr float SpeedPerAmp = 547.0F;
constexpr float Tau = 0.1F;
constexpr float Dt = 0.001F;

Snapshot OnlineSnapshot(float rotorSpeedRpm, float totalAngleDegrees) {
  Snapshot snapshot{};
  snapshot.feedbackValid = true;
  snapshot.online = true;
  snapshot.speedRpm = static_cast<float>(static_cast<std::int16_t>(rotorSpeedRpm)) / Ratio;
  snapshot.angleDegrees = totalAngleDegrees / Ratio;
  return snapshot;
}

} // namespace


int main() {
  // ---- 配置校验 ----
  Controller controller;
  assert(!controller.IsReady());
  assert(controller.Init(
      application::task::ChassisTask::ControllerConfig()));
  assert(controller.IsReady());
  assert(controller.Mode() == Mode::Speed);
  {
    Controller bad;
    Controller::Config config =
        application::task::ChassisTask::ControllerConfig();
    config.maxWheelSpeedRpm = 0.0F;
    assert(!bad.Init(config));
    config = application::task::ChassisTask::ControllerConfig();
    config.maxTorqueNewtonMeter = std::numeric_limits<float>::quiet_NaN();
    assert(!bad.Init(config));
  }

  Snapshot snapshots[Controller::MotorCount]{};
  float out[Controller::MotorCount]{};

  // ---- 禁用 / 离线 / 非法 dt 恒 0 ----
  snapshots[0] = OnlineSnapshot(0.0F, 0.0F);
  controller.SetWheelTarget(0U, 0.5F);
  controller.Update(snapshots, Dt, out);
  assert(out[0] == 0.0F); // 未使能

  controller.SetEnabled(true);
  snapshots[1] = OnlineSnapshot(0.0F, 0.0F);
  snapshots[1].online = false;
  controller.Update(snapshots, Dt, out);
  assert(out[1] == 0.0F); // 离线恒 0

  controller.Update(snapshots, 0.0F, out);
  assert(out[0] == 0.0F); // 非法 dt

  // ---- Torque 模式：开环直通与钳位（M2006 maxTorque=1.0 N·m）----
  controller.SetMode(Mode::Torque);
  assert(controller.Mode() == Mode::Torque);
  controller.SetWheelTarget(0U, 0.5F);
  controller.Update(snapshots, Dt, out);
  assert(out[0] == 0.5F); // 直通
  controller.SetWheelTarget(0U, 99.0F);
  controller.Update(snapshots, Dt, out);
  assert(out[0] == 1.0F); // maxTorque 钳位
  controller.SetWheelTarget(0U, -99.0F);
  controller.Update(snapshots, Dt, out);
  assert(out[0] == -1.0F);

  // ---- 模式切换复位：积分不跨模式泄漏 ----
  controller.SetMode(Mode::Speed);
  float rpm = 0.0F;
  for (int step = 0; step < 200; ++step) { // 误差累积建立积分
    Snapshot sim[Controller::MotorCount]{};
    sim[0] = OnlineSnapshot(rpm, 0.0F);
    controller.SetWheelTarget(0U, 100.0F); // 输出轴 100rpm → 转子 3600
    controller.Update(sim, Dt, out);
  }
  {
    Snapshot sim[Controller::MotorCount]{};
    sim[0] = OnlineSnapshot(3600.0F, 0.0F);
    controller.SetWheelTarget(0U, 100.0F);
    controller.Update(sim, Dt, out);
    assert(out[0] > 0.0F); // 误差已 0，积分仍在维持输出
  }
  controller.SetMode(Mode::Speed); // 同模式重设即复位
  controller.SetWheelTarget(0U, 100.0F);
  {
    Snapshot sim[Controller::MotorCount]{};
    sim[0] = OnlineSnapshot(3600.0F, 0.0F);
    controller.Update(sim, Dt, out);
    // 积分已清：残差仅为 100×36f 与 3600 的浮点尾数差
    assert(std::fabs(out[0]) < 1.0e-4F);
  }

  // ---- Speed 模式闭环仿真（输出为输出轴力矩）----
  controller.SetMode(Mode::Speed);
  controller.SetWheelTarget(0U, 40.0F); // 输出轴 40rpm → 转子 1440
  rpm = 0.0F;
  float peakAbsTorque = 0.0F;
  for (int step = 0; step < 4000; ++step) {
    Snapshot sim[Controller::MotorCount]{};
    sim[0] = OnlineSnapshot(rpm, 0.0F);
    controller.Update(sim, Dt, out);
    if (std::fabs(out[0]) > peakAbsTorque) {
      peakAbsTorque = std::fabs(out[0]);
    }
    assert(std::fabs(out[0]) <= 1.0F + 1.0e-3F); // maxTorque 总限幅
    assert(std::fabs(out[0]) <= 0.55F); // 3A 安培限幅 × Kt=0.18
    rpm += ((out[0] / Kt * SpeedPerAmp) - rpm) * Dt / Tau;
  }
  assert(std::fabs(rpm - 1440.0F) < 80.0F); // 收敛到转子目标
  assert(peakAbsTorque >= 0.5F); // 启动阶段曾接近安培限幅

  // 转速目标钳位：超大目标 → 转子目标被钳 18000 → 输出为安培限幅
  controller.SetWheelTarget(0U, 100000.0F);
  {
    Snapshot sim[Controller::MotorCount]{};
    sim[0] = OnlineSnapshot(0.0F, 0.0F);
    controller.Update(sim, Dt, out);
    assert(out[0] > 0.5F);
  }

  // ---- Angle 模式串级仿真：目标输出轴 90° ----
  controller.SetMode(Mode::Angle);
  controller.SetWheelTarget(0U, 90.0F);
  rpm = 0.0F;
  float angleOut = 0.0F;
  for (int step = 0; step < 6000; ++step) { // 6 秒
    Snapshot sim[Controller::MotorCount]{};
    sim[0] = OnlineSnapshot(rpm, angleOut * Ratio);
    controller.Update(sim, Dt, out);
    assert(std::fabs(out[0]) <= 1.0F + 1.0e-3F);
    rpm += ((out[0] / Kt * SpeedPerAmp) - rpm) * Dt / Tau;
    angleOut += (rpm / Ratio) * Dt * 6.0F; // 输出轴 rpm → deg/s
  }
  assert(std::fabs(angleOut - 90.0F) < 5.0F);

  // ---- 轮间独立 ----
  controller.SetMode(Mode::Speed);
  controller.SetWheelTarget(1U, -100.0F);
  {
    Snapshot sim[Controller::MotorCount]{};
    sim[0] = OnlineSnapshot(0.0F, 0.0F);
    sim[1] = OnlineSnapshot(0.0F, 0.0F);
    controller.Update(sim, Dt, out);
    assert(out[0] == 0.0F);
    assert(out[1] < 0.0F);
  }
  // Invalid inputs reset accumulated state instead of replaying an old output.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  for (const float badValue : {nan, inf, -inf}) {
    auto valid = OnlineSnapshot(0.0F, 0.0F);
    Snapshot states[Controller::MotorCount]{};
    states[0] = valid;
    controller.SetMode(Mode::Speed);
    controller.SetWheelTarget(0U, 100.0F);
    controller.Update(states, Dt, out);
    assert(out[0] > 0.0F);
    controller.Update(states, badValue, out);
    assert(out[0] == 0.0F);
    controller.SetWheelTarget(0U, badValue);
    controller.Update(states, Dt, out);
    assert(out[0] == 0.0F);
    controller.SetWheelTarget(0U, 100.0F);
    states[0].speedRpm = badValue;
    controller.Update(states, Dt, out);
    assert(out[0] == 0.0F);
    states[0] = valid;
    states[0].angleDegrees = badValue;
    controller.Update(states, Dt, out);
    assert(out[0] == 0.0F);
    states[0] = valid;
    states[0].feedbackValid = false;
    controller.Update(states, Dt, out);
    assert(out[0] == 0.0F);
    states[0] = valid;
    controller.SetWheelTarget(0U, 0.0F);
    controller.Update(states, Dt, out);
    assert(out[0] == 0.0F);
  }
  // Finite but overflowing PID arithmetic must also reset and output zero.
  auto extremeConfig = application::task::ChassisTask::ControllerConfig();
  extremeConfig.speedLoop.Kp = std::numeric_limits<float>::max();
  assert(controller.Init(extremeConfig));
  controller.SetEnabled(true);
  controller.SetWheelTarget(0U, 2.0F);
  snapshots[0] = OnlineSnapshot(0.0F, 0.0F);
  controller.Update(snapshots, Dt, out);
  assert(out[0] == 0.0F);
  controller.SetWheelTarget(0U, 0.0F);
  controller.Update(snapshots, Dt, out);
  assert(out[0] == 0.0F);
  return 0;
}
