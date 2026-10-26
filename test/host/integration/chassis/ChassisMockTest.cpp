#include "Application/Task/1_chassis/ChassisTask.hpp"
#include "MockMotor.hpp"
#include <cassert>
#include <cmath>
#include <string>
#include <cstdlib>

namespace {
unsigned delays=0;
std::string scenario;
}
extern "C" {
bool TimePort_Init() { return true; }
bool TimePort_IsReady() { return true; }
std::uint32_t TimePort_NowTicks() { return delays; }
std::uint32_t TimePort_FrequencyHz() { return 1000; }
void TimePort_DelayUs(std::uint32_t) {}
}
[[noreturn]] void osThreadExit() { std::abort(); }
std::int32_t osDelay(std::uint32_t ticks) {
  assert(ticks==1);
  ++delays;
  using Task=application::task::ChassisTask;
  if (delays==1) {
    Task::SetMode(Task::WheelControlMode::Torque);
    Task::SetWheelTarget(0,0.3F);
  }
  if (delays==2) {
    assert(mock::processes==1 && mock::sent[0]==0);
    assert(std::fabs(mock::staged[0]-0.3F)<1e-6F);
    mock::readFailure=scenario=="read_failure";
    mock::commandFailure=scenario=="command_failure";
  }
  if (delays==3) {
    assert(mock::processes==2);
    assert(std::fabs(mock::sent[0]-0.3F)<1e-6F); // previous cycle's command
    if (mock::readFailure || mock::commandFailure) assert(mock::staged[0]==0);
    mock::readFailure=false;
    mock::commandFailure=false;
    Task::SetWheelTarget(0,0.3F); // recover after failure/reset
  }
  if (delays==4) {
    assert(mock::processes==3);
    if (scenario!="normal") assert(mock::sent[0]==0);
    assert(std::fabs(mock::staged[0]-0.3F)<1e-6F);
    Task::SetEnabled(false);
  }
  if (delays==5) {
    assert(mock::processes==4 && mock::staged[0]==0);
  }
  if (delays==6) {
    assert(mock::processes==5 && mock::sent[0]==0);
    if (scenario=="command_failure") assert(mock::clears==1);
    std::exit(0);
  }
  return 0;
}
int main(int argc, char **argv) {
  assert(argc==2);
  scenario=argv[1];
  using Task=application::task::ChassisTask;
  if (scenario=="init_failure") mock::initResult=false;
  if (scenario=="init_failure") {
    assert(!Task::Init());
    assert(!Task::IsReady() && mock::clears==1);
    mock::initResult=true;
    assert(Task::Init());
    return 0;
  }
  assert(Task::Init());
  assert(Task::Init());
  Task::Run(nullptr);
}
