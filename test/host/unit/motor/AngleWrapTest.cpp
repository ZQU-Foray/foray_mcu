#include "Tests/Motor/Support/DjiPlatformStub.hpp"
#include <initializer_list>

void Check(std::initializer_list<std::uint16_t> angles, std::int32_t expected) {
  device::DjiMotor bus{platform::Can::Channel::Channel1, device::DjiMotor::Profile::C610()};
  assert(bus.Init());
  assert(bus.EnableMotor(1));
  for (const auto angle : angles) {
    ResetRx();
    FeedRx(0x201, {static_cast<std::uint8_t>(angle >> 8),
                   static_cast<std::uint8_t>(angle),0,0,0,0,0,0});
    bus.Process();
  }
  device::DjiMotor::Snapshot state{};
  assert(bus.GetSnapshot(1, state));
  assert(state.totalAngleCounts == expected);
}
int main() {
  Check({4096}, 0);
  Check({4096,8191,15}, 4111);
  Check({15,8191,4096}, -4111);
  Check({8191,0}, 1);
  Check({0,8191}, -1);
  Check({0,2048,4096,6144,0,2048,4096,6144,0}, 16384);
  Check({0,6144,4096,2048,0,6144,4096,2048,0}, -16384);
  Check({0,4096}, 4096);
  Check({4096,0}, -4096);
  Check({0,4095}, 4095);
  Check({0,4097}, -4095);
}
