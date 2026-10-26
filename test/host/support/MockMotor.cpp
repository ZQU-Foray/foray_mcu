#include "MockMotor.hpp"
#include <cmath>

namespace mock {
bool Adapter::Init() noexcept {
  ready = initResult;
  return ready;
}
void Adapter::Process() noexcept {
  ++processes;
  sent = staged;
}
bool Adapter::ReadState(std::size_t id, device::MotorState &state) const noexcept {
  state = {};
  if (!ready || id >= staged.size() || readFailure) {
    return false;
  }
  state.feedbackValid = true;
  state.online = true;
  return true;
}
bool Adapter::SetTorque(std::size_t id, float value) noexcept {
  ++writes;
  if (!ready || id >= staged.size() || commandFailure) {
    return false;
  }
  if (!std::isfinite(value)) {
    staged[id] = 0;
    return false;
  }
  staged[id] = value;
  return true;
}
void Adapter::ClearCommands() noexcept {
  ++clears;
  staged = {};
}
} // namespace mock
