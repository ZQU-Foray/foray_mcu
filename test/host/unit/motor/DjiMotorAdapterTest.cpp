#include "Tests/Motor/Support/DjiPlatformStub.hpp"
#include "Libraries/Device/motor/dji/DjiMotorAdapter.hpp"
#include <cmath>
#include <limits>

using Adapter = device::DjiMotorAdapter;
Adapter::Config Config() {
  return {platform::Can::Channel::Channel1, device::DjiMotor::Profile::C610(),
          {{{2U, 36.0F, -1}, {1U, 36.0F, 1}, {3U, 36.0F, 1}, {4U, 36.0F, 1}}}, 4U};
}
int main() {
  auto config = Config();
  Adapter adapter{config};
  device::MotorState state{1.0F, 2.0F, true, true, 1U};
  assert(!adapter.ReadState(0, state) && !state.feedbackValid && state.angleDegrees == 0);
  assert(!adapter.SetTorque(0, 0.1F));
  adapter.Process();
  assert(canStub.txCount == 0);
  assert(adapter.Init());
  assert(adapter.Count() == 4);
  assert(adapter.ReadState(0, state) && !state.feedbackValid && !state.online);
  adapter.Process();
  assert(canStub.txCount == 1 && canStub.lastTxIdentifier == 0x200);
  assert(canStub.lastTxData == (std::array<std::uint8_t, 8>{}));
  // Logical 0 -> device 2, negative direction; output-shaft Kt is not geared twice.
  assert(adapter.SetTorque(0, 0.18F));
  assert(adapter.SetTorque(1, 0.36F));
  assert(adapter.SetTorque(2, 0.54F));
  assert(adapter.SetTorque(3, 0.72F));
  assert(canStub.txCount == 1); // setters do not send
  assert(adapter.Init()); // idempotent, retains staged commands
  adapter.Process();
  assert(canStub.txCount == 2);
  assert(canStub.lastTxData == (std::array<std::uint8_t, 8>{0x07,0xD0,0xFC,0x18,0x0B,0xB8,0x0F,0xA0}));
  timeStub.ticks = 100;
  FeedRx(0x202, {0x10,0x00,0x0E,0x10,0,0,0,0}); // 3600 rotor rpm
  FeedRx(0x201, {0x00,0x00,0x0E,0x10,0,0,0,0});
  adapter.Process();
  assert(canStub.rxIndex == canStub.rxCount);
  assert(adapter.ReadState(0, state));
  assert(state.feedbackValid && state.online && state.lastUpdateTick == 100);
  assert(state.angleDegrees == 0 && state.speedRpm == -100);
  assert(adapter.ReadState(1, state) && state.speedRpm == 100);
  ResetRx();
  FeedRx(0x202, {0x18,0x00,0x0E,0x10,0,0,0,0}); // +90 rotor deg
  adapter.Process();
  assert(adapter.ReadState(0, state) && std::fabs(state.angleDegrees + 2.5F) < 1e-6F);
  timeStub.ticks = 110;
  adapter.Process();
  assert(adapter.ReadState(0, state) && !state.online && state.feedbackValid);
  ResetRx();
  FeedRx(0x202, {0x18,0x00,0,0,0,0,0,0});
  adapter.Process();
  assert(adapter.ReadState(0, state) && state.online);
  assert(!adapter.ReadState(4, state));
  assert(!state.online && !state.feedbackValid && state.angleDegrees == 0 && state.lastUpdateTick == 0);
  assert(!adapter.SetTorque(4, 1));
  for (float value : {std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity()}) {
    assert(adapter.SetTorque(0, 0.18F));
    assert(!adapter.SetTorque(0, value));
    adapter.Process();
    assert(canStub.lastTxData[2] == 0 && canStub.lastTxData[3] == 0);
    assert(canStub.lastTxData[0] == 0x07); // other slots retained
  }
  assert(adapter.SetTorque(0, std::numeric_limits<float>::max()));
  adapter.Process(); // saturated to -10000
  assert(canStub.lastTxData[2] == 0xD8 && canStub.lastTxData[3] == 0xF0);
  const auto txBefore = canStub.txCount;
  adapter.ClearCommands();
  assert(canStub.txCount == txBefore);
  adapter.Process();
  assert(canStub.lastTxData == (std::array<std::uint8_t, 8>{}));
  const auto rejects = [](const Adapter::Config &bad) {
    Adapter invalid{bad};
    auto count = canStub.txCount;
    assert(!invalid.Init());
    invalid.Process();
    assert(canStub.txCount == count);
  };
  config = Config(); config.channel = static_cast<platform::Can::Channel>(255); rejects(config);
  config = Config(); config.count = 0; rejects(config);
  config = Config(); config.count = 9; rejects(config);
  config = Config(); config.mappings[1].deviceId = 2; rejects(config);
  config = Config(); config.mappings[0].deviceId = 0; rejects(config);
  config = Config(); config.mappings[0].deviceId = 9; rejects(config);
  config = Config(); config.mappings[0].direction = 0; rejects(config);
  config = Config(); config.mappings[0].direction = 2; rejects(config);
  for (float bad : {0.0F, -1.0F, 0.5F, std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::infinity()}) {
    config = Config(); config.mappings[0].gearRatio = bad; rejects(config);
  }
  config = Config(); config.profile = device::DjiMotor::Profile::Gm6020Voltage(); rejects(config);
  config = Config(); config.profile.torqueConstantNewtonMeterPerAmpere = 0; rejects(config);
  config = Config(); config.profile.currentFullScaleAmpere = std::numeric_limits<float>::infinity(); rejects(config);
  canStub.ready = false;
  rejects(Config());
  canStub.ready = true;
  config = Config(); config.profile = device::DjiMotor::Profile::Gm6020Current();
  Adapter currentGm{config};
  assert(currentGm.Init());
}
