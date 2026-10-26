// 非 DJI 品牌扩展机制验证：独立最小适配器 + 虚拟型号映射，不依赖任何
// DJI 实现——本翻译单元只包含通用 MotorGroup.hpp，证明电机组与映射机制
// 品牌无关。
#include "Libraries/Device/motor/MotorGroup.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace {

// 虚拟品牌型号：不占用正式型号枚举值，仅在本测试内有效。
constexpr device::MotorModel kFictionalModel{
    static_cast<device::MotorModel>(254U)};

unsigned inits{0};
unsigned processes{0};
unsigned clears{0};
unsigned writes{0};
std::array<float, 2> lastCommands{};
bool initResult{true};

class FictionalAdapter final {
public:
  struct Config final {
    platform::Can::Channel channel{};
    std::array<device::MotorConnection, 2> connections{};
    std::size_t count{0U};
  };
  static constexpr std::size_t MaximumMotors{2U};

  explicit FictionalAdapter(const Config &config) noexcept : config_{config} {}
  [[nodiscard]] bool Init() noexcept {
    ++inits;
    return initResult;
  }
  void Process() noexcept { ++processes; }
  [[nodiscard]] bool ReadState(std::size_t id,
                               device::MotorState &state) const noexcept {
    state = {};
    if (id >= config_.count) {
      return false;
    }
    state.feedbackValid = true;
    state.online = true;
    return true;
  }
  [[nodiscard]] bool SetTorque(std::size_t id, float value) noexcept {
    if (id >= config_.count || !std::isfinite(value)) {
      return false;
    }
    ++writes;
    lastCommands[id] = value;
    return true;
  }
  void ClearCommands() noexcept {
    ++clears;
    lastCommands = {};
  }

private:
  Config config_;
};

} // namespace

namespace device {
template <> struct MotorAdapterFor<kFictionalModel> {
  using Adapter = FictionalAdapter;
  static constexpr std::size_t MaximumMotors{FictionalAdapter::MaximumMotors};
  template <std::size_t N>
  [[nodiscard]] static FictionalAdapter::Config
  MakeConfig(platform::Can::Channel channel,
             const std::array<MotorConnection, N> &connections) noexcept {
    FictionalAdapter::Config config{};
    config.channel = channel;
    config.count = N;
    for (std::size_t index = 0U; index < N; ++index) {
      config.connections[index] = connections[index];
    }
    return config;
  }
};
} // namespace device

using Fiction = device::MotorGroup<kFictionalModel, 2>;
static_assert(Fiction::Count() == 2U);

int main() {
  // 型号映射：适配器收到品牌装配参数（连接与数量）。
  Fiction fiction{platform::Can::Channel::Channel1, {{{3U, 1}, {5U, -1}}}};

  // 通道占用检查位于通用电机组：与非 DJI 适配器同样生效。
  initResult = false;
  Fiction failing{platform::Can::Channel::Channel2, {{{1U, 1}, {2U, 1}}}};
  assert(!failing.Init()); // 适配器 Init 失败
  assert(inits == 1U);
  initResult = true;
  Fiction owner{platform::Can::Channel::Channel2, {{{1U, 1}, {2U, 1}}}};
  assert(owner.Init() && owner.Init()); // 成功后幂等
  assert(inits == 3U);
  Fiction conflict{platform::Can::Channel::Channel2, {{{1U, 1}, {2U, 1}}}};
  assert(!conflict.Init()); // 同通道第二组拒绝（失败实例不占用通道）

  // 控制契约直通适配器：暂存、发送节拍、清零。
  assert(fiction.Init());
  assert(fiction.SetTorque(0U, 0.5F) && fiction.SetTorque(1U, -0.25F));
  assert(lastCommands[0] == 0.5F && lastCommands[1] == -0.25F);
  fiction.Process();
  assert(processes == 1U);
  device::MotorState state{};
  assert(fiction.ReadState(0U, state) && state.online);
  assert(!fiction.ReadState(2U, state)); // 越界编号
  fiction.ClearCommands();
  assert(clears == 1U && lastCommands == (std::array<float, 2>{}));
  return 0;
}
