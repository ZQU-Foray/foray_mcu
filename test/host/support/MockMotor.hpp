#pragma once
#include "Libraries/Device/motor/MotorGroup.hpp"
#include "Platform/Interface/Can.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace mock {
inline bool initResult{true};
inline bool readFailure{false}, commandFailure{false};
inline unsigned processes{0}, clears{0}, writes{0};
inline std::array<float, 4> staged{}, sent{};
inline bool ready{false};

// 最小非 DJI 适配器：与 DjiMotorAdapter 同契约，直连全局测试观测量。
class Adapter final {
public:
  struct Config final {
    platform::Can::Channel channel{};
    std::array<device::MotorConnection, 4> connections{};
    std::size_t count{0U};
  };
  static constexpr std::size_t MaximumMotors{4U};

  explicit Adapter(const Config &config) noexcept : config_{config} {}
  [[nodiscard]] bool Init() noexcept;
  void Process() noexcept;
  [[nodiscard]] bool ReadState(std::size_t id,
                               device::MotorState &state) const noexcept;
  [[nodiscard]] bool SetTorque(std::size_t id, float value) noexcept;
  void ClearCommands() noexcept;
  [[nodiscard]] const Config &GetConfig() const noexcept { return config_; }

private:
  Config config_;
};
} // namespace mock

namespace device {

// 模拟映射共用装配：全部型号指向模拟适配器。
template <std::size_t N>
[[nodiscard]] mock::Adapter::Config
MakeMockConfig(platform::Can::Channel channel,
               const std::array<MotorConnection, N> &connections) noexcept {
  static_assert(N > 0U && N <= mock::Adapter::MaximumMotors,
                "电机数量超出模拟适配器能力");
  mock::Adapter::Config config{};
  config.channel = channel;
  config.count = N;
  for (std::size_t index = 0U; index < N; ++index) {
    config.connections[index] = connections[index];
  }
  return config;
}

// 测试映射：仅经 Support 包含路径可见（拦截生产的 DJI 映射头）。
template <> struct MotorAdapterFor<MotorModel::M2006> {
  using Adapter = mock::Adapter;
  static constexpr std::size_t MaximumMotors{mock::Adapter::MaximumMotors};
  template <std::size_t N>
  [[nodiscard]] static mock::Adapter::Config
  MakeConfig(platform::Can::Channel channel,
             const std::array<MotorConnection, N> &connections) noexcept {
    return MakeMockConfig(channel, connections);
  }
};

template <> struct MotorAdapterFor<MotorModel::M3508> {
  using Adapter = mock::Adapter;
  static constexpr std::size_t MaximumMotors{mock::Adapter::MaximumMotors};
  template <std::size_t N>
  [[nodiscard]] static mock::Adapter::Config
  MakeConfig(platform::Can::Channel channel,
             const std::array<MotorConnection, N> &connections) noexcept {
    return MakeMockConfig(channel, connections);
  }
};

template <> struct MotorAdapterFor<MotorModel::GM6020Current> {
  using Adapter = mock::Adapter;
  static constexpr std::size_t MaximumMotors{mock::Adapter::MaximumMotors};
  template <std::size_t N>
  [[nodiscard]] static mock::Adapter::Config
  MakeConfig(platform::Can::Channel channel,
             const std::array<MotorConnection, N> &connections) noexcept {
    return MakeMockConfig(channel, connections);
  }
};

} // namespace device
