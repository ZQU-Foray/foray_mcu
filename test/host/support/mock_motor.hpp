#pragma once
#include "Libraries/Device/motor/MotorGroup.hpp"
#include "Platform/Interface/Can.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace mock
{
inline bool g_init_result{true};
inline bool g_read_failure{false}, g_command_failure{false};
inline unsigned g_processes{0}, g_clears{0}, g_writes{0};
inline std::array<float, 4> g_staged{}, g_sent{};
inline bool g_ready{false};
// 输出轴最大力矩能力（N·m）；默认取底盘所用 M2006/C610 的 10 A × 0.18 N·m/A。
inline float g_maximum_torque{1.8F};

// 最小非 DJI 适配器：与 DjiMotorAdapter 同契约，直连全局测试观测量。
class Adapter final
{
  public:
    struct Config final
    {
        platform::Can::Channel channel{};
        std::array<device::MotorConnection, 4> connections{};
        std::size_t count{0U};
    };
    static constexpr std::size_t MAXIMUM_MOTORS{4U};

    explicit Adapter(const Config &config) noexcept : config_{config} {}
    [[nodiscard]] bool init() noexcept;
    void process() noexcept;
    [[nodiscard]] bool read_state(std::size_t id, device::MotorState &state) const noexcept;
    [[nodiscard]] bool set_torque(std::size_t id, float value) noexcept;
    void clear_commands() noexcept;
    // 与 DjiMotorAdapter 同语义：Init 成功后返回能力值，未就绪返回 0。
    [[nodiscard]] float maximum_torque_newton_meter() const noexcept;
    [[nodiscard]] const Config &get_config() const noexcept
    {
        return config_;
    }

  private:
    Config config_;
};
} // namespace mock

namespace device
{

// 模拟映射共用装配：全部型号指向模拟适配器。
template <std::size_t N>
[[nodiscard]] mock::Adapter::Config make_mock_config(platform::Can::Channel channel,
                                                     const std::array<MotorConnection, N> &connections) noexcept
{
    static_assert(N > 0U && N <= mock::Adapter::MAXIMUM_MOTORS, "电机数量超出模拟适配器能力");
    mock::Adapter::Config config{};
    config.channel = channel;
    config.count = N;
    for (std::size_t index = 0U; index < N; ++index)
    {
        config.connections[index] = connections[index];
    }
    return config;
}

// 测试映射：仅经 Support 包含路径可见（拦截生产的 DJI 映射头）。
template <> struct MotorAdapterFor<MotorModel::M2006>
{
    using Adapter = mock::Adapter;
    static constexpr std::size_t MAXIMUM_MOTORS{mock::Adapter::MAXIMUM_MOTORS};
    template <std::size_t N>
    [[nodiscard]] static mock::Adapter::Config make_config(platform::Can::Channel channel,
                                                           const std::array<MotorConnection, N> &connections) noexcept
    {
        return make_mock_config(channel, connections);
    }
};

template <> struct MotorAdapterFor<MotorModel::M3508>
{
    using Adapter = mock::Adapter;
    static constexpr std::size_t MAXIMUM_MOTORS{mock::Adapter::MAXIMUM_MOTORS};
    template <std::size_t N>
    [[nodiscard]] static mock::Adapter::Config make_config(platform::Can::Channel channel,
                                                           const std::array<MotorConnection, N> &connections) noexcept
    {
        return make_mock_config(channel, connections);
    }
};

template <> struct MotorAdapterFor<MotorModel::GM6020_CURRENT>
{
    using Adapter = mock::Adapter;
    static constexpr std::size_t MAXIMUM_MOTORS{mock::Adapter::MAXIMUM_MOTORS};
    template <std::size_t N>
    [[nodiscard]] static mock::Adapter::Config make_config(platform::Can::Channel channel,
                                                           const std::array<MotorConnection, N> &connections) noexcept
    {
        return make_mock_config(channel, connections);
    }
};

} // namespace device
