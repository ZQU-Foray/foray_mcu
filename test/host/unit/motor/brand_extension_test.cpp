// 非 DJI 品牌扩展机制验证：独立最小适配器 + 虚拟型号映射，不依赖任何
// DJI 实现——本翻译单元只包含通用 MotorGroup.hpp，证明电机组与映射机制
// 品牌无关。
#include "driver/motor/motor_group.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace
{

// 虚拟品牌型号：不占用正式型号枚举值，仅在本测试内有效。
constexpr device::MotorModel K_FICTIONAL_MODEL{static_cast<device::MotorModel>(254U)};

unsigned g_inits{0};
unsigned g_processes{0};
unsigned g_clears{0};
unsigned g_writes{0};
std::array<float, 2> g_last_commands{};
bool g_init_result{true};

class FictionalAdapter final
{
  public:
    struct Config final
    {
        hal::Can bus{0U};
        std::array<device::MotorConnection, 2> connections{};
        std::size_t count{0U};
    };
    static constexpr std::size_t MAXIMUM_MOTORS{2U};

    explicit FictionalAdapter(const Config &config) noexcept : config_{config} {}
    [[nodiscard]] bool init() noexcept
    {
        ++g_inits;
        return g_init_result;
    }
    void process() noexcept
    {
        ++g_processes;
    }
    [[nodiscard]] bool read_state(std::size_t id, device::MotorState &state) const noexcept
    {
        state = {};
        if (id >= config_.count)
        {
            return false;
        }
        state.feedback_valid = true;
        state.online = true;
        return true;
    }
    [[nodiscard]] bool set_torque(std::size_t id, float value) noexcept
    {
        if (id >= config_.count || !std::isfinite(value))
        {
            return false;
        }
        ++g_writes;
        g_last_commands[id] = value;
        return true;
    }
    void clear_commands() noexcept
    {
        ++g_clears;
        g_last_commands = {};
    }

  private:
    Config config_;
};

} // namespace

namespace device
{
template <>
struct MotorAdapterFor<K_FICTIONAL_MODEL>
{
    using Adapter = FictionalAdapter;
    static constexpr std::size_t MAXIMUM_MOTORS{FictionalAdapter::MAXIMUM_MOTORS};
    template <std::size_t N>
    [[nodiscard]] static FictionalAdapter::Config
    make_config(hal::Can bus, const std::array<MotorConnection, N> &connections) noexcept
    {
        FictionalAdapter::Config config{};
        config.bus = bus;
        config.count = N;
        for (std::size_t index = 0U; index < N; ++index)
        {
            config.connections[index] = connections[index];
        }
        return config;
    }
};
} // namespace device

using Fiction = device::MotorGroup<K_FICTIONAL_MODEL, 2>;
static_assert(Fiction::count() == 2U);

int main()
{
    // 型号映射：适配器收到品牌装配参数（连接与数量）。
    Fiction fiction{hal::Can{0U}, {{{3U, 1}, {5U, -1}}}};

    // 电机组的 Init 直接交给适配器：失败照实返回，重复调用由适配器决定。
    // 总线占用由用到 CAN 的适配器自己用 hal::Can::claim_receiver() 检查（DJI 见 motor_group 测试），
    // 电机组本身不再维护占用表。
    g_init_result = false;
    Fiction failing{hal::Can{1U}, {{{1U, 1}, {2U, 1}}}};
    assert(!failing.init()); // 适配器 Init 失败
    assert(g_inits == 1U);
    g_init_result = true;
    Fiction owner{hal::Can{1U}, {{{1U, 1}, {2U, 1}}}};
    assert(owner.init() && owner.init());
    assert(g_inits == 3U);

    // 控制契约直通适配器：暂存、发送节拍、清零。
    assert(fiction.init());
    assert(fiction.set_torque(0U, 0.5F) && fiction.set_torque(1U, -0.25F));
    assert(g_last_commands[0] == 0.5F && g_last_commands[1] == -0.25F);
    fiction.process();
    assert(g_processes == 1U);
    device::MotorState state{};
    assert(fiction.read_state(0U, state) && state.online);
    assert(!fiction.read_state(2U, state)); // 越界编号
    fiction.clear_commands();
    assert(g_clears == 1U && g_last_commands == (std::array<float, 2>{}));
    return 0;
}
