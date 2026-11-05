#ifndef COMMAND_ARBITER_HPP
#define COMMAND_ARBITER_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/command.hpp"
#include "topics/operator_input.hpp"
#include "topics/pc_link.hpp"
#include "topics/remote.hpp"
#include "topics/safety.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace command
{

/** 操作输入的来源。第二部分加 VIDEO_LINK。 */
enum class Source : std::uint8_t
{
    REMOTE = 0U
};

/** 绑定里用的控件。 */
enum class Control : std::uint8_t
{
    NONE = 0U,
    SWITCH = 1U,       // 拨杆到某一档
    KEYS = 2U,         // 键盘组合键
    MOUSE_BUTTONS = 3U // 鼠标键组合
};

/** 按下之后怎么算。 */
enum class Trigger : std::uint8_t
{
    HOLD = 0U,  // 按住生效
    TOGGLE = 1U // 按一下切换；来源断开时清零
};

/** 一条绑定：来源 + 控件 + 触发方式。功能成立 = 它的任一条绑定成立。 */
struct Binding
{
    Source source;
    Control control;
    std::uint8_t index;              // SWITCH：第几个拨杆
    topics::SwitchPosition position; // SWITCH：哪一档
    std::uint16_t mask;              // KEYS / MOUSE_BUTTONS：这些位都按下才算
    Trigger trigger;
};

using Bindings = std::array<Binding, 2>;

/** 不用的那一条。 */
constexpr Binding unbound() noexcept
{
    return Binding{Source::REMOTE, Control::NONE, 0U, topics::SwitchPosition::NONE, 0U, Trigger::HOLD};
}

/** 拨杆到某一档。 */
constexpr Binding on_switch(Source source,
                            std::uint8_t index,
                            topics::SwitchPosition position,
                            Trigger trigger = Trigger::HOLD) noexcept
{
    return Binding{source, Control::SWITCH, index, position, 0U, trigger};
}

/** 键盘组合键：mask 里的位都按下才算。 */
constexpr Binding on_keys(Source source, std::uint16_t mask, Trigger trigger = Trigger::HOLD) noexcept
{
    return Binding{source, Control::KEYS, 0U, topics::SwitchPosition::NONE, mask, trigger};
}

/** 鼠标键组合：mask 里的位都按下才算。 */
constexpr Binding on_mouse(Source source, std::uint8_t mask, Trigger trigger = Trigger::HOLD) noexcept
{
    return Binding{source, Control::MOUSE_BUTTONS, 0U, topics::SwitchPosition::NONE, mask, trigger};
}

/** 功能：顺序即 Parameters::functions 的下标。 */
enum class Function : std::uint8_t
{
    RUN = 0U,      // 放开安全门
    AUTO_AIM = 1U, // 自瞄开关
    FIRE = 2U      // 开火
};

inline constexpr std::size_t FUNCTION_COUNT{3U};

/** 手动瞄准：哪根摇杆管 yaw、哪根管 pitch，满量程对应的角速度（带符号），以及角速度上限。 */
struct ManualAim
{
    std::size_t yaw_stick;
    std::size_t pitch_stick;
    float yaw_rate_full_scale;   // rad/s
    float pitch_rate_full_scale; // rad/s
    float maximum_rate;          // rad/s，正数
};

/** 来自 profile 的参数（params.hpp）。 */
struct Parameters
{
    std::array<Bindings, FUNCTION_COUNT> functions;
    ManualAim manual;
    float engage_range_rad;                  // 自瞄接管范围，0 表示不限制
    hal::os::Microseconds input_timeout_us;  // 这么久收不到操作输入就当来源断了
    hal::os::Microseconds target_timeout_us; // PC 目标的默认有效期
};

/** 统计与诊断，调试器可读（只有运行 step() 的任务写）。 */
struct Report
{
    bool input_connected; // 操作输入连着
    bool run;             // 运行功能成立（安全门放开）
    std::uint32_t arm_sequence;
    bool auto_aim;        // 自瞄开关成立
    bool fire;            // 开火成立
    bool pc_target_valid;
    bool engaged;         // 自瞄接管中（发出的模式是 ABSOLUTE）
    float engage_error_rad; // PC 目标离当前瞄准方向多远（yaw 回绕后的绝对值与 pitch 的较大者）
    topics::command::GimbalMode mode;
    std::uint32_t release_count; // 从"没运行"变成"运行"的次数
    std::uint32_t stop_count;    // 从"运行"变成"没运行"的次数
    std::uint32_t engage_count;  // 进入自瞄接管的次数
    hal::os::Microseconds last_input_us;
};

/**
 * @brief 仲裁：把操作输入翻译成功能，发布 `safety.stop` 与 `command.gimbal`。
 *
 * - 功能（RUN、AUTO_AIM、FIRE）各有最多两条绑定，任一条成立即成立；`TOGGLE` 在控件上升沿翻转，
 *   来源断开时清零。
 * - 安全：`stop = !(连着 && RUN)`；`arm_sequence` 只在 RUN 的上升沿加 1；上电后的第一条消息与
 *   断开后重连都取基线（不算上升沿，要拨回去再拨上）。每轮都发布，安全门靠超时判断发布者挂了。
 * - 连着：收到过消息、最近一次**收到**的时刻在 `input_timeout_us` 内、消息里的 `connected` 为真。
 *   不用消息里的时间戳：别的任务填的时刻可能晚于本轮的 now，而且发布任务挂掉时最后一条消息还写着"连着"。
 * - 云台指令：不在运行 → HOLD；自瞄开着、按住开火、PC 目标有效、目标在接管范围内 → ABSOLUTE；
 *   其余 → RATE（手动角速度）。
 *
 * @note 在 gimbal 任务里、云台之前调用：急停当轮生效，自瞄目标不走长周期。
 */
class Arbiter final
{
  public:
    [[nodiscard]] bool init(const Parameters &parameters) noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    void read_inputs(hal::os::Microseconds now) noexcept;
    [[nodiscard]] bool input_connected(hal::os::Microseconds now) const noexcept;
    [[nodiscard]] bool binding_active(const Binding &binding, std::size_t function, std::size_t slot) noexcept;
    [[nodiscard]] bool pc_target_valid(hal::os::Microseconds now) const noexcept;
    [[nodiscard]] float manual_rate(std::size_t stick, float full_scale) const noexcept;
    void publish_stop(hal::os::Microseconds now) noexcept;
    void publish_command(hal::os::Microseconds now) noexcept;

    Parameters parameters_{};
    topic::Subscriber<topics::OperatorInput> remote_{topics::remote::input};
    topic::Subscriber<topics::pc_link::Control> control_{topics::pc_link::control};
    topic::Publisher<topics::safety::Stop> stop_publisher_{};
    topic::Publisher<topics::command::Gimbal> command_publisher_{};

    topics::OperatorInput input_{};
    topics::pc_link::Control control_message_{};
    hal::os::Microseconds input_received_us_{0U};
    hal::os::Microseconds control_received_us_{0U};
    bool has_input_{false};
    bool has_control_{false};
    bool connected_{false};
    bool run_{false};
    bool run_known_{false};
    bool run_last_{false};
    bool engaged_{false};
    bool was_running_{false};
    std::uint32_t arm_sequence_{0U};
    std::array<std::array<bool, 2>, FUNCTION_COUNT> binding_level_{};  // 控件上一拍的按下状态（TOGGLE 用）
    std::array<std::array<bool, 2>, FUNCTION_COUNT> binding_toggle_{}; // TOGGLE 出来的状态
    std::array<bool, FUNCTION_COUNT> function_active_{};
    Report report_{};
    bool initialized_{false};
};

} // namespace command

#endif
