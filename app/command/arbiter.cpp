#include "command/arbiter.hpp"

#include "utils/math/basic_math.hpp"

#include <cmath>

namespace command
{

namespace
{

// pc_link.control.flags 的 bit1：云台子系统的字段有效
constexpr std::uint16_t GIMBAL_FLAG{0x0002U};
// flags 的 bit3：PC 请求急停。只当"PC 目标无效"处理，不当急停：急停走遥控 → 安全门
constexpr std::uint16_t ESTOP_REQUEST_FLAG{0x0008U};
// PC 目标有效期的上限：协议的 validity_hint_ms 不超过 50 ms，参数给大了也按这个上限
constexpr hal::os::Microseconds TARGET_AGE_LIMIT_US{200000U};

bool binding_valid(const Binding &binding) noexcept
{
    if (binding.source != Source::REMOTE)
    {
        return false;
    }
    switch (binding.control)
    {
    case Control::NONE:
        return true;
    case Control::SWITCH:
        return binding.index < topics::SWITCH_COUNT && binding.position != topics::SwitchPosition::NONE;
    case Control::KEYS:
        return binding.mask != 0U;
    case Control::MOUSE_BUTTONS:
        return binding.mask != 0U && binding.mask <= 0xFFU;
    }
    return false;
}

bool parameters_valid(const Parameters &parameters) noexcept
{
    for (const Bindings &bindings : parameters.functions)
    {
        for (const Binding &binding : bindings)
        {
            if (!binding_valid(binding))
            {
                return false;
            }
        }
    }
    const ManualAim &manual = parameters.manual;
    if (manual.yaw_stick >= topics::STICK_COUNT || manual.pitch_stick >= topics::STICK_COUNT)
    {
        return false;
    }
    if (!std::isfinite(manual.yaw_rate_full_scale) || !std::isfinite(manual.pitch_rate_full_scale) ||
        !std::isfinite(manual.maximum_rate) || manual.maximum_rate <= 0.0F)
    {
        return false;
    }
    if (!std::isfinite(parameters.engage_range_rad) || parameters.engage_range_rad < 0.0F)
    {
        return false;
    }
    return parameters.input_timeout_us != 0U && parameters.target_timeout_us != 0U;
}

} // namespace

bool Arbiter::init(const Parameters &parameters) noexcept
{
    initialized_ = false;
    if (!parameters_valid(parameters))
    {
        return false;
    }
    parameters_ = parameters;
    report_ = Report{};
    input_ = {};
    control_message_ = {};
    gimbal_state_message_ = {};
    input_received_us_ = 0U;
    control_received_us_ = 0U;
    has_input_ = false;
    has_control_ = false;
    has_gimbal_state_ = false;
    connected_ = false;
    run_ = false;
    run_known_ = false;
    run_last_ = false;
    engaged_ = false;
    was_running_ = false;
    arm_sequence_ = 0U;
    binding_level_ = {};
    binding_toggle_ = {};
    function_active_ = {};
    stop_publisher_ = topics::safety::stop.claim_publisher();
    command_publisher_ = topics::command::gimbal.claim_publisher();
    if (!stop_publisher_ || !command_publisher_)
    {
        return false;
    }
    initialized_ = true;
    return true;
}

void Arbiter::step(hal::os::Microseconds now) noexcept
{
    if (!initialized_)
    {
        return;
    }
    read_inputs(now);
    connected_ = input_connected(now);
    if (!connected_)
    {
        // 断开：切换出来的状态清零、控件上一拍的状态清掉，重连时重新取基线
        for (auto &levels : binding_level_)
        {
            levels.fill(false);
        }
        for (auto &toggles : binding_toggle_)
        {
            toggles.fill(false);
        }
        run_known_ = false;
    }

    for (std::size_t function = 0U; function < FUNCTION_COUNT; ++function)
    {
        bool active = false;
        for (std::size_t slot = 0U; slot < parameters_.functions[function].size(); ++slot)
        {
            active = binding_active(parameters_.functions[function][slot], function, slot) || active;
        }
        function_active_[function] = active;
    }

    run_ = connected_ && function_active_[static_cast<std::size_t>(Function::RUN)];
    if (!connected_)
    {
        // 断开期间不取基线：等重新连上来的第一轮再取（这时看到的才是恢复时的状态）
        run_last_ = false;
    }
    else if (!run_known_)
    {
        run_known_ = true; // 上电第一条消息、断开后重连：取基线，不算上升沿
        run_last_ = run_;
    }
    else
    {
        if (run_ && !run_last_)
        {
            ++arm_sequence_;
            ++report_.release_count;
        }
        run_last_ = run_;
    }
    if (was_running_ && !run_)
    {
        ++report_.stop_count;
    }
    was_running_ = run_;

    publish_stop(now);
    publish_command(now);

    report_.input_connected = connected_;
    report_.run = run_;
    report_.arm_sequence = arm_sequence_;
    report_.auto_aim = function_active_[static_cast<std::size_t>(Function::AUTO_AIM)];
    report_.fire = function_active_[static_cast<std::size_t>(Function::FIRE)];
    report_.pc_target_valid = pc_target_valid(now);
    report_.last_input_us = input_received_us_;
}

void Arbiter::read_inputs(hal::os::Microseconds now) noexcept
{
    topics::OperatorInput input{};
    if (remote_.update(input))
    {
        input_ = input;
        input_received_us_ = now; // 新鲜度按"这一轮收到的时刻"算，不用消息里的时间戳
        has_input_ = true;
    }
    topics::pc_link::Control control{};
    if (control_.update(control))
    {
        control_message_ = control;
        control_received_us_ = now;
        has_control_ = true;
    }
    topics::gimbal::State state{};
    if (gimbal_state_.update(state))
    {
        gimbal_state_message_ = state;
        has_gimbal_state_ = true;
    }
}

bool Arbiter::input_connected(hal::os::Microseconds now) const noexcept
{
    return has_input_ && !hal::os::has_elapsed(input_received_us_, parameters_.input_timeout_us, now) &&
           input_.connected;
}

bool Arbiter::binding_active(const Binding &binding, std::size_t function, std::size_t slot) noexcept
{
    bool pressed = false;
    switch (binding.control)
    {
    case Control::NONE:
        return false;
    case Control::SWITCH:
        pressed =
            connected_ && binding.index < topics::SWITCH_COUNT && input_.switches[binding.index] == binding.position;
        break;
    case Control::KEYS:
        pressed = connected_ && (input_.keys & binding.mask) == binding.mask;
        break;
    case Control::MOUSE_BUTTONS:
        pressed = connected_ && (input_.mouse_buttons & binding.mask) == binding.mask;
        break;
    }
    if (binding.trigger == Trigger::HOLD)
    {
        return pressed;
    }
    bool &level = binding_level_[function][slot];
    bool &toggled = binding_toggle_[function][slot];
    if (pressed && !level)
    {
        toggled = !toggled; // 上升沿翻转
    }
    level = pressed;
    return toggled;
}

bool Arbiter::pc_target_valid(hal::os::Microseconds now) const noexcept
{
    if (!has_control_ || (control_message_.flags & GIMBAL_FLAG) == 0U ||
        (control_message_.flags & ESTOP_REQUEST_FLAG) != 0U)
    {
        return false;
    }
    hal::os::Microseconds validity = parameters_.target_timeout_us;
    if (control_message_.validity_hint_ms != 0U)
    {
        validity = static_cast<hal::os::Microseconds>(control_message_.validity_hint_ms) *
                   hal::os::MICROSECONDS_PER_MILLISECOND;
    }
    validity = validity < TARGET_AGE_LIMIT_US ? validity : TARGET_AGE_LIMIT_US;
    return !hal::os::has_elapsed(control_received_us_, validity, now);
}

float Arbiter::engage_error() const noexcept
{
    if (!has_gimbal_state_)
    {
        return 0.0F;
    }
    const float yaw_error =
        std::fabs(math::wrap_pi(control_message_.gimbal_yaw - gimbal_state_message_.yaw_target_rad));
    const float pitch_error = std::fabs(control_message_.gimbal_pitch - gimbal_state_message_.pitch_target_rad);
    return yaw_error > pitch_error ? yaw_error : pitch_error;
}

float Arbiter::manual_rate(std::size_t stick, float full_scale) const noexcept
{
    if (!connected_ || stick >= topics::STICK_COUNT)
    {
        return 0.0F;
    }
    return math::limit(
        input_.sticks[stick] * full_scale, -parameters_.manual.maximum_rate, parameters_.manual.maximum_rate);
}

void Arbiter::publish_stop(hal::os::Microseconds now) noexcept
{
    topics::safety::Stop message{};
    message.stop = !run_;
    message.arm_sequence = arm_sequence_;
    message.timestamp_us = now;
    stop_publisher_.publish(message);
}

void Arbiter::publish_command(hal::os::Microseconds now) noexcept
{
    topics::command::Gimbal command{};
    command.mode = topics::command::GimbalMode::HOLD;
    command.yaw_rad = 0.0F;
    command.pitch_rad = 0.0F;
    command.yaw_rate = 0.0F;
    command.pitch_rate = 0.0F;
    command.timestamp_us = now;

    bool engaged = false;
    if (run_)
    {
        const float distance = engage_error();
        const bool in_range =
            parameters_.engage_range_rad <= 0.0F || (has_gimbal_state_ && distance <= parameters_.engage_range_rad);
        engaged = function_active_[static_cast<std::size_t>(Function::AUTO_AIM)] &&
                  function_active_[static_cast<std::size_t>(Function::FIRE)] && pc_target_valid(now) && in_range;
        if (engaged)
        {
            command.mode = topics::command::GimbalMode::ABSOLUTE;
            command.yaw_rad = control_message_.gimbal_yaw;
            command.pitch_rad = control_message_.gimbal_pitch;
            command.yaw_rate = control_message_.gimbal_yaw_rate;
            command.pitch_rate = control_message_.gimbal_pitch_rate;
        }
        else
        {
            command.mode = topics::command::GimbalMode::RATE;
            command.yaw_rate = manual_rate(parameters_.manual.yaw_stick, parameters_.manual.yaw_rate_full_scale);
            command.pitch_rate = manual_rate(parameters_.manual.pitch_stick, parameters_.manual.pitch_rate_full_scale);
        }
    }
    if (engaged && !engaged_)
    {
        ++report_.engage_count;
    }
    engaged_ = engaged;
    report_.engaged = engaged;
    report_.engage_error_rad = engage_error();
    report_.mode = command.mode;
    command_publisher_.publish(command);
}

} // namespace command
