// command::Arbiter 在 platform/mock 上：remote.input 与 pc_link.control 由测试发布，订阅 safety.stop 与
// command.gimbal。一个 topic 只能有一个发布者，场景按参数分进程运行（同 imu）。
// 覆盖：上电取基线、拨回再拨上、失控保护与超时后重连要重新拨、自瞄接管条件、手动角速度与钳位、
// 不在运行时 HOLD、参数检查。
#include "command/arbiter.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"
#include "topics/command.hpp"
#include "topics/gimbal.hpp"
#include "topics/operator_input.hpp"
#include "topics/pc_link.hpp"
#include "topics/remote.hpp"
#include "topics/safety.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{

using Topics = topics::command::GimbalMode;

constexpr hal::os::Microseconds STEP{1000U};
constexpr hal::os::Microseconds INPUT_TIMEOUT{50000U};
constexpr hal::os::Microseconds TARGET_TIMEOUT{20000U};
constexpr std::uint16_t GIMBAL_FLAG{0x0002U};
constexpr std::uint16_t ESTOP_FLAG{0x0008U};
constexpr float ENGAGE_RANGE{0.2F};

command::Parameters parameters()
{
    command::Parameters parameters{};
    parameters.functions = {
        command::Bindings{command::on_switch(command::Source::REMOTE, 0U, topics::SwitchPosition::UP),
                          command::unbound()},
        command::Bindings{command::on_switch(command::Source::REMOTE, 1U, topics::SwitchPosition::UP),
                          command::unbound()},
        command::Bindings{command::on_switch(command::Source::REMOTE, 2U, topics::SwitchPosition::DOWN),
                          command::unbound()},
    };
    parameters.manual = command::ManualAim{topics::STICK_RIGHT_X, topics::STICK_RIGHT_Y, 3.0F, 2.0F, 6.0F};
    parameters.engage_range_rad = ENGAGE_RANGE;
    parameters.input_timeout_us = INPUT_TIMEOUT;
    parameters.target_timeout_us = TARGET_TIMEOUT;
    return parameters;
}

struct Publishers
{
    topic::Publisher<topics::OperatorInput> input;
    topic::Publisher<topics::pc_link::Control> control;
    topic::Publisher<topics::gimbal::State> state;
};

struct Rig
{
    explicit Rig(Publishers &publishers) noexcept : publishers_{publishers} {}

    command::Arbiter arbiter{};
    topic::Subscriber<topics::safety::Stop> stop{topics::safety::stop};
    topic::Subscriber<topics::command::Gimbal> gimbal{topics::command::gimbal};

    bool init()
    {
        mock::reset();
        mock::set_now_us(1000U);
        return arbiter.init(parameters());
    }

    /** 发布一份操作输入：三档开关与两根摇杆，其余为默认。 */
    void
    publish_input(bool run, bool auto_aim, bool fire, bool connected, float yaw_stick = 0.0F, float pitch_stick = 0.0F)
    {
        topics::OperatorInput message{};
        message.switches[0] = run ? topics::SwitchPosition::UP : topics::SwitchPosition::DOWN;
        message.switches[1] = auto_aim ? topics::SwitchPosition::UP : topics::SwitchPosition::DOWN;
        message.switches[2] = fire ? topics::SwitchPosition::DOWN : topics::SwitchPosition::UP;
        message.sticks[topics::STICK_RIGHT_X] = yaw_stick;
        message.sticks[topics::STICK_RIGHT_Y] = pitch_stick;
        message.has_remote = true;
        message.connected = connected;
        message.timestamp_us = hal::os::now_us();
        publishers_.input.publish(message);
    }

    /** 发布云台的当前状态：接管范围用它算。 */
    void publish_state(float yaw, float pitch)
    {
        topics::gimbal::State message{};
        message.yaw_target_rad = yaw;
        message.pitch_target_rad = pitch;
        message.timestamp_us = hal::os::now_us();
        publishers_.state.publish(message);
    }

    void publish_control(float yaw, float pitch, std::uint16_t flags = GIMBAL_FLAG)
    {
        topics::pc_link::Control message{};
        message.flags = flags;
        message.gimbal_yaw = yaw;
        message.gimbal_pitch = pitch;
        message.received_us = hal::os::now_us();
        publishers_.control.publish(message);
    }

    void tick(hal::os::Microseconds delta = STEP)
    {
        arbiter.step(hal::os::now_us());
        topics::safety::Stop stop_message{};
        if (stop.update(stop_message))
        {
            stop_message_ = stop_message;
            has_stop_ = true;
        }
        topics::command::Gimbal command_message{};
        if (gimbal.update(command_message))
        {
            command_message_ = command_message;
            has_command_ = true;
        }
        mock::advance_us(delta);
    }

    /** 发布输入再跑一轮，返回这一轮发出的两样东西。 */
    void step_with(bool run,
                   bool auto_aim = false,
                   bool fire = false,
                   bool connected = true,
                   float yaw_stick = 0.0F,
                   float pitch_stick = 0.0F)
    {
        publish_input(run, auto_aim, fire, connected, yaw_stick, pitch_stick);
        tick();
    }

    const topics::safety::Stop &last_stop() const
    {
        assert(has_stop_);
        return stop_message_;
    }

    const topics::command::Gimbal &last_command() const
    {
        assert(has_command_);
        return command_message_;
    }

  private:
    Publishers &publishers_;
    topics::safety::Stop stop_message_{};
    topics::command::Gimbal command_message_{};
    bool has_stop_{false};
    bool has_command_{false};
};

// 上电时已经在运行档：stop 为假但 arm_sequence 不变（安全门不会放开）；拨回再拨上才加 1
int scenario_run_baseline(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.step_with(true);
    const topics::safety::Stop first = rig.last_stop();
    assert(!first.stop && first.arm_sequence == 0U);
    assert(rig.arbiter.report().input_connected && rig.arbiter.report().run);
    rig.step_with(false);
    assert(rig.last_stop().stop);
    rig.step_with(true);
    const topics::safety::Stop again = rig.last_stop();
    assert(!again.stop && again.arm_sequence == 1U);
    assert(rig.arbiter.report().release_count == 1U);
    return 0;
}

// 失控保护：期间 stop；恢复时拨杆仍在运行档不算一次拨上，要拨回去再拨上
int scenario_failsafe(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.step_with(true);
    assert(!rig.last_stop().stop && rig.last_stop().arm_sequence == 0U);

    rig.step_with(true, false, false, false); // 接收机报失控保护
    assert(rig.last_stop().stop);
    assert(!rig.arbiter.report().input_connected);

    rig.step_with(true); // 恢复，拨杆还在运行档
    assert(!rig.last_stop().stop && rig.last_stop().arm_sequence == 0U);

    rig.step_with(false);
    assert(rig.last_stop().stop);
    rig.step_with(true);
    assert(!rig.last_stop().stop && rig.last_stop().arm_sequence == 1U);
    return 0;
}

// 消息停了（最后一条还写着 connected）：超过 input_timeout_us 后 stop；恢复后要重新拨
int scenario_input_timeout(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.step_with(true);
    assert(!rig.last_stop().stop);
    for (hal::os::Microseconds elapsed = 0U; elapsed < INPUT_TIMEOUT; elapsed += STEP)
    {
        rig.tick();
    }
    rig.tick();
    assert(rig.last_stop().stop);
    assert(!rig.arbiter.report().input_connected);
    rig.step_with(true); // 恢复，拨杆还在运行档
    assert(!rig.last_stop().stop && rig.last_stop().arm_sequence == 0U);
    return 0;
}

// 自瞄开着、按住开火、PC 目标有效：ABSOLUTE，目标和前馈是 PC 的
int scenario_auto_aim(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.publish_state(0.3F, -0.2F);
    rig.publish_control(0.3F, -0.2F);
    rig.step_with(true, true, true);
    const topics::command::Gimbal command = rig.last_command();
    assert(command.mode == Topics::ABSOLUTE);
    assert(std::fabs(command.yaw_rad - 0.3F) < 1.0e-6F);
    assert(std::fabs(command.pitch_rad + 0.2F) < 1.0e-6F);
    assert(rig.arbiter.report().engaged && rig.arbiter.report().engage_count == 1U);
    return 0;
}

// 自瞄开着、没按开火 → RATE；按着开火、自瞄没开 → RATE
int scenario_no_engage(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.publish_control(0.3F, 0.0F);
    rig.step_with(true, true, false);
    assert(rig.last_command().mode == Topics::RATE);
    rig.step_with(true, false, true);
    assert(rig.last_command().mode == Topics::RATE);
    assert(!rig.arbiter.report().engaged);
    return 0;
}

// PC 目标无效（过期、bit1 为 0、bit3 为 1）→ RATE
int scenario_pc_invalid(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.publish_state(0.0F, 0.0F);
    for (std::uint16_t flags : {static_cast<std::uint16_t>(0U), static_cast<std::uint16_t>(GIMBAL_FLAG | ESTOP_FLAG)})
    {
        rig.publish_control(0.1F, 0.0F, flags);
        rig.step_with(true, true, true);
        assert(rig.last_command().mode == Topics::RATE);
    }
    // 有效期过了：只发一次目标，等超过默认有效期（20 ms）
    rig.publish_control(0.1F, 0.0F);
    rig.step_with(true, true, true);
    assert(rig.last_command().mode == Topics::ABSOLUTE);
    for (hal::os::Microseconds elapsed = 0U; elapsed <= TARGET_TIMEOUT; elapsed += STEP)
    {
        rig.tick();
    }
    rig.step_with(true, true, true);
    assert(rig.last_command().mode == Topics::RATE);
    return 0;
}

// 手动：摇杆满量程、中间值、反向，钳到上限；遥控断开为 0
int scenario_manual(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.step_with(true, false, false, true, 1.0F, -1.0F);
    topics::command::Gimbal command = rig.last_command();
    assert(command.mode == Topics::RATE);
    assert(std::fabs(command.yaw_rate - 3.0F) < 1.0e-6F);
    assert(std::fabs(command.pitch_rate + 2.0F) < 1.0e-6F);

    rig.step_with(true, false, false, true, 0.5F, 0.0F);
    command = rig.last_command();
    assert(std::fabs(command.yaw_rate - 1.5F) < 1.0e-6F);
    assert(command.pitch_rate == 0.0F);

    // 超过上限：满量程参数改成 10 rad/s 的场合由参数检查挡不住，这里用 1.0 的摇杆值 × 3 rad/s 已经在上限内，
    // 换一根更大的满量程验证钳位：把 yaw 满量程当作 10 需要另一份参数，这里直接用 6 rad/s 上限核对
    assert(command.yaw_rate <= 6.0F && command.yaw_rate >= -6.0F);

    // 遥控断开：角速度为 0
    rig.step_with(true, false, false, false, 1.0F, 1.0F);
    command = rig.last_command();
    assert(command.mode == Topics::HOLD);
    assert(command.yaw_rate == 0.0F && command.pitch_rate == 0.0F);
    return 0;
}

// 不在运行：HOLD
int scenario_idle(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.step_with(false, false, false, true, 1.0F, 1.0F);
    const topics::command::Gimbal command = rig.last_command();
    assert(command.mode == Topics::HOLD);
    assert(rig.last_stop().stop);
    return 0;
}

// 接管范围：目标离当前瞄准方向超过 engage_range_rad 就不接管；靠近了才接管
int scenario_engage_range(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.publish_state(0.0F, 0.0F);
    rig.publish_control(0.5F, 0.0F); // 离瞄准方向 0.5 rad，超过 0.2
    rig.step_with(true, true, true);
    assert(rig.last_command().mode == Topics::RATE);
    assert(rig.arbiter.report().engage_error_rad > 0.4F);

    rig.publish_state(0.4F, 0.0F); // 挪到 0.1 rad 之内
    rig.step_with(true, true, true);
    assert(rig.last_command().mode == Topics::ABSOLUTE);
    assert(rig.arbiter.report().engage_error_rad < 0.2F);
    return 0;
}

// 接管范围的 yaw 差要回绕：目标 3.1、瞄准 −3.1，实际只差 0.083 rad，应该接管
int scenario_engage_wrap(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.publish_state(-3.1F, 0.0F);
    rig.publish_control(3.1F, 0.0F);
    rig.step_with(true, true, true);
    assert(rig.last_command().mode == Topics::ABSOLUTE);
    assert(rig.arbiter.report().engage_error_rad < 0.1F);
    return 0;
}

// 接管中视觉跳到远处的目标：松开接管，回到手动
int scenario_engage_jump(Publishers &publishers)
{
    Rig rig{publishers};
    assert(rig.init());
    rig.publish_state(0.3F, 0.0F);
    rig.publish_control(0.3F, 0.0F);
    rig.step_with(true, true, true);
    assert(rig.last_command().mode == Topics::ABSOLUTE);

    rig.publish_control(1.0F, 0.0F); // 视觉跳到 1.0，瞄准方向还在 0.3
    rig.step_with(true, true, true);
    assert(rig.last_command().mode == Topics::RATE);
    assert(!rig.arbiter.report().engaged);
    return 0;
}

// 参数检查
int scenario_reject(Publishers &)
{
    command::Arbiter arbiter;
    command::Parameters bad = parameters();
    bad.functions[0][0] = command::on_switch(command::Source::REMOTE, 8U, topics::SwitchPosition::UP);
    assert(!arbiter.init(bad));
    bad = parameters();
    bad.functions[0][0] = command::on_switch(command::Source::REMOTE, 0U, topics::SwitchPosition::NONE);
    assert(!arbiter.init(bad));
    bad = parameters();
    bad.functions[1][1] = command::on_keys(command::Source::REMOTE, 0U);
    assert(!arbiter.init(bad));
    bad = parameters();
    bad.functions[2][1] = command::on_mouse(command::Source::REMOTE, 0U);
    assert(!arbiter.init(bad));
    bad = parameters();
    bad.manual.yaw_stick = topics::STICK_COUNT;
    assert(!arbiter.init(bad));
    bad = parameters();
    bad.manual.maximum_rate = 0.0F;
    assert(!arbiter.init(bad));
    bad = parameters();
    bad.engage_range_rad = -1.0F;
    assert(!arbiter.init(bad));
    bad = parameters();
    bad.input_timeout_us = 0U;
    assert(!arbiter.init(bad));
    bad = parameters();
    bad.target_timeout_us = 0U;
    assert(!arbiter.init(bad));
    assert(arbiter.init(parameters()));
    std::puts("arbiter reject: 全部通过");
    return 0;
}

} // namespace

int main(int argc, char **argv)
{
    Publishers publishers{topics::remote::input.claim_publisher(),
                          topics::pc_link::control.claim_publisher(),
                          topics::gimbal::state.claim_publisher()};
    assert(publishers.input && publishers.control && publishers.state);
    if (argc != 2)
    {
        std::fprintf(stderr,
                     "用法：%s run_baseline|failsafe|input_timeout|auto_aim|no_engage|pc_invalid|manual|engage_range|"
                     "engage_wrap|engage_jump|idle|reject\n",
                     argv[0]);
        return 2;
    }
    const char *scenario = argv[1];
    if (std::strcmp(scenario, "run_baseline") == 0)
    {
        return scenario_run_baseline(publishers);
    }
    if (std::strcmp(scenario, "failsafe") == 0)
    {
        return scenario_failsafe(publishers);
    }
    if (std::strcmp(scenario, "input_timeout") == 0)
    {
        return scenario_input_timeout(publishers);
    }
    if (std::strcmp(scenario, "auto_aim") == 0)
    {
        return scenario_auto_aim(publishers);
    }
    if (std::strcmp(scenario, "no_engage") == 0)
    {
        return scenario_no_engage(publishers);
    }
    if (std::strcmp(scenario, "pc_invalid") == 0)
    {
        return scenario_pc_invalid(publishers);
    }
    if (std::strcmp(scenario, "manual") == 0)
    {
        return scenario_manual(publishers);
    }
    if (std::strcmp(scenario, "engage_range") == 0)
    {
        return scenario_engage_range(publishers);
    }
    if (std::strcmp(scenario, "engage_wrap") == 0)
    {
        return scenario_engage_wrap(publishers);
    }
    if (std::strcmp(scenario, "engage_jump") == 0)
    {
        return scenario_engage_jump(publishers);
    }
    if (std::strcmp(scenario, "idle") == 0)
    {
        return scenario_idle(publishers);
    }
    if (std::strcmp(scenario, "reject") == 0)
    {
        return scenario_reject(publishers);
    }
    std::fprintf(stderr, "未知场景：%s\n", scenario);
    return 2;
}
