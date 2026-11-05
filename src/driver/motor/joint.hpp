#ifndef DRIVER_MOTOR_JOINT_HPP
#define DRIVER_MOTOR_JOINT_HPP

#include "hal/os/clock.hpp"

#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * 关节：一台电机折算到机构上之后的状态与指令。能力只用它，不知道电机的品牌与型号。
 *
 * - 单位一律 SI：rad、rad/s、N·m（D5）。方向、外接传动、零点在 JointMechanics 里，由组合根按 profile 给出。
 * - 位置：型号有绝对编码器（达妙、GM6020）时是绝对位置，否则从第一帧反馈起算；看 state().position_absolute（D4）。
 * - 出力要先 enable()；没有请求出力时 set_*() 返回 false。电机出故障、离线时驱动撤销请求，
 *   能力要再 enable() 才会重试。
 * - 指令超过 command_timeout_us 没有刷新，驱动按 0 力矩发（D7）。
 * - 同一个任务：关节与它所在总线的 CanBus、品牌驱动在同一个任务里使用（D3），不做跨任务同步。
 */
enum class JointMode : std::uint8_t
{
    TORQUE = 0U,   // 只给力矩：全部型号
    IMPEDANCE = 1U // 电机内阻抗：τ = Kp(p* − p) + Kd(v* − v) + τff，只有支持的型号（达妙 MIT）
};

struct JointState final
{
    float position_rad{0.0F};
    float velocity_rad_per_second{0.0F};
    float torque_newton_meter{0.0F};    // 反馈力矩（DJI 由电流 × Kt 换算）
    float temperature_celsius{0.0F};    // has_temperature 为 false 时无意义
    hal::os::Microseconds stamp_us{0U}; // 这份反馈的接收时刻（hal::Can::Frame::received_us）
    bool feedback_valid{false};         // 收到过有效反馈，上面的物理量有意义
    bool online{false};
    bool enabled{false}; // 正在出力
    bool fault{false};   // 电机报故障、设置与型号不符或使能失败
    bool position_absolute{false};
    bool has_temperature{false};
    std::uint8_t fault_code{0U}; // 品牌原始故障码（达妙的状态码），排查用
};

/** 一条指令。能力给的是关节单位；驱动从 JointLink::command() 拿到的是电机输出轴单位。 */
struct JointCommand final
{
    JointMode mode{JointMode::TORQUE};
    float torque_newton_meter{0.0F}; // 力矩，或阻抗模式的前馈
    float position_rad{0.0F};
    float velocity_rad_per_second{0.0F};
    float stiffness{0.0F}; // N·m/rad
    float damping{0.0F};   // N·m·s/rad
};

/**
 * 关节与电机输出轴之间的机构。电机输出轴转 gear_ratio rad，关节转 1 rad；zero_offset_rad 是关节零点处的电机位置。
 * q = direction × (p − zero_offset) / gear_ratio；关节力矩 = direction × 电机力矩 × gear_ratio（按理想传动）。
 */
struct JointMechanics final
{
    int direction{1}; // +1 或 −1
    float gear_ratio{1.0F};
    float zero_offset_rad{0.0F};
    hal::os::Microseconds command_timeout_us{3000U}; // 1 kHz 控制约 3 个周期；慢的控制环要放宽
};

class JointLink;

class Joint final
{
  public:
    constexpr Joint() noexcept = default;
    Joint(const Joint &) = delete;
    Joint &operator=(const Joint &) = delete;
    Joint(Joint &&) = delete;
    Joint &operator=(Joint &&) = delete;

    [[nodiscard]] const JointState &state() const noexcept
    {
        return state_;
    }

    /** 现在能不能使能：接了驱动、驱动就绪（达妙核对通过）、没有被安全门禁止。 */
    [[nodiscard]] bool available() const noexcept
    {
        return bound_ && available_ && !inhibited_;
    }

    /** 请求出力；available() 为 false 时返回 false。已经请求过的直接返回 true。 */
    [[nodiscard]] bool enable() noexcept;

    /** 撤销出力请求并清掉指令。 */
    void disable() noexcept;

    /** 力矩指令（关节 N·m）。没有请求出力、非有限值时返回 false；非有限值同时清掉指令。超出能力由驱动钳位。 */
    [[nodiscard]] bool set_torque(float newton_meter) noexcept;

    /** 阻抗指令（关节单位）。型号不支持、没有请求出力、非有限值或刚度阻尼为负时返回 false。 */
    [[nodiscard]] bool set_impedance(float position_rad,
                                     float velocity_rad_per_second,
                                     float stiffness,
                                     float damping,
                                     float torque_newton_meter) noexcept;

    [[nodiscard]] bool supports(JointMode mode) const noexcept
    {
        return bound_ && (modes_ & mode_bit(mode)) != 0U;
    }

    /** 关节侧的最大力矩（电机能力 × 外接传动比）；没接驱动时为 0。 */
    [[nodiscard]] float maximum_torque_newton_meter() const noexcept
    {
        return bound_ ? maximum_torque_newton_meter_ : 0.0F;
    }

    /** 指令超时的次数（每次从"有新指令"变成"超时"计一次）。 */
    [[nodiscard]] std::uint32_t command_timeout_count() const noexcept
    {
        return command_timeout_count_;
    }

    /** 安全门用（D7）：true 时立即撤销出力请求，之后 enable() 返回 false，直到解除。 */
    void set_inhibited(bool inhibited) noexcept;

    [[nodiscard]] bool inhibited() const noexcept
    {
        return inhibited_;
    }

  private:
    friend class JointLink;

    [[nodiscard]] static constexpr std::uint8_t mode_bit(JointMode mode) noexcept
    {
        return static_cast<std::uint8_t>(1U << static_cast<std::uint8_t>(mode));
    }
    void clear_command() noexcept;

    // 驱动写
    JointState state_{};
    JointMechanics mechanics_{};
    bool bound_{false};
    bool available_{false};
    std::uint8_t modes_{0U};
    float maximum_torque_newton_meter_{0.0F};
    // 能力与安全门写
    bool enable_requested_{false};
    bool inhibited_{false};
    bool has_command_{false};
    bool timed_out_{false};
    JointCommand command_{};
    hal::os::Microseconds command_stamp_us_{0U};
    std::uint32_t command_timeout_count_{0U};
};

/** 一份电机反馈（电机输出轴单位），驱动交给 JointLink::publish() 换算成关节单位。 */
struct MotorFeedback final
{
    float position_rad{0.0F};
    float velocity_rad_per_second{0.0F};
    float torque_newton_meter{0.0F};
    float temperature_celsius{0.0F};
    hal::os::Microseconds stamp_us{0U};
};

/**
 * 只给品牌驱动用：把关节接到一台电机上、写状态、取本周期的指令。能力不要用它。
 * 全部函数在拥有这条总线的任务里调用。
 */
class JointLink final
{
  public:
    struct Binding final
    {
        std::uint8_t modes{0U};                        // 支持的 JointMode（位）
        float motor_maximum_torque_newton_meter{0.0F}; // 电机输出轴的最大力矩
        bool position_absolute{false};
        bool has_temperature{false};
        JointMechanics mechanics{};
    };

    static constexpr std::uint8_t TORQUE_ONLY{1U << static_cast<std::uint8_t>(JointMode::TORQUE)};
    static constexpr std::uint8_t TORQUE_AND_IMPEDANCE{
        TORQUE_ONLY | static_cast<std::uint8_t>(1U << static_cast<std::uint8_t>(JointMode::IMPEDANCE))};

    /** 接上一台电机。关节已经接过、方向不是 ±1、传动比不是有限正数、零点非有限、不支持力矩时返回 false。 */
    [[nodiscard]] static bool bind(Joint &joint, const Binding &binding) noexcept;

    /** 驱动是否就绪（达妙核对通过）。变为 false 时同时撤销出力请求。 */
    static void set_available(Joint &joint, bool available) noexcept;

    /** 写一份反馈（电机输出轴单位），换算成关节单位；同时标记 feedback_valid。 */
    static void publish(Joint &joint, const MotorFeedback &feedback) noexcept;

    /** 写在线、出力、故障。 */
    static void set_status(Joint &joint, bool online, bool enabled, bool fault, std::uint8_t fault_code) noexcept;

    /** 能力是否请求出力（且没有被禁止）。 */
    [[nodiscard]] static bool enable_requested(const Joint &joint) noexcept
    {
        return joint.enable_requested_ && !joint.inhibited_;
    }

    /** 驱动撤销出力（故障、离线、使能失败）：清掉请求与指令，同时把关节状态刷成没在出力，能力要再 enable()。 */
    static void drop_enable(Joint &joint) noexcept;

    /**
     * 本周期要发的指令，换算成电机输出轴单位。没有请求出力、被禁止、还没有指令、指令超时，都给 0 力矩；
     * 超时在这里计数。now 是本周期的时刻。
     */
    [[nodiscard]] static JointCommand command(Joint &joint, hal::os::Microseconds now) noexcept;
};

/** 全组请求出力：任何一个不能使能就一个都不请求，返回 false。 */
[[nodiscard]] bool enable_all(Joint *const *joints, std::size_t count) noexcept;

/** 全组撤销出力请求。 */
void disable_all(Joint *const *joints, std::size_t count) noexcept;

} // namespace device

#endif // DRIVER_MOTOR_JOINT_HPP
