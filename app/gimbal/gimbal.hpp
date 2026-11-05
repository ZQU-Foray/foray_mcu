#ifndef GIMBAL_GIMBAL_HPP
#define GIMBAL_GIMBAL_HPP

#include "driver/motor/joint.hpp"
#include "gimbal/axis_loop.hpp"
#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/imu.hpp"
#include "topics/pc_link.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace gimbal
{

/**
 * @brief 云台能力：按 IMU 稳住世界系指向，跟踪上位机给的目标；yaw 与 pitch 两个关节同进同出。
 *
 * - 输入：`imu.attitude`（云台系 → 世界系的姿态、已扣零偏的角速度）与 `pc_link.control`（第一版的目标来源）；
 *   输出：两个关节的力矩（`JointMode::TORQUE`）。能力不认识电机品牌，只用 `device::Joint`。
 * - 状态机：DISABLED → ENABLING → HOLD ⇄ TRACK。
 *   条件不满足（姿态无效或过期、还在 SETTLING、关节不可用或故障）立即两个一起撤销；
 *   进入 HOLD 时两个环清零、目标取当前姿态，所以一使能不会甩；ENABLING 超时回 DISABLED 并计数。
 * - 目标：世界系 yaw / pitch。yaw 的误差回绕（目标展开到测量值附近）；有限位的轴按"世界系 − 关节角"的偏移
 *   把目标钳在机械限位内。
 * - 出力：两轴串级（`AxisLoop`），pitch 加 τg·cos(pitch) 的重力前馈，最后钳到总限幅。
 *   每轮必须 `set_torque()`：超过驱动的指令超时时间电机就按 0 力矩发。
 *
 * @note 在拥有这条 CAN 总线的任务里使用（收 → 算 → 安全门 → 发）。能力零 RTOS。
 * @note `init()` 要用到关节的最大力矩，组合根里品牌驱动的 `init()`（把关节接上）必须排在前面。
 */
class Gimbal final
{
  public:
    enum class Mode : std::uint8_t
    {
        DISABLED = 0U, // 两个环清零，驱动发"失能"保活
        ENABLING = 1U, // 已请求出力，等两个关节都确认
        HOLD = 2U,     // 目标不变（进入时取当前姿态）
        TRACK = 3U     // 跟踪 PC 目标
    };

    /** 最近一次回 DISABLED 的原因，调试器里看。 */
    enum class DisableReason : std::uint8_t
    {
        NONE = 0U,
        IMU_INVALID = 1U,       // 还没有姿态，或姿态解算无效
        IMU_SETTLING = 2U,      // 零偏标定中（yaw 零点未定）
        IMU_STALE = 3U,         // 姿态超过 attitude_timeout_us 没更新
        JOINT_UNAVAILABLE = 4U, // 关节不可用：安全门锁住、驱动没就绪
        JOINT_FAULT = 5U,       // 关节报故障
        JOINT_DROPPED = 6U,     // 出力中有一路掉了
        ENABLE_TIMEOUT = 7U,    // ENABLING 超时没确认
        COMMAND_REJECTED = 8U   // 力矩指令被关节拒绝
    };

    /**
     * 一个环的增益（就是 `AxisLoop::Gains`，profile 里只填这几个数）：
     * integral_limit 是积分项的上限（输出单位），output_limit 是输出上限——角度环是角速度上限 rad/s，
     * 角速度环是力矩上限 N·m。换控制律时跟着 `AxisLoop` 换。
     */
    using LoopGains = AxisLoop::Gains;

    struct AxisParameters
    {
        LoopGains angle; // rad → rad/s
        LoopGains rate;  // rad/s → N·m
        bool limited;    // 有机械限位（pitch 有；小 yaw 按线缆限位）
        float minimum_rad;
        float maximum_rad;
    };

    /** 来自 profile 的参数（params.hpp）。 */
    struct Parameters
    {
        AxisParameters yaw;
        AxisParameters pitch;
        float pitch_gravity_newton_meter;          // 云台水平时的重力矩，0 表示不补
        float torque_limit_newton_meter;           // 总限幅，不超过关节的 maximum_torque_newton_meter()
        hal::os::Microseconds attitude_timeout_us; // 姿态这么久没更新就停
        hal::os::Microseconds enable_timeout_us;   // ENABLING 这么久没确认就回 DISABLED
        hal::os::Microseconds target_timeout_us;   // PC 目标的默认有效期
    };

    /** 组合根注入：两个关节与参数。以后加大 yaw 时这里再加一个可选的指针，`JOINT_COUNT` 跟着变。 */
    struct Config
    {
        device::Joint *yaw;
        device::Joint *pitch;
        Parameters parameters;
    };

    /** "选目标"的输出。以后从 pc_link.control 换成仲裁发布的 command.gimbal，只改 select_target()。 */
    struct Setpoint
    {
        float yaw_rad;
        float pitch_rad;
        float yaw_rate;   // 前馈
        float pitch_rate; // 前馈
        bool valid;
    };

    /** 统计与诊断，调试器可读（只有运行 step() 的任务写）。 */
    struct Report
    {
        Mode mode;
        DisableReason last_disable_reason;
        // 输入：第 2 步核对方向与零点就看姿态与关节这两组
        topics::imu::State imu_state;
        bool attitude_valid;
        hal::os::Microseconds last_attitude_us; // 最近一份姿态的采样时刻
        float yaw_rad;                          // IMU，世界系
        float pitch_rad;
        bool target_valid;    // PC 目标有效（TRACK）
        float yaw_target_rad; // 回绕、钳位之后实际用的目标
        float pitch_target_rad;
        // 关节
        float yaw_joint_rad;
        float pitch_joint_rad;
        bool yaw_available;
        bool pitch_available;
        bool yaw_online;
        bool pitch_online;
        bool yaw_fault;
        bool pitch_fault;
        bool yaw_enabled;
        bool pitch_enabled;
        // 出力
        float yaw_torque_newton_meter; // 实际下发的力矩
        float pitch_torque_newton_meter;
        float pitch_gravity_newton_meter; // 加在 pitch 上的重力前馈
        AxisLoop::Diagnostics yaw_diagnostics;
        AxisLoop::Diagnostics pitch_diagnostics;
        std::uint32_t enable_failure_count;
    };

    [[nodiscard]] bool init(const Config &config) noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    static constexpr std::size_t JOINT_COUNT{2U};

    void read_inputs() noexcept;
    [[nodiscard]] DisableReason blocked_reason(hal::os::Microseconds now) const noexcept;
    [[nodiscard]] bool joint_dropped() const noexcept;
    [[nodiscard]] bool all_joints_enabled() const noexcept;
    [[nodiscard]] Setpoint select_target(hal::os::Microseconds now) const noexcept;
    void disable(DisableReason reason) noexcept;
    void update_report() noexcept;

    Config config_{};
    Report report_{};
    std::array<device::Joint *, JOINT_COUNT> joints_{nullptr, nullptr};
    AxisLoop yaw_loop_{};
    AxisLoop pitch_loop_{};
    topic::Subscriber<topics::imu::Attitude> attitude_{topics::imu::attitude};
    topic::Subscriber<topics::pc_link::Control> control_{topics::pc_link::control};
    topics::imu::Attitude attitude_message_{};
    topics::pc_link::Control control_message_{};

    // 从姿态换算出来的量
    float yaw_rad_{0.0F};
    float pitch_rad_{0.0F};
    float yaw_rate_{0.0F};
    float pitch_rate_{0.0F};
    float roll_rad_{0.0F};
    hal::os::Microseconds attitude_timestamp_us_{0U};
    float dt_seconds_{0.001F};
    bool has_attitude_{false};
    bool has_control_{false};

    // 状态机与目标
    Mode mode_{Mode::DISABLED};
    DisableReason reason_{DisableReason::NONE};
    hal::os::Microseconds enable_start_us_{0U};
    float yaw_target_rad_{0.0F};
    float pitch_target_rad_{0.0F};
    float yaw_rate_feedforward_{0.0F};
    float pitch_rate_feedforward_{0.0F};
    float applied_yaw_target_rad_{0.0F};
    float applied_pitch_target_rad_{0.0F};
    std::uint32_t enable_failure_count_{0U};
    bool initialized_{false};
};

} // namespace gimbal

#endif
