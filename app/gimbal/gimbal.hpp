#ifndef GIMBAL_GIMBAL_HPP
#define GIMBAL_GIMBAL_HPP

#include "driver/motor/joint.hpp"
#include "gimbal/axis_loop.hpp"
#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/command.hpp"
#include "topics/gimbal.hpp"
#include "topics/imu.hpp"
#include "utils/math/basic_math.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace gimbal
{

/**
 * @brief 云台能力：按 IMU 稳住世界系指向，跟踪上位机给的目标；yaw 与 pitch 两个关节同进同出。
 *
 * - 输入：`imu.attitude`（云台系 → 世界系的姿态、已扣零偏的角速度）与 `command.gimbal`（仲裁给的指令）；
 *   输出：两个关节的力矩（`JointMode::TORQUE`）。能力不认识电机品牌，只用 `device::Joint`。
 * - 状态机：DISABLED → ENABLING → HOLD ⇄ TRACK（自瞄）⇄ MANUAL（手动）。
 *   条件不满足（姿态无效或过期、还在 SETTLING、关节不可用或故障）立即两个一起撤销；
 *   进入 HOLD 时两个控制律与连续角度一起复位、目标取当前姿态，所以一使能不会甩；ENABLING 超时回 DISABLED 并计数。
 * - 目标：世界系 yaw / pitch。HOLD 不变；RATE 每份新姿态 `目标 += 角速度 × dt`；ABSOLUTE 朝指令目标平滑，
 *   每拍最多走 `maximum_target_rate × dt`。无限位的轴把目标展开到测量值附近，误差不绕远路；有限位的轴先在
 *   关节角里（世界系目标减去底座倾斜）绕限位中点回绕、钳到机械限位，再换回世界系，所以不会先钳后绕穿过限位。
 *   被钳住时把钳完的目标写回（否则摇杆一直往限位外推会积分饱和），前馈同时清零。
 * - 出力：两轴各一个控制律（`AxisLoop`），pitch 加 τg·cos(pitch) 的重力前馈，最后钳到总限幅。
 *   每轮必须 `set_torque()`：超过驱动的指令超时时间电机就按 0 力矩发。
 *   这一轮没有新姿态时不重复积分，重发上一拍的力矩；目标被限位钳住的轴，这一拍不加角速度前馈。
 *
 * - 观测：每轮发布 `gimbal.state`（模式、实际用的目标、测量值、yaw 关节角），仲裁与以后的底盘用。
 *
 * @note 在拥有这条 CAN 总线的任务里使用（收 → 仲裁 → 算 → 安全门 → 发）。能力零 RTOS。
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
        TRACK = 3U,    // 绝对目标（自瞄、全自动）
        MANUAL = 4U    // 手动角速度（云台自己积分）
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

    struct AxisParameters
    {
        AxisLoop::Parameters controller; // 控制律的参数，Gimbal 不看里面有什么
        bool limited;                    // 有机械限位（pitch 有；小 yaw 按线缆限位）
        float minimum_rad;
        float maximum_rad;
        float maximum_target_rate; // rad/s，目标转动速度上限：平滑目标每拍最多走这么多
    };

    /** 来自 profile 的参数（params.hpp）。 */
    struct Parameters
    {
        AxisParameters yaw;
        AxisParameters pitch;
        float pitch_gravity_newton_meter; // 云台水平时的重力矩，0 表示不补；低头方向为正，枪管一侧偏重时是负数
        float torque_limit_newton_meter;           // 总限幅，不超过关节的 maximum_torque_newton_meter()
        hal::os::Microseconds attitude_timeout_us; // 姿态这么久没更新就停
        hal::os::Microseconds enable_timeout_us;   // ENABLING 这么久没确认就回 DISABLED
        hal::os::Microseconds command_timeout_us;  // 这么久收不到 command.gimbal 就按 HOLD
    };

    /** 组合根注入：两个关节与参数。以后加大 yaw 时这里再加一个可选的指针，`JOINT_COUNT` 跟着变。 */
    struct Config
    {
        device::Joint *yaw;
        device::Joint *pitch;
        Parameters parameters;
    };

    /** 统计与诊断，调试器可读（只有运行 step() 的任务写）。 */
    struct Report
    {
        Mode mode;
        DisableReason last_disable_reason;
        // 输入：核对方向与零点就看姿态与关节这两组
        topics::imu::State imu_state;
        bool attitude_valid;
        hal::os::Microseconds last_attitude_us; // 最近一次收到新姿态的时刻（任务里的 now，判新鲜度用它）
        float yaw_rad;                          // IMU，世界系，回绕在 (−π, π]
        float yaw_continuous_rad;               // IMU，连续（不在 ±π 处跳变），控制律用的是它
        float pitch_rad;
        topics::command::GimbalMode command; // 这一轮收到的云台指令模式（超时按 HOLD）
        float yaw_target_rad; // 实际用的目标（有限位时钳在限位内，无限位时展开到测量值附近）
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

    [[nodiscard]] bool read_inputs(hal::os::Microseconds now) noexcept;
    [[nodiscard]] topics::command::GimbalMode current_command_mode(hal::os::Microseconds now) const noexcept;
    void apply_target(const topics::command::GimbalMode &command_mode) noexcept;
    [[nodiscard]] DisableReason stop_reason(hal::os::Microseconds now) const noexcept;
    [[nodiscard]] bool all_joints_enabled() const noexcept;
    [[nodiscard]] bool command_torque(float yaw_torque, float pitch_torque) noexcept;
    void anchor_yaw_continuous() noexcept;
    void disable(DisableReason reason) noexcept;
    void update_report(hal::os::Microseconds now) noexcept;

    Config config_{};
    Report report_{};
    std::array<device::Joint *, JOINT_COUNT> joints_{nullptr, nullptr};
    AxisLoop yaw_loop_{};
    AxisLoop pitch_loop_{};
    topic::Subscriber<topics::imu::Attitude> attitude_{topics::imu::attitude};
    topic::Subscriber<topics::command::Gimbal> command_{topics::command::gimbal};
    topic::Publisher<topics::gimbal::State> state_publisher_{};
    topics::imu::Attitude attitude_message_{};
    topics::command::Gimbal command_message_{};

    // 从姿态换算出来的量
    float yaw_rad_{0.0F};
    math::AngleUnwrapper yaw_unwrapper_{}; // 连续 yaw：每次复位时锚定到 yaw_rad_，之后记圈数
    float pitch_rad_{0.0F};
    float yaw_rate_{0.0F};
    float pitch_rate_{0.0F};
    float roll_rad_{0.0F};
    hal::os::Microseconds attitude_received_us_{0U};  // 最近一次收到新姿态的时刻，判新鲜度用
    hal::os::Microseconds attitude_timestamp_us_{0U}; // 最近一份姿态的采样时刻，只用来算 dt
    float dt_seconds_{0.001F};
    hal::os::Microseconds command_received_us_{0U};
    bool has_attitude_{false};
    bool has_command_{false};

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
