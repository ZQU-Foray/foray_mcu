#ifndef BRINGUP_DM_CHECK_HPP
#define BRINGUP_DM_CHECK_HPP

#include "driver/can_bus/can_bus.hpp"
#include "driver/motor/dm/dm_motor.hpp"
#include "driver/motor/joint.hpp"
#include "hal/os/clock.hpp"

#include <cstdint>

namespace bringup
{

/**
 * @brief 达妙电机检查（Decision-20261102-02）：上电后按顺序跑一遍，在真电机上验证 device::DmMotor 的回读核对、
 *        使能、MIT 指令、通讯超时与 CAN 发送。只检查驱动里的第一台电机。
 *
 * 阶段：核对 → 失能保活 → 使能 → 零力矩 → 转动（小力矩加阻尼）→ 刹车 → 通讯超时（暂停调用 process()）
 * → 恢复（电机按写进去的超时报"通讯丢失"；驱动按帧的接收时刻判出离线、撤销出力并重新核对）→ 再使能（先清错）→ 失能 → 结束。
 * 之后是 1 kHz 控制环基准（benchmark，Decision-20261102-03 的 M0）：使能后用 PD 保持位置，记录单轮耗时、周期抖动、
 * 从收到反馈到下发指令的延迟，再失能。结束或失败后一直发失能保活。
 * 每轮 step()：CanBus 收帧 → 按阶段推进、经关节下指令 → CanBus 发送（通讯超时一步两头都不做）；约 1 ms 一次。
 * 控制只经关节 Joint（Decision-20261102-03），驱动的快照只用来读核对的值与诊断。
 *
 * @note 会让电机转一小段：稳态转速不超过力矩 / Kd，电机要能自由转动。总线上没有这台电机时停在核对阶段，
 *       不发控制帧。
 */
class DmCheck final
{
  public:
    enum class Stage : std::uint8_t
    {
        VERIFY = 0U,
        IDLE = 1U,
        ENABLE = 2U,
        ZERO_TORQUE = 3U,
        SPIN = 4U,
        BRAKE = 5U,
        SILENCE = 6U,
        RECOVER = 7U,
        REENABLE = 8U,
        DISABLE = 9U,
        BENCH_ENABLE = 10U,
        BENCH = 11U,
        BENCH_DISABLE = 12U,
        DONE = 13U,
        FAILED = 14U
    };

    /** 来自 profile 的参数（params.hpp）。 */
    struct Parameters
    {
        std::uint16_t esc_id; // 接线，组合根据此构造驱动
        std::uint16_t master_id;
        bool spin;                               // false：跳过转动与刹车
        float spin_torque_newton_meter;          // 转动时的力矩
        float damping;                           // 转动与刹车时的 Kd，N·m·s/rad
        hal::os::Microseconds verify_timeout_us; // 从第一次 step() 起等核对完成
        hal::os::Microseconds idle_us;           // 核对完成后先失能保活这么久
        hal::os::Microseconds enable_timeout_us; // 使能、再使能、失能各自的时限
        hal::os::Microseconds zero_torque_us;
        hal::os::Microseconds spin_us;
        hal::os::Microseconds brake_us;
        hal::os::Microseconds silence_us; // 暂停收发的时长，要长于写进电机的通讯超时
        hal::os::Microseconds recover_timeout_us;
        // 基准：PD 保持起点位置
        bool benchmark;                       // false：失能后直接结束
        hal::os::Microseconds benchmark_us;   // 基准时长
        float hold_stiffness;                 // Kp，N·m/rad
        float hold_damping;                   // Kd，N·m·s/rad
        float hold_torque_limit_newton_meter; // 力矩限幅
    };

    /** 组合根注入：这条总线、驱动（构造时已接好关节与接线）、关节与参数。 */
    struct Config
    {
        device::CanBus *bus;
        device::DmMotor *motor;
        device::Joint *joint;
        Parameters parameters;
    };

    /** 结果与各阶段测得的值；物理量为电机输出轴，单位 rad、rad/s、N·m。 */
    struct Report
    {
        Stage stage;
        Stage failed_stage; // stage 为 FAILED 时：失败在哪一步
        bool passed;        // 必过项（状态机与时限）全部通过
        bool motion_ok;     // 转动时速度与位置都朝正方向变（跳过转动时为 false）
        bool reverified;    // 通讯超时后驱动重新核对过（因为离线），记录用，不是必过项
        std::uint8_t silence_fault_state; // 通讯超时后读到的故障状态码（应为 0xD 通讯丢失），0：没看到
        hal::os::Microseconds verified_after_us;   // 第一次 step() 到核对完成
        hal::os::Microseconds enable_latency_us;   // enable() 到电机确认使能
        hal::os::Microseconds reenable_latency_us; // 通讯超时恢复后再使能的确认时间
        hal::os::Microseconds recover_after_us;    // 恢复调用 process() 到驱动不再出力、又收到反馈
        float zero_torque_drift_rad;               // 零力矩期间离起点最远的位置差
        float spin_max_velocity_rad_per_second;
        float spin_position_delta_rad;
        float spin_torque_feedback_newton_meter; // 转动结束时反馈的力矩
        float spin_command_newton_meter;         // 转动指令的力矩（参数，供探测脚本对照）
        float spin_damping;                      // 转动指令的 Kd（参数）
        float brake_end_velocity_rad_per_second;
        // 1 kHz 控制环基准（每轮：t0 = 本轮开始，收帧，算 PD，发送）
        hal::os::Microseconds bench_duration_us; // 基准时长（参数，供探测脚本对照轮数）
        std::uint32_t bench_cycles;
        hal::os::Microseconds bench_exec_max_us; // 单轮耗时：t0 → 指令放进发送队列
        std::uint32_t bench_exec_sum_us;
        hal::os::Microseconds bench_jitter_max_us;  // 相邻两轮开始的间隔与 1 ms 之差的绝对值
        std::uint32_t bench_late_cycles;            // 间隔超过 1.1 ms 的轮数
        hal::os::Microseconds bench_latency_max_us; // 反馈的接收时刻 → 用它算出的指令放进发送队列
        std::uint32_t bench_latency_sum_us;
        std::uint32_t bench_latency_samples;
        hal::os::Microseconds bench_feedback_age_max_us; // 计算时用的反馈已经多旧
        float bench_hold_error_max_rad;
        // 驱动的当前状态
        device::DmMotor::Phase phase;
        std::uint8_t state; // 最后一帧反馈的状态码
        bool online;
        bool output_active;
        float position_rad;
        float velocity_rad_per_second;
        float torque_newton_meter;
        std::uint8_t mos_temperature_celsius;
        std::uint8_t rotor_temperature_celsius;
        std::uint32_t feedback_count;
        // 核对读到的值
        std::uint32_t control_mode;
        float position_max_rad;
        float velocity_max_rad_per_second;
        float torque_max_newton_meter;
        std::uint32_t timeout_ticks;
        std::uint8_t mismatch_register;
        // 驱动的总线统计（展开成标量，探测脚本按字段读）
        std::uint32_t tx_queued;
        std::uint32_t tx_rejected; // 没有放进发送队列的帧（队列满、Bus-Off 等）
        std::uint32_t register_replies;
        std::uint32_t stale_replies;
        std::uint32_t request_timeouts;
        std::uint32_t enable_failures;
        std::uint32_t unknown_frames;
        std::uint32_t rx_dropped;
        std::uint32_t bus_off;
    };

    /** 初始化驱动（检查配置、占住总线的接收）；驱动初始化失败时返回 false。 */
    [[nodiscard]] bool init(const Config &config) noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    void record(const device::DmMotor::Snapshot &snapshot) noexcept;
    void advance(const device::DmMotor::Snapshot &snapshot, hal::os::Microseconds now) noexcept;
    void enter(Stage stage, hal::os::Microseconds now) noexcept;
    void fail() noexcept;

    Config config_{};
    Report report_{};
    bool started_{false};
    hal::os::Microseconds start_us_{0U};
    hal::os::Microseconds stage_start_us_{0U};
    float stage_start_position_rad_{0.0F};
    std::uint32_t stage_start_feedback_count_{0U};
    // 基准
    void measure(hal::os::Microseconds now, hal::os::Microseconds computed_us) noexcept;
    void finish_measure(hal::os::Microseconds now, hal::os::Microseconds transmitted_us) noexcept;
    bool bench_previous_{false}; // 上一轮是否算过指令
    bool bench_measured_{false}; // 本轮是否计入基准
    hal::os::Microseconds bench_previous_start_us_{0U};
    hal::os::Microseconds bench_feedback_us_{0U}; // 本轮算指令时所用反馈的接收时刻
    float bench_target_rad_{0.0F};
};

} // namespace bringup

#endif // BRINGUP_DM_CHECK_HPP
