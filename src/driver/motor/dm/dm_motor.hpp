#ifndef DRIVER_MOTOR_DM_DM_MOTOR_HPP
#define DRIVER_MOTOR_DM_DM_MOTOR_HPP

#include "driver/protocol/dm/dm_esc.hpp"
#include "hal/can.hpp"
#include "hal/os/clock.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief 达妙一体化电机的总线会话（MIT 模式，Decision-20261102-01）：一条 CAN 总线上最多 4 台。
 *        启动时回读核对电机里的设置，核对通过后按请求使能、发 MIT 指令、解析反馈，监视离线与故障。
 *        编解码全部委托 protocol::DmEsc。
 *
 * 每台电机的阶段：
 * - VERIFYING：读控制模式 → 按读到的模式发"失能"并等一帧反馈 → 读 PMAX、VMAX、TMAX → 把档案的通讯超时
 *   写进电机 RAM 并核对回显。每一步 REQUEST_TIMEOUT_US 内没有应答就重发，一直重试（电机晚上电也能接上）；
 * - READY：模式是 MIT，量程与超时都与档案一致。没有请求出力时每周期发"失能"作保活（取得反馈，并确保电机
 *   不出力）；enable() 之后先发一次"清除错误"，再每周期发"使能"，直到反馈确认使能，之后每周期发 MIT 指令；
 * - MISMATCH：模式不是 MIT，或量程、超时与档案不符。不发任何帧，不允许使能；重新 init() 才会再核对。
 *
 * READY 时出现下列情况就撤销出力请求，驱动不自动重新使能，由持有它的能力决定：
 * - 反馈报故障（过温、欠压、通讯丢失等）；
 * - 使能后反馈变成"失能"（电机复位）：同时重新核对，电机重启后 RAM 里写的超时已经丢了；
 * - LINK_TIMEOUT_US 没有反馈（离线）：同时重新核对；
 * - enable() 之后 ENABLE_TIMEOUT_US 内没有确认使能：记为使能失败。
 *
 * 一帧属于哪台电机：帧 ID 等于它的 MST_ID，且寄存器应答的 D0–D1 等于 ESC_ID，或反馈 D0 的低 4 位等于 ESC_ID。
 * 所以几台电机共用 MST_ID 也能分开（推荐仍按 ESC_ID + 0x10 设置）。
 *
 * @note 全部接口在同一个任务里调用，process() 约 1 ms 一次：它是这条总线软件接收队列的唯一消费者
 *       （init() 用 hal::Can::claim_receiver() 占住总线），也决定发送节拍。READY 时每台电机每次 process()
 *       发一帧、收一帧反馈，1 Mbps、1 kHz 时每台约占 26% 总线负载，一条总线建议不超过 3 台。
 * @note 驱动不做跨任务同步：电机状态要给别的能力用，由持有驱动的能力发布 topic（架构 §3.4、§3.5）。
 * @note 位置在 ±PMAX 处回绕，驱动按 65536 个计数累计多圈，前提是相邻两帧之间转过不到 PMAX。电机掉电重启后
 *       位置从 [−π, π] 重新开始，累计值可能差 2π 的整数倍；离线期间的运动不做推断。
 */
class DmMotor final
{
  public:
    static constexpr std::size_t MAXIMUM_MOTORS{4U};
    static constexpr std::uint16_t MAXIMUM_ESC_ID{15U};                // 反馈帧 D0 只有低 4 位放 ESC_ID
    static constexpr std::uint16_t MAXIMUM_MASTER_ID{0x7FEU};          // 0x7FF 是寄存器请求的帧 ID
    static constexpr hal::os::Microseconds REQUEST_TIMEOUT_US{20000U}; // 核对时一步没有应答就重发
    static constexpr hal::os::Microseconds ENABLE_TIMEOUT_US{50000U};  // enable() 后这么久没确认就放弃
    static constexpr hal::os::Microseconds ENABLE_CONFIRM_GUARD_US{2000U}; // 清除错误后这段时间内的反馈不算确认
    static constexpr hal::os::Microseconds LINK_TIMEOUT_US{50000U}; // READY 时这么久没有反馈即离线
    static constexpr hal::os::Microseconds TIMEOUT_TICK_US{50U};    // 通讯超时寄存器（0x09）的计数周期

    /** 型号档案：电机里必须是什么设置，以及写进电机 RAM 的通讯超时。 */
    struct Profile final
    {
        protocol::DmEsc::Limits limits;                 // PMAX、VMAX、TMAX：核对用，也是编解码的量程
        float peak_torque_newton_meter;                 // 型号的峰值扭矩（说明书"特征参数"）
        hal::os::Microseconds communication_timeout_us; // 使能后这么久收不到指令，电机自己失能并报"通讯丢失"

        /** DM-J4310-2EC V1.2：出厂量程 12.5 rad、30 rad/s、10 N·m（2026-11-01 回读），峰值扭矩 12.5 N·m。 */
        [[nodiscard]] static constexpr Profile j4310() noexcept
        {
            return {{12.5F, 30.0F, 10.0F}, 12.5F, 50000U};
        }
    };

    struct Motor final
    {
        std::uint16_t esc_id{0U};    // 1–15，组内互不相同
        std::uint16_t master_id{0U}; // MST_ID：反馈与应答的帧 ID，低 8 位不能等于组内任何 ESC_ID
    };

    struct Config final
    {
        hal::Can bus; // 组合根注入的总线句柄
        Profile profile;
        std::array<Motor, MAXIMUM_MOTORS> motors{};
        std::size_t count{0U};
    };

    enum class Phase : std::uint8_t
    {
        STOPPED = 0U, // 还没有 init()，或 init() 失败
        VERIFYING = 1U,
        READY = 2U,
        MISMATCH = 3U
    };

    /** 单台电机的快照。物理量指电机输出轴，单位 rad、rad/s、N·m，没有反馈时为 0。 */
    struct Snapshot final
    {
        Motor motor{};
        Phase phase{Phase::STOPPED};
        bool output_requested{false}; // enable() 之后、撤销之前
        bool output_active{false};    // 电机已确认使能，正在发 MIT 指令
        bool enable_failed{false};    // 最近一次 enable() 没有成功：超时未确认，或电机报故障
        bool online{false};           // READY，且 LINK_TIMEOUT_US 内收到过这台电机的帧
        bool has_feedback{false};     // 核对通过后收到过反馈，下面的物理量有效
        hal::os::Microseconds last_update_us{0U}; // 最后一次收到这台电机的帧（反馈或应答）
        std::uint32_t feedback_count{0U};
        protocol::DmEsc::State state{protocol::DmEsc::State::DISABLED}; // 最后一帧反馈的状态
        float position_rad{0.0F};                                       // 多圈累计的位置
        float position_from_first_feedback_rad{0.0F};                   // 从核对后第一帧反馈起转过的角度
        float velocity_rad_per_second{0.0F};
        float torque_newton_meter{0.0F};
        std::uint8_t mos_temperature_celsius{0U};
        std::uint8_t rotor_temperature_celsius{0U};
        // 核对时读到的值（还没读到时为 0），核对不符时用来排查
        std::uint32_t control_mode{0U};
        protocol::DmEsc::Limits limits{};
        std::uint32_t timeout_ticks{0U};
        std::uint8_t mismatch_register{0U}; // MISMATCH 时第一个不符的寄存器地址
    };

    /**
     * @brief 总线级统计。tx_result_counts 按 hal::Can::SendResult 的值索引。
     */
    struct Statistics final
    {
        std::array<std::uint32_t, hal::Can::SEND_RESULT_COUNT> tx_result_counts{};
        std::uint32_t feedback_count{0U};
        std::uint32_t register_reply_count{0U}; // 收到的寄存器应答，含下面丢弃的重复应答
        std::uint32_t stale_reply_count{0U}; // 不在等待中的寄存器应答（重发造成的重复应答），丢弃
        std::uint32_t request_timeout_count{0U}; // 核对时请求超时而重发的次数
        std::uint32_t enable_failure_count{0U};
        std::uint32_t unknown_frame_count{0U}; // 不属于组内任何电机的帧
        std::uint32_t rx_dropped_count{0U};
        std::uint32_t rx_hardware_loss_event_count{0U};
        std::uint32_t bus_off_count{0U};
    };

    explicit DmMotor(const Config &config) noexcept;

    /**
     * @brief 检查配置、占住总线的接收，全部电机从头开始核对。调度器启动前调用。
     * @return 配置有效、总线就绪且接收未被别人占用时返回 true；失败时不占用总线。
     *         可以重复调用：复位全部状态并重新核对（核对不符的电机改好设置后用它重试）。
     */
    [[nodiscard]] bool init() noexcept;

    /** @brief 周期服务：排空接收队列，推进核对、使能与指令发送，读取总线统计。 */
    void process() noexcept;

    [[nodiscard]] std::size_t count() const noexcept
    {
        return config_.count;
    }

    /** @brief 这台电机现在能不能使能：已初始化且核对通过（READY）。 */
    [[nodiscard]] bool can_enable(std::size_t index) const noexcept;

    /**
     * @brief 请求出力：不是 READY 时返回 false。已经请求过的不重复清错。
     * @note 指令清零；之后的 process() 先清除错误、再使能，确认使能后才发指令。
     */
    [[nodiscard]] bool enable(std::size_t index) noexcept;

    /** @brief 撤销出力请求并清零指令，之后每周期发"失能"。编号越界时返回 false。 */
    [[nodiscard]] bool disable(std::size_t index) noexcept;

    /**
     * @brief 暂存完整的 MIT 指令，下次 process() 发出。没有请求出力时返回 false。
     * @note 非有限值：指令清零并返回 false。各量超出映射范围时由编码钳位。
     *       控制位置时 kd 不能为 0（说明书：否则会振荡甚至失控）。
     */
    [[nodiscard]] bool set_command(std::size_t index, const protocol::DmEsc::MitCommand &command) noexcept;

    /** @brief 只给力矩（Kp = Kd = 0），输出轴 N·m；超出 TMAX 时由编码钳位。 */
    [[nodiscard]] bool set_torque(std::size_t index, float newton_meter) noexcept;

    /** @brief 清零全部暂存指令，不撤销出力请求。 */
    void clear_all_commands() noexcept;

    /** @return 编号越界时返回 false，snapshot 不变。 */
    [[nodiscard]] bool get_snapshot(std::size_t index, Snapshot &snapshot) const noexcept;

    [[nodiscard]] bool get_statistics(Statistics &statistics) const noexcept;

    /** @brief 输出轴最大力矩（N·m）：峰值扭矩与 TMAX 的较小者；init() 成功前为 0。 */
    [[nodiscard]] float maximum_torque_newton_meter() const noexcept;

    /** @brief 反馈状态是不是故障（失能、使能以外的状态码）。 */
    [[nodiscard]] static constexpr bool is_fault(protocol::DmEsc::State state) noexcept
    {
        return state != protocol::DmEsc::State::DISABLED && state != protocol::DmEsc::State::ENABLED;
    }

  private:
    enum class Step : std::uint8_t
    {
        READ_CONTROL_MODE,
        DISABLE,
        READ_POSITION_MAX,
        READ_VELOCITY_MAX,
        READ_TORQUE_MAX,
        WRITE_TIMEOUT
    };

    struct Slot final
    {
        Phase phase{Phase::STOPPED};
        // 核对
        Step step{Step::READ_CONTROL_MODE};
        bool request_pending{false};
        hal::os::Microseconds request_sent_us{0U};
        std::uint32_t control_mode{0U};
        protocol::DmEsc::Limits limits{};
        std::uint32_t timeout_ticks{0U};
        std::uint8_t mismatch_register{0U};
        // 出力
        bool output_requested{false};
        bool clear_sent{false};
        bool output_active{false};
        bool enable_failed{false};
        hal::os::Microseconds clear_sent_us{0U};
        protocol::DmEsc::MitCommand command{};
        // 反馈
        hal::os::Microseconds last_update_us{0U};
        protocol::DmEsc::State state{protocol::DmEsc::State::DISABLED};
        bool has_feedback{false};
        protocol::DmEsc::FeedbackRaw feedback{};
        std::uint32_t feedback_count{0U};
        std::int32_t position_counts{0}; // 多圈累计的原始计数
        std::int32_t origin_counts{0};   // 核对后第一帧反馈的计数
    };

    [[nodiscard]] bool config_valid() const noexcept;
    void receive_frames(hal::os::Microseconds now) noexcept;
    [[nodiscard]] bool
    handle_frame(std::size_t index, const protocol::DmEsc::Frame &frame, hal::os::Microseconds now) noexcept;
    void accept_reply(std::size_t index, const protocol::DmEsc::RegisterReply &reply) noexcept;
    void accept_feedback(Slot &slot, const protocol::DmEsc::FeedbackRaw &feedback, hal::os::Microseconds now) noexcept;
    void finish_verification(Slot &slot) noexcept;
    void service_verification(std::size_t index, hal::os::Microseconds now) noexcept;
    void service_output(std::size_t index, hal::os::Microseconds now) noexcept;
    [[nodiscard]] bool send(const protocol::DmEsc::Frame &frame) noexcept;
    static void restart_verification(Slot &slot) noexcept;
    static void drop_output(Slot &slot) noexcept;

    Config config_;
    std::uint32_t timeout_ticks_; // 档案的通讯超时换算成寄存器计数
    bool ready_{false};
    bool receiver_claimed_{false};
    std::array<Slot, MAXIMUM_MOTORS> slots_{};
    Statistics statistics_{};
};

} // namespace device

#endif // DRIVER_MOTOR_DM_DM_MOTOR_HPP
