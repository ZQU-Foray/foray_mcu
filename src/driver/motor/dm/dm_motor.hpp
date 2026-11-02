#ifndef DRIVER_MOTOR_DM_DM_MOTOR_HPP
#define DRIVER_MOTOR_DM_DM_MOTOR_HPP

#include "driver/can_bus/can_bus.hpp"
#include "driver/motor/joint.hpp"
#include "driver/protocol/dm/dm_esc.hpp"
#include "hal/can.hpp"
#include "hal/os/clock.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief 一条 CAN 总线上的达妙一体化电机（MIT 模式，Decision-20261102-01、-03）：最多 4 台，可以是不同型号。
 *        登记到这条总线的 CanBus 上收帧；能力经关节 Joint 使能、下指令、读状态。编解码全部委托 protocol::DmEsc。
 *
 * 每台电机的阶段：
 * - VERIFYING：读控制模式 → 按读到的模式发"失能"并等一帧反馈 → 读 PMAX、VMAX、TMAX → 把型号的通讯超时
 *   写进电机 RAM 并核对回显。每一步 REQUEST_TIMEOUT_US 内没有应答就重发，一直重试（电机晚上电也能接上）；
 * - READY：模式是 MIT，量程与超时都与型号一致，关节可以使能。没有请求出力时每周期发"失能"作保活（取得反馈，
 *   并确保电机不出力）；关节 enable() 之后先发一次"清除错误"，再每周期发"使能"，直到反馈确认，之后每周期发
 *   关节的指令（力矩，或阻抗：MIT 的 p、v、Kp、Kd、τ）；
 * - MISMATCH：模式不是 MIT，或量程、超时与型号不符。不发任何帧，关节不能使能；重新 init() 才会再核对。
 *
 * READY 时出现下列情况，驱动撤销出力（关节要再 enable() 才会重试，驱动不自动重新使能）：
 * - 反馈报故障（过温、欠压、通讯丢失等）；
 * - 使能后反馈变成"失能"（电机复位）：同时重新核对，电机重启后 RAM 里写的超时已经丢了；
 * - LINK_TIMEOUT_US 没有反馈（离线）：同时重新核对；
 * - 使能后 ENABLE_TIMEOUT_US 内没有确认：记为使能失败。
 *
 * 一帧属于哪台电机：帧 ID 等于它的 MST_ID，且寄存器应答的 D0–D1 等于 ESC_ID，或反馈 D0 的低 4 位等于 ESC_ID。
 * 所以几台电机共用 MST_ID 也能分开（推荐仍按 ESC_ID + 0x10 设置）。
 *
 * @note 在拥有这条总线的任务里使用（Decision-20261102-03 D3）：CanBus::receive() 把帧交给本驱动，
 *       CanBus::transmit(now) 调用本驱动推进核对、使能与发送，约 1 ms 一次。READY 时每台电机每周期发一帧、收一帧，
 *       1 Mbps、1 kHz 时每台约占 27% 总线负载（frames_per_second()）。
 * @note 收到帧的时刻用 hal::Can::Frame::received_us（Decision-20261102-04）。
 * @note 位置：输出轴有绝对编码器，上电时在 [−π, π] 内；在 ±PMAX 处回绕，驱动按 65536 个计数累计多圈，前提是相邻两帧之间
 *       转过不到 PMAX。电机掉电重启后从 [−π, π] 重新开始，累计值可能差 2π 的整数倍。阻抗模式的位置给定按电机当前所在的
 *       那一圈换算回电机的量程，超出 ±PMAX 时被钳位。
 */
class DmMotor final
{
  public:
    static constexpr std::size_t MAXIMUM_MOTORS{4U};
    static constexpr std::uint16_t MAXIMUM_ESC_ID{15U};                // 反馈帧 D0 只有低 4 位放 ESC_ID
    static constexpr std::uint16_t MAXIMUM_MASTER_ID{0x7FEU};          // 0x7FF 是寄存器请求的帧 ID
    static constexpr hal::os::Microseconds REQUEST_TIMEOUT_US{20000U}; // 核对时一步没有应答就重发
    static constexpr hal::os::Microseconds ENABLE_TIMEOUT_US{50000U};  // 使能后这么久没确认就放弃
    // 清除错误后这段时间内收到的反馈不算确认：排除清错之前发出的帧的应答（按接收时刻计，Decision-20261102-04）
    static constexpr hal::os::Microseconds ENABLE_CONFIRM_GUARD_US{1000U};
    static constexpr hal::os::Microseconds LINK_TIMEOUT_US{50000U}; // READY 时这么久没有反馈即离线
    static constexpr hal::os::Microseconds TIMEOUT_TICK_US{50U};    // 通讯超时寄存器（0x09）的计数周期

    /** 型号：电机里必须是什么设置，以及写进电机 RAM 的通讯超时。型号常量见 dm_models.hpp。 */
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

        /** 电机输出轴的最大力矩：峰值扭矩与 TMAX 的较小者。 */
        [[nodiscard]] constexpr float maximum_torque_newton_meter() const noexcept
        {
            return peak_torque_newton_meter < limits.torque_newton_meter ? peak_torque_newton_meter
                                                                         : limits.torque_newton_meter;
        }
    };

    /** 一台电机：接到哪个关节、什么型号、接线与机构。 */
    struct Motor final
    {
        Joint *joint{nullptr};
        Profile profile{Profile::j4310()};
        std::uint16_t esc_id{0U};    // 1–15，组内互不相同
        std::uint16_t master_id{0U}; // MST_ID：反馈与应答的帧 ID，低 8 位不能等于组内任何 ESC_ID
        JointMechanics mechanics{};
    };

    struct Config final
    {
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

    /** 单台电机的诊断快照（电机输出轴，单位 rad、rad/s、N·m）。控制用关节的状态，这里给排查与 bringup 检查。 */
    struct Snapshot final
    {
        std::uint16_t esc_id{0U};
        std::uint16_t master_id{0U};
        Phase phase{Phase::STOPPED};
        bool output_requested{false}; // 关节请求出力（驱动这边已接受）
        bool output_active{false};    // 电机已确认使能，正在发关节的指令
        bool enable_failed{false};    // 最近一次使能没有成功：超时未确认，或电机报故障
        bool online{false};           // READY，且 LINK_TIMEOUT_US 内收到过这台电机的帧
        bool has_feedback{false};     // 核对通过后收到过反馈，下面的物理量有效
        hal::os::Microseconds last_update_us{0U}; // 最后一次收到这台电机的帧（反馈或应答）的接收时刻
        std::uint32_t feedback_count{0U};
        protocol::DmEsc::State state{protocol::DmEsc::State::DISABLED}; // 最后一帧反馈的状态
        float position_rad{0.0F};                                       // 多圈累计的位置
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

    /** 本驱动的统计。tx_result_counts 按 hal::Can::SendResult 的值索引。 */
    struct Statistics final
    {
        std::array<std::uint32_t, hal::Can::SEND_RESULT_COUNT> tx_result_counts{};
        std::uint32_t feedback_count{0U};
        std::uint32_t register_reply_count{0U}; // 收到的寄存器应答，含下面丢弃的重复应答
        std::uint32_t stale_reply_count{0U}; // 不在等待中的寄存器应答（重发造成的重复应答），丢弃
        std::uint32_t request_timeout_count{0U}; // 核对时请求超时而重发的次数
        std::uint32_t enable_failure_count{0U};
        std::uint32_t unknown_frame_count{0U}; // 帧 ID 是本组的 MST_ID，内容却不属于组内任何电机
    };

    /** @brief 只保存总线与配置，不碰硬件；constexpr，全局对象常量初始化。 */
    constexpr DmMotor(CanBus &bus, const Config &config) noexcept : bus_{bus}, config_{config} {}
    DmMotor(const DmMotor &) = delete;
    DmMotor &operator=(const DmMotor &) = delete;
    DmMotor(DmMotor &&) = delete;
    DmMotor &operator=(DmMotor &&) = delete;

    /**
     * @brief 检查配置，第一次调用时把关节接上、登记到 CanBus；全部电机从头开始核对。调度器启动前调用。
     * @return 配置无效（关节为空或已被别的电机接走、ID 不合规、型号无效、机构参数无效）或登记失败时返回 false。
     *         可以重复调用：复位全部状态并重新核对（核对不符的电机改好设置后用它重试）。
     * @note 总线的接收由 CanBus::init() 占住，与本函数的先后不限。
     */
    [[nodiscard]] bool init() noexcept;

    [[nodiscard]] constexpr std::size_t count() const noexcept
    {
        return config_.count;
    }

    /** @return 编号越界时返回 false，snapshot 不变。 */
    [[nodiscard]] bool get_snapshot(std::size_t index, Snapshot &snapshot) const noexcept;

    [[nodiscard]] bool get_statistics(Statistics &statistics) const noexcept;

    /** @brief 反馈状态是不是故障（失能、使能以外的状态码）。 */
    [[nodiscard]] static constexpr bool is_fault(protocol::DmEsc::State state) noexcept
    {
        return state != protocol::DmEsc::State::DISABLED && state != protocol::DmEsc::State::ENABLED;
    }

    /** @brief 控制频率 rate_hz 下 motor_count 台电机每秒的帧数（每台每周期一帧指令加一帧反馈），供负载检查。 */
    [[nodiscard]] static constexpr std::uint32_t frames_per_second(std::size_t motor_count,
                                                                   std::uint32_t rate_hz) noexcept
    {
        return static_cast<std::uint32_t>(motor_count) * 2U * rate_hz;
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
        // 反馈
        hal::os::Microseconds last_update_us{0U};
        protocol::DmEsc::State state{protocol::DmEsc::State::DISABLED};
        bool has_feedback{false};
        protocol::DmEsc::FeedbackRaw feedback{};
        std::uint32_t feedback_count{0U};
        std::int32_t position_counts{0}; // 多圈累计的原始计数
    };

    static void on_frame(void *context, const hal::Can::Frame &frame) noexcept;
    static void on_transmit(void *context, hal::os::Microseconds now) noexcept;

    [[nodiscard]] bool config_valid() const noexcept;
    [[nodiscard]] bool attach() noexcept;
    void receive(const hal::Can::Frame &frame) noexcept;
    void transmit(hal::os::Microseconds now) noexcept;
    [[nodiscard]] bool
    handle_frame(std::size_t index, const protocol::DmEsc::Frame &frame, hal::os::Microseconds received_us) noexcept;
    void accept_reply(std::size_t index, const protocol::DmEsc::RegisterReply &reply) noexcept;
    void accept_feedback(std::size_t index,
                         const protocol::DmEsc::FeedbackRaw &feedback,
                         hal::os::Microseconds received_us) noexcept;
    void finish_verification(std::size_t index) noexcept;
    void service_verification(std::size_t index, hal::os::Microseconds now) noexcept;
    void service_output(std::size_t index, hal::os::Microseconds now) noexcept;
    void publish_status(std::size_t index, hal::os::Microseconds now) noexcept;
    [[nodiscard]] float position_rad(std::size_t index) const noexcept;
    [[nodiscard]] bool send(const protocol::DmEsc::Frame &frame) noexcept;
    void restart_verification(std::size_t index) noexcept;
    void drop_output(std::size_t index) noexcept;
    void abandon_output(std::size_t index) noexcept;

    CanBus &bus_;
    Config config_;
    bool ready_{false};
    bool attached_{false};
    std::array<Slot, MAXIMUM_MOTORS> slots_{};
    Statistics statistics_{};
};

} // namespace device

#endif // DRIVER_MOTOR_DM_DM_MOTOR_HPP
