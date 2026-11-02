#ifndef DRIVER_MOTOR_DJI_DJI_MOTOR_HPP
#define DRIVER_MOTOR_DJI_DJI_MOTOR_HPP

#include "driver/can_bus/can_bus.hpp"
#include "driver/motor/joint.hpp"
#include "driver/protocol/dji/dji_esc.hpp"
#include "hal/can.hpp"
#include "hal/os/clock.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief 一条 CAN 总线上的大疆电调电机（C610、C620、GM6020 电流模式可以混接，Decision-20261102-03 D6）：最多 8 台。
 *        登记到这条总线的 CanBus 上收反馈；能力经关节 Joint 使能、给力矩、读状态。编解码委托 protocol::DjiEsc。
 *
 * - 控制帧：每个控制帧 ID 带四台电机的电流（大端 int16）。init() 按每台电机的方言算出它在哪一帧的哪一格；
 *   每次 transmit() 把用到的帧全部发出，没有请求出力的电机那一格为 0。C610/C620 要先收到控制帧才回反馈，
 *   所以总线上一直有这些帧（保活）。
 * - 冲突检查（init() 拒绝）：两台电机的反馈帧 ID 相同（如 C620 的 5 号与 GM6020 的 1 号都是 0x205），或者占了同一帧的
 *   同一格（如 C620 的 5–8 号与 GM6020 电压模式的 1–4 号都用 0x1FF）。只支持电流（力矩）控制，电压模式的档案被拒绝。
 * - 力矩：关节给的电机输出轴力矩 ÷ Kt 得电流，按满量程换成计数并饱和；反馈的转矩电流同样换回力矩。
 * - 位置：转子编码器每圈 8192 计数，相邻两帧之间按不超过半圈累计多圈。GM6020 转子即输出轴，位置上电即绝对；
 *   M3508、M2006 的编码器在减速箱前面，从第一帧反馈起算（型号的 position_absolute）。离线期间的运动不做推断。
 * - DJI 电调没有使能命令：关节 enable() 之后即出力；关节状态里没有故障信息。
 * - 一条总线上的 DJI 电机放在同一个 DjiMotor 里（关节可以分给不同的能力）：控制帧一帧带四台，两个实例各发一份同 ID 的帧
 *   会互相覆盖。所以本驱动把用到的控制帧 ID 也登记到 CanBus：第二个实例用到同一个 ID 时登记失败；总线上若收到这些 ID
 *   的帧（别的主机也在发），计入 foreign_control_frame_count。
 *
 * @note 在拥有这条总线的任务里使用（D3）：CanBus::receive() 把反馈交给本驱动，CanBus::transmit(now) 调用本驱动发控制帧。
 * @note 收到帧的时刻用 hal::Can::Frame::received_us（Decision-20261102-04）；LINK_TIMEOUT_US 没有反馈即离线。
 */
class DjiMotor final
{
  public:
    static constexpr std::size_t MAXIMUM_MOTORS{8U};
    static constexpr std::size_t MAXIMUM_CONTROL_FRAMES{4U}; // 电流模式用到的控制帧 ID：0x200、0x1FF、0x1FE、0x2FE
    static constexpr hal::os::Microseconds LINK_TIMEOUT_US{50000U}; // 超过这么久没有反馈即离线
    static constexpr std::uint32_t FEEDBACK_RATE_HZ{1000U};         // 电调的反馈频率，与控制频率无关

    /** 电调档案：协议方言 + 电流满量程 + 转矩常数 + 有无温度反馈。 */
    struct Profile final
    {
        protocol::DjiEsc::Dialect dialect;
        float current_full_scale_ampere;
        /** 转矩常数（N·m/A），已是电机输出轴口径：GM6020 直驱 0.741；M3508、M2006 取自电机手册（减速后）。 */
        float torque_constant_newton_meter_per_ampere;
        bool has_temperature_feedback;

        [[nodiscard]] static constexpr Profile c620() noexcept
        {
            return {protocol::DjiEsc::c620(), 20.0F, 0.3F, true};
        }
        [[nodiscard]] static constexpr Profile c610() noexcept
        {
            return {protocol::DjiEsc::c610(), 10.0F, 0.18F, false};
        }
        [[nodiscard]] static constexpr Profile gm6020_current() noexcept
        {
            return {protocol::DjiEsc::gm6020_current(), 3.0F, 0.741F, true};
        }
        /** GM6020 电压模式：没有电流满量程，不能做力矩控制，init() 拒绝；只为说明方言而保留。 */
        [[nodiscard]] static constexpr Profile gm6020_voltage() noexcept
        {
            return {protocol::DjiEsc::gm6020_voltage(), 0.0F, 0.741F, true};
        }
    };

    /** 型号：电调档案 + 转子到电机输出轴的减速比 + 位置是否上电即绝对。型号常量见 dji_models.hpp。 */
    struct Model final
    {
        Profile profile;
        float gear_ratio; // 转子转 gear_ratio 圈，电机输出轴转 1 圈（M3508 19.2，M2006 36，GM6020 1）
        bool position_absolute;

        /** 电机输出轴的最大力矩：Kt × 电流满量程。 */
        [[nodiscard]] constexpr float maximum_torque_newton_meter() const noexcept
        {
            return profile.torque_constant_newton_meter_per_ampere * profile.current_full_scale_ampere;
        }
    };

    /** 一台电机：接到哪个关节、什么型号、电调 ID、机构。 */
    struct Motor final
    {
        Joint *joint{nullptr};
        Model model{Profile::c620(), 1.0F, false};
        std::uint8_t device_id{0U}; // 电调 ID：C610/C620 1–8，GM6020 1–7
        JointMechanics mechanics{};
    };

    struct Config final
    {
        std::array<Motor, MAXIMUM_MOTORS> motors{};
        std::size_t count{0U};
    };

    /** 单台电机的诊断快照。 */
    struct Snapshot final
    {
        std::uint8_t device_id{0U};
        std::uint32_t feedback_identifier{0U};
        std::uint32_t control_identifier{0U};
        bool ever_received{false};
        bool online{false};
        hal::os::Microseconds last_update_us{0U};
        std::uint32_t feedback_count{0U};
        std::uint16_t rotor_angle_raw{0U};
        std::int32_t total_angle_counts{0}; // 多圈累计的转子计数（绝对型号从上电时的角度起）
        std::int16_t rotor_speed_rpm{0};
        std::int16_t torque_current_raw{0};
        std::uint8_t motor_temperature_celsius{0U};
        std::int16_t command_raw{0}; // 最后一次发出的控制值
    };

    /** 本驱动的统计。tx_result_counts 按 hal::Can::SendResult 的值索引。 */
    struct Statistics final
    {
        std::array<std::uint32_t, hal::Can::SEND_RESULT_COUNT> tx_result_counts{};
        std::uint32_t feedback_count{0U};
        std::uint32_t rejected_frame_count{0U}; // 帧 ID 属于本组，内容却解不出（长度不对、扩展帧）
        std::uint32_t foreign_control_frame_count{0U}; // 收到本组控制帧 ID 的帧：总线上还有别的主机在发
    };

    /** @brief 只保存总线与配置，不碰硬件；constexpr，全局对象常量初始化。 */
    constexpr DjiMotor(CanBus &bus, const Config &config) noexcept : bus_{bus}, config_{config} {}
    DjiMotor(const DjiMotor &) = delete;
    DjiMotor &operator=(const DjiMotor &) = delete;
    DjiMotor(DjiMotor &&) = delete;
    DjiMotor &operator=(DjiMotor &&) = delete;

    /**
     * @brief 检查配置与 ID 冲突，第一次调用时把关节接上、登记到 CanBus。调度器启动前调用。
     * @return 配置无效、ID 冲突、关节已被别的电机接走或登记失败时返回 false。可以重复调用（复位状态）。
     */
    [[nodiscard]] bool init() noexcept;

    [[nodiscard]] constexpr std::size_t count() const noexcept
    {
        return config_.count;
    }

    /** @return 编号越界时返回 false，snapshot 不变。 */
    [[nodiscard]] bool get_snapshot(std::size_t index, Snapshot &snapshot) const noexcept;

    [[nodiscard]] bool get_statistics(Statistics &statistics) const noexcept;

    /** @brief 电机的控制帧 ID 与在帧里的格（0–3）；型号不支持电流控制或 ID 越界时返回 false。 */
    [[nodiscard]] static constexpr bool
    control_slot(const Motor &motor, std::uint32_t &identifier, std::size_t &slot) noexcept
    {
        const protocol::DjiEsc::Dialect &dialect = motor.model.profile.dialect;
        const protocol::DjiEsc::CommandSpec &current =
            dialect.commands[static_cast<std::size_t>(protocol::DjiEsc::CommandKind::CURRENT)];
        if (motor.device_id == 0U || motor.device_id > dialect.maximum_device_count)
        {
            return false;
        }
        const bool first = motor.device_id <= protocol::DjiEsc::DEVICES_PER_FRAME;
        identifier = first ? current.group_one_control_identifier : current.group_two_control_identifier;
        slot = (motor.device_id - 1U) % protocol::DjiEsc::DEVICES_PER_FRAME;
        return identifier != 0U;
    }

    /** @brief 控制频率 rate_hz 下这组电机每秒的帧数：每台 1 kHz 反馈，加上每周期每个用到的控制帧，供负载检查。 */
    [[nodiscard]] static constexpr std::uint32_t frames_per_second(const Config &config, std::uint32_t rate_hz) noexcept
    {
        std::array<std::uint32_t, MAXIMUM_CONTROL_FRAMES + MAXIMUM_MOTORS> identifiers{};
        std::size_t frames = 0U;
        for (std::size_t index = 0U; index < config.count && index < config.motors.size(); ++index)
        {
            std::uint32_t identifier = 0U;
            std::size_t slot = 0U;
            if (!control_slot(config.motors[index], identifier, slot))
            {
                continue;
            }
            bool seen = false;
            for (std::size_t other = 0U; other < frames; ++other)
            {
                seen = seen || identifiers[other] == identifier;
            }
            if (!seen)
            {
                identifiers[frames++] = identifier;
            }
        }
        return static_cast<std::uint32_t>(config.count) * FEEDBACK_RATE_HZ +
               static_cast<std::uint32_t>(frames) * rate_hz;
    }

  private:
    struct Slot final
    {
        bool ever_received{false};
        hal::os::Microseconds last_update_us{0U};
        protocol::DjiEsc::Feedback feedback{};
        std::int32_t total_counts{0};
        std::uint32_t feedback_count{0U};
        std::int16_t command{0};
        std::uint32_t feedback_identifier{0U};
        std::uint8_t frame{0U}; // 在 frames_ 里的下标
        std::uint8_t slot{0U};  // 在帧里的格
    };

    struct ControlFrame final
    {
        std::uint32_t identifier{0U};
        std::array<std::int8_t, protocol::DjiEsc::DEVICES_PER_FRAME> motor{{-1, -1, -1, -1}}; // 每格的电机下标，−1 为空
    };

    static void on_frame(void *context, const hal::Can::Frame &frame) noexcept;
    static void on_transmit(void *context, hal::os::Microseconds now) noexcept;

    [[nodiscard]] bool config_valid() const noexcept;
    [[nodiscard]] bool plan() noexcept;
    [[nodiscard]] bool attach() noexcept;
    void receive(const hal::Can::Frame &frame) noexcept;
    void transmit(hal::os::Microseconds now) noexcept;
    void accept_feedback(std::size_t index,
                         const protocol::DjiEsc::Feedback &feedback,
                         hal::os::Microseconds received_us) noexcept;
    [[nodiscard]] std::int16_t command_counts(std::size_t index, hal::os::Microseconds now) noexcept;

    CanBus &bus_;
    Config config_;
    bool ready_{false};
    bool attached_{false};
    std::array<Slot, MAXIMUM_MOTORS> slots_{};
    std::array<ControlFrame, MAXIMUM_CONTROL_FRAMES> frames_{};
    std::size_t frame_count_{0U};
    Statistics statistics_{};
};

} // namespace device

#endif // DRIVER_MOTOR_DJI_DJI_MOTOR_HPP
