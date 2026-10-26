#ifndef LIBRARIES_DEVICE_MOTOR_DJI_DJI_MOTOR_HPP
#define LIBRARIES_DEVICE_MOTOR_DJI_DJI_MOTOR_HPP

#include "Libraries/Protocol/dji/DjiEsc.hpp"
#include "Platform/Interface/Can.hpp"
#include "Platform/Interface/Time.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief DJI 电调电机总线会话：绑定一路 CAN 与一种电调产品线，维护该总线
 *        上至多 8 台电机的指令聚合发送、反馈解析、多圈累计、链路老化与
 *        跨任务一致快照。协议编解码全部委托 protocol::DjiEsc。
 * @note C610/C620 只有在总线上出现主机控制帧后才发送反馈（实机验证结论），
 *       因此必须先 enable_motor 声明在役设备：会话按 Process 调用节拍向含
 *       在役设备的组发送控制帧（未设置指令时即全零保活帧），这是 C610/
 *       C620 反馈流动的前提；未声明任何设备的总线保持静默。GM6020 无此
 *       依赖，但随控制环自然满足。
 * @note 一条总线只允许一种产品线：GM6020 反馈标识符 0x205~0x20B 与
 *       C610/C620 设备 5~8 的 0x205~0x208 重叠，且 0x1FF 在两条产品线上
 *       命令含义不同。分通道布局时每通道独立一个会话实例。
 * @note Process 必须由单一任务以约 1ms 周期调用：既是本通道软件队列的唯一
 *       消费者（见 platform::Can 契约），也决定控制帧节拍。指令写入与快照
 *       读取可来自其他任务；快照与统计内部用序列锁保证一致性。
 * @note 多圈累计假设相邻两帧机械角差小于半圈（4096 计数）：1kHz 反馈下
 *       各款电机转子转速上限均满足。指令侧不做斜率限制与指令源失联清零，
 *       该策略由应用层负责（可调用 clear_all_commands 实现）。
 */
class DjiMotor final
{
  public:
    static constexpr std::size_t MAXIMUM_MOTORS{8U};
    static constexpr std::uint32_t LINK_TIMEOUT_MS{50U};

    /**
   * @brief 产品档案：协议方言 + 电气满量程 + 反馈能力，按总线实际产品选择。
   */
    struct Profile final
    {
        protocol::DjiEsc::Dialect dialect;
        float current_full_scale_ampere;
        /**
     * @brief 转矩常数（N·m/A），电流环即力矩环：T = Kt × I。GM6020 取自
     *        v1.4 手册"电机特征值"（741 mN·m/A，直驱输出轴）；M3508/M2006
     *        取自对应电机手册（减速电机为输出轴值）。为 0 表示未知，力矩
     *        接口不可用。
     */
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
        [[nodiscard]] static constexpr Profile gm6020_voltage() noexcept
        {
            return {protocol::DjiEsc::gm6020_voltage(), 0.0F, 0.741F, true};
        }
        [[nodiscard]] static constexpr Profile gm6020_current() noexcept
        {
            return {protocol::DjiEsc::gm6020_current(), 3.0F, 0.741F, true};
        }
    };

    /**
   * @brief 单台电机的遥测快照。
   * @note motor_temperature_celsius 仅在产品档案 has_temperature_feedback 为
   *       true 时有效；C610 反馈 DATA[6] 为 Null，恒读 0。torque_current_ratio
   *       按命令满量程同尺度换算（手册未标注反馈电流量程，按惯例一致）。
   */
    struct Snapshot final
    {
        std::uint8_t device_id{0U};
        bool enabled{false};
        bool ever_received{false};
        bool online{false};
        std::uint32_t last_update_tick{0U};
        std::uint32_t accepted_frame_count{0U};
        std::uint16_t rotor_angle_raw{0U};
        float rotor_angle_degrees{0.0F};
        std::int32_t total_angle_counts{0};
        std::int32_t total_turns{0};
        float total_angle_degrees{0.0F};
        std::int16_t rotor_speed_rpm{0};
        std::int16_t torque_current_raw{0};
        float torque_current_ratio{0.0F};
        std::uint8_t motor_temperature_celsius{0U};
    };

    /**
   * @brief 总线级健康统计快照：发送结果分布、解码统计与平台丢帧统计。
   * @note tx_result_counts 按 platform::Can::SendResult 枚举值索引，
   *       [0]=Queued 持续增长表示控制帧正常完成（总线有节点 ACK）。
   */
    struct Statistics final
    {
        std::array<std::uint32_t, 6> tx_result_counts{};
        std::uint32_t accepted_frame_count{0U};
        std::uint32_t rejected_frame_count{0U};
        std::uint32_t unknown_identifier_count{0U};
        std::uint32_t rx_dropped_count{0U};
        std::uint32_t rx_hardware_loss_event_count{0U};
        std::uint32_t bus_off_count{0U};
    };

    /**
   * @brief 绑定 CAN 通道与产品档案。档案方言必须至少支持一种命令种类。
   */
    DjiMotor(platform::Can::Channel channel, const Profile &profile) noexcept;

    /**
   * @brief 校验通道与时基可用并复位会话状态。调度器启动前调用一次。
   * @return 通道与时基均就绪且档案有效时返回 true。
   */
    [[nodiscard]] bool init() noexcept;

    /**
   * @brief 周期服务：排空接收队列并更新电机状态、执行链路老化、向含在役
   *        设备的组聚合发送控制帧、刷新并发布快照。
   */
    void process() noexcept;

    [[nodiscard]] bool is_ready() const noexcept
    {
        return ready_;
    }

    /**
   * @brief 返回档案允许的最大设备编号（C610/C620 为 8，GM6020 为 7）。
   */
    [[nodiscard]] std::uint8_t maximum_device_count() const noexcept
    {
        return profile_.dialect.maximum_device_count;
    }

    /**
   * @brief 声明在役设备：启用其指令槽位，并使所在组开始周期发送控制帧。
   * @note C610/C620 反馈依赖此调用（保活前提），应在 Init 后尽早执行。
   */
    [[nodiscard]] bool enable_motor(std::uint8_t device_id) noexcept;

    /**
   * @brief 设置指定设备的原始控制值（电流/电压原始计数），越界按满量程钳位。
   */
    [[nodiscard]] bool set_raw_command(std::uint8_t device_id, std::int16_t raw_command) noexcept;

    /**
   * @brief 按安培设置电流类产品的控制值；电压类产品（GM6020 电压模式）
   *        不适用，返回 false。输入按满量程双向钳位。
   */
    [[nodiscard]] bool set_current_ampere(std::uint8_t device_id, float ampere) noexcept;

    /**
   * @brief 开环力矩指令：按档案转矩常数换算为电流指令
   *        （I = torque / Kt），无额外闭环；电流类产品且 Kt 有效时可用，
   *        输出经电流满量程自然钳位。
   */
    [[nodiscard]] bool set_torque(std::uint8_t device_id, float newton_meter) noexcept;

    /**
   * @brief 按 [-1,1] 归一化比例设置控制值，适用于所有产品线；越界钳位。
   */
    [[nodiscard]] bool set_torque_ratio(std::uint8_t device_id, float ratio) noexcept;

    /**
   * @brief 清零全部指令槽位（指令源失联等安全策略由应用层调用）。
   */
    void clear_all_commands() noexcept;

    /**
   * @brief 读取指定设备的遥测快照（序列锁保证跨任务一致）。
   * @return 设备编号在 [1, MAXIMUM_MOTORS] 内时返回 true。
   */
    [[nodiscard]] bool get_snapshot(std::uint8_t device_id, Snapshot &snapshot) const noexcept;

    /**
   * @brief 读取总线健康统计快照（序列锁保证跨任务一致）。
   */
    [[nodiscard]] bool get_statistics(Statistics &statistics) const noexcept;

  private:
    struct MotorState final
    {
        bool enabled{false};
        bool ever_received{false};
        bool stale{false};
        bool has_angle{false};
        std::uint32_t last_update_tick{0U};
        std::uint32_t accepted_frame_count{0U};
        std::uint16_t last_angle_raw{0U};
        std::int32_t total_angle_counts{0};
        protocol::DjiEsc::Feedback feedback{};
        std::int16_t command{0};
    };

    void receive_frames() noexcept;
    void update_link_aging() noexcept;
    void transmit_commands() noexcept;
    void publish() noexcept;
    void fill_snapshot(std::size_t index, Snapshot &snapshot) const noexcept;
    [[nodiscard]] bool device_id_valid(std::uint8_t device_id) const noexcept;
    [[nodiscard]] bool group_enabled(protocol::DjiEsc::Group group) const noexcept;

    platform::Can::Channel channel_;
    protocol::DjiEsc esc_;
    Profile profile_;
    protocol::DjiEsc::CommandKind command_kind_;
    bool ready_{false};
    bool dirty_{false};
    protocol::DjiEsc::Feedback feedback_{};
    std::array<MotorState, MAXIMUM_MOTORS> motors_{};
    std::array<std::uint32_t, 6> tx_result_counts_{};
    platform::Can::Statistics port_statistics_{};

    static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
                  "DjiMotor snapshot sequence lock requires lock-free atomics");
    mutable std::atomic<std::uint32_t> snapshot_sequence_{0U};
    std::array<Snapshot, MAXIMUM_MOTORS> published_snapshots_{};
    Statistics published_statistics_{};
};

} // namespace device

#endif
