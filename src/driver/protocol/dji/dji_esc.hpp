#ifndef DRIVER_PROTOCOL_DJI_DJI_ESC_HPP
#define DRIVER_PROTOCOL_DJI_DJI_ESC_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace protocol
{

/**
 * @brief 大疆电调家族 CAN 协议编解码器。下行控制帧承载同组四台设备的有符号
 *        16 位大端控制值，上行每台设备独立回报一条反馈帧；帧骨架各产品线
 *        一致，语义差异（控制帧 ID 分组、命令种类与满量程、反馈 ID 基准、
 *        设备总数）通过 Dialect 注入，新增产品线只增工厂不改编解码逻辑。
 */
class DjiEsc final
{
  public:
    static constexpr std::size_t DEVICES_PER_FRAME{4U};
    static constexpr std::size_t FRAME_DATA_LENGTH{8U};
    static constexpr std::uint16_t ANGLE_COUNTS_PER_REVOLUTION{8192U};

    enum class CommandKind : std::uint8_t
    {
        CURRENT = 0U,
        VOLTAGE = 1U
    };
    enum class Group : std::uint8_t
    {
        FIRST = 0U,
        SECOND = 1U
    };

    enum class DecodeResult : std::uint8_t
    {
        ACCEPTED,
        INVALID_ARGUMENT,
        UNKNOWN_IDENTIFIER,
        UNSUPPORTED
    };

    struct CommandSpec final
    {
        std::uint32_t group_one_control_identifier;
        std::uint32_t group_two_control_identifier;
        std::uint16_t control_full_scale_counts;
    };

    struct Dialect final
    {
        std::array<CommandSpec, 2> commands;
        std::uint32_t feedback_identifier_base;
        std::uint8_t maximum_device_count;
    };

    [[nodiscard]] static constexpr Dialect make_dialect(const CommandSpec &current,
                                                        const CommandSpec &voltage,
                                                        std::uint32_t feedback_identifier_base,
                                                        std::uint8_t maximum_device_count) noexcept
    {
        return {{{current, voltage}}, feedback_identifier_base, maximum_device_count};
    }

    /**
   * @brief C610：电流控制 0x200 对应设备 1~4、0x1FF 对应 5~8，
   *        满量程 ±10000 ↔ ±10 A；反馈 0x200+ID，设备总数 8。
   */
    [[nodiscard]] static constexpr Dialect c610() noexcept
    {
        return make_dialect({0x200U, 0x1FFU, 10000U}, {0U, 0U, 0U}, 0x200U, 8U);
    }

    /**
   * @brief C620：控制 ID 分组与 C610 一致，满量程 ±16384 ↔ ±20 A。
   */
    [[nodiscard]] static constexpr Dialect c620() noexcept
    {
        return make_dialect({0x200U, 0x1FFU, 16384U}, {0U, 0U, 0U}, 0x200U, 8U);
    }

    /**
   * @brief GM6020 电流环模式（需固件 >=1.0.11.2 并在 Assistant 开启电流
   *        环）：0x1FE 对应设备 1~4、0x2FE 对应 5~7，满量程
   *        ±16384 ↔ ±3 A；反馈 0x204+ID，设备总数 7。
   */
    [[nodiscard]] static constexpr Dialect gm6020_current() noexcept
    {
        return make_dialect({0x1FEU, 0x2FEU, 16384U}, {0U, 0U, 0U}, 0x204U, 7U);
    }

    /**
   * @brief GM6020 电压模式（默认）：0x1FF 对应设备 1~4、0x2FF 对应 5~7，
   *        满量程 ±25000；反馈 0x204+ID，设备总数 7。
   */
    [[nodiscard]] static constexpr Dialect gm6020_voltage() noexcept
    {
        return make_dialect({0U, 0U, 0U}, {0x1FFU, 0x2FFU, 25000U}, 0x204U, 7U);
    }

    struct ControlFrame final
    {
        std::uint32_t identifier{0U};
        std::array<std::uint8_t, FRAME_DATA_LENGTH> data{};
    };

    struct Feedback final
    {
        std::uint8_t device_id{0U};
        std::uint16_t rotor_angle_raw{0U};
        std::int16_t rotor_speed_raw{0};
        std::int16_t torque_current_raw{0};
        std::uint8_t motor_temperature_celsius{0U};
    };

    struct Statistics final
    {
        std::uint32_t encoded_control_frame_count{0U};
        std::uint32_t accepted_frame_count{0U};
        std::uint32_t rejected_frame_count{0U};
        std::uint32_t unknown_identifier_count{0U};
        std::uint32_t unsupported_command_count{0U};
    };

    explicit DjiEsc(const Dialect &dialect) noexcept;

    /**
   * @brief 清零全部统计，保留绑定的 Dialect。
   */
    void reset() noexcept;

    /**
   * @brief 将同组四台设备的控制值编码为一条指定种类的命令帧。
   * @param kind 命令种类，当前产品线未提供该种类时拒绝并计入统计。
   * @param group 控制帧对应的设备组。
   * @param control_counts 控制值，任一值超出该命令种类有符号满量程时拒绝。
   * @param frame 输出帧，仅编码成功时写入。
   * @return 种类可用、该组在当前 Dialect 下存在设备且控制值全部合法时
   *         返回 true。
   */
    [[nodiscard]] bool encode(CommandKind kind,
                              Group group,
                              const std::array<std::int16_t, DEVICES_PER_FRAME> &control_counts,
                              ControlFrame &frame) noexcept;

    /**
   * @brief 解析一条反馈帧，仅返回 Accepted 时写入 feedback。
   * @param identifier 收到的帧标识符。
   * @param extended_identifier 为 true 表示扩展帧，家族反馈帧均为标准帧。
   * @param data_length 收到的帧数据长度。
   * @param data 帧数据指针。
   * @param feedback 输出解析结果。
   */
    [[nodiscard]] DecodeResult decode_feedback(std::uint32_t identifier,
                                               bool extended_identifier,
                                               std::uint8_t data_length,
                                               const std::uint8_t *data,
                                               Feedback &feedback) noexcept;

    /**
   * @brief 把任意控制值钳位到指定命令种类的有符号满量程内。
   * @note 种类不可用时返回 0。
   */
    [[nodiscard]] std::int16_t saturate(std::int32_t control_counts, CommandKind kind) const noexcept;

    /**
   * @brief 将控制值换算为 [-1, 1] 的有符号比例，供设备层乘具体电气量程。
   * @note 种类不可用或满量程为 0 时返回 0。
   */
    [[nodiscard]] static constexpr float control_ratio(std::int16_t control_counts, const CommandSpec &spec) noexcept
    {
        if (spec.control_full_scale_counts == 0U)
        {
            return 0.0F;
        }
        return static_cast<float>(control_counts) / static_cast<float>(spec.control_full_scale_counts);
    }

    /**
   * @brief 返回当前 Dialect 中指定命令种类的规格描述。
   */
    [[nodiscard]] constexpr const CommandSpec &get_command_spec(CommandKind kind) const noexcept
    {
        return dialect_.commands[static_cast<std::size_t>(kind)];
    }

    /**
   * @brief 返回指定设备编号所属的控制组。
   * @note device_id 必须已确认处于 [1, maximum_device_count]。
   */
    [[nodiscard]] static Group group_of(std::uint8_t device_id) noexcept
    {
        return device_id <= static_cast<std::uint8_t>(DEVICES_PER_FRAME) ? Group::FIRST : Group::SECOND;
    }

    /**
   * @brief 将转子机械角原始计数换算为机械角度数。
   */
    [[nodiscard]] static constexpr float rotor_angle_degrees(std::uint16_t rotor_angle_raw) noexcept
    {
        return static_cast<float>(rotor_angle_raw) * 360.0F / static_cast<float>(ANGLE_COUNTS_PER_REVOLUTION);
    }

    [[nodiscard]] const Statistics &get_statistics() const noexcept
    {
        return statistics_;
    }

    [[nodiscard]] const Dialect &get_dialect() const noexcept
    {
        return dialect_;
    }

  private:
    Dialect dialect_;
    Statistics statistics_{};
};

} // namespace protocol

#endif
