#ifndef LIBRARIES_PROTOCOL_DJI_DJI_ESC_HPP
#define LIBRARIES_PROTOCOL_DJI_DJI_ESC_HPP

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
  static constexpr std::size_t DevicesPerFrame{4U};
  static constexpr std::size_t FrameDataLength{8U};
  static constexpr std::uint16_t AngleCountsPerRevolution{8192U};

  enum class CommandKind : std::uint8_t
  {
    Current = 0U,
    Voltage = 1U
  };
  enum class Group : std::uint8_t
  {
    First = 0U,
    Second = 1U
  };

  enum class DecodeResult : std::uint8_t
  {
    Accepted,
    InvalidArgument,
    UnknownIdentifier,
    Unsupported
  };

  struct CommandSpec final
  {
    std::uint32_t groupOneControlIdentifier;
    std::uint32_t groupTwoControlIdentifier;
    std::uint16_t controlFullScaleCounts;
  };

  struct Dialect final
  {
    std::array<CommandSpec, 2> commands;
    std::uint32_t feedbackIdentifierBase;
    std::uint8_t maximumDeviceCount;
  };

  [[nodiscard]] static constexpr Dialect MakeDialect(const CommandSpec &current,
                                                     const CommandSpec &voltage,
                                                     std::uint32_t feedbackIdentifierBase,
                                                     std::uint8_t maximumDeviceCount) noexcept
  {
    return {{{current, voltage}}, feedbackIdentifierBase, maximumDeviceCount};
  }

  /**
   * @brief C610：电流控制 0x200 对应设备 1~4、0x1FF 对应 5~8，
   *        满量程 ±10000 ↔ ±10 A；反馈 0x200+ID，设备总数 8。
   */
  [[nodiscard]] static constexpr Dialect C610() noexcept
  {
    return MakeDialect({0x200U, 0x1FFU, 10000U}, {0U, 0U, 0U}, 0x200U, 8U);
  }

  /**
   * @brief C620：控制 ID 分组与 C610 一致，满量程 ±16384 ↔ ±20 A。
   */
  [[nodiscard]] static constexpr Dialect C620() noexcept
  {
    return MakeDialect({0x200U, 0x1FFU, 16384U}, {0U, 0U, 0U}, 0x200U, 8U);
  }

  /**
   * @brief GM6020 电流环模式（需固件 >=1.0.11.2 并在 Assistant 开启电流
   *        环）：0x1FE 对应设备 1~4、0x2FE 对应 5~7，满量程
   *        ±16384 ↔ ±3 A；反馈 0x204+ID，设备总数 7。
   */
  [[nodiscard]] static constexpr Dialect Gm6020Current() noexcept
  {
    return MakeDialect({0x1FEU, 0x2FEU, 16384U}, {0U, 0U, 0U}, 0x204U, 7U);
  }

  /**
   * @brief GM6020 电压模式（默认）：0x1FF 对应设备 1~4、0x2FF 对应 5~7，
   *        满量程 ±25000；反馈 0x204+ID，设备总数 7。
   */
  [[nodiscard]] static constexpr Dialect Gm6020Voltage() noexcept
  {
    return MakeDialect({0U, 0U, 0U}, {0x1FFU, 0x2FFU, 25000U}, 0x204U, 7U);
  }

  struct ControlFrame final
  {
    std::uint32_t identifier{0U};
    std::array<std::uint8_t, FrameDataLength> data{};
  };

  struct Feedback final
  {
    std::uint8_t deviceId{0U};
    std::uint16_t rotorAngleRaw{0U};
    std::int16_t rotorSpeedRaw{0};
    std::int16_t torqueCurrentRaw{0};
    std::uint8_t motorTemperatureCelsius{0U};
  };

  struct Statistics final
  {
    std::uint32_t encodedControlFrameCount{0U};
    std::uint32_t acceptedFrameCount{0U};
    std::uint32_t rejectedFrameCount{0U};
    std::uint32_t unknownIdentifierCount{0U};
    std::uint32_t unsupportedCommandCount{0U};
  };

  explicit DjiEsc(const Dialect &dialect) noexcept;

  /**
   * @brief 清零全部统计，保留绑定的 Dialect。
   */
  void Reset() noexcept;

  /**
   * @brief 将同组四台设备的控制值编码为一条指定种类的命令帧。
   * @param kind 命令种类，当前产品线未提供该种类时拒绝并计入统计。
   * @param group 控制帧对应的设备组。
   * @param controlCounts 控制值，任一值超出该命令种类有符号满量程时拒绝。
   * @param frame 输出帧，仅编码成功时写入。
   * @return 种类可用、该组在当前 Dialect 下存在设备且控制值全部合法时
   *         返回 true。
   */
  [[nodiscard]] bool Encode(CommandKind kind,
                            Group group,
                            const std::array<std::int16_t, DevicesPerFrame> &controlCounts,
                            ControlFrame &frame) noexcept;

  /**
   * @brief 解析一条反馈帧，仅返回 Accepted 时写入 feedback。
   * @param identifier 收到的帧标识符。
   * @param extendedIdentifier 为 true 表示扩展帧，家族反馈帧均为标准帧。
   * @param dataLength 收到的帧数据长度。
   * @param data 帧数据指针。
   * @param feedback 输出解析结果。
   */
  [[nodiscard]] DecodeResult DecodeFeedback(std::uint32_t identifier,
                                            bool extendedIdentifier,
                                            std::uint8_t dataLength,
                                            const std::uint8_t *data,
                                            Feedback &feedback) noexcept;

  /**
   * @brief 把任意控制值钳位到指定命令种类的有符号满量程内。
   * @note 种类不可用时返回 0。
   */
  [[nodiscard]] std::int16_t Saturate(std::int32_t controlCounts, CommandKind kind) const noexcept;

  /**
   * @brief 将控制值换算为 [-1, 1] 的有符号比例，供设备层乘具体电气量程。
   * @note 种类不可用或满量程为 0 时返回 0。
   */
  [[nodiscard]] static constexpr float ControlRatio(std::int16_t controlCounts, const CommandSpec &spec) noexcept
  {
    if (spec.controlFullScaleCounts == 0U)
    {
      return 0.0F;
    }
    return static_cast<float>(controlCounts) / static_cast<float>(spec.controlFullScaleCounts);
  }

  /**
   * @brief 返回当前 Dialect 中指定命令种类的规格描述。
   */
  [[nodiscard]] constexpr const CommandSpec &GetCommandSpec(CommandKind kind) const noexcept
  {
    return dialect_.commands[static_cast<std::size_t>(kind)];
  }

  /**
   * @brief 返回指定设备编号所属的控制组。
   * @note deviceId 必须已确认处于 [1, maximumDeviceCount]。
   */
  [[nodiscard]] static Group GroupOf(std::uint8_t deviceId) noexcept
  {
    return deviceId <= static_cast<std::uint8_t>(DevicesPerFrame) ? Group::First : Group::Second;
  }

  /**
   * @brief 将转子机械角原始计数换算为机械角度数。
   */
  [[nodiscard]] static constexpr float RotorAngleDegrees(std::uint16_t rotorAngleRaw) noexcept
  {
    return static_cast<float>(rotorAngleRaw) * 360.0F / static_cast<float>(AngleCountsPerRevolution);
  }

  [[nodiscard]] const Statistics &GetStatistics() const noexcept
  {
    return statistics_;
  }

  [[nodiscard]] const Dialect &GetDialect() const noexcept
  {
    return dialect_;
  }

private:
  Dialect dialect_;
  Statistics statistics_{};
};

} // namespace protocol

#endif
