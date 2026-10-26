#include "Libraries/Protocol/dji/DjiEsc.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace
{

using Codec = protocol::DjiEsc;
using Kind = Codec::CommandKind;
using Group = Codec::Group;

using Targets = std::array<std::int16_t, Codec::DEVICES_PER_FRAME>;

// 反馈帧 DLC 按手册取字面量 8，避免与被测实现的常量来源耦合。
constexpr std::uint8_t FEEDBACK_DLC{8U};

Targets make_targets(std::int16_t first, std::int16_t second, std::int16_t third, std::int16_t fourth) noexcept
{
    return {first, second, third, fourth};
}

} // namespace

int main()
{
    // C610：电流控制双组 ID 与固定外部黄金帧，防止测试编码器与实现编码器
    // 因共享错误而形成假通过。
    const std::array<std::uint8_t, Codec::FRAME_DATA_LENGTH> golden_c610_data{
        0x03U, 0xE8U, 0xFCU, 0x18U, 0x00U, 0x00U, 0xD8U, 0xF0U};
    const auto c610_targets = make_targets(1000, -1000, 0, -10000);
    Codec c610{Codec::c610()};
    Codec::ControlFrame frame{};
    assert(c610.encode(Kind::CURRENT, Group::FIRST, c610_targets, frame));
    assert(frame.identifier == 0x200U);
    assert(frame.data == golden_c610_data);
    assert(c610.encode(Kind::CURRENT, Group::SECOND, c610_targets, frame));
    assert(frame.identifier == 0x1FFU);
    assert(frame.data == golden_c610_data);

    Codec::ControlFrame untouched{};
    untouched.identifier = 0xABCDU;
    assert(!c610.encode(Kind::VOLTAGE, Group::FIRST, c610_targets, untouched));
    assert(!c610.encode(Kind::VOLTAGE, Group::FIRST, c610_targets, untouched));
    assert(untouched.identifier == 0xABCDU);
    assert(!c610.encode(Kind::CURRENT, Group::FIRST, make_targets(10001, 0, 0, 0), untouched));
    assert(!c610.encode(Kind::CURRENT, Group::FIRST, make_targets(0, 0, 0, -10001), untouched));
    assert(untouched.identifier == 0xABCDU);

    // C620：分组同 C610，满量程 ±16384。
    const std::array<std::uint8_t, Codec::FRAME_DATA_LENGTH> golden_c620_data{
        0x40U, 0x00U, 0xC0U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    Codec c620{Codec::c620()};
    assert(c620.encode(Kind::CURRENT, Group::FIRST, make_targets(16384, -16384, 0, 0), frame));
    assert(frame.identifier == 0x200U);
    assert(frame.data == golden_c620_data);
    assert(!c620.encode(Kind::CURRENT, Group::FIRST, make_targets(16384, 16385, 0, 0), frame));
    assert(c620.control_ratio(16384, c620.get_command_spec(Kind::CURRENT)) == 1.0F);

    // GM6020 电压模式：控制 ID 分组、反馈基准与设备总数均为另一套方言。
    const std::array<std::uint8_t, Codec::FRAME_DATA_LENGTH> golden_gm_voltage_data{
        0x61U, 0xA8U, 0x9EU, 0x58U, 0x00U, 0x01U, 0xFFU, 0xFFU};
    Codec gm_voltage{Codec::gm6020_voltage()};
    const auto gm_voltage_targets = make_targets(25000, -25000, 1, -1);
    assert(gm_voltage.encode(Kind::VOLTAGE, Group::FIRST, gm_voltage_targets, frame));
    assert(frame.identifier == 0x1FFU);
    assert(frame.data == golden_gm_voltage_data);
    assert(gm_voltage.encode(Kind::VOLTAGE, Group::SECOND, gm_voltage_targets, frame));
    assert(frame.identifier == 0x2FFU);
    assert(!gm_voltage.encode(Kind::CURRENT, Group::FIRST, gm_voltage_targets, frame));
    assert(!gm_voltage.encode(Kind::VOLTAGE, Group::FIRST, make_targets(25000, 25001, 0, 0), frame));
    assert(gm_voltage.saturate(-30000, Kind::VOLTAGE) == -25000);
    assert(gm_voltage.saturate(300, Kind::CURRENT) == 0);
    const auto &gm_voltage_spec = gm_voltage.get_command_spec(Kind::VOLTAGE);
    assert(gm_voltage.control_ratio(25000, gm_voltage_spec) == 1.0F);

    // GM6020 电流模式：满量程 ±16384。
    const std::array<std::uint8_t, Codec::FRAME_DATA_LENGTH> golden_gm_current_data{
        0x40U, 0x00U, 0xC0U, 0x00U, 0x00U, 0x00U, 0xFFU, 0xFDU};
    Codec gm_current{Codec::gm6020_current()};
    assert(gm_current.encode(Kind::CURRENT, Group::FIRST, make_targets(16384, -16384, 0, -3), frame));
    assert(frame.identifier == 0x1FEU);
    assert(frame.data == golden_gm_current_data);
    assert(gm_current.encode(Kind::CURRENT, Group::SECOND, make_targets(0, 0, 0, 0), frame));
    assert(frame.identifier == 0x2FEU);
    assert(!gm_current.encode(Kind::VOLTAGE, Group::FIRST, gm_voltage_targets, frame));

    // 反馈：C610 家族基 0x200，设备 5 落在第二组。
    const std::array<std::uint8_t, Codec::FRAME_DATA_LENGTH> golden_c610_feedback{
        0x1FU, 0xFFU, 0x3AU, 0x98U, 0xECU, 0x78U, 0x00U, 0x00U};
    Codec::Feedback feedback{};
    assert(c610.decode_feedback(0x205U, false, FEEDBACK_DLC, golden_c610_feedback.data(), feedback) ==
           Codec::DecodeResult::ACCEPTED);
    assert(feedback.device_id == 5U);
    assert(Codec::group_of(feedback.device_id) == Group::SECOND);
    assert(feedback.rotor_angle_raw == 8191U);
    assert(feedback.rotor_speed_raw == 15000);
    assert(feedback.torque_current_raw == -5000);
    assert(feedback.motor_temperature_celsius == 0U);

    // 反馈：转速域为有符号 16 位，电机反转（输出轴顺时针）时为负值。
    const std::array<std::uint8_t, Codec::FRAME_DATA_LENGTH> reverse_feedback{
        0x00U, 0x01U, 0xFFU, 0x9CU, 0x00U, 0x64U, 0x00U, 0x00U};
    assert(c610.decode_feedback(0x202U, false, FEEDBACK_DLC, reverse_feedback.data(), feedback) ==
           Codec::DecodeResult::ACCEPTED);
    assert(feedback.device_id == 2U);
    assert(feedback.rotor_angle_raw == 1U);
    assert(feedback.rotor_speed_raw == -100);
    assert(feedback.torque_current_raw == 100);

    // 反馈：GM6020 基 0x204，0x205 是设备 1，且温度字段有效。
    const std::array<std::uint8_t, Codec::FRAME_DATA_LENGTH> golden_gm_feedback{
        0x08U, 0x00U, 0x00U, 0x64U, 0xFFU, 0xF4U, 0x32U, 0x00U};
    assert(gm_voltage.decode_feedback(0x205U, false, FEEDBACK_DLC, golden_gm_feedback.data(), feedback) ==
           Codec::DecodeResult::ACCEPTED);
    assert(feedback.device_id == 1U);
    assert(Codec::group_of(feedback.device_id) == Group::FIRST);
    assert(feedback.rotor_angle_raw == 2048U);
    assert(feedback.rotor_speed_raw == 100);
    assert(feedback.torque_current_raw == -12);
    assert(feedback.motor_temperature_celsius == 50U);
    assert(Codec::rotor_angle_degrees(feedback.rotor_angle_raw) == 90.0F);

    // GM6020 上限 7 台：0x20B 是设备 7，基值之上越界与 C610 家族标识符
    // 均为未知。
    Codec::Feedback stale{7U, 7U, 7, 7, 7U};
    assert(gm_voltage.decode_feedback(0x20CU, false, FEEDBACK_DLC, golden_gm_feedback.data(), stale) ==
           Codec::DecodeResult::UNKNOWN_IDENTIFIER);
    assert(gm_voltage.decode_feedback(0x204U, false, FEEDBACK_DLC, golden_gm_feedback.data(), stale) ==
           Codec::DecodeResult::UNKNOWN_IDENTIFIER);
    assert(gm_current.decode_feedback(0x201U, false, FEEDBACK_DLC, golden_c610_feedback.data(), stale) ==
           Codec::DecodeResult::UNKNOWN_IDENTIFIER);
    assert(stale.device_id == 7U);
    assert(stale.rotor_speed_raw == 7);
    assert(gm_voltage.decode_feedback(0x20BU, false, FEEDBACK_DLC, golden_gm_feedback.data(), feedback) ==
           Codec::DecodeResult::ACCEPTED);
    assert(feedback.device_id == 7U);

    // 非法形态：扩展帧、错误长度与空指针均拒绝且不写输出。
    assert(gm_current.decode_feedback(0x205U, true, FEEDBACK_DLC, golden_gm_feedback.data(), stale) ==
           Codec::DecodeResult::INVALID_ARGUMENT);
    assert(gm_current.decode_feedback(0x205U, false, 7U, golden_gm_feedback.data(), stale) ==
           Codec::DecodeResult::INVALID_ARGUMENT);
    assert(gm_current.decode_feedback(0x205U, false, FEEDBACK_DLC, nullptr, stale) ==
           Codec::DecodeResult::INVALID_ARGUMENT);
    assert(stale.device_id == 7U);

    // 换算与钳位原语。
    assert(Codec::rotor_angle_degrees(0U) == 0.0F);
    assert(Codec::rotor_angle_degrees(8191U) == 359.9560546875F);
    const auto &c610_spec = c610.get_command_spec(Kind::CURRENT);
    assert(c610.control_ratio(5000, c610_spec) == 0.5F);
    assert(c610.control_ratio(-10000, c610_spec) == -1.0F);
    assert(c610.control_ratio(100, c610.get_command_spec(Kind::VOLTAGE)) == 0.0F);
    assert(c610.saturate(20000, Kind::CURRENT) == 10000);
    assert(c610.saturate(-20000, Kind::CURRENT) == -10000);
    assert(c610.saturate(300, Kind::CURRENT) == 300);

    // 合成 4 设备方言：第二组无设备，编码拒绝但不计入不支持统计。
    const Codec::Dialect four_device_dialect = Codec::make_dialect({0x200U, 0x1FFU, 10000U}, {0U, 0U, 0U}, 0x200U, 4U);
    Codec narrow{four_device_dialect};
    const auto before_narrow = narrow.get_statistics();
    assert(!narrow.encode(Kind::CURRENT, Group::SECOND, c610_targets, frame));
    assert(narrow.get_statistics().unsupported_command_count == before_narrow.unsupported_command_count);
    assert(narrow.encode(Kind::CURRENT, Group::FIRST, c610_targets, frame));

    // 统计按实例独立：c610 只经历了 2 次成功编码、2 次接受解码与 2 次
    // 不支持命令拒绝，非法形态与未知标识符均发生在 GM 实例上。
    const Codec::Statistics &statistics = c610.get_statistics();
    assert(statistics.encoded_control_frame_count == 2U);
    assert(statistics.accepted_frame_count == 2U);
    assert(statistics.rejected_frame_count == 0U);
    assert(statistics.unknown_identifier_count == 0U);
    assert(statistics.unsupported_command_count == 2U);

    const Codec::Statistics &gm_statistics = gm_current.get_statistics();
    assert(gm_statistics.accepted_frame_count == 0U);
    assert(gm_statistics.rejected_frame_count == 3U);
    assert(gm_statistics.unknown_identifier_count == 1U);
    assert(gm_statistics.unsupported_command_count == 1U);

    const Codec::Statistics &gm_voltage_statistics = gm_voltage.get_statistics();
    assert(gm_voltage_statistics.encoded_control_frame_count == 2U);
    assert(gm_voltage_statistics.accepted_frame_count == 2U);
    assert(gm_voltage_statistics.unknown_identifier_count == 2U);
    assert(gm_voltage_statistics.unsupported_command_count == 1U);

    c610.reset();
    const Codec::Statistics &cleared = c610.get_statistics();
    assert(cleared.encoded_control_frame_count == 0U);
    assert(cleared.accepted_frame_count == 0U);
    assert(cleared.rejected_frame_count == 0U);
    assert(cleared.unknown_identifier_count == 0U);
    assert(cleared.unsupported_command_count == 0U);
    return 0;
}
