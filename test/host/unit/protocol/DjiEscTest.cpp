#include "Libraries/Protocol/dji/DjiEsc.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace {

using Codec = protocol::DjiEsc;
using Kind = Codec::CommandKind;
using Group = Codec::Group;

using Targets = std::array<std::int16_t, Codec::DevicesPerFrame>;

// 反馈帧 DLC 按手册取字面量 8，避免与被测实现的常量来源耦合。
constexpr std::uint8_t FeedbackDlc{8U};

Targets MakeTargets(std::int16_t first, std::int16_t second,
                    std::int16_t third, std::int16_t fourth) noexcept {
  return {first, second, third, fourth};
}

} // namespace

int main() {
  // C610：电流控制双组 ID 与固定外部黄金帧，防止测试编码器与实现编码器
  // 因共享错误而形成假通过。
  const std::array<std::uint8_t, Codec::FrameDataLength> goldenC610Data{
      0x03U, 0xE8U, 0xFCU, 0x18U, 0x00U, 0x00U, 0xD8U, 0xF0U};
  const auto c610Targets = MakeTargets(1000, -1000, 0, -10000);
  Codec c610{Codec::C610()};
  Codec::ControlFrame frame{};
  assert(c610.Encode(Kind::Current, Group::First, c610Targets, frame));
  assert(frame.identifier == 0x200U);
  assert(frame.data == goldenC610Data);
  assert(c610.Encode(Kind::Current, Group::Second, c610Targets, frame));
  assert(frame.identifier == 0x1FFU);
  assert(frame.data == goldenC610Data);

  Codec::ControlFrame untouched{};
  untouched.identifier = 0xABCDU;
  assert(!c610.Encode(Kind::Voltage, Group::First, c610Targets, untouched));
  assert(!c610.Encode(Kind::Voltage, Group::First, c610Targets, untouched));
  assert(untouched.identifier == 0xABCDU);
  assert(!c610.Encode(Kind::Current, Group::First,
                      MakeTargets(10001, 0, 0, 0), untouched));
  assert(!c610.Encode(Kind::Current, Group::First,
                      MakeTargets(0, 0, 0, -10001), untouched));
  assert(untouched.identifier == 0xABCDU);

  // C620：分组同 C610，满量程 ±16384。
  const std::array<std::uint8_t, Codec::FrameDataLength> goldenC620Data{
      0x40U, 0x00U, 0xC0U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  Codec c620{Codec::C620()};
  assert(c620.Encode(Kind::Current, Group::First,
                     MakeTargets(16384, -16384, 0, 0), frame));
  assert(frame.identifier == 0x200U);
  assert(frame.data == goldenC620Data);
  assert(!c620.Encode(Kind::Current, Group::First,
                      MakeTargets(16384, 16385, 0, 0), frame));
  assert(c620.ControlRatio(16384, c620.GetCommandSpec(Kind::Current)) == 1.0F);

  // GM6020 电压模式：控制 ID 分组、反馈基准与设备总数均为另一套方言。
  const std::array<std::uint8_t, Codec::FrameDataLength> goldenGmVoltageData{
      0x61U, 0xA8U, 0x9EU, 0x58U, 0x00U, 0x01U, 0xFFU, 0xFFU};
  Codec gmVoltage{Codec::Gm6020Voltage()};
  const auto gmVoltageTargets = MakeTargets(25000, -25000, 1, -1);
  assert(gmVoltage.Encode(Kind::Voltage, Group::First, gmVoltageTargets,
                          frame));
  assert(frame.identifier == 0x1FFU);
  assert(frame.data == goldenGmVoltageData);
  assert(gmVoltage.Encode(Kind::Voltage, Group::Second, gmVoltageTargets,
                          frame));
  assert(frame.identifier == 0x2FFU);
  assert(!gmVoltage.Encode(Kind::Current, Group::First, gmVoltageTargets,
                           frame));
  assert(!gmVoltage.Encode(Kind::Voltage, Group::First,
                           MakeTargets(25000, 25001, 0, 0), frame));
  assert(gmVoltage.Saturate(-30000, Kind::Voltage) == -25000);
  assert(gmVoltage.Saturate(300, Kind::Current) == 0);
  const auto &gmVoltageSpec = gmVoltage.GetCommandSpec(Kind::Voltage);
  assert(gmVoltage.ControlRatio(25000, gmVoltageSpec) == 1.0F);

  // GM6020 电流模式：满量程 ±16384。
  const std::array<std::uint8_t, Codec::FrameDataLength> goldenGmCurrentData{
      0x40U, 0x00U, 0xC0U, 0x00U, 0x00U, 0x00U, 0xFFU, 0xFDU};
  Codec gmCurrent{Codec::Gm6020Current()};
  assert(gmCurrent.Encode(Kind::Current, Group::First,
                          MakeTargets(16384, -16384, 0, -3), frame));
  assert(frame.identifier == 0x1FEU);
  assert(frame.data == goldenGmCurrentData);
  assert(gmCurrent.Encode(Kind::Current, Group::Second,
                          MakeTargets(0, 0, 0, 0), frame));
  assert(frame.identifier == 0x2FEU);
  assert(!gmCurrent.Encode(Kind::Voltage, Group::First, gmVoltageTargets,
                           frame));

  // 反馈：C610 家族基 0x200，设备 5 落在第二组。
  const std::array<std::uint8_t, Codec::FrameDataLength> goldenC610Feedback{
      0x1FU, 0xFFU, 0x3AU, 0x98U, 0xECU, 0x78U, 0x00U, 0x00U};
  Codec::Feedback feedback{};
  assert(c610.DecodeFeedback(0x205U, false, FeedbackDlc,
                             goldenC610Feedback.data(),
                             feedback) == Codec::DecodeResult::Accepted);
  assert(feedback.deviceId == 5U);
  assert(Codec::GroupOf(feedback.deviceId) == Group::Second);
  assert(feedback.rotorAngleRaw == 8191U);
  assert(feedback.rotorSpeedRaw == 15000);
  assert(feedback.torqueCurrentRaw == -5000);
  assert(feedback.motorTemperatureCelsius == 0U);

  // 反馈：转速域为有符号 16 位，电机反转（输出轴顺时针）时为负值。
  const std::array<std::uint8_t, Codec::FrameDataLength> reverseFeedback{
      0x00U, 0x01U, 0xFFU, 0x9CU, 0x00U, 0x64U, 0x00U, 0x00U};
  assert(c610.DecodeFeedback(0x202U, false, FeedbackDlc,
                             reverseFeedback.data(),
                             feedback) == Codec::DecodeResult::Accepted);
  assert(feedback.deviceId == 2U);
  assert(feedback.rotorAngleRaw == 1U);
  assert(feedback.rotorSpeedRaw == -100);
  assert(feedback.torqueCurrentRaw == 100);

  // 反馈：GM6020 基 0x204，0x205 是设备 1，且温度字段有效。
  const std::array<std::uint8_t, Codec::FrameDataLength> goldenGmFeedback{
      0x08U, 0x00U, 0x00U, 0x64U, 0xFFU, 0xF4U, 0x32U, 0x00U};
  assert(gmVoltage.DecodeFeedback(0x205U, false, FeedbackDlc,
                                  goldenGmFeedback.data(),
                                  feedback) == Codec::DecodeResult::Accepted);
  assert(feedback.deviceId == 1U);
  assert(Codec::GroupOf(feedback.deviceId) == Group::First);
  assert(feedback.rotorAngleRaw == 2048U);
  assert(feedback.rotorSpeedRaw == 100);
  assert(feedback.torqueCurrentRaw == -12);
  assert(feedback.motorTemperatureCelsius == 50U);
  assert(Codec::RotorAngleDegrees(feedback.rotorAngleRaw) == 90.0F);

  // GM6020 上限 7 台：0x20B 是设备 7，基值之上越界与 C610 家族标识符
  // 均为未知。
  Codec::Feedback stale{7U, 7U, 7, 7, 7U};
  assert(gmVoltage.DecodeFeedback(0x20CU, false, FeedbackDlc,
                                  goldenGmFeedback.data(),
                                  stale) ==
         Codec::DecodeResult::UnknownIdentifier);
  assert(gmVoltage.DecodeFeedback(0x204U, false, FeedbackDlc,
                                  goldenGmFeedback.data(),
                                  stale) ==
         Codec::DecodeResult::UnknownIdentifier);
  assert(gmCurrent.DecodeFeedback(0x201U, false, FeedbackDlc,
                                  goldenC610Feedback.data(),
                                  stale) ==
         Codec::DecodeResult::UnknownIdentifier);
  assert(stale.deviceId == 7U);
  assert(stale.rotorSpeedRaw == 7);
  assert(gmVoltage.DecodeFeedback(0x20BU, false, FeedbackDlc,
                                  goldenGmFeedback.data(),
                                  feedback) == Codec::DecodeResult::Accepted);
  assert(feedback.deviceId == 7U);

  // 非法形态：扩展帧、错误长度与空指针均拒绝且不写输出。
  assert(gmCurrent.DecodeFeedback(0x205U, true, FeedbackDlc,
                                  goldenGmFeedback.data(),
                                  stale) ==
         Codec::DecodeResult::InvalidArgument);
  assert(gmCurrent.DecodeFeedback(0x205U, false, 7U,
                                  goldenGmFeedback.data(),
                                  stale) ==
         Codec::DecodeResult::InvalidArgument);
  assert(gmCurrent.DecodeFeedback(0x205U, false, FeedbackDlc, nullptr,
                                  stale) ==
         Codec::DecodeResult::InvalidArgument);
  assert(stale.deviceId == 7U);

  // 换算与钳位原语。
  assert(Codec::RotorAngleDegrees(0U) == 0.0F);
  assert(Codec::RotorAngleDegrees(8191U) == 359.9560546875F);
  const auto &c610Spec = c610.GetCommandSpec(Kind::Current);
  assert(c610.ControlRatio(5000, c610Spec) == 0.5F);
  assert(c610.ControlRatio(-10000, c610Spec) == -1.0F);
  assert(c610.ControlRatio(100, c610.GetCommandSpec(Kind::Voltage)) == 0.0F);
  assert(c610.Saturate(20000, Kind::Current) == 10000);
  assert(c610.Saturate(-20000, Kind::Current) == -10000);
  assert(c610.Saturate(300, Kind::Current) == 300);

  // 合成 4 设备方言：第二组无设备，编码拒绝但不计入不支持统计。
  const Codec::Dialect fourDeviceDialect =
      Codec::MakeDialect({0x200U, 0x1FFU, 10000U}, {0U, 0U, 0U}, 0x200U, 4U);
  Codec narrow{fourDeviceDialect};
  const auto beforeNarrow = narrow.GetStatistics();
  assert(!narrow.Encode(Kind::Current, Group::Second, c610Targets, frame));
  assert(narrow.GetStatistics().unsupportedCommandCount ==
         beforeNarrow.unsupportedCommandCount);
  assert(narrow.Encode(Kind::Current, Group::First, c610Targets, frame));

  // 统计按实例独立：c610 只经历了 2 次成功编码、2 次接受解码与 2 次
  // 不支持命令拒绝，非法形态与未知标识符均发生在 GM 实例上。
  const Codec::Statistics &statistics = c610.GetStatistics();
  assert(statistics.encodedControlFrameCount == 2U);
  assert(statistics.acceptedFrameCount == 2U);
  assert(statistics.rejectedFrameCount == 0U);
  assert(statistics.unknownIdentifierCount == 0U);
  assert(statistics.unsupportedCommandCount == 2U);

  const Codec::Statistics &gmStatistics = gmCurrent.GetStatistics();
  assert(gmStatistics.acceptedFrameCount == 0U);
  assert(gmStatistics.rejectedFrameCount == 3U);
  assert(gmStatistics.unknownIdentifierCount == 1U);
  assert(gmStatistics.unsupportedCommandCount == 1U);

  const Codec::Statistics &gmVoltageStatistics = gmVoltage.GetStatistics();
  assert(gmVoltageStatistics.encodedControlFrameCount == 2U);
  assert(gmVoltageStatistics.acceptedFrameCount == 2U);
  assert(gmVoltageStatistics.unknownIdentifierCount == 2U);
  assert(gmVoltageStatistics.unsupportedCommandCount == 1U);

  c610.Reset();
  const Codec::Statistics &cleared = c610.GetStatistics();
  assert(cleared.encodedControlFrameCount == 0U);
  assert(cleared.acceptedFrameCount == 0U);
  assert(cleared.rejectedFrameCount == 0U);
  assert(cleared.unknownIdentifierCount == 0U);
  assert(cleared.unsupportedCommandCount == 0U);
  return 0;
}
