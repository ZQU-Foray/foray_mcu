#include "Libraries/Protocol/dji/DjiEsc.hpp"

namespace protocol
{

namespace
{

std::uint16_t ReadBigEndian16(const std::uint8_t *bytes) noexcept
{
  return static_cast<std::uint16_t>((static_cast<std::uint32_t>(bytes[0]) << 8U) |
                                    static_cast<std::uint32_t>(bytes[1]));
}

void WriteBigEndian16(std::uint8_t *bytes, std::int16_t value) noexcept
{
  const std::uint16_t raw = static_cast<std::uint16_t>(value);
  bytes[0] = static_cast<std::uint8_t>(raw >> 8U);
  bytes[1] = static_cast<std::uint8_t>(raw & 0xFFU);
}

} // namespace

DjiEsc::DjiEsc(const Dialect &dialect) noexcept : dialect_{dialect} {}

void DjiEsc::Reset() noexcept
{
  statistics_ = {};
}

bool DjiEsc::Encode(CommandKind kind,
                    Group group,
                    const std::array<std::int16_t, DevicesPerFrame> &controlCounts,
                    ControlFrame &frame) noexcept
{
  const CommandSpec &spec = GetCommandSpec(kind);
  const std::uint32_t identifier =
      group == Group::First ? spec.groupOneControlIdentifier : spec.groupTwoControlIdentifier;
  if (identifier == 0U)
  {
    ++statistics_.unsupportedCommandCount;
    return false;
  }

  if (group == Group::Second && dialect_.maximumDeviceCount <= static_cast<std::uint8_t>(DevicesPerFrame))
  {
    return false;
  }

  const std::int32_t fullScale = static_cast<std::int32_t>(spec.controlFullScaleCounts);
  for (const std::int16_t counts : controlCounts)
  {
    if (static_cast<std::int32_t>(counts) > fullScale || static_cast<std::int32_t>(counts) < -fullScale)
    {
      return false;
    }
  }

  ControlFrame encodedFrame{};
  encodedFrame.identifier = identifier;
  for (std::size_t index = 0U; index < DevicesPerFrame; ++index)
  {
    WriteBigEndian16(&encodedFrame.data[index * 2U], controlCounts[index]);
  }

  frame = encodedFrame;
  ++statistics_.encodedControlFrameCount;
  return true;
}

DjiEsc::DecodeResult DjiEsc::DecodeFeedback(std::uint32_t identifier,
                                            bool extendedIdentifier,
                                            std::uint8_t dataLength,
                                            const std::uint8_t *data,
                                            Feedback &feedback) noexcept
{
  if (data == nullptr || extendedIdentifier || dataLength != static_cast<std::uint8_t>(FrameDataLength))
  {
    ++statistics_.rejectedFrameCount;
    return DecodeResult::InvalidArgument;
  }

  if (identifier <= dialect_.feedbackIdentifierBase ||
      identifier - dialect_.feedbackIdentifierBase > static_cast<std::uint32_t>(dialect_.maximumDeviceCount))
  {
    ++statistics_.unknownIdentifierCount;
    return DecodeResult::UnknownIdentifier;
  }

  Feedback decoded{};
  decoded.deviceId = static_cast<std::uint8_t>(identifier - dialect_.feedbackIdentifierBase);
  decoded.rotorAngleRaw = ReadBigEndian16(&data[0]);
  decoded.rotorSpeedRaw = static_cast<std::int16_t>(ReadBigEndian16(&data[2]));
  decoded.torqueCurrentRaw = static_cast<std::int16_t>(ReadBigEndian16(&data[4]));
  decoded.motorTemperatureCelsius = data[6];

  feedback = decoded;
  ++statistics_.acceptedFrameCount;
  return DecodeResult::Accepted;
}

std::int16_t DjiEsc::Saturate(std::int32_t controlCounts, CommandKind kind) const noexcept
{
  const CommandSpec &spec = GetCommandSpec(kind);
  if (spec.controlFullScaleCounts == 0U)
  {
    return 0;
  }

  const std::int32_t fullScale = static_cast<std::int32_t>(spec.controlFullScaleCounts);
  if (controlCounts > fullScale)
  {
    return static_cast<std::int16_t>(fullScale);
  }
  if (controlCounts < -fullScale)
  {
    return static_cast<std::int16_t>(-fullScale);
  }
  return static_cast<std::int16_t>(controlCounts);
}

} // namespace protocol
