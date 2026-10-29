#include "driver/protocol/dji/dji_esc.hpp"

namespace protocol
{

namespace
{

std::uint16_t read_big_endian16(const std::uint8_t *bytes) noexcept
{
    return static_cast<std::uint16_t>((static_cast<std::uint32_t>(bytes[0]) << 8U) |
                                      static_cast<std::uint32_t>(bytes[1]));
}

void write_big_endian16(std::uint8_t *bytes, std::int16_t value) noexcept
{
    const std::uint16_t raw = static_cast<std::uint16_t>(value);
    bytes[0] = static_cast<std::uint8_t>(raw >> 8U);
    bytes[1] = static_cast<std::uint8_t>(raw & 0xFFU);
}

} // namespace

DjiEsc::DjiEsc(const Dialect &dialect) noexcept : dialect_{dialect} {}

void DjiEsc::reset() noexcept
{
    statistics_ = {};
}

bool DjiEsc::encode(CommandKind kind,
                    Group group,
                    const std::array<std::int16_t, DEVICES_PER_FRAME> &control_counts,
                    ControlFrame &frame) noexcept
{
    const CommandSpec &spec = get_command_spec(kind);
    const std::uint32_t identifier =
        group == Group::FIRST ? spec.group_one_control_identifier : spec.group_two_control_identifier;
    if (identifier == 0U)
    {
        ++statistics_.unsupported_command_count;
        return false;
    }

    if (group == Group::SECOND && dialect_.maximum_device_count <= static_cast<std::uint8_t>(DEVICES_PER_FRAME))
    {
        return false;
    }

    const std::int32_t full_scale = static_cast<std::int32_t>(spec.control_full_scale_counts);
    for (const std::int16_t counts : control_counts)
    {
        if (static_cast<std::int32_t>(counts) > full_scale || static_cast<std::int32_t>(counts) < -full_scale)
        {
            return false;
        }
    }

    ControlFrame encoded_frame{};
    encoded_frame.identifier = identifier;
    for (std::size_t index = 0U; index < DEVICES_PER_FRAME; ++index)
    {
        write_big_endian16(&encoded_frame.data[index * 2U], control_counts[index]);
    }

    frame = encoded_frame;
    ++statistics_.encoded_control_frame_count;
    return true;
}

DjiEsc::DecodeResult DjiEsc::decode_feedback(std::uint32_t identifier,
                                             bool extended_identifier,
                                             std::uint8_t data_length,
                                             const std::uint8_t *data,
                                             Feedback &feedback) noexcept
{
    if (data == nullptr || extended_identifier || data_length != static_cast<std::uint8_t>(FRAME_DATA_LENGTH))
    {
        ++statistics_.rejected_frame_count;
        return DecodeResult::INVALID_ARGUMENT;
    }

    if (identifier <= dialect_.feedback_identifier_base ||
        identifier - dialect_.feedback_identifier_base > static_cast<std::uint32_t>(dialect_.maximum_device_count))
    {
        ++statistics_.unknown_identifier_count;
        return DecodeResult::UNKNOWN_IDENTIFIER;
    }

    Feedback decoded{};
    decoded.device_id = static_cast<std::uint8_t>(identifier - dialect_.feedback_identifier_base);
    decoded.rotor_angle_raw = read_big_endian16(&data[0]);
    decoded.rotor_speed_raw = static_cast<std::int16_t>(read_big_endian16(&data[2]));
    decoded.torque_current_raw = static_cast<std::int16_t>(read_big_endian16(&data[4]));
    decoded.motor_temperature_celsius = data[6];

    feedback = decoded;
    ++statistics_.accepted_frame_count;
    return DecodeResult::ACCEPTED;
}

std::int16_t DjiEsc::saturate(std::int32_t control_counts, CommandKind kind) const noexcept
{
    const CommandSpec &spec = get_command_spec(kind);
    if (spec.control_full_scale_counts == 0U)
    {
        return 0;
    }

    const std::int32_t full_scale = static_cast<std::int32_t>(spec.control_full_scale_counts);
    if (control_counts > full_scale)
    {
        return static_cast<std::int16_t>(full_scale);
    }
    if (control_counts < -full_scale)
    {
        return static_cast<std::int16_t>(-full_scale);
    }
    return static_cast<std::int16_t>(control_counts);
}

} // namespace protocol
