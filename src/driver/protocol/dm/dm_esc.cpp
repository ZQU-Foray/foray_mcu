#include "driver/protocol/dm/dm_esc.hpp"

#include <cmath>
#include <cstring>

namespace protocol
{

namespace
{

constexpr std::uint8_t POSITION_BITS{16U};
constexpr std::uint8_t TWELVE_BITS{12U};
constexpr std::uint8_t SAVE_REGISTERS_CODE{0x01U};
constexpr std::uint8_t SYSTEM_COMMAND_FILL{0xFFU};
constexpr float FORCE_POSITION_VELOCITY_SCALE{100.0F};
constexpr float FORCE_POSITION_CURRENT_SCALE{10000.0F};

bool valid_esc_id(std::uint16_t esc_id)
{
    return esc_id >= 1U && esc_id <= DmEsc::MAX_ESC_ID;
}

std::uint32_t float_bits(float value)
{
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

void put_u32(std::array<std::uint8_t, DmEsc::FRAME_DATA_LENGTH> &data, std::size_t offset, std::uint32_t value)
{
    data[offset] = static_cast<std::uint8_t>(value & 0xFFU);
    data[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    data[offset + 2U] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    data[offset + 3U] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

void put_u16(std::array<std::uint8_t, DmEsc::FRAME_DATA_LENGTH> &data, std::size_t offset, std::uint16_t value)
{
    data[offset] = static_cast<std::uint8_t>(value & 0xFFU);
    data[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

std::uint32_t get_u32(const std::array<std::uint8_t, DmEsc::FRAME_DATA_LENGTH> &data, std::size_t offset)
{
    return static_cast<std::uint32_t>(data[offset]) | (static_cast<std::uint32_t>(data[offset + 1U]) << 8U) |
           (static_cast<std::uint32_t>(data[offset + 2U]) << 16U) |
           (static_cast<std::uint32_t>(data[offset + 3U]) << 24U);
}

std::uint16_t scale_to_u16(float value, float scale)
{
    const float scaled = value * scale;
    if (!(scaled > 0.0F))
    {
        return 0U;
    }
    const float limit = static_cast<float>(DmEsc::FORCE_POSITION_SCALE_MAX);
    return scaled >= limit ? DmEsc::FORCE_POSITION_SCALE_MAX : static_cast<std::uint16_t>(std::lround(scaled));
}

DmEsc::Frame register_request(std::uint16_t esc_id, DmEsc::RegisterOperation operation, std::uint8_t address)
{
    DmEsc::Frame frame{};
    frame.identifier = DmEsc::REGISTER_IDENTIFIER;
    frame.data[0] = static_cast<std::uint8_t>(esc_id & 0xFFU);
    frame.data[1] = static_cast<std::uint8_t>((esc_id >> 8U) & 0xFFU);
    frame.data[2] = static_cast<std::uint8_t>(operation);
    frame.data[3] = address;
    return frame;
}

} // namespace

std::uint16_t DmEsc::to_unsigned(float value, float minimum, float maximum, std::uint8_t bits) noexcept
{
    const float span = maximum - minimum;
    if (!std::isfinite(value) || !(span > 0.0F) || bits == 0U || bits > 16U)
    {
        return 0U;
    }
    const float clamped = value < minimum ? minimum : (value > maximum ? maximum : value);
    const float full_scale = static_cast<float>((1UL << bits) - 1UL);
    // 与说明书 float_to_uint 相同：截断取整
    return static_cast<std::uint16_t>((clamped - minimum) * full_scale / span);
}

float DmEsc::from_unsigned(std::uint16_t value, float minimum, float maximum, std::uint8_t bits) noexcept
{
    if (bits == 0U || bits > 16U)
    {
        return 0.0F;
    }
    const float full_scale = static_cast<float>((1UL << bits) - 1UL);
    return static_cast<float>(value) * (maximum - minimum) / full_scale + minimum;
}

bool DmEsc::is_valid(const Limits &limits) noexcept
{
    return std::isfinite(limits.position_rad) && limits.position_rad > 0.0F &&
           std::isfinite(limits.velocity_rad_per_second) && limits.velocity_rad_per_second > 0.0F &&
           std::isfinite(limits.torque_newton_meter) && limits.torque_newton_meter > 0.0F;
}

std::uint32_t DmEsc::control_identifier(std::uint16_t esc_id, ControlMode mode) noexcept
{
    if (!valid_esc_id(esc_id))
    {
        return 0U;
    }
    switch (mode)
    {
    case ControlMode::MIT:
        return esc_id;
    case ControlMode::POSITION_VELOCITY:
        return 0x100U + esc_id;
    case ControlMode::VELOCITY:
        return 0x200U + esc_id;
    case ControlMode::FORCE_POSITION:
        return 0x300U + esc_id;
    }
    return 0U;
}

bool DmEsc::encode_mit(std::uint16_t esc_id, const MitCommand &command, const Limits &limits, Frame &frame) noexcept
{
    if (!valid_esc_id(esc_id) || !is_valid(limits) || !std::isfinite(command.position_rad) ||
        !std::isfinite(command.velocity_rad_per_second) || !std::isfinite(command.kp) || !std::isfinite(command.kd) ||
        !std::isfinite(command.torque_newton_meter))
    {
        return false;
    }
    const std::uint16_t position =
        to_unsigned(command.position_rad, -limits.position_rad, limits.position_rad, POSITION_BITS);
    const std::uint16_t velocity = to_unsigned(
        command.velocity_rad_per_second, -limits.velocity_rad_per_second, limits.velocity_rad_per_second, TWELVE_BITS);
    const std::uint16_t kp = to_unsigned(command.kp, 0.0F, MIT_KP_MAX, TWELVE_BITS);
    const std::uint16_t kd = to_unsigned(command.kd, 0.0F, MIT_KD_MAX, TWELVE_BITS);
    const std::uint16_t torque =
        to_unsigned(command.torque_newton_meter, -limits.torque_newton_meter, limits.torque_newton_meter, TWELVE_BITS);

    Frame result{};
    result.identifier = control_identifier(esc_id, ControlMode::MIT);
    result.length = static_cast<std::uint8_t>(FRAME_DATA_LENGTH);
    result.data[0] = static_cast<std::uint8_t>(position >> 8U);
    result.data[1] = static_cast<std::uint8_t>(position & 0xFFU);
    result.data[2] = static_cast<std::uint8_t>(velocity >> 4U);
    result.data[3] = static_cast<std::uint8_t>(((velocity & 0x0FU) << 4U) | (kp >> 8U));
    result.data[4] = static_cast<std::uint8_t>(kp & 0xFFU);
    result.data[5] = static_cast<std::uint8_t>(kd >> 4U);
    result.data[6] = static_cast<std::uint8_t>(((kd & 0x0FU) << 4U) | (torque >> 8U));
    result.data[7] = static_cast<std::uint8_t>(torque & 0xFFU);
    frame = result;
    return true;
}

bool DmEsc::encode_position_velocity(std::uint16_t esc_id,
                                     float position_rad,
                                     float velocity_rad_per_second,
                                     Frame &frame) noexcept
{
    if (!valid_esc_id(esc_id) || !std::isfinite(position_rad) || !std::isfinite(velocity_rad_per_second))
    {
        return false;
    }
    Frame result{};
    result.identifier = control_identifier(esc_id, ControlMode::POSITION_VELOCITY);
    result.length = static_cast<std::uint8_t>(FRAME_DATA_LENGTH);
    put_u32(result.data, 0U, float_bits(position_rad));
    put_u32(result.data, 4U, float_bits(velocity_rad_per_second));
    frame = result;
    return true;
}

bool DmEsc::encode_velocity(std::uint16_t esc_id, float velocity_rad_per_second, Frame &frame) noexcept
{
    if (!valid_esc_id(esc_id) || !std::isfinite(velocity_rad_per_second))
    {
        return false;
    }
    Frame result{};
    result.identifier = control_identifier(esc_id, ControlMode::VELOCITY);
    result.length = 4U;
    put_u32(result.data, 0U, float_bits(velocity_rad_per_second));
    frame = result;
    return true;
}

bool DmEsc::encode_force_position(std::uint16_t esc_id,
                                  float position_rad,
                                  float velocity_limit_rad_per_second,
                                  float current_limit_per_unit,
                                  Frame &frame) noexcept
{
    if (!valid_esc_id(esc_id) || !std::isfinite(position_rad) || !std::isfinite(velocity_limit_rad_per_second) ||
        !std::isfinite(current_limit_per_unit))
    {
        return false;
    }
    Frame result{};
    result.identifier = control_identifier(esc_id, ControlMode::FORCE_POSITION);
    result.length = static_cast<std::uint8_t>(FRAME_DATA_LENGTH);
    put_u32(result.data, 0U, float_bits(position_rad));
    put_u16(result.data, 4U, scale_to_u16(velocity_limit_rad_per_second, FORCE_POSITION_VELOCITY_SCALE));
    put_u16(result.data, 6U, scale_to_u16(current_limit_per_unit, FORCE_POSITION_CURRENT_SCALE));
    frame = result;
    return true;
}

bool DmEsc::encode_system(std::uint16_t esc_id, ControlMode mode, SystemCommand command, Frame &frame) noexcept
{
    const std::uint32_t identifier = control_identifier(esc_id, mode);
    if (identifier == 0U)
    {
        return false;
    }
    Frame result{};
    result.identifier = identifier;
    result.length = static_cast<std::uint8_t>(FRAME_DATA_LENGTH);
    result.data.fill(SYSTEM_COMMAND_FILL);
    result.data[FRAME_DATA_LENGTH - 1U] = static_cast<std::uint8_t>(command);
    frame = result;
    return true;
}

bool DmEsc::encode_read_register(std::uint16_t esc_id, Register address, Frame &frame) noexcept
{
    if (!valid_esc_id(esc_id))
    {
        return false;
    }
    Frame result = register_request(esc_id, RegisterOperation::READ, static_cast<std::uint8_t>(address));
    result.length = 4U;
    frame = result;
    return true;
}

bool DmEsc::encode_write_register(std::uint16_t esc_id, Register address, std::uint32_t value, Frame &frame) noexcept
{
    if (!valid_esc_id(esc_id))
    {
        return false;
    }
    Frame result = register_request(esc_id, RegisterOperation::WRITE, static_cast<std::uint8_t>(address));
    result.length = static_cast<std::uint8_t>(FRAME_DATA_LENGTH);
    put_u32(result.data, 4U, value);
    frame = result;
    return true;
}

bool DmEsc::encode_write_register_float(std::uint16_t esc_id, Register address, float value, Frame &frame) noexcept
{
    return std::isfinite(value) && encode_write_register(esc_id, address, float_bits(value), frame);
}

bool DmEsc::encode_save_registers(std::uint16_t esc_id, Frame &frame) noexcept
{
    if (!valid_esc_id(esc_id))
    {
        return false;
    }
    Frame result = register_request(esc_id, RegisterOperation::SAVE, SAVE_REGISTERS_CODE);
    result.length = 4U;
    frame = result;
    return true;
}

bool DmEsc::decode_feedback(const Frame &frame, FeedbackRaw &feedback) noexcept
{
    if (frame.length != FRAME_DATA_LENGTH)
    {
        return false;
    }
    const auto &data = frame.data;
    FeedbackRaw result{};
    result.id_low_bits = static_cast<std::uint8_t>(data[0] & 0x0FU);
    result.state = static_cast<State>(data[0] >> 4U);
    result.position = static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[1]) << 8U) | data[2]);
    result.velocity = static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[3]) << 4U) | (data[4] >> 4U));
    result.torque = static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[4] & 0x0FU) << 8U) | data[5]);
    result.mos_temperature_celsius = data[6];
    result.rotor_temperature_celsius = data[7];
    feedback = result;
    return true;
}

bool DmEsc::decode_feedback(const Frame &frame, const Limits &limits, Feedback &feedback) noexcept
{
    FeedbackRaw raw{};
    if (!is_valid(limits) || !decode_feedback(frame, raw))
    {
        return false;
    }
    Feedback result{};
    result.id_low_bits = raw.id_low_bits;
    result.state = raw.state;
    result.position_rad = from_unsigned(raw.position, -limits.position_rad, limits.position_rad, POSITION_BITS);
    result.velocity_rad_per_second =
        from_unsigned(raw.velocity, -limits.velocity_rad_per_second, limits.velocity_rad_per_second, TWELVE_BITS);
    result.torque_newton_meter =
        from_unsigned(raw.torque, -limits.torque_newton_meter, limits.torque_newton_meter, TWELVE_BITS);
    result.mos_temperature_celsius = raw.mos_temperature_celsius;
    result.rotor_temperature_celsius = raw.rotor_temperature_celsius;
    feedback = result;
    return true;
}

bool DmEsc::decode_register_reply(const Frame &frame, std::uint16_t esc_id, RegisterReply &reply) noexcept
{
    if (frame.length < 4U)
    {
        return false;
    }
    const auto &data = frame.data;
    const auto id = static_cast<std::uint16_t>(data[0] | (static_cast<std::uint16_t>(data[1]) << 8U));
    if (id != esc_id)
    {
        return false;
    }
    RegisterReply result{};
    result.esc_id = id;
    result.register_address = data[3];
    switch (data[2])
    {
    case static_cast<std::uint8_t>(RegisterOperation::READ):
    case static_cast<std::uint8_t>(RegisterOperation::WRITE):
        if (frame.length != FRAME_DATA_LENGTH)
        {
            return false;
        }
        result.operation = static_cast<RegisterOperation>(data[2]);
        result.value = get_u32(data, 4U);
        break;
    case static_cast<std::uint8_t>(RegisterOperation::SAVE):
        if (data[3] != SAVE_REGISTERS_CODE)
        {
            return false;
        }
        result.operation = RegisterOperation::SAVE;
        break;
    default:
        return false;
    }
    reply = result;
    return true;
}

float DmEsc::value_as_float(std::uint32_t value) noexcept
{
    float result = 0.0F;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

} // namespace protocol
