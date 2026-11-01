// protocol::DmEsc：样例按《DM-J4310-2EC V1.2 减速电机使用说明书》V1.4 的帧格式手算，
// 寄存器应答取自 2026-11-01 在 DM-MC02 的 CAN1 上读 DM-J4310（ESC_ID 1、MST_ID 0）的实测帧。
#include "driver/protocol/dm/dm_esc.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>

namespace
{

using Dm = protocol::DmEsc;
using Bytes = std::array<std::uint8_t, Dm::FRAME_DATA_LENGTH>;

// DM-J4310 出厂量程（实测回读：PMAX 12.5、VMAX 30、TMAX 10）
constexpr Dm::Limits J4310{12.5F, 30.0F, 10.0F};

Dm::Frame frame_of(std::uint32_t identifier, std::uint8_t length, const Bytes &data)
{
    Dm::Frame frame{};
    frame.identifier = identifier;
    frame.length = length;
    frame.data = data;
    return frame;
}

bool near(float actual, float expected, float tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

} // namespace

int main()
{
    Dm::Frame frame{};

    // ---- 帧 ID：控制模式的偏移 + ESC_ID；ESC_ID 只能是 1–0xFF ----
    assert(Dm::control_identifier(1U, Dm::ControlMode::MIT) == 0x001U);
    assert(Dm::control_identifier(1U, Dm::ControlMode::POSITION_VELOCITY) == 0x101U);
    assert(Dm::control_identifier(1U, Dm::ControlMode::VELOCITY) == 0x201U);
    assert(Dm::control_identifier(0x12U, Dm::ControlMode::FORCE_POSITION) == 0x312U);
    assert(Dm::control_identifier(0U, Dm::ControlMode::MIT) == 0U);
    assert(Dm::control_identifier(0x100U, Dm::ControlMode::MIT) == 0U);

    // ---- MIT：零指令是常见的 7F FF 7F F0 00 00 07 FF ----
    assert(Dm::encode_mit(1U, Dm::MitCommand{}, J4310, frame));
    assert(frame.identifier == 0x001U && frame.length == 8U);
    assert((frame.data == Bytes{0x7FU, 0xFFU, 0x7FU, 0xF0U, 0x00U, 0x00U, 0x07U, 0xFFU}));
    // p 1.0 → 35388（0x8A3C），v −3 → 1842（0x732），kp 50 → 409（0x199），kd 1 → 819（0x333），t 2.5 → 2559（0x9FF）
    assert(Dm::encode_mit(1U, Dm::MitCommand{1.0F, -3.0F, 50.0F, 1.0F, 2.5F}, J4310, frame));
    assert((frame.data == Bytes{0x8AU, 0x3CU, 0x73U, 0x21U, 0x99U, 0x33U, 0x39U, 0xFFU}));
    // 满量程与钳位
    assert(Dm::encode_mit(1U, Dm::MitCommand{12.5F, 30.0F, 500.0F, 5.0F, 10.0F}, J4310, frame));
    assert((frame.data == Bytes{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU}));
    assert(Dm::encode_mit(1U, Dm::MitCommand{100.0F, 100.0F, 1000.0F, 9.0F, 100.0F}, J4310, frame));
    assert((frame.data == Bytes{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU}));
    assert(Dm::encode_mit(1U, Dm::MitCommand{-100.0F, -100.0F, -1.0F, -1.0F, -100.0F}, J4310, frame));
    assert((frame.data == Bytes{}));
    // 参数无效：frame 不变
    const Dm::Frame before = frame;
    assert(!Dm::encode_mit(1U, Dm::MitCommand{NAN, 0.0F, 0.0F, 0.0F, 0.0F}, J4310, frame));
    assert(!Dm::encode_mit(1U, Dm::MitCommand{}, Dm::Limits{0.0F, 30.0F, 10.0F}, frame));
    assert(!Dm::encode_mit(0U, Dm::MitCommand{}, J4310, frame));
    assert(frame.data == before.data && frame.identifier == before.identifier);

    // ---- 位置速度：两个小端 float；速度：一个小端 float ----
    assert(Dm::encode_position_velocity(1U, 1.0F, 2.0F, frame));
    assert(frame.identifier == 0x101U && frame.length == 8U);
    assert((frame.data == Bytes{0x00U, 0x00U, 0x80U, 0x3FU, 0x00U, 0x00U, 0x00U, 0x40U}));
    assert(Dm::encode_velocity(1U, -1.5F, frame));
    assert(frame.identifier == 0x201U && frame.length == 4U);
    assert(frame.data[0] == 0x00U && frame.data[1] == 0x00U && frame.data[2] == 0xC0U && frame.data[3] == 0xBFU);

    // ---- 力位混控：float 位置 + u16 限速 ×100 + u16 电流标幺 ×10000，超出钳位 ----
    assert(Dm::encode_force_position(1U, 0.5F, 12.34F, 0.25F, frame));
    assert(frame.identifier == 0x301U && frame.length == 8U);
    assert((frame.data == Bytes{0x00U, 0x00U, 0x00U, 0x3FU, 0xD2U, 0x04U, 0xC4U, 0x09U}));
    assert(Dm::encode_force_position(1U, 0.0F, 200.0F, 1.5F, frame));
    assert(frame.data[4] == 0x10U && frame.data[5] == 0x27U && frame.data[6] == 0x10U && frame.data[7] == 0x27U);
    assert(Dm::encode_force_position(1U, 0.0F, -5.0F, -0.1F, frame));
    assert(frame.data[4] == 0U && frame.data[5] == 0U && frame.data[6] == 0U && frame.data[7] == 0U);

    // ---- 系统命令：帧 ID 跟随当前模式 ----
    assert(Dm::encode_system(1U, Dm::ControlMode::MIT, Dm::SystemCommand::ENABLE, frame));
    assert(frame.identifier == 0x001U && frame.length == 8U);
    assert((frame.data == Bytes{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU}));
    assert(Dm::encode_system(1U, Dm::ControlMode::VELOCITY, Dm::SystemCommand::DISABLE, frame));
    assert(frame.identifier == 0x201U && frame.data[7] == 0xFDU);
    assert(Dm::encode_system(1U, Dm::ControlMode::MIT, Dm::SystemCommand::CLEAR_ERROR, frame) &&
           frame.data[7] == 0xFBU);
    assert(Dm::encode_system(1U, Dm::ControlMode::MIT, Dm::SystemCommand::SAVE_ZERO, frame) && frame.data[7] == 0xFEU);
    assert(!Dm::encode_system(0U, Dm::ControlMode::MIT, Dm::SystemCommand::ENABLE, frame));

    // ---- 寄存器请求：ID 0x7FF ----
    assert(Dm::encode_read_register(1U, Dm::Register::CONTROL_MODE, frame));
    assert(frame.identifier == 0x7FFU && frame.length == 4U);
    assert(frame.data[0] == 0x01U && frame.data[1] == 0x00U && frame.data[2] == 0x33U && frame.data[3] == 0x0AU);
    assert(Dm::encode_write_register_float(1U, Dm::Register::TORQUE_MAX, 10.0F, frame));
    assert(frame.identifier == 0x7FFU && frame.length == 8U);
    assert((frame.data == Bytes{0x01U, 0x00U, 0x55U, 0x17U, 0x00U, 0x00U, 0x20U, 0x41U}));
    assert(Dm::encode_write_register(0x12U, Dm::Register::CONTROL_MODE, 1U, frame));
    assert((frame.data == Bytes{0x12U, 0x00U, 0x55U, 0x0AU, 0x01U, 0x00U, 0x00U, 0x00U}));
    assert(!Dm::encode_write_register_float(1U, Dm::Register::TORQUE_MAX, INFINITY, frame));
    assert(Dm::encode_save_registers(1U, frame));
    assert(frame.identifier == 0x7FFU && frame.length == 4U);
    assert(frame.data[0] == 0x01U && frame.data[1] == 0x00U && frame.data[2] == 0xAAU && frame.data[3] == 0x01U);

    // ---- 寄存器应答（实测帧，发到 MST_ID 0x000）----
    Dm::RegisterReply reply{};
    assert(Dm::decode_register_reply(
        frame_of(0x000U, 8U, {0x01U, 0x00U, 0x33U, 0x15U, 0x00U, 0x00U, 0x48U, 0x41U}), 1U, reply));
    assert(reply.operation == Dm::RegisterOperation::READ && reply.register_address == 0x15U);
    assert(Dm::value_as_float(reply.value) == 12.5F);
    assert(Dm::decode_register_reply(
        frame_of(0x000U, 8U, {0x01U, 0x00U, 0x33U, 0x0AU, 0x01U, 0x00U, 0x00U, 0x00U}), 1U, reply));
    assert(reply.value == static_cast<std::uint32_t>(Dm::ControlMode::MIT));
    assert(Dm::decode_register_reply(
        frame_of(0x000U, 8U, {0x01U, 0x00U, 0x33U, 0x0EU, 0x37U, 0x30U, 0x32U, 0x30U}), 1U, reply));
    assert(reply.value == 0x30323037U); // 软件版本："7020"
    assert(Dm::decode_register_reply(frame_of(0x000U, 4U, {0x01U, 0x00U, 0xAAU, 0x01U}), 1U, reply));
    assert(reply.operation == Dm::RegisterOperation::SAVE);
    // 别的电机、长度不对、操作码不对：不是应答
    assert(!Dm::decode_register_reply(
        frame_of(0x000U, 8U, {0x02U, 0x00U, 0x33U, 0x15U, 0x00U, 0x00U, 0x48U, 0x41U}), 1U, reply));
    assert(!Dm::decode_register_reply(frame_of(0x000U, 4U, {0x01U, 0x00U, 0x33U, 0x15U}), 1U, reply));
    assert(!Dm::decode_register_reply(
        frame_of(0x000U, 8U, {0x01U, 0x80U, 0x00U, 0x7FU, 0xF8U, 0x00U, 0x1AU, 0x1CU}), 1U, reply));

    // ---- 反馈：D0 = ID|ERR<<4，位置 16 位，速度、扭矩 12 位，两个温度 ----
    const Dm::Frame feedback_frame = frame_of(0x000U, 8U, {0x11U, 0x8AU, 0x3CU, 0x73U, 0x29U, 0xFFU, 35U, 40U});
    Dm::FeedbackRaw raw{};
    assert(Dm::decode_feedback(feedback_frame, raw));
    assert(raw.id_low_bits == 1U && raw.state == Dm::State::ENABLED);
    assert(raw.position == 0x8A3CU && raw.velocity == 0x732U && raw.torque == 0x9FFU);
    assert(raw.mos_temperature_celsius == 35U && raw.rotor_temperature_celsius == 40U);
    Dm::Feedback feedback{};
    assert(Dm::decode_feedback(feedback_frame, J4310, feedback));
    assert(near(feedback.position_rad, 1.0F, 0.001F));
    assert(near(feedback.velocity_rad_per_second, -3.0F, 0.02F));
    assert(near(feedback.torque_newton_meter, 2.5F, 0.01F));
    // 量程决定换算：同一帧按两倍量程解读，物理量也是两倍（说明书：量程不一致时控制量等比例缩放）
    assert(Dm::decode_feedback(feedback_frame, Dm::Limits{25.0F, 60.0F, 20.0F}, feedback));
    assert(near(feedback.position_rad, 2.0F, 0.002F) && near(feedback.torque_newton_meter, 5.0F, 0.02F));
    assert(!Dm::decode_feedback(frame_of(0x000U, 4U, {}), raw));
    assert(!Dm::decode_feedback(feedback_frame, Dm::Limits{12.5F, 0.0F, 10.0F}, feedback));

    // ---- 映射往返：误差不超过一个最小刻度 ----
    for (float value = -12.5F; value <= 12.5F; value += 0.37F)
    {
        const std::uint16_t encoded = Dm::to_unsigned(value, -12.5F, 12.5F, 16U);
        assert(near(Dm::from_unsigned(encoded, -12.5F, 12.5F, 16U), value, 25.0F / 65535.0F + 1e-5F));
    }
    return 0;
}
