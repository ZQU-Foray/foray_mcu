#ifndef TEST_HOST_UNIT_MOTOR_DM_FAKE_MOTOR_HPP
#define TEST_HOST_UNIT_MOTOR_DM_FAKE_MOTOR_HPP

// 测试里扮演达妙电机：从 mock CAN 的发送记录取帧，按《DM-J4310-2EC V1.2 减速电机使用说明书》V1.4 的格式应答。
// - 寄存器读写（0x7FF，D0–D1 = ESC_ID）：应答发到 MST_ID；写入只改"RAM"（成员变量）。
// - 控制帧（帧 ID 低 8 位 = ESC_ID，高 3 位忽略）：系统命令改状态，MIT 帧记下指令；每帧都回一帧反馈。
// - 使能只在没有故障时生效；故障要"清除错误"才回到失能。
// - update()：使能时超过 TIMEOUT × 50 µs 没收到控制帧，自己失能并报"通讯丢失"（TIMEOUT 为 0 时不保护）。
// - simulate_motion：按 MIT 指令的力矩与 Kd 取稳态转速（力矩 / Kd，不计惯量与摩擦），位置按收到 MIT 帧的间隔积分。
#include "driver/can_bus/can_bus.hpp"
#include "driver/protocol/dm/dm_esc.hpp"
#include "hal/can.hpp"
#include "hal/os/clock.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace dm_test
{

using Dm = protocol::DmEsc;
using Bytes = std::array<std::uint8_t, 8>;

inline std::uint32_t float_bits(float value)
{
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

inline void inject(std::uint8_t bus, std::uint32_t identifier, std::uint8_t length, const Bytes &data)
{
    hal::Can::Frame frame{};
    frame.identifier = identifier;
    frame.length = length;
    frame.data = data;
    assert(mock::can::inject(bus, frame));
}

inline bool is_system_command(const hal::Can::Frame &frame, Dm::SystemCommand command)
{
    if (frame.length != 8U || frame.data[7] != static_cast<std::uint8_t>(command))
    {
        return false;
    }
    for (std::size_t index = 0U; index < 7U; ++index)
    {
        if (frame.data[index] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

inline bool is_register_request(const hal::Can::Frame &frame,
                                std::uint16_t esc_id,
                                Dm::RegisterOperation operation,
                                Dm::Register address)
{
    return frame.identifier == Dm::REGISTER_IDENTIFIER && frame.length >= 4U &&
           frame.data[0] == static_cast<std::uint8_t>(esc_id & 0xFFU) && frame.data[1] == (esc_id >> 8U) &&
           frame.data[2] == static_cast<std::uint8_t>(operation) && frame.data[3] == static_cast<std::uint8_t>(address);
}

class FakeMotor final
{
  public:
    FakeMotor(std::uint8_t bus, std::uint16_t esc_id, std::uint16_t master_id) noexcept
        : bus_{bus}, esc_id_{esc_id}, master_id_{master_id}
    {
    }

    // 电机里的寄存器（出厂 J4310）
    std::uint32_t control_mode{1U};
    float position_max{12.5F};
    float velocity_max{30.0F};
    float torque_max{10.0F};
    std::uint32_t timeout{0U};
    bool echo_wrong_timeout{false}; // 写超时的回显故意不同（模拟写失败）

    // 运行状态
    bool powered{true};         // 没电时什么都不回
    bool refuse_enable{false};  // 收到"使能"也不使能
    bool fault_clearable{true}; // false：故障清不掉（模拟一直欠压）
    Dm::State state{Dm::State::DISABLED};
    std::uint16_t position{32767U}; // 原始计数，0 rad
    std::uint16_t velocity{2047U};
    std::uint16_t torque{2047U};
    std::uint8_t mos_temperature{30U};
    std::uint8_t rotor_temperature{35U};
    bool simulate_motion{false}; // true：position、velocity、torque 由 MIT 指令算出

    // 收到的东西，给测试检查
    std::uint32_t frames_received{0U};
    std::uint32_t last_system_identifier{0U};
    Dm::SystemCommand last_system_command{Dm::SystemCommand::DISABLE};
    bool has_mit{false};
    hal::Can::Frame last_mit{};

    /** 时间推进：检查通讯超时（Bench::respond() 每轮先调用）。 */
    void update(hal::os::Microseconds now)
    {
        const hal::os::Microseconds timeout_us = timeout * 50U;
        if (powered && state == Dm::State::ENABLED && timeout != 0U && now - last_control_us_ >= timeout_us)
        {
            state = Dm::State::COMMUNICATION_LOST;
        }
    }

    /** 处理一帧主机发出的帧；不是发给自己的返回 false。 */
    bool handle(const hal::Can::Frame &frame)
    {
        if (!powered || frame.identifier_type != hal::Can::IdentifierType::STANDARD)
        {
            return false;
        }
        if (frame.identifier == Dm::REGISTER_IDENTIFIER)
        {
            return handle_register(frame);
        }
        if ((frame.identifier & 0xFFU) != esc_id_)
        {
            return false;
        }
        ++frames_received;
        last_control_us_ = hal::os::now_us();
        handle_control(frame);
        send_feedback();
        return true;
    }

    void send_feedback() const
    {
        Bytes data{};
        data[0] = static_cast<std::uint8_t>((esc_id_ & 0x0FU) | (static_cast<std::uint8_t>(state) << 4U));
        data[1] = static_cast<std::uint8_t>(position >> 8U);
        data[2] = static_cast<std::uint8_t>(position & 0xFFU);
        data[3] = static_cast<std::uint8_t>(velocity >> 4U);
        data[4] = static_cast<std::uint8_t>(((velocity & 0x0FU) << 4U) | (torque >> 8U));
        data[5] = static_cast<std::uint8_t>(torque & 0xFFU);
        data[6] = mos_temperature;
        data[7] = rotor_temperature;
        inject(bus_, master_id_, 8U, data);
    }

  private:
    bool handle_register(const hal::Can::Frame &frame)
    {
        if (frame.length < 4U || frame.data[0] != (esc_id_ & 0xFFU) || frame.data[1] != (esc_id_ >> 8U))
        {
            return false;
        }
        ++frames_received;
        const std::uint8_t operation = frame.data[2];
        const auto address = static_cast<Dm::Register>(frame.data[3]);
        Bytes reply{frame.data[0], frame.data[1], operation, frame.data[3], 0U, 0U, 0U, 0U};
        std::uint32_t value = 0U;
        if (operation == static_cast<std::uint8_t>(Dm::RegisterOperation::WRITE))
        {
            value = static_cast<std::uint32_t>(frame.data[4]) | (static_cast<std::uint32_t>(frame.data[5]) << 8U) |
                    (static_cast<std::uint32_t>(frame.data[6]) << 16U) |
                    (static_cast<std::uint32_t>(frame.data[7]) << 24U);
            write(address, value);
        }
        value = read(address);
        reply[4] = static_cast<std::uint8_t>(value & 0xFFU);
        reply[5] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
        reply[6] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
        reply[7] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
        inject(bus_, master_id_, 8U, reply);
        return true;
    }

    void write(Dm::Register address, std::uint32_t value)
    {
        if (address == Dm::Register::TIMEOUT)
        {
            timeout = echo_wrong_timeout ? value + 1U : value;
        }
    }

    [[nodiscard]] std::uint32_t read(Dm::Register address) const
    {
        switch (address)
        {
        case Dm::Register::CONTROL_MODE:
            return control_mode;
        case Dm::Register::POSITION_MAX:
            return float_bits(position_max);
        case Dm::Register::VELOCITY_MAX:
            return float_bits(velocity_max);
        case Dm::Register::TORQUE_MAX:
            return float_bits(torque_max);
        case Dm::Register::TIMEOUT:
            return timeout;
        case Dm::Register::MASTER_ID:
            return master_id_;
        case Dm::Register::ESC_ID:
            return esc_id_;
        default:
            return 0U;
        }
    }

    void handle_control(const hal::Can::Frame &frame)
    {
        const Dm::SystemCommand commands[]{Dm::SystemCommand::CLEAR_ERROR,
                                           Dm::SystemCommand::ENABLE,
                                           Dm::SystemCommand::DISABLE,
                                           Dm::SystemCommand::SAVE_ZERO};
        for (const Dm::SystemCommand command : commands)
        {
            if (!is_system_command(frame, command))
            {
                continue;
            }
            last_system_identifier = frame.identifier;
            last_system_command = command;
            const bool fault = state != Dm::State::DISABLED && state != Dm::State::ENABLED;
            if (command == Dm::SystemCommand::CLEAR_ERROR && fault && fault_clearable)
            {
                state = Dm::State::DISABLED;
            }
            else if (command == Dm::SystemCommand::ENABLE && !fault && !refuse_enable)
            {
                state = Dm::State::ENABLED;
            }
            else if (command == Dm::SystemCommand::DISABLE && !fault)
            {
                state = Dm::State::DISABLED;
            }
            return;
        }
        has_mit = true;
        last_mit = frame;
        if (simulate_motion)
        {
            move(frame);
        }
    }

    void move(const hal::Can::Frame &frame)
    {
        const hal::os::Microseconds now = hal::os::now_us();
        const float dt = moved_ ? hal::os::to_seconds(now - last_mit_us_) : 0.0F;
        moved_ = true;
        last_mit_us_ = now;
        const auto kd_raw = static_cast<std::uint16_t>((frame.data[5] << 4U) | (frame.data[6] >> 4U));
        const auto torque_raw = static_cast<std::uint16_t>(((frame.data[6] & 0x0FU) << 8U) | frame.data[7]);
        const float kd = Dm::from_unsigned(kd_raw, 0.0F, Dm::MIT_KD_MAX, 12U);
        const float command = Dm::from_unsigned(torque_raw, -torque_max, torque_max, 12U);
        const float speed = state == Dm::State::ENABLED && kd > 0.01F ? command / kd : 0.0F;
        const float applied = state == Dm::State::ENABLED ? command : 0.0F;
        position_rad_ += speed * dt;
        position = Dm::to_unsigned(position_rad_, -position_max, position_max, 16U);
        velocity = Dm::to_unsigned(speed, -velocity_max, velocity_max, 12U);
        torque = Dm::to_unsigned(applied, -torque_max, torque_max, 12U);
    }

    std::uint8_t bus_;
    std::uint16_t esc_id_;
    std::uint16_t master_id_;
    hal::os::Microseconds last_control_us_{0U};
    hal::os::Microseconds last_mit_us_{0U};
    bool moved_{false};
    float position_rad_{0.0F};
};

/** 一条总线上的模拟电机：取出本轮发送记录，交给每台电机应答。 */
class Bench final
{
  public:
    static constexpr std::size_t MAXIMUM_FRAMES{32U};

    explicit Bench(std::uint8_t bus) noexcept : bus_{bus} {}

    void add(FakeMotor &motor)
    {
        assert(count_ < motors_.size());
        motors_[count_++] = &motor;
    }

    /** 取出发送记录并应答；记下本轮发出的帧。 */
    void respond()
    {
        sent_count_ = 0U;
        for (std::size_t index = 0U; index < count_; ++index)
        {
            motors_[index]->update(hal::os::now_us());
        }
        hal::Can::Frame frame{};
        while (mock::can::pop_sent(bus_, frame))
        {
            assert(sent_count_ < sent_.size());
            sent_[sent_count_++] = frame;
            for (std::size_t index = 0U; index < count_; ++index)
            {
                (void)motors_[index]->handle(frame);
            }
        }
    }

    [[nodiscard]] std::size_t sent_count() const noexcept
    {
        return sent_count_;
    }
    [[nodiscard]] const hal::Can::Frame &sent(std::size_t index) const noexcept
    {
        assert(index < sent_count_);
        return sent_[index];
    }

  private:
    std::uint8_t bus_;
    std::array<FakeMotor *, 4> motors_{};
    std::size_t count_{0U};
    std::array<hal::Can::Frame, MAXIMUM_FRAMES> sent_{};
    std::size_t sent_count_{0U};
};

/** 一条总线的一轮：CanBus 收帧分给驱动，再让驱动推进与发送（Decision-20261102-03）。 */
struct BusLoop final
{
    device::CanBus &bus;
    void process() const
    {
        bus.receive();
        bus.transmit(hal::os::now_us());
    }
};

/** 一轮：driver.process()（收上一轮的应答、发本轮的帧）→ 电机应答 → 时间前进 step_us。 */
template <typename Driver>
void cycle(Driver &driver, Bench &bench, std::size_t rounds = 1U, hal::os::Microseconds step_us = 1000U)
{
    for (std::size_t round = 0U; round < rounds; ++round)
    {
        driver.process();
        bench.respond();
        mock::advance_us(step_us);
    }
}

} // namespace dm_test

#endif // TEST_HOST_UNIT_MOTOR_DM_FAKE_MOTOR_HPP
