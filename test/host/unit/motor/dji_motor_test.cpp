#include "Libraries/Device/motor/dji/DjiMotor.hpp"

#include <array>
#include <cassert>
#include <cstdint>

// ---- 平台 C ABI 桩：被测代码经 platform::Can / platform::Time 调用 ----
namespace
{

struct CanStub
{
    bool ready{true};
    std::uint8_t rx_count{0U};
    std::uint8_t rx_index{0U};
    struct RxFrame
    {
        std::uint32_t identifier{0U};
        std::array<std::uint8_t, 8U> data{};
    };
    std::array<RxFrame, 8U> rx{};
    std::uint32_t tx_count{0U};
    std::uint32_t last_tx_identifier{0U};
    std::array<std::uint8_t, 8U> last_tx_data{};
    CanPort_SendResult tx_result{CAN_PORT_SEND_QUEUED};
    CanPort_Statistics statistics{};
} g_can_stub;

struct TimeStub
{
    std::uint32_t ticks{0U};
} g_time_stub;

void feed_rx(std::uint32_t identifier, const std::array<std::uint8_t, 8U> &data)
{
    g_can_stub.rx[g_can_stub.rx_count] = CanStub::RxFrame{identifier, data};
    ++g_can_stub.rx_count;
}

void reset_rx()
{
    g_can_stub.rx_count = 0U;
    g_can_stub.rx_index = 0U;
}

} // namespace

extern "C"
{
    bool CanPort_Init(void)
    {
        return true;
    }

    bool CanPort_IsReady(CanPort_Channel)
    {
        return g_can_stub.ready;
    }

    bool CanPort_ConfigureStandardReceiveFilter(CanPort_Channel, bool, std::uint32_t, std::uint32_t)
    {
        return false;
    }

    bool CanPort_SetReceiveNotification(CanPort_Channel, CanPort_ReceiveNotification, void *)
    {
        return false;
    }

    CanPort_SendResult CanPort_TrySend(CanPort_Channel,
                                       std::uint32_t identifier,
                                       CanPort_IdentifierType,
                                       std::uint8_t length,
                                       const std::uint8_t *data)
    {
        ++g_can_stub.tx_count;
        g_can_stub.last_tx_identifier = identifier;
        for (std::uint8_t index = 0U; index < 8U; ++index)
        {
            g_can_stub.last_tx_data[index] = index < length ? data[index] : 0U;
        }
        return g_can_stub.tx_result;
    }

    CanPort_ReceiveResult CanPort_TryReceive(CanPort_Channel,
                                             std::uint32_t *identifier,
                                             CanPort_IdentifierType *identifier_type,
                                             std::uint8_t *length,
                                             std::uint8_t *data,
                                             std::uint8_t data_capacity)
    {
        if (g_can_stub.rx_index >= g_can_stub.rx_count)
        {
            return CAN_PORT_RECEIVE_EMPTY;
        }
        const CanStub::RxFrame &frame = g_can_stub.rx[g_can_stub.rx_index];
        ++g_can_stub.rx_index;
        *identifier = frame.identifier;
        *identifier_type = CAN_PORT_IDENTIFIER_STANDARD;
        *length = 8U;
        for (std::uint8_t index = 0U; index < 8U && index < data_capacity; ++index)
        {
            data[index] = frame.data[index];
        }
        return CAN_PORT_RECEIVE_RECEIVED;
    }

    bool CanPort_GetStatistics(CanPort_Channel, CanPort_Statistics *statistics)
    {
        *statistics = g_can_stub.statistics;
        return true;
    }

    bool TimePort_Init(void)
    {
        return true;
    }
    bool TimePort_IsReady(void)
    {
        return true;
    }
    std::uint32_t TimePort_NowTicks(void)
    {
        return g_time_stub.ticks;
    }
    std::uint32_t TimePort_FrequencyHz(void)
    {
        return 1000U;
    }
    void TimePort_DelayUs(std::uint32_t) {}

} // extern "C"

using Bus = device::DjiMotor;

int main()
{
    // ---- C620 会话：保活发送（C610/C620 反馈的前提）----
    Bus c620_bus{platform::Can::Channel::CHANNEL_1, Bus::Profile::c620()};
    assert(c620_bus.init());
    assert(c620_bus.is_ready());
    assert(c620_bus.maximum_device_count() == 8U);
    assert(c620_bus.enable_motor(1U));
    assert(!c620_bus.enable_motor(0U));
    assert(!c620_bus.enable_motor(9U));

    reset_rx();
    c620_bus.process();
    assert(g_can_stub.tx_count == 1U);
    assert(g_can_stub.last_tx_identifier == 0x200U);
    assert(g_can_stub.last_tx_data == (std::array<std::uint8_t, 8U>{}));

    // ---- 反馈解析与单圈换算（0x201 = 设备 1）----
    g_time_stub.ticks = 100U;
    reset_rx();
    feed_rx(0x201U, {0x10U, 0x00U, 0x00U, 0x64U, 0x00U, 0x32U, 30U, 0U});
    c620_bus.process();
    Bus::Snapshot snapshot{};
    assert(c620_bus.get_snapshot(1U, snapshot));
    assert(snapshot.device_id == 1U);
    assert(snapshot.enabled && snapshot.ever_received && snapshot.online);
    assert(snapshot.last_update_tick == 100U);
    assert(snapshot.accepted_frame_count == 1U);
    assert(snapshot.rotor_angle_raw == 0x1000U);
    assert(snapshot.rotor_angle_degrees == 180.0F);
    assert(snapshot.rotor_speed_rpm == 100);
    assert(snapshot.torque_current_raw == 50);
    assert(snapshot.motor_temperature_celsius == 30U);
    assert(snapshot.total_angle_counts == 0); // 首帧只建立多圈基准

    // 未启用/未见过反馈的设备：快照可读但离线
    assert(c620_bus.get_snapshot(3U, snapshot));
    assert(!snapshot.enabled && !snapshot.online && !snapshot.ever_received);
    assert(!c620_bus.get_snapshot(0U, snapshot));
    assert(!c620_bus.get_snapshot(9U, snapshot));

    // ---- 多圈累计：+4095 后跨零前进 +16，净 +4111 ----
    g_time_stub.ticks = 101U;
    reset_rx();
    feed_rx(0x201U, {0x1FU, 0xFFU, 0, 0, 0, 0, 0, 0});
    c620_bus.process();
    assert(c620_bus.get_snapshot(1U, snapshot));
    assert(snapshot.total_angle_counts == 4095);

    g_time_stub.ticks = 102U;
    reset_rx();
    feed_rx(0x201U, {0x00U, 0x0FU, 0, 0, 0, 0, 0, 0});
    c620_bus.process();
    assert(c620_bus.get_snapshot(1U, snapshot));
    assert(snapshot.total_angle_counts == 4111);
    assert(snapshot.total_turns == 0);

    // ---- 指令换算：安培 / 比例 / 原始值，全部钳位 ----
    assert(c620_bus.set_current_ampere(1U, 5.0F)); // 5A/20A*16384 = 4096
    c620_bus.process();
    assert(g_can_stub.last_tx_identifier == 0x200U);
    assert(g_can_stub.last_tx_data[0] == 0x10U && g_can_stub.last_tx_data[1] == 0x00U);

    assert(c620_bus.set_current_ampere(1U, 999.0F)); // 钳位到 +16384
    c620_bus.process();
    assert(g_can_stub.last_tx_data[0] == 0x40U && g_can_stub.last_tx_data[1] == 0x00U);

    assert(c620_bus.set_torque_ratio(1U, -1.0F)); // -16384
    c620_bus.process();
    assert(g_can_stub.last_tx_data[0] == 0xC0U && g_can_stub.last_tx_data[1] == 0x00U);

    assert(!c620_bus.set_current_ampere(0U, 1.0F));
    assert(!c620_bus.set_current_ampere(9U, 1.0F));

    assert(c620_bus.set_torque(1U, 1.5F)); // 1.5 N·m / 0.3 = 5A → 4096
    c620_bus.process();
    assert(g_can_stub.last_tx_identifier == 0x200U);
    assert(g_can_stub.last_tx_data[0] == 0x10U && g_can_stub.last_tx_data[1] == 0x00U);
    assert(!c620_bus.set_torque(9U, 1.0F));

    // ---- 第二组聚合：设备 6 走 0x1FF 槽位 1 ----
    Bus second_bus{platform::Can::Channel::CHANNEL_2, Bus::Profile::c620()};
    assert(second_bus.init());
    assert(second_bus.enable_motor(6U));
    assert(second_bus.set_raw_command(6U, 1000));
    reset_rx();
    second_bus.process();
    assert(g_can_stub.last_tx_identifier == 0x1FFU);
    assert(g_can_stub.last_tx_data[2] == 0x03U && g_can_stub.last_tx_data[3] == 0xE8U);
    Bus::Statistics second_statistics{};
    assert(second_bus.get_statistics(second_statistics));
    assert(second_statistics.tx_result_counts[0] == 1U);

    // ---- GM6020 电压模式：无电流接口、比例走电压、ID 上限 7、范围外 ID 未知 ----
    Bus gm_bus{platform::Can::Channel::CHANNEL_3, Bus::Profile::gm6020_voltage()};
    assert(gm_bus.init());
    assert(gm_bus.maximum_device_count() == 7U);
    assert(gm_bus.enable_motor(1U));
    assert(!gm_bus.enable_motor(8U));
    assert(!gm_bus.set_current_ampere(1U, 1.0F)); // 电压模式无电流满量程
    assert(!gm_bus.set_torque(1U, 1.0F));         // 电压模式力矩接口同样不可用
    assert(gm_bus.set_torque_ratio(1U, 0.5F));    // 0.5*25000 = 12500 = 0x30D4
    reset_rx();
    gm_bus.process(); // 仅设备 1 在役：单帧 0x1FF
    assert(g_can_stub.last_tx_identifier == 0x1FFU);
    assert(g_can_stub.last_tx_data[0] == 0x30U && g_can_stub.last_tx_data[1] == 0xD4U);

    assert(gm_bus.enable_motor(7U)); // 第二组开始随节拍发送 0x2FF
    g_time_stub.ticks = 103U;
    reset_rx();
    feed_rx(0x205U, {0x08U, 0x00U, 0x00U, 0x64U, 0xFFU, 0xF4U, 50U, 0U});
    feed_rx(0x201U, {0, 0, 0, 0, 0, 0, 0, 0}); // C 家族 ID 对 GM 方言为未知
    gm_bus.process();                          // 双组各一帧：0x1FF 之后 0x2FF，末帧为设备 7 所在组
    assert(g_can_stub.last_tx_identifier == 0x2FFU);
    assert(g_can_stub.last_tx_data == (std::array<std::uint8_t, 8U>{}));
    Bus::Statistics gm_statistics{};
    assert(gm_bus.get_statistics(gm_statistics));
    assert(gm_statistics.accepted_frame_count == 1U);
    assert(gm_statistics.unknown_identifier_count == 1U);
    assert(gm_statistics.tx_result_counts[0] == 3U); // 1 帧 + 双组 2 帧
    assert(gm_bus.get_snapshot(1U, snapshot));
    assert(snapshot.online);
    assert(snapshot.rotor_angle_raw == 0x0800U);
    assert(snapshot.rotor_angle_degrees == 90.0F);
    assert(snapshot.rotor_speed_rpm == 100);
    assert(snapshot.torque_current_raw == -12);
    assert(snapshot.motor_temperature_celsius == 50U);

    // ---- C610：温度槽手册标注 Null，快照恒 0 ----
    Bus c610_bus{platform::Can::Channel::CHANNEL_1, Bus::Profile::c610()};
    assert(c610_bus.init());
    assert(c610_bus.enable_motor(1U));
    g_time_stub.ticks = 110U;
    reset_rx();
    feed_rx(0x201U, {0x00U, 0x01U, 0, 0, 0, 0, 40U, 0U});
    c610_bus.process();
    assert(c610_bus.get_snapshot(1U, snapshot));
    assert(snapshot.online);
    assert(snapshot.motor_temperature_celsius == 0U);

    assert(c610_bus.set_torque(1U, 0.5F)); // 0.5 N·m / 0.18 ≈ 2.778A → 2777
    c610_bus.process();
    assert(g_can_stub.last_tx_identifier == 0x200U);
    assert(g_can_stub.last_tx_data[0] == 0x0AU && g_can_stub.last_tx_data[1] == 0xD9U);

    // ---- 链路老化：静默超过 10ms 后离线 ----
    g_time_stub.ticks = 200U;
    reset_rx();
    c620_bus.process();
    assert(c620_bus.get_snapshot(1U, snapshot));
    assert(!snapshot.online);
    assert(snapshot.ever_received);
    assert(snapshot.accepted_frame_count == 3U);

    // ---- 总线统计：发送结果分布与解码计数 ----
    Bus::Statistics c620_statistics{};
    assert(c620_bus.get_statistics(c620_statistics));
    assert(c620_statistics.tx_result_counts[0] == 9U); // 含一次力矩指令发送
    assert(c620_statistics.accepted_frame_count == 3U);
    assert(c620_statistics.unknown_identifier_count == 0U);
    assert(c620_statistics.rx_dropped_count == 0U);

    // ---- 清零指令 ----
    c620_bus.clear_all_commands();
    reset_rx();
    c620_bus.process();
    assert(g_can_stub.last_tx_data == (std::array<std::uint8_t, 8U>{}));
    return 0;
}
