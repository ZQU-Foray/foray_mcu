#include "driver/motor/dji/dji_motor.hpp"

#include <array>
#include <cassert>
#include <cstdint>

#include "mock_hal.hpp"
#include "mock_os.hpp"

// DJI 电机会话（device::DjiMotor）在 platform/mock 上：总线收发经 hal::Can 替身，时间经 hal_os 替身。
namespace
{

constexpr std::uint8_t C620_BUS{0U};
constexpr std::uint8_t SECOND_BUS{1U};
constexpr std::uint8_t GM_BUS{2U};
constexpr std::uint8_t C610_BUS{3U};

// 时刻按毫秒给（历史测试用 1 kHz 时基写成）
void set_ms(std::uint32_t milliseconds)
{
    mock::set_now_us(milliseconds * 1000U);
}

hal::Can::Frame last_sent(std::uint8_t bus)
{
    hal::Can::Frame frame{};
    assert(mock::can::last_sent(bus, frame));
    return frame;
}

} // namespace

using Bus = device::DjiMotor;

int main()
{
    mock::reset();
    mock::can::reset();

    // ---- C620 会话：保活发送（C610/C620 反馈的前提）----
    Bus c620_bus{hal::Can{C620_BUS}, Bus::Profile::c620()};
    assert(c620_bus.init());
    assert(c620_bus.is_ready());
    assert(c620_bus.maximum_device_count() == 8U);
    assert(c620_bus.enable_motor(1U));
    assert(!c620_bus.enable_motor(0U));
    assert(!c620_bus.enable_motor(9U));

    c620_bus.process();
    assert(mock::can::sent_count(C620_BUS) == 1U);
    assert(last_sent(C620_BUS).identifier == 0x200U);
    assert(last_sent(C620_BUS).data == (std::array<std::uint8_t, 8U>{}));

    // 每条总线只有一个接收方：同一总线上的第二个会话 init() 失败
    Bus duplicate{hal::Can{C620_BUS}, Bus::Profile::c620()};
    assert(!duplicate.init());
    assert(!duplicate.is_ready());

    // ---- 反馈解析与单圈换算（0x201 = 设备 1）----
    set_ms(100U);
    (void)mock::can::inject(C620_BUS, 0x201U, {0x10U, 0x00U, 0x00U, 0x64U, 0x00U, 0x32U, 30U, 0U});
    c620_bus.process();
    Bus::Snapshot snapshot{};
    assert(c620_bus.get_snapshot(1U, snapshot));
    assert(snapshot.device_id == 1U);
    assert(snapshot.enabled && snapshot.ever_received && snapshot.online);
    assert(snapshot.last_update_us == 100000U);
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
    set_ms(101U);
    (void)mock::can::inject(C620_BUS, 0x201U, {0x1FU, 0xFFU, 0, 0, 0, 0, 0, 0});
    c620_bus.process();
    assert(c620_bus.get_snapshot(1U, snapshot));
    assert(snapshot.total_angle_counts == 4095);

    set_ms(102U);
    (void)mock::can::inject(C620_BUS, 0x201U, {0x00U, 0x0FU, 0, 0, 0, 0, 0, 0});
    c620_bus.process();
    assert(c620_bus.get_snapshot(1U, snapshot));
    assert(snapshot.total_angle_counts == 4111);
    assert(snapshot.total_turns == 0);

    // ---- 指令换算：安培 / 比例 / 原始值，全部钳位 ----
    assert(c620_bus.set_current_ampere(1U, 5.0F)); // 5A/20A*16384 = 4096
    c620_bus.process();
    assert(last_sent(C620_BUS).identifier == 0x200U);
    assert(last_sent(C620_BUS).data[0] == 0x10U && last_sent(C620_BUS).data[1] == 0x00U);

    assert(c620_bus.set_current_ampere(1U, 999.0F)); // 钳位到 +16384
    c620_bus.process();
    assert(last_sent(C620_BUS).data[0] == 0x40U && last_sent(C620_BUS).data[1] == 0x00U);

    assert(c620_bus.set_torque_ratio(1U, -1.0F)); // -16384
    c620_bus.process();
    assert(last_sent(C620_BUS).data[0] == 0xC0U && last_sent(C620_BUS).data[1] == 0x00U);

    assert(!c620_bus.set_current_ampere(0U, 1.0F));
    assert(!c620_bus.set_current_ampere(9U, 1.0F));

    assert(c620_bus.set_torque(1U, 1.5F)); // 1.5 N·m / 0.3 = 5A → 4096
    c620_bus.process();
    assert(last_sent(C620_BUS).identifier == 0x200U);
    assert(last_sent(C620_BUS).data[0] == 0x10U && last_sent(C620_BUS).data[1] == 0x00U);
    assert(!c620_bus.set_torque(9U, 1.0F));

    // ---- 第二组聚合：设备 6 走 0x1FF 槽位 1 ----
    Bus second_bus{hal::Can{SECOND_BUS}, Bus::Profile::c620()};
    assert(second_bus.init());
    assert(second_bus.enable_motor(6U));
    assert(second_bus.set_raw_command(6U, 1000));
    second_bus.process();
    assert(last_sent(SECOND_BUS).identifier == 0x1FFU);
    assert(last_sent(SECOND_BUS).data[2] == 0x03U && last_sent(SECOND_BUS).data[3] == 0xE8U);
    Bus::Statistics second_statistics{};
    assert(second_bus.get_statistics(second_statistics));
    assert(second_statistics.tx_result_counts[0] == 1U);

    // ---- GM6020 电压模式：无电流接口、比例走电压、ID 上限 7、范围外 ID 未知 ----
    Bus gm_bus{hal::Can{GM_BUS}, Bus::Profile::gm6020_voltage()};
    assert(gm_bus.init());
    assert(gm_bus.maximum_device_count() == 7U);
    assert(gm_bus.enable_motor(1U));
    assert(!gm_bus.enable_motor(8U));
    assert(!gm_bus.set_current_ampere(1U, 1.0F)); // 电压模式无电流满量程
    assert(!gm_bus.set_torque(1U, 1.0F));         // 电压模式力矩接口同样不可用
    assert(gm_bus.set_torque_ratio(1U, 0.5F));    // 0.5*25000 = 12500 = 0x30D4
    gm_bus.process();                             // 仅设备 1 在役：单帧 0x1FF
    assert(last_sent(GM_BUS).identifier == 0x1FFU);
    assert(last_sent(GM_BUS).data[0] == 0x30U && last_sent(GM_BUS).data[1] == 0xD4U);

    assert(gm_bus.enable_motor(7U)); // 第二组开始随节拍发送 0x2FF
    set_ms(103U);
    (void)mock::can::inject(GM_BUS, 0x205U, {0x08U, 0x00U, 0x00U, 0x64U, 0xFFU, 0xF4U, 50U, 0U});
    (void)mock::can::inject(GM_BUS, 0x201U, {0, 0, 0, 0, 0, 0, 0, 0}); // C 家族 ID 对 GM 方言为未知
    gm_bus.process(); // 双组各一帧：0x1FF 之后 0x2FF，末帧为设备 7 所在组
    assert(last_sent(GM_BUS).identifier == 0x2FFU);
    assert(last_sent(GM_BUS).data == (std::array<std::uint8_t, 8U>{}));
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
    Bus c610_bus{hal::Can{C610_BUS}, Bus::Profile::c610()};
    assert(c610_bus.init());
    assert(c610_bus.enable_motor(1U));
    set_ms(110U);
    (void)mock::can::inject(C610_BUS, 0x201U, {0x00U, 0x01U, 0, 0, 0, 0, 40U, 0U});
    c610_bus.process();
    assert(c610_bus.get_snapshot(1U, snapshot));
    assert(snapshot.online);
    assert(snapshot.motor_temperature_celsius == 0U);

    assert(c610_bus.set_torque(1U, 0.5F)); // 0.5 N·m / 0.18 ≈ 2.778A → 2777
    c610_bus.process();
    assert(last_sent(C610_BUS).identifier == 0x200U);
    assert(last_sent(C610_BUS).data[0] == 0x0AU && last_sent(C610_BUS).data[1] == 0xD9U);

    // ---- 链路老化：静默超过 LINK_TIMEOUT_US（50 ms）后离线 ----
    set_ms(200U);
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

    // 总线统计随 process() 读入；发送失败按结果分类计数
    mock::can::set_statistics(C620_BUS, hal::Can::Statistics{2U, 1U, 3U});
    mock::can::set_send_result(C620_BUS, hal::Can::SendResult::QUEUE_FULL);
    c620_bus.process();
    mock::can::set_send_result(C620_BUS, hal::Can::SendResult::QUEUED);
    assert(c620_bus.get_statistics(c620_statistics));
    assert(c620_statistics.rx_dropped_count == 2U);
    assert(c620_statistics.rx_hardware_loss_event_count == 1U);
    assert(c620_statistics.bus_off_count == 3U);
    assert(c620_statistics.tx_result_counts[static_cast<std::size_t>(hal::Can::SendResult::QUEUE_FULL)] == 1U);
    assert(c620_statistics.tx_result_counts[0] == 9U);

    // ---- 清零指令 ----
    c620_bus.clear_all_commands();
    c620_bus.process();
    assert(last_sent(C620_BUS).data == (std::array<std::uint8_t, 8U>{}));
    return 0;
}
