// bringup 的外设检查（Decision-20261101-03）在 platform/mock 上：CanCheck（只收）、SbusCheck（UART → SBUS →
// bringup.sbus）、UsbReport（回显与状态行）、Indicator（蜂鸣器与 RGB 灯）、TextLine（定点格式化）。
#include "bringup/can_check.hpp"
#include "bringup/indicator.hpp"
#include "bringup/sbus_check.hpp"
#include "bringup/text_line.hpp"
#include "bringup/usb_report.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"
#include "topics/bringup.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace
{

std::string text(const bringup::TextLine &line)
{
    return std::string(reinterpret_cast<const char *>(line.data()), line.size());
}

void test_text_line()
{
    bringup::TextLine line;
    line.append("x=");
    line.append_unsigned(0U);
    line.append(",");
    line.append_signed(-2147483647 - 1);
    line.append(",");
    line.append_fixed(-1.2345F, 3U);
    line.append(",");
    line.append_fixed(0.0004F, 3U);
    line.append(",");
    line.append_fixed(-0.0004F, 3U); // 四舍五入到 0，不输出负号
    line.append(",");
    line.append_fixed(9.99F, 1U);
    line.append(",");
    line.append_fixed(3.0e9F, 0U);
    assert(text(line) == "x=0,-2147483648,-1.235,0.000,0.000,10.0,inf");
    assert(!line.overflowed());
    for (int index = 0; index < 100; ++index)
    {
        line.append("abc");
    }
    assert(line.overflowed() && line.size() == bringup::TextLine::CAPACITY);
    line.clear();
    assert(line.size() == 0U && !line.overflowed());
}

void test_can_check()
{
    mock::can::reset();
    bringup::CanCheck check{hal::Can{1U}};
    assert(check.init());
    bringup::CanCheck duplicate{hal::Can{1U}};
    assert(!duplicate.init()); // 每条总线一个接收方
    assert(mock::can::inject(1U, 0x201U, {}));
    assert(mock::can::inject(1U, 0x205U, {}));
    assert(mock::can::inject(1U, 0x123U, {}));
    check.step(5000U);
    assert(check.report().frames == 3U);
    assert(check.report().last_identifier == 0x123U && check.report().last_frame_us == 5000U);
    assert(check.report().dji_feedback_seen == ((1U << 0U) | (1U << 4U)));
    assert(check.report().bus_ready);
    assert(mock::can::sent_count(1U) == 0U); // 只收不发
}

// 按 SBUS 格式编码一帧：16 路 11 位通道小端连续打包
std::array<std::uint8_t, 25> sbus_frame(std::uint16_t value, std::uint8_t flags)
{
    std::array<std::uint8_t, 25> frame{};
    frame[0] = 0x0FU;
    std::uint32_t bits = 0U;
    std::uint32_t bit_count = 0U;
    std::size_t index = 1U;
    for (std::size_t channel = 0U; channel < 16U; ++channel)
    {
        bits |= static_cast<std::uint32_t>(value + channel) << bit_count;
        bit_count += 11U;
        while (bit_count >= 8U)
        {
            frame[index++] = static_cast<std::uint8_t>(bits & 0xFFU);
            bits >>= 8U;
            bit_count -= 8U;
        }
    }
    frame[23] = flags;
    frame[24] = 0x00U;
    return frame;
}

void test_sbus_check()
{
    mock::uart::reset();
    bringup::SbusCheck check{hal::Uart{0U}};
    assert(check.init());
    topic::Subscriber<topics::bringup::Sbus> sbus{topics::bringup::sbus};
    const auto first = sbus_frame(1000U, 0x00U);
    const auto second = sbus_frame(1500U, 0x08U); // 失控保护位
    // 分两段注入：解析器要能跨段拼帧
    assert(mock::uart::inject(0U, first.data(), 10U) == 10U);
    check.step(1000U);
    topics::bringup::Sbus message{};
    assert(!sbus.update(message));
    assert(mock::uart::inject(0U, first.data() + 10U, first.size() - 10U) == first.size() - 10U);
    assert(mock::uart::inject(0U, second.data(), second.size()) == second.size());
    check.step(2000U);
    assert(check.report().frames == 2U && check.report().failsafe == 1U);
    assert(check.report().bytes == 50U);
    assert(sbus.update(message));
    assert(message.channels[0] == 1500U && message.channels[15] == 1515U);
    assert(message.failsafe && message.frame_count == 2U && message.timestamp_us == 2000U);
}

// 心跳与 IMU 的发布句柄每个 topic 只能取一次：main() 取得后交给用到的测试
using HeartbeatPublisher = topic::Publisher<topics::bringup::Heartbeat>;
using ImuPublisher = topic::Publisher<topics::bringup::Imu>;

void test_usb_report(HeartbeatPublisher &heartbeat, ImuPublisher &imu)
{
    mock::usb_cdc::reset();
    bringup::UsbReport report{hal::UsbCdc{0U}};
    assert(report.init(bringup::UsbReport::Parameters{100000U}));

    report.step(0U); // 没连上：什么都不做
    assert(report.report().lines == 0U && !report.report().connected);

    mock::usb_cdc::set_connected(true);
    heartbeat.publish(topics::bringup::Heartbeat{42U, 0U});
    topics::bringup::Imu sample{};
    sample.quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
    sample.gyro_rad_per_sec = {0.0F, 0.0F, -0.5F};
    sample.accel_m_per_s2 = {0.0F, 0.0F, 9.81F};
    sample.temperature_celsius = 49.96F;
    sample.gyro_sample_count = 7U;
    imu.publish(sample);
    report.step(1234000U);
    assert(report.report().lines == 1U);
    const std::string expected = "t=1234 hb=42 imu=7 q=1.0000,0.0000,0.0000,0.0000 g=0.000,0.000,-0.500 "
                                 "a=0.00,0.00,9.81 T=50.0 sbus=2 ch=1500,1501,1502,1503 fs=1\r\n";
    assert(text(report.last_line()) == expected);
    std::array<std::uint8_t, 256> written{};
    assert(mock::usb_cdc::take_written(written.data(), written.size()) == expected.size());
    assert(mock::usb_cdc::complete_write());

    // 两行之间回显收到的字节
    const std::array<std::uint8_t, 3> hello{'h', 'i', '\n'};
    assert(mock::usb_cdc::inject(hello.data(), hello.size()) == 3U);
    report.step(1240000U);
    assert(report.report().echoed_bytes == 3U);
    assert(mock::usb_cdc::take_written(written.data(), written.size()) == 3U && written[0] == 'h');
    // 上一次发送没结束：到期的状态行跳过
    report.step(1340000U);
    assert(report.report().skipped == 1U && report.report().lines == 1U);
}

void test_indicator(HeartbeatPublisher &heartbeat, ImuPublisher &imu)
{
    mock::pwm::reset();
    mock::spi::reset();
    mock::spi::set_capabilities(2U, true, true, false);
    device::Buzzer buzzer{hal::Pwm{0U}};
    device::Ws2812 led{hal::Spi{2U}};
    bringup::Indicator indicator;
    // 100 ms 没有新数据算停。test_usb_report 先发布过心跳与 IMU：订阅者第一次 step() 会把它们当作新数据
    assert(indicator.init(bringup::Indicator::Config{&buzzer, &led, {100000U, 255U, true}}));

    // 上电两声：2000 Hz、2600 Hz，各 80 ms，然后停
    indicator.step(0U);
    assert(mock::pwm::output(0U).frequency_hz == 2000U && mock::pwm::output(0U).duty_permille > 0U);
    indicator.step(90000U);
    assert(mock::pwm::output(0U).frequency_hz == 2600U);
    indicator.step(170000U);
    assert(mock::pwm::output(0U).duty_permille == 0U);
    assert(indicator.status() == bringup::Indicator::Status::STOPPED); // 170 ms 没有新数据

    // 状态：IMU 数据 → 绿；只有心跳 → 黄；都停 → 红。灯每轮刷新，亮半秒、灭半秒
    imu.publish(topics::bringup::Imu{});
    heartbeat.publish(topics::bringup::Heartbeat{1U, 0U});
    indicator.step(200000U);
    assert(indicator.status() == bringup::Indicator::Status::IMU_RUNNING);
    assert(led.get_color() == device::ws2812_color::GREEN);
    heartbeat.publish(topics::bringup::Heartbeat{2U, 0U});
    indicator.step(450000U); // IMU 已停 250 ms，心跳刚刚更新（450 ms 时在亮的半秒里）
    assert(indicator.status() == bringup::Indicator::Status::HEARTBEAT);
    assert(led.get_color() == device::ws2812_color::YELLOW);
    indicator.step(600000U); // 后半秒灭
    assert(led.get_color() == device::ws2812_color::BLACK);
    indicator.step(1100000U); // 都停了
    assert(indicator.status() == bringup::Indicator::Status::STOPPED);
    assert(led.get_color() == device::ws2812_color::RED);
    assert(mock::spi::transaction_count(2U) > 0U); // 灯确实在刷新
}

} // namespace

int main()
{
    mock::reset();
    test_text_line();
    test_can_check();
    test_sbus_check();
    HeartbeatPublisher heartbeat = topics::bringup::heartbeat.claim_publisher();
    ImuPublisher imu = topics::bringup::imu.claim_publisher();
    assert(heartbeat && imu);
    test_usb_report(heartbeat, imu);
    test_indicator(heartbeat, imu);
    return 0;
}
