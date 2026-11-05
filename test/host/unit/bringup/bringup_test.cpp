// bringup 能力：Heartbeat 发布心跳、Monitor 统计收到、丢失、乱序与延迟。
// 能力零 RTOS，宿主上直接调用 step()；topic 用 platform_mock 的 hal_os。
#include "bringup/heartbeat.hpp"
#include "bringup/monitor.hpp"
#include "mock_os.hpp"
#include "topics/all.hpp"
#include "topics/bringup.hpp"

#include <cassert>

int main()
{
    mock::reset();
    bringup::Heartbeat heartbeat;
    assert(heartbeat.init());
    bringup::Heartbeat second;
    assert(!second.init()); // 每个 topic 只有一个发布者

    bringup::Monitor monitor;
    assert(monitor.init(bringup::Monitor::Config{100U}));
    monitor.step(0U);
    assert(monitor.report().received == 0U); // 还没有心跳

    heartbeat.step(1000U);
    monitor.step(1030U);
    assert(monitor.report().received == 1U);
    assert(monitor.report().last_sequence == 1U);
    assert(monitor.report().last_latency_us == 30U);
    assert(monitor.report().lost == 0U);
    assert(monitor.report().late == 0U);

    monitor.step(1100U); // 没有新心跳：不计数
    assert(monitor.report().received == 1U);

    heartbeat.step(2000U);
    heartbeat.step(3000U); // 2 号被 3 号覆盖
    monitor.step(3500U);
    assert(monitor.report().received == 2U);
    assert(monitor.report().last_sequence == 3U);
    assert(monitor.report().lost == 1U);
    assert(monitor.report().last_latency_us == 500U);
    assert(monitor.report().max_latency_us == 500U);
    assert(monitor.report().late == 1U); // 超过 100 µs
    assert(monitor.report().out_of_order == 0U);
    assert(heartbeat.sequence() == 3U);

    // 延迟跨 2^32 µs 回绕仍按无符号差计算
    mock::set_now_us(0xFFFFFFF0U);
    heartbeat.step(0xFFFFFFF0U);
    monitor.step(0x10U);
    assert(monitor.report().last_latency_us == 0x20U);

    // 清单列出全部 topic
    assert(topics::ALL.size() == 10U);
    assert(topics::ALL[0] == &topics::bringup::heartbeat);
    assert(topics::ALL[4] == &topics::safety::stop);     // 安全门订阅的急停
    assert(topics::ALL[7] == &topics::pc_link::control); // 上位机链路
    assert(topics::ALL[8] == &topics::remote::input);    // SBUS 遥控的操作输入
    assert(topics::ALL[9] == &topics::command::gimbal);  // 仲裁发给云台的指令
    assert(topics::ALL[5] == &topics::imu::event && topics::ALL[6] == &topics::imu::attitude); // IMU 能力
    assert(topics::bringup::heartbeat.publish_count() == 4U);
    return 0;
}
