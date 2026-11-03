// 用 PC 端 foray_platform 的链路状态机（RoboLinkHostLink + 串口类）直接连板子：
// 视觉组实际用的就是这套代码。不进本仓构建：编译命令见 tool/robolink/README.md。
//
// 按真实时间跑：每约 1 ms 调用一次 step()，Active 后可选每轮发一份控制快照；结束时打印 PC 端看到的状态、
// 拿到新姿态的轮数与最后的姿态。PC 端只保留最新一份姿态，一轮最多数到一份，姿态的真实频率用 link_probe.py 量。
// 用法：pc_board_check <串口> [秒数] [--control]
#include "foray/platform/link/robolink_host_link.hpp"
#include "foray/platform/link/usb_cdc_port.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

namespace
{

const char *state_name(foray::platform::link::HostLinkState state)
{
    switch (state)
    {
    case foray::platform::link::HostLinkState::Disconnected:
        return "Disconnected";
    case foray::platform::link::HostLinkState::HelloPending:
        return "HelloPending";
    case foray::platform::link::HostLinkState::Ready:
        return "Ready";
    case foray::platform::link::HostLinkState::Active:
        return "Active";
    }
    return "?";
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "用法：%s <串口> [秒数] [--control]\n", argv[0]);
        return 2;
    }
    const double seconds = argc >= 3 ? std::atof(argv[2]) : 10.0;
    const bool send_control = argc >= 4 && std::strcmp(argv[3], "--control") == 0;

    foray::platform::link::UsbCdcPort port{argv[1]};
    foray::platform::link::RoboLinkHostLink link{port};
    const auto start = std::chrono::steady_clock::now();
    auto now_ms = [&]
    {
        return static_cast<std::uint64_t>(
                   std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start)
                       .count()) +
               1000U;
    };

    std::uint64_t active_at = 0U;
    std::uint32_t attitudes = 0U;    // 拿到新姿态的轮数
    std::uint32_t active_steps = 0U; // Active 时的轮数
    std::uint32_t controls = 0U;
    std::uint32_t disconnects = 0U;
    std::uint32_t previous_sample = 0U;
    auto previous_state = foray::platform::link::HostLinkState::Disconnected;
    const std::uint64_t end = now_ms() + static_cast<std::uint64_t>(seconds * 1000.0);
    while (now_ms() < end)
    {
        const std::uint64_t now = now_ms();
        const auto state = link.step(now);
        if (state != previous_state)
        {
            std::printf("%6.3f s  PC 状态 %s\n", static_cast<double>(now - 1000U) / 1000.0, state_name(state));
            disconnects += state == foray::platform::link::HostLinkState::Disconnected ? 1U : 0U;
            previous_state = state;
        }
        if (state == foray::platform::link::HostLinkState::Active && active_at == 0U)
        {
            active_at = now;
        }
        active_steps += state == foray::platform::link::HostLinkState::Active ? 1U : 0U;
        if (send_control && state == foray::platform::link::HostLinkState::Active)
        {
            foray::platform::link::ControlSnapshot control;
            control.flags = 0x0002U;
            controls += link.queue_control(control, now) ? 1U : 0U;
        }
        const auto &attitude = link.latest_attitude();
        if (attitude && attitude->attitude.sample_time_us != previous_sample)
        {
            previous_sample = attitude->attitude.sample_time_us;
            ++attitudes;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    const double active_seconds = active_at == 0U ? 0.0 : static_cast<double>(now_ms() - active_at) / 1000.0;
    std::printf("\nActive 后 %.1f s：%u 轮里 %u 轮拿到新姿态，发出控制快照 %u 份，断链 %u 次\n",
                active_seconds,
                active_steps,
                attitudes,
                controls,
                disconnects > 0U ? disconnects - 1U : 0U);
    if (const auto &status = link.latest_status())
    {
        std::printf("最后的状态：uptime %u ms，mode %u，source %u，safety %u，last_rx_control_seq %u，link_state %u\n",
                    status->uptime_ms,
                    status->mode,
                    status->control_source,
                    status->safety_state,
                    status->last_rx_control_seq,
                    status->link_state);
    }
    if (const auto &attitude = link.latest_attitude())
    {
        const auto &q = attitude->attitude.quaternion_wxyz;
        const double yaw = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        const double pitch = std::asin(2.0 * (q[0] * q[2] - q[3] * q[1]));
        std::printf("最后的姿态：imu_state %u，valid %d，q = (%.4f, %.4f, %.4f, %.4f)，yaw %.2f°，pitch %.2f°，"
                    "帧头 − 采样 %u µs\n",
                    attitude->attitude.imu_state,
                    attitude->attitude.attitude_valid,
                    q[0],
                    q[1],
                    q[2],
                    q[3],
                    yaw * 180.0 / M_PI,
                    pitch * 180.0 / M_PI,
                    attitude->frame_timestamp_us - attitude->attitude.sample_time_us);
    }
    const bool ok = active_at != 0U && link.state() == foray::platform::link::HostLinkState::Active &&
                    disconnects <= 1U && attitudes >= active_steps / 100U * 95U;
    std::printf("%s\n", ok ? "通过：PC 端链路代码与板子握手、不断链，几乎每轮都拿到新姿态" : "失败");
    return ok ? 0 : 1;
}
