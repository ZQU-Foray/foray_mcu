// 宿主下位机可执行目标（I26）：把 app/pc_link 与 platform/host_sim 组装成一个在宿主机上跑的上位机链路端点。
//
// 与 config/assembly/link_dm_mc02.cpp 的差别只在"板"：那边从 board.hpp 取 USB 句柄、由任务表按 topic 唤醒；
// 这边用 virtual_cdc.py 建出的 PTY 从端，主循环按 1 ms 步进（对应 DM-MC02 上的 pc_link 任务节拍）。
// 参数与 link_dm_mc02 一致（固件版本 0.1.0、心跳 20 ms、状态 200 Hz、诊断 1 s、判活 500 ms、每轮 8 帧）。
// 只做记账：控制快照发布到 pc_link.control，不驱动任何执行器。
//
// 环境变量：DIESEL_SIM_USB_CDC（设备路径，缺省 /tmp/diesel_mcu）。
// 输出：状态变化时打一行，之后每秒打一行统计（state/controls/attitudes/frames/errors）。

#include "hal/os/clock.hpp"
#include "hal/usb_cdc.hpp"
#include "pc_link/pc_link.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace
{

volatile std::sig_atomic_t g_stop{0};

void on_signal(int) noexcept
{
    g_stop = 1;
}

constexpr pc_link::PcLink::Parameters PARAMETERS{
    robolink::Session::Config{{0U, 1U, 0U}, 0U, 0U, 20U, 5000U, 1000000U, 500000U, 8U},
    8U,
};

const char *state_name(robolink::LinkState state) noexcept
{
    switch (state)
    {
    case robolink::LinkState::DISCONNECTED:
        return "DISCONNECTED";
    case robolink::LinkState::USB_CONNECTED:
        return "USB_CONNECTED";
    case robolink::LinkState::READY:
        return "READY";
    case robolink::LinkState::ACTIVE:
        return "ACTIVE";
    }
    return "UNKNOWN";
}

} // namespace

int main()
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    pc_link::PcLink link{hal::UsbCdc{0U}};
    if (!link.init(PARAMETERS))
    {
        std::fprintf(stderr, "firmware_sim: pc_link.init() 失败\n");
        return 1;
    }
    std::fprintf(stderr,
                 "firmware_sim: 启动，设备 %s\n",
                 std::getenv("DIESEL_SIM_USB_CDC") != nullptr ? std::getenv("DIESEL_SIM_USB_CDC") : "/tmp/diesel_mcu");

    robolink::LinkState last_state = robolink::LinkState::DISCONNECTED;
    hal::os::Microseconds last_report = hal::os::now_us();
    while (g_stop == 0)
    {
        const hal::os::Microseconds now = hal::os::now_us();
        link.step(now);

        const pc_link::PcLink::Report &report = link.report();
        if (report.state != last_state)
        {
            std::fprintf(stderr,
                         "firmware_sim: state=%s connected=%d controls=%u\n",
                         state_name(report.state),
                         report.connected ? 1 : 0,
                         report.controls);
            last_state = report.state;
        }
        if (hal::os::has_elapsed(last_report, 1000000U, now))
        {
            last_report = now;
            std::fprintf(stderr,
                         "firmware_sim: state=%s controls=%u attitudes=%u frames=%u rx=%u busy=%u err=%u\n",
                         state_name(report.state),
                         report.controls,
                         report.attitudes,
                         report.written_frames,
                         report.received_bytes,
                         report.write_busy,
                         report.write_errors);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const pc_link::PcLink::Report &report = link.report();
    std::fprintf(stderr,
                 "firmware_sim: 退出，state=%s controls=%u attitudes=%u\n",
                 state_name(report.state),
                 report.controls,
                 report.attitudes);
    return 0;
}
