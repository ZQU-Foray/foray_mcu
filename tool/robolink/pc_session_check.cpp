// PC 端 foray_platform 的链路状态机（RoboLinkHostLink + 串口类）与 MCU 端的 robolink::Session
// 在伪终端（pty）上联调（Decision-20261103-01）。不进本仓构建：PC 端代码在另一个仓库，编译命令见
// tool/robolink/README.md。
//
// 时间按 1 ms 一步模拟（PC 用 ms，MCU 用 µs，起点不同）。场景：
//   1. 握手：PC 打开串口发 HELLO，MCU 回应，PC 进入 Active；
//   2. 稳态 2 s：MCU 每毫秒给一份姿态，PC 收到的姿态与状态和 MCU 发的一致；PC 每毫秒发一份控制快照，
//      MCU 收到的序号连续，状态里的 last_rx_control_seq 回到 PC；
//   3. MCU 停发 700 ms（模拟卡死）：PC 判断断链、重开串口、重新握手；MCU 判活超时后重新握手；
//   4. PC 卡住 600 ms（不调用 step）：MCU 判活超时、停发；PC 恢复后断链重连，两端重新握手。
#include "foray/platform/link/robolink_host_link.hpp"
#include "foray/platform/link/usb_cdc_port.hpp"
#include "middleware/robolink/session.hpp"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <thread>
#include <unistd.h>

namespace
{

int g_failures = 0;

void check(bool ok, const char *what)
{
    std::printf("%s %s\n", ok ? "通过" : "失败", what);
    g_failures += ok ? 0 : 1;
}

const char *pc_state_name(foray::platform::link::HostLinkState state)
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

struct Mcu
{
    int fd;
    robolink::Session session;
    std::uint32_t attitude_count{0U};
    std::uint32_t controls{0U};
    std::uint32_t last_control_seq{0U};
    std::uint32_t control_gaps{0U};
    bool frozen{false};

    void step(std::uint32_t now_us)
    {
        session.set_connected(true, now_us);
        std::uint8_t buffer[256];
        for (;;)
        {
            const std::size_t space = session.receive_space();
            if (space == 0U)
            {
                break;
            }
            const ssize_t got = ::read(fd, buffer, space < sizeof(buffer) ? space : sizeof(buffer));
            if (got <= 0)
            {
                break; // EAGAIN；PC 关了串口时是 EIO
            }
            session.receive(buffer, static_cast<std::size_t>(got));
        }
        session.process(now_us);
        robolink::ControlSnapshot control{};
        std::uint32_t received = 0U;
        if (session.take_control(control, received))
        {
            if (controls != 0U && control.control_seq != last_control_seq + 1U)
            {
                ++control_gaps;
            }
            last_control_seq = control.control_seq;
            ++controls;
        }
        robolink::GimbalAttitude attitude{};
        ++attitude_count;
        attitude.sample_time_us = now_us - 300U;
        attitude.quaternion_wxyz = {0.5F, 0.5F, -0.5F, 0.5F};
        attitude.gyro_rad_per_sec = {0.01F * static_cast<float>(attitude_count % 100U), -1.0F, 2.0F};
        attitude.imu_state = robolink::ImuState::READY;
        attitude.attitude_valid = true;
        session.set_attitude(attitude);
        if (frozen)
        {
            return;
        }
        const std::size_t size = session.prepare(now_us);
        if (size == 0U)
        {
            return;
        }
        const ssize_t written = ::write(fd, session.frame(), size);
        if (written == static_cast<ssize_t>(size))
        {
            session.commit(now_us);
        }
        else
        {
            session.report_busy();
        }
    }
};

} // namespace

int main()
{
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0)
    {
        std::perror("posix_openpt");
        return 2;
    }
    ::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL) | O_NONBLOCK);
    const std::string slave = ::ptsname(master);

    foray::platform::link::UsbCdcPort port{slave};
    foray::platform::link::RoboLinkHostLink pc{port};
    Mcu mcu{master, {}};
    robolink::Session::Config config{};
    config.fw_version = {0U, 1U, 0U};
    const std::uint32_t mcu_start_us = 0xFFF00000U; // MCU 时基快回绕：顺带验证回绕
    if (!mcu.session.init(config, mcu_start_us))
    {
        return 2;
    }

    std::uint64_t now_ms = 1000U;
    std::uint32_t pc_controls = 0U;
    std::uint32_t pc_attitudes = 0U;
    std::uint32_t last_pc_status_rx = 0U;
    std::uint32_t handshake_ms = 0U;
    auto run = [&](std::uint32_t steps, bool send_control, bool pc_paused = false)
    {
        std::uint32_t previous_sample = 0U;
        for (std::uint32_t step = 0U; step < steps; ++step, ++now_ms)
        {
            const auto state = pc_paused ? pc.state() : pc.step(now_ms);
            if (state == foray::platform::link::HostLinkState::Active && handshake_ms == 0U)
            {
                handshake_ms = static_cast<std::uint32_t>(now_ms);
            }
            if (send_control && !pc_paused && state == foray::platform::link::HostLinkState::Active)
            {
                foray::platform::link::ControlSnapshot control;
                control.flags = 0x0002U;
                control.gimbal_yaw_rad = 0.5F;
                pc_controls += pc.queue_control(control, now_ms) ? 1U : 0U;
            }
            if (pc.latest_attitude() && pc.latest_attitude()->attitude.sample_time_us != previous_sample)
            {
                previous_sample = pc.latest_attitude()->attitude.sample_time_us;
                ++pc_attitudes;
            }
            if (pc.latest_status())
            {
                last_pc_status_rx = pc.latest_status()->last_rx_control_seq;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(150)); // pty 在内核里异步转发
            mcu.step(mcu_start_us + static_cast<std::uint32_t>((now_ms - 1000U) * 1000U));
        }
    };

    // 1. 握手
    run(100U, false);
    std::printf("信息 PC 状态 %s，握手用时 %u ms；MCU 状态 %u\n",
                pc_state_name(pc.state()),
                handshake_ms - 1000U,
                static_cast<unsigned>(mcu.session.state()));
    check(pc.state() == foray::platform::link::HostLinkState::Active, "PC 进入 Active");
    check(mcu.session.state() == robolink::LinkState::ACTIVE, "MCU 进入 ACTIVE");

    // 2. 稳态 2 s
    pc_attitudes = 0U;
    run(2000U, true);
    const auto &attitude = pc.latest_attitude();
    const auto &status = pc.latest_status();
    std::printf("信息 2 s 内 PC 收到姿态 %u 份、发出控制快照 %u 份；MCU 收到快照 %u 份，序号跳变 %u 次\n",
                pc_attitudes,
                pc_controls,
                mcu.controls,
                mcu.control_gaps);
    check(attitude && attitude->attitude.attitude_valid && attitude->attitude.quaternion_wxyz[2] == -0.5F &&
              attitude->attitude.gyro_rad_s[2] == 2.0F,
          "PC 解出的姿态与 MCU 发的一致");
    check(attitude &&
              static_cast<std::uint32_t>(attitude->frame_timestamp_us - attitude->attitude.sample_time_us) < 2000U,
          "帧头时刻 − 采样时刻 < 2 ms（§4.8）");
    check(pc_attitudes >= 1900U, "姿态接近 1 kHz（2 s 内 ≥ 1900 份）");
    check(status && status->link_state == 3U && status->mode == 7U, "PC 收到的状态：ACTIVE、IDLE");
    check(mcu.controls >= 1900U && mcu.control_gaps == 0U, "MCU 收到的控制快照接近 1 kHz、序号连续");
    check(last_pc_status_rx + 10U >= mcu.last_control_seq && last_pc_status_rx != 0U,
          "状态里的 last_rx_control_seq 回到 PC");

    // 3. MCU 卡死 700 ms
    mcu.frozen = true;
    run(700U, false);
    std::printf("信息 MCU 停发 700 ms 后：PC 状态 %s，MCU 状态 %u\n",
                pc_state_name(pc.state()),
                static_cast<unsigned>(mcu.session.state()));
    check(pc.state() != foray::platform::link::HostLinkState::Active, "PC 判断断链");
    mcu.frozen = false;
    run(1000U, true);
    std::printf("信息 恢复后：PC 状态 %s，MCU 状态 %u，握手 %u 次，判活超时 %u 次\n",
                pc_state_name(pc.state()),
                static_cast<unsigned>(mcu.session.state()),
                mcu.session.statistics().handshakes,
                mcu.session.statistics().link_timeouts);
    check(pc.state() == foray::platform::link::HostLinkState::Active, "PC 重新进入 Active");
    check(mcu.session.state() == robolink::LinkState::ACTIVE && mcu.session.statistics().handshakes >= 2U,
          "MCU 重新握手");

    // 4. PC 卡住 600 ms
    run(600U, false, true);
    std::printf("信息 PC 卡住 600 ms 后：MCU 状态 %u，判活超时 %u 次\n",
                static_cast<unsigned>(mcu.session.state()),
                mcu.session.statistics().link_timeouts);
    check(mcu.session.state() == robolink::LinkState::USB_CONNECTED && mcu.session.statistics().link_timeouts == 1U,
          "MCU 判活超时，回到 USB_CONNECTED");
    run(1000U, true);
    std::printf("信息 PC 恢复后：PC 状态 %s，MCU 状态 %u，握手 %u 次\n",
                pc_state_name(pc.state()),
                static_cast<unsigned>(mcu.session.state()),
                mcu.session.statistics().handshakes);
    check(pc.state() == foray::platform::link::HostLinkState::Active &&
              mcu.session.state() == robolink::LinkState::ACTIVE && mcu.session.statistics().handshakes >= 3U,
          "两端重新握手");

    const auto stats = mcu.session.statistics();
    std::printf("信息 MCU 计数：收 %u 帧，发 %u 帧，CRC 错 %u，帧头错 %u，帧错 %u，忙丢 %u，内容拒收 %u，方向不对 %u\n",
                stats.rx_frames,
                stats.tx_frames,
                stats.crc_errors,
                stats.header_errors,
                stats.frame_errors,
                stats.tx_busy_drop,
                stats.invalid_payloads,
                stats.unexpected_messages);
    // 场景 4 里 PC 卡住期间串口攒了 MCU 早先发的帧：PC 恢复后读到它们，以为链路还活着，继续发控制快照；
    // MCU 已判超时、等重新握手，这些快照计入"方向不对"（没握手的控制快照），直到 PC 自己判出断链。
    // 同一段时间 PC 不读串口，MCU 写不进去，计入忙丢。
    check(stats.crc_errors == 0U && stats.header_errors == 0U && stats.frame_errors == 0U &&
              stats.invalid_payloads == 0U,
          "MCU 收到的帧全部合法（CRC、帧头、槽位、内容）");
    std::printf(g_failures == 0 ? "\n全部通过\n" : "\n%d 项失败\n", g_failures);
    ::close(master);
    return g_failures == 0 ? 0 : 1;
}
