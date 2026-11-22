// host_sim × pc_link 的整链测试（I26）：真实 PTY 上跑一次上位机握手。
//
// 被测：platform/host_sim 的 hal::os 与 hal::UsbCdc，加上 app/pc_link 与 middleware/robolink。
// 被测端从 PTY 从端收发（DIESEL_SIM_USB_CDC 指过去），测试扮上位机在 PTY 主端：
//   1. 发 HELLO（PC）→ 收到 HELLO_ACK 且 status=OK，state 变 READY；
//   2. 发 CONTROL_SNAPSHOT → PcLink 的 controls 计数加一（只记账，不执行）；
//   3. 主端静默超过 500 ms → 会话判活失败，state 退回 USB_CONNECTED；
//   4. 主端关闭 → 判为断开（DISCONNECTED）。
// 时间是真的：每步之间用 std::this_thread::sleep_for 推进，判活门限不可压缩。

#include "hal/os/clock.hpp"
#include "hal/usb_cdc.hpp"
#include "middleware/robolink/messages.hpp"
#include "middleware/robolink/wire.hpp"
#include "pc_link/pc_link.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{

int g_failures{0};

void check(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL %s\n", message);
        ++g_failures;
    }
    else
    {
        std::fprintf(stderr, "ok   %s\n", message);
    }
}

constexpr hal::os::Microseconds MS{1000U};

// 主端：非阻塞读写
struct Host
{
    int fd{-1};
    std::array<std::uint8_t, 512> buffer{};
    std::size_t used{0U};

    bool open(const char *path)
    {
        fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        return fd >= 0;
    }

    bool send(const std::vector<std::uint8_t> &bytes) const
    {
        std::size_t sent = 0U;
        while (sent < bytes.size())
        {
            const ssize_t written = ::write(fd, bytes.data() + sent, bytes.size() - sent);
            if (written <= 0)
            {
                return false;
            }
            sent += static_cast<std::size_t>(written);
        }
        return true;
    }

    void close()
    {
        if (fd >= 0)
        {
            ::close(fd);
            fd = -1;
        }
    }
};

std::vector<std::uint8_t>
frame(robolink::MessageType type, std::uint16_t sequence, const std::uint8_t *payload, std::size_t length)
{
    std::array<std::uint8_t, robolink::MAX_FRAME_SIZE> buffer{};
    robolink::FrameBuilder builder{buffer.data(), buffer.size()};
    builder.begin(0U, 0U);
    if (!builder.add(type, sequence, payload, length))
    {
        return {};
    }
    builder.finish();
    return std::vector<std::uint8_t>(builder.data(), builder.data() + builder.size());
}

// 从主端读到的字节里找一帧指定类型的消息；返回是否找到（同时把该槽位载荷拷出来）
bool poll_frame(Host &host, robolink::MessageType wanted, std::array<std::uint8_t, 512> &payload, std::size_t &length)
{
    for (;;)
    {
        const ssize_t count = ::read(host.fd, host.buffer.data() + host.used, host.buffer.size() - host.used);
        if (count > 0)
        {
            host.used += static_cast<std::size_t>(count);
            continue;
        }
        break;
    }
    robolink::FrameParser parser{};
    parser.push(host.buffer.data(), host.used);
    host.used = 0U;
    robolink::Frame frame{};
    std::size_t consumed = 0U;
    bool found = false;
    while (parser.next(frame))
    {
        consumed = frame.size;
        robolink::SlotReader reader{frame.data, frame.size};
        robolink::Slot slot{};
        while (reader.next(slot))
        {
            if (slot.type == static_cast<std::uint8_t>(wanted))
            {
                length = slot.length;
                std::memcpy(payload.data(), slot.payload, slot.length);
                found = true;
            }
        }
    }
    (void)consumed;
    return found;
}

// 跑 PcLink 若干次 step，每次 1 ms
template <typename Predicate>
bool run_until(pc_link::PcLink &link, Predicate done, int max_steps)
{
    for (int index = 0; index < max_steps; ++index)
    {
        link.step(hal::os::now_us());
        if (done())
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

} // namespace

int main()
{
    // 1. 建一对 PTY，把从端路径交给 host_sim
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0)
    {
        std::fprintf(stderr, "FAIL 建 PTY 失败\n");
        return 1;
    }
    const char *slave_path = ::ptsname(master);
    if (slave_path == nullptr)
    {
        std::fprintf(stderr, "FAIL ptsname 失败\n");
        return 1;
    }
    ::setenv("DIESEL_SIM_USB_CDC", slave_path, 1);

    Host host{};
    host.fd = master;
    (void)::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL, 0) | O_NONBLOCK); // posix_openpt 建的主端是阻塞的
    // 从端已由 host_sim 打开后才算连上：先让 PcLink 跑起来

    pc_link::PcLink link{hal::UsbCdc{0U}};
    pc_link::PcLink::Parameters parameters{};
    parameters.protocol.fw_version = {0U, 1U, 0U};
    parameters.max_reads_per_step = 8U;
    check(link.init(parameters), "pc_link.init() 成功");

    check(run_until(
              link, [&] { return link.report().state == robolink::LinkState::USB_CONNECTED; }, 200),
          "从端打开后进入 USB_CONNECTED");
    check(link.report().connected, "report().connected 为真");

    // 2. 发 HELLO（PC 声明支持 GIMBAL_ATTITUDE）
    robolink::Hello hello{};
    hello.device_type = robolink::DeviceType::PC;
    hello.capabilities = robolink::capability_bit(robolink::MessageType::GIMBAL_ATTITUDE);
    const auto hello_payload = robolink::encode_hello(hello);
    const auto hello_frame = frame(robolink::MessageType::HELLO, 0U, hello_payload.bytes.data(), hello_payload.size);
    check(host.send(hello_frame), "主端送出 HELLO");

    std::array<std::uint8_t, 512> payload{};
    std::size_t length{0U};
    check(
        run_until(link, [&] { return link.report().state == robolink::LinkState::READY; }, 500), "HELLO 后进入 READY");
    check(link.report().peer_accepts_attitude, "记录了对端支持 GIMBAL_ATTITUDE");

    // 3. 发一个控制快照：只记账
    robolink::ControlSnapshot snapshot{};
    snapshot.control_seq = 7U;
    snapshot.chassis_vx = 0.5F;
    snapshot.validity_hint_ms = 50U;
    const auto control_payload = robolink::encode_control_snapshot(snapshot);
    const auto control_frame =
        frame(robolink::MessageType::CONTROL_SNAPSHOT, 1U, control_payload.bytes.data(), control_payload.size);
    check(host.send(control_frame), "主端送出 CONTROL_SNAPSHOT");
    check(run_until(link, [&] { return link.report().controls >= 1U; }, 500), "控制快照被接收并记账");

    check(poll_frame(host, robolink::MessageType::HELLO_ACK, payload, length), "收到 HELLO_ACK");
    robolink::HelloAck ack{};
    if (length >= robolink::HELLO_ACK_SIZE)
    {
        check(robolink::decode_hello_ack(payload.data(), length, ack), "HELLO_ACK 可解码");
        check(ack.status == robolink::HelloStatus::OK, "HELLO_ACK status=OK");
        check(ack.negotiated_major == robolink::VERSION, "协商版本为 v0.2");
    }

    // 4. 静默超过判活门限（500 ms）：退回 USB_CONNECTED
    check(run_until(
              link, [&] { return link.report().state == robolink::LinkState::USB_CONNECTED; }, 900),
          "静默 500 ms 后判活失败，退回 USB_CONNECTED");

    // 5. 主端关闭：判为断开
    host.close();
    check(run_until(
              link, [&] { return link.report().state == robolink::LinkState::DISCONNECTED; }, 500),
          "主端关闭后进入 DISCONNECTED");

    if (g_failures != 0)
    {
        std::fprintf(stderr, "host_sim_link: %d 项失败\n", g_failures);
        return 1;
    }
    std::fprintf(stderr, "host_sim_link: 全部通过（判活门限 %u µs）\n", static_cast<unsigned>(500U * MS));
    return 0;
}
