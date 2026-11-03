// 用 PC 端 foray_platform 的 RoboLink 编解码核对本仓的标准帧（Decision-20261103-01）。
// 不进本仓构建：PC 端代码在另一个仓库，编译命令见 tool/robolink/README.md。
//
// 核对四件事：
//   1. PC 的帧级解码对每个标准帧的接受与拒绝，与标准帧的期望一致；
//   2. PC 的字节流解析器从全部标准帧拼成的流里取出的帧数；
//   3. MCU → PC 的标准帧，PC 解出的字段与生成时的取值一致；
//   4. PC → MCU 的帧，PC 编出的字节与标准帧逐字节相同。
#include "foray/platform/link/robolink_codec.hpp"
#include "foray/platform/link/robolink_frame_parser.hpp"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

using foray::platform::link::ControlSnapshot;
using foray::platform::link::Hello;
using foray::platform::link::RoboLinkFrameParser;
using foray::platform::link::RoboLinkSlot;

struct Vector
{
    std::string expect;
    std::vector<std::uint8_t> bytes;
};

std::map<std::string, Vector> load(const char *path)
{
    std::map<std::string, Vector> table;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream fields(line);
        std::string name;
        std::string hex;
        Vector vector;
        fields >> name >> vector.expect >> hex;
        for (std::size_t index = 0U; index + 1U < hex.size(); index += 2U)
        {
            vector.bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(index, 2U), nullptr, 16)));
        }
        table[name] = vector;
    }
    return table;
}

int g_failures = 0;

void check(bool ok, const char *what)
{
    std::printf("%s %s\n", ok ? "通过" : "失败", what);
    g_failures += ok ? 0 : 1;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "用法：%s test/data/robolink/golden_v0_2.txt\n", argv[0]);
        return 2;
    }
    auto table = load(argv[1]);
    if (table.empty())
    {
        std::fprintf(stderr, "读不到标准帧 %s\n", argv[1]);
        return 2;
    }

    // 1. 帧级接受与拒绝（payload 类的帧在帧层合法）
    for (const auto &[name, vector] : table)
    {
        const bool accepted = foray::platform::link::decode_frame(vector.bytes).has_value();
        const bool expected = vector.expect == "ok" || vector.expect == "payload";
        if (name == "pc_control_extended")
        {
            // 协议 §7.1 允许载荷末尾追加字段；PC 端要求长度正好 48 B，记为差异
            std::printf("差异 pc_control_extended：PC 端%s\n", accepted ? "接受" : "拒收（要求载荷正好 48 B）");
            continue;
        }
        if (accepted != expected)
        {
            std::printf(
                "失败 帧级 %s：期望 %s，PC 端%s\n", name.c_str(), vector.expect.c_str(), accepted ? "接受" : "拒收");
            ++g_failures;
        }
    }

    // 2. 字节流解析器
    {
        RoboLinkFrameParser parser;
        std::size_t frames = 0U;
        std::size_t expected = 0U;
        for (const auto &[name, vector] : table)
        {
            (void)name;
            expected += (vector.expect == "header" || vector.expect == "crc") ? 0U : 1U;
            for (const std::uint8_t byte : vector.bytes)
            {
                frames += parser.feed(&byte, 1U).size();
            }
        }
        std::printf("信息 PC 端解析器取出 %zu 帧（帧头与 CRC 合法的标准帧 %zu 个）\n", frames, expected);
        check(frames == expected, "PC 端解析器取出帧头与 CRC 合法的全部帧");
    }

    // 3. MCU → PC
    {
        const auto slots = foray::platform::link::decode_frame(table["mcu_status_attitude"].bytes);
        check(slots && slots->size() == 2U, "状态与姿态同帧：两个槽位");
        if (slots && slots->size() == 2U)
        {
            const auto status = foray::platform::link::decode_system_status_payload((*slots)[0].payload.data(),
                                                                                    (*slots)[0].payload.size());
            check(status && status->uptime_ms == 123456U && status->mode == 7U && status->last_rx_control_seq == 41U &&
                      status->control_msg_age_ms == 65535U && status->link_state == 3U &&
                      status->fault_flags == 0x80000001U && !status->diagnostics,
                  "SYSTEM_STATUS 核心字段");
            const auto attitude = foray::platform::link::decode_gimbal_attitude_payload((*slots)[1].payload.data(),
                                                                                        (*slots)[1].payload.size());
            check(attitude && attitude->sample_time_us == 0xFFFFFE00U && attitude->quaternion_wxyz[1] == -0.5F &&
                      attitude->gyro_rad_s[1] == -2.5F && attitude->imu_state == 1U && attitude->attitude_valid,
                  "GIMBAL_ATTITUDE 字段");
        }
    }
    {
        const auto status = foray::platform::link::decode_system_status_frame(table["mcu_status_full"].bytes);
        check(status && status->diagnostics && status->diagnostics->rx_frames == 1000U &&
                  status->diagnostics->crc_errors == 3U && status->diagnostics->event_queue_depth == 7U,
              "SYSTEM_STATUS 诊断尾部");
    }
    {
        const auto slots = foray::platform::link::decode_frame(table["mcu_handshake"].bytes);
        check(slots && slots->size() == 2U, "握手应答：两个槽位");
        if (slots && slots->size() == 2U)
        {
            const auto ack =
                foray::platform::link::decode_hello_ack_payload((*slots)[0].payload.data(), (*slots)[0].payload.size());
            check(ack && ack->acked_msg_seq == 7U && ack->negotiated_major == 2U && ack->status == 0U &&
                      ack->heartbeat_period_ms == 20U,
                  "HELLO_ACK 字段");
            const auto hello =
                foray::platform::link::decode_hello_payload((*slots)[1].payload.data(), (*slots)[1].payload.size());
            check(hello && hello->device_type == 0U && hello->git_short == 0x1234ABCDU && hello->fw_version[1] == 1U &&
                      hello->max_frame_bytes == 512U && ((hello->capabilities >> 48U) & 1U) != 0U,
                  "MCU 的 HELLO 字段");
        }
    }
    {
        const auto slots = foray::platform::link::decode_frame(table["mcu_attitude_invalid"].bytes);
        const auto attitude =
            slots ? foray::platform::link::decode_gimbal_attitude_payload((*slots)[0].payload.data(), 36U)
                  : std::nullopt;
        check(attitude && !attitude->attitude_valid && attitude->imu_state == 0U, "姿态无效时 PC 端仍接受该帧");
    }

    // 4. PC → MCU
    {
        ControlSnapshot control;
        control.control_seq = 2U;
        control.source = 2U;
        control.mode = 2U;
        control.flags = 2U;
        control.chassis_vx = 0.5F;
        control.chassis_vy = -0.25F;
        control.chassis_wz = 1.5F;
        control.gimbal_yaw_rad = -3.0F;
        control.gimbal_pitch_rad = 0.125F;
        control.gimbal_yaw_rate = 0.5F;
        control.gimbal_pitch_rate = -0.0625F;
        control.friction_rads = 600.0F;
        control.feeder_rads = 12.5F;
        control.validity_hint_ms = 30U;
        check(foray::platform::link::make_control_snapshot_frame(3U, 123456789U, 2U, control) ==
                  table["pc_control"].bytes,
              "PC 端编出的控制快照 = 标准帧");
    }
    {
        // 与 RoboLinkHostLink::queue_hello() 的取值相同
        Hello hello;
        for (const unsigned type : {1U, 2U, 3U, 4U, 5U, 16U, 32U, 48U})
        {
            hello.capabilities |= std::uint64_t{1U} << type;
        }
        RoboLinkSlot slot;
        slot.type = 0x01U;
        slot.flags = 0x04U;
        slot.sequence = 0U;
        slot.payload = foray::platform::link::encode_hello_payload(hello);
        check(foray::platform::link::encode_single_slot_frame(0U, 5000U, slot) == table["pc_hello"].bytes,
              "PC 端编出的 HELLO = 标准帧");
    }
    check(foray::platform::link::encode_frame(0x1234U, 0x89ABCDEFU, {}) == table["keepalive"].bytes,
          "PC 端编出的保活帧 = 标准帧");

    std::printf(g_failures == 0 ? "\n全部通过\n" : "\n%d 项失败\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
