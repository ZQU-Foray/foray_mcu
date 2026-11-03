#ifndef TOPICS_PC_LINK_HPP
#define TOPICS_PC_LINK_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"

#include <cstdint>

// 上位机链路能力 pc_link::PcLink 发布的 topic（Decision-20261103-01 第 8 点）。

namespace topics::pc_link
{

/**
 * PC 发来的最新控制快照（RoboLink CONTROL_SNAPSHOT，字段与单位同协议 §4.7）。
 * 第一版只记账：没有能力订阅它执行；接受与执行随云台与仲裁能力重建时再做。
 * 未使能子系统的字段为 0（协议：视为缺失）。
 */
struct Control
{
    std::uint32_t control_seq;
    std::uint8_t source;               // 0 NONE · 1 REMOTE · 2 PC · 3 AUTO · 4 SAFETY
    std::uint8_t mode;                 // 0 MANUAL … 7 IDLE（协议 §4.6）
    std::uint16_t flags;               // bit0 底盘 · bit1 云台 · bit2 射击 · bit3 急停请求
    float chassis_vx;                  // m/s，机体系 x 前
    float chassis_vy;                  // m/s，机体系 y 左
    float chassis_wz;                  // rad/s，逆时针为正
    float gimbal_yaw;                  // rad，世界系，向左为正，不区分圈数
    float gimbal_pitch;                // rad，世界系，向下为正
    float gimbal_yaw_rate;             // rad/s，前馈
    float gimbal_pitch_rate;           // rad/s，前馈
    float friction_rads;               // rad/s
    float feeder_rads;                 // rad/s
    std::uint16_t validity_hint_ms;    // 0 = 无提示，不超过 50
    hal::os::Microseconds received_us; // MCU 处理这份快照的时刻
};

inline topic::Topic<Control> control{"pc_link.control"};

} // namespace topics::pc_link

#endif // TOPICS_PC_LINK_HPP
