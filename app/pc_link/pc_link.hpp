#ifndef PC_LINK_PC_LINK_HPP
#define PC_LINK_PC_LINK_HPP

#include "hal/os/clock.hpp"
#include "hal/usb_cdc.hpp"
#include "middleware/robolink/session.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/imu.hpp"
#include "topics/pc_link.hpp"

#include <cstdint>

namespace pc_link
{

/**
 * @brief 上位机链路能力：RoboLink v0.2 经 USB 虚拟串口。
 *
 * 每轮 step()：
 *   1. 传输层连没连上，把 USB 收到的字节交给会话（robolink::Session），处理握手、控制快照与判活；
 *   2. 新的控制快照发布到 pc_link.control（第一版只记账，不执行）；
 *   3. imu.attitude 的新姿态交给会话，PC 声明支持时以 GIMBAL_ATTITUDE 发出；
 *   4. 会话组下一帧（握手应答、状态、姿态或保活），用 USB 发出；USB 忙就下一轮按最新值重组。
 *
 * SYSTEM_STATUS 第一版：模式 IDLE、来源 NONE、安全状态"未收到快照"、accepted/applied 为 0、age 65535。
 * 帧头时刻与 IMU 采样时刻都是 hal::os::now_us() 的同一时钟。能力零 RTOS；USB 句柄由组合根在构造时注入。
 */
class PcLink final
{
  public:
    /** 来自 profile 的参数（params.hpp）。 */
    struct Parameters
    {
        robolink::Session::Config protocol; // 本端 HELLO 的固件版本、心跳、状态与诊断周期、判活门限
        std::uint8_t max_reads_per_step;    // 每轮最多读几次 USB（每次最多 64 B）
    };

    /** 统计，调试器可读（只有运行 step() 的任务写）。 */
    struct Report
    {
        robolink::LinkState state;
        bool connected;               // USB 已被主机打开
        bool peer_accepts_attitude;   // PC 的 HELLO 声明了 GIMBAL_ATTITUDE
        std::uint32_t received_bytes; // 交给会话的字节
        std::uint32_t written_frames; // USB 开始发送的帧
        std::uint32_t write_busy;     // USB 上一次发送没结束
        std::uint32_t write_errors;   // USB 没连上或出错
        std::uint32_t controls;       // 发布的控制快照
        std::uint32_t attitudes;      // 交给会话的新姿态
        robolink::Session::Statistics session;
    };

    constexpr explicit PcLink(hal::UsbCdc usb) noexcept : usb_{usb} {}

    [[nodiscard]] bool init(const Parameters &parameters) noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    void receive() noexcept;
    void transmit() noexcept;

    hal::UsbCdc usb_;
    Parameters parameters_{};
    robolink::Session session_{};
    topic::Publisher<topics::pc_link::Control> control_publisher_{};
    topic::Subscriber<topics::imu::Attitude> attitude_{topics::imu::attitude};
    Report report_{};
};

} // namespace pc_link

#endif // PC_LINK_PC_LINK_HPP
