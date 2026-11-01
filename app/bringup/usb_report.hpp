#ifndef BRINGUP_USB_REPORT_HPP
#define BRINGUP_USB_REPORT_HPP

#include "bringup/text_line.hpp"
#include "hal/os/clock.hpp"
#include "hal/usb_cdc.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/bringup.hpp"

#include <cstdint>

namespace bringup
{

/**
 * @brief USB 报告（Decision-20261101-03）：在 USB 虚拟串口上回显收到的字节，并按周期输出一行状态
 *        （心跳、IMU 读数与姿态、SBUS），验证 hal::UsbCdc 的收发；也是 F4 示范"IMU → 姿态 topic → USB"的输出端。
 *
 * 状态行格式（ASCII，\r\n 结尾）：
 *   t=<ms> hb=<心跳序号> imu=<陀螺样本数> q=<w>,<x>,<y>,<z> g=<x>,<y>,<z> a=<x>,<y>,<z> T=<°C>
 *   sbus=<帧数> ch=<1>,<2>,<3>,<4> fs=<0|1>
 * 角速度单位 rad/s，加速度 m/s²。用任意串口终端打开即可看到（波特率不影响 USB CDC）。
 */
class UsbReport final
{
  public:
    struct Parameters
    {
        hal::os::Microseconds period_us; // 状态行的间隔
    };

    struct Report
    {
        std::uint32_t lines;        // 发出的状态行
        std::uint32_t echoed_bytes; // 回显的字节
        std::uint32_t skipped;      // 上一次发送没结束而跳过的状态行或回显
        bool connected;             // 主机已打开端口
    };

    constexpr explicit UsbReport(hal::UsbCdc usb) noexcept : usb_{usb} {}

    [[nodiscard]] bool init(const Parameters &parameters) noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

    /** 最后一次组好的状态行（宿主测试检查格式）。 */
    [[nodiscard]] const TextLine &last_line() const noexcept
    {
        return line_;
    }

  private:
    void format_line(hal::os::Microseconds now) noexcept;

    hal::UsbCdc usb_;
    Parameters parameters_{0U};
    topic::Subscriber<topics::bringup::Heartbeat> heartbeat_{topics::bringup::heartbeat};
    topic::Subscriber<topics::bringup::Imu> imu_{topics::bringup::imu};
    topic::Subscriber<topics::bringup::Sbus> sbus_{topics::bringup::sbus};
    topics::bringup::Heartbeat latest_heartbeat_{};
    topics::bringup::Imu latest_imu_{};
    topics::bringup::Sbus latest_sbus_{};
    hal::os::Microseconds last_line_us_{0U};
    TextLine line_{};
    Report report_{};
};

} // namespace bringup

#endif // BRINGUP_USB_REPORT_HPP
