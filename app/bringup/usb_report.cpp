#include "bringup/usb_report.hpp"

#include <array>
#include <cstddef>

namespace bringup
{

bool UsbReport::init(const Parameters &parameters) noexcept
{
    if (parameters.period_us == 0U)
    {
        return false;
    }
    parameters_ = parameters;
    report_ = Report{};
    latest_imu_.quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
    // 不检查 usb_.is_ready()：DM-MC02 的 USB 在内核启动后才由生成区的默认任务初始化，连上之前 step() 什么都不做
    return true;
}

void UsbReport::step(hal::os::Microseconds now) noexcept
{
    (void)heartbeat_.update(latest_heartbeat_);
    (void)imu_.update(latest_imu_);
    (void)sbus_.update(latest_sbus_);
    report_.connected = usb_.is_connected();
    if (!report_.connected)
    {
        return;
    }

    // 状态行优先；同时只能有一次发送在进行，发不出去就等下一个周期
    if (hal::os::has_elapsed(last_line_us_, parameters_.period_us, now))
    {
        last_line_us_ = now;
        format_line(now);
        if (usb_.write(line_.data(), line_.size()) == hal::UsbCdc::WriteResult::STARTED)
        {
            ++report_.lines;
        }
        else
        {
            ++report_.skipped;
        }
        return;
    }

    std::array<std::uint8_t, 64> echo{};
    const std::size_t length = usb_.read(echo.data(), echo.size());
    if (length == 0U)
    {
        return;
    }
    if (usb_.write(echo.data(), length) == hal::UsbCdc::WriteResult::STARTED)
    {
        report_.echoed_bytes += static_cast<std::uint32_t>(length);
    }
    else
    {
        ++report_.skipped;
    }
}

void UsbReport::format_line(hal::os::Microseconds now) noexcept
{
    line_.clear();
    line_.append("t=");
    line_.append_unsigned(now / hal::os::MICROSECONDS_PER_MILLISECOND);
    line_.append(" hb=");
    line_.append_unsigned(latest_heartbeat_.sequence);
    line_.append(" imu=");
    line_.append_unsigned(latest_imu_.gyro_sample_count);
    line_.append(" q=");
    for (std::size_t index = 0U; index < latest_imu_.quaternion.size(); ++index)
    {
        line_.append(index == 0U ? "" : ",");
        line_.append_fixed(latest_imu_.quaternion[index], 4U);
    }
    line_.append(" g=");
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        line_.append(index == 0U ? "" : ",");
        line_.append_fixed(latest_imu_.gyro_rad_per_sec[index], 3U);
    }
    line_.append(" a=");
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        line_.append(index == 0U ? "" : ",");
        line_.append_fixed(latest_imu_.accel_m_per_s2[index], 2U);
    }
    line_.append(" T=");
    line_.append_fixed(latest_imu_.temperature_celsius, 1U);
    line_.append(" sbus=");
    line_.append_unsigned(latest_sbus_.frame_count);
    line_.append(" ch=");
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        line_.append(index == 0U ? "" : ",");
        line_.append_unsigned(latest_sbus_.channels[index]);
    }
    line_.append(" fs=");
    line_.append(latest_sbus_.failsafe ? "1" : "0");
    line_.append("\r\n");
}

} // namespace bringup
