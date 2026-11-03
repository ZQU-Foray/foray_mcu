#ifndef BRINGUP_SBUS_CHECK_HPP
#define BRINGUP_SBUS_CHECK_HPP

#include "driver/protocol/sbus/sbus.hpp"
#include "hal/os/clock.hpp"
#include "hal/uart.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/bringup.hpp"

#include <cstdint>

namespace bringup
{

/**
 * @brief SBUS 检查：从 UART 读字节流、解析 SBUS 帧，发布 bringup.sbus，
 *        验证 hal::Uart 的接收路径与 SBUS 协议解析。
 */
class SbusCheck final
{
  public:
    struct Report
    {
        std::uint32_t bytes;    // 读到的字节
        std::uint32_t frames;   // 解析出的帧
        std::uint32_t rejected; // 格式不对的帧
        std::uint32_t failsafe; // 接收机报告失控保护的帧
        hal::Uart::Statistics statistics;
        bool port_ready;
    };

    constexpr explicit SbusCheck(hal::Uart port) noexcept : port_{port} {}

    /** 取得 bringup.sbus 的发布句柄。 */
    [[nodiscard]] bool init() noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    hal::Uart port_;
    protocol::SbusParser parser_{};
    topic::Publisher<topics::bringup::Sbus> publisher_{};
    Report report_{};
};

} // namespace bringup

#endif // BRINGUP_SBUS_CHECK_HPP
