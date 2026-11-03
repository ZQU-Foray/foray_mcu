#ifndef BRINGUP_CAN_CHECK_HPP
#define BRINGUP_CAN_CHECK_HPP

#include "hal/can.hpp"
#include "hal/os/clock.hpp"

#include <cstdint>

namespace bringup
{

/**
 * @brief CAN 检查：只收不发，统计一条总线上收到的帧，验证 hal::Can 的接收路径。
 *        接上 DJI 电调时能看到 0x201–0x20B 的反馈（电调要先收到控制帧才回反馈，C610/C620 不会自己发）。
 * @note 不发送任何帧：bringup 固件接在整车上也不会让电机动。
 */
class CanCheck final
{
  public:
    struct Report
    {
        std::uint32_t frames;                // 收到的帧
        std::uint32_t last_identifier;       // 最后一帧的标识符
        std::uint16_t dji_feedback_seen;     // 位 n：见过 0x201 + n 的标准帧（n = 0–10）
        hal::os::Microseconds last_frame_us; // 最后一帧的时刻
        hal::Can::Statistics statistics;     // port 的计数
        bool bus_ready;                      // 总线启动且不在 Bus-Off
    };

    constexpr explicit CanCheck(hal::Can bus) noexcept : bus_{bus} {}

    /** 占住总线的接收；被占用时返回 false。 */
    [[nodiscard]] bool init() noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    hal::Can bus_;
    Report report_{};
};

} // namespace bringup

#endif // BRINGUP_CAN_CHECK_HPP
