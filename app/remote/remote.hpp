#ifndef REMOTE_REMOTE_HPP
#define REMOTE_REMOTE_HPP

#include "driver/protocol/sbus/sbus.hpp"
#include "hal/os/clock.hpp"
#include "hal/uart.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/safety.hpp"

#include <cstdint>

namespace remote
{

/**
 * @brief 遥控的最小版：SBUS → safety.stop，给安全门提供急停与人工确认。
 *
 * 每个 step() 都发布一次 `safety.stop`，不管这一轮有没有收到新帧：安全门靠超时判断发布者挂了。
 * "运行"要三条同时满足：最近一帧在 `frame_timeout_us` 以内、帧里没有 failsafe、拨杆在运行档。
 * `arm_sequence` 只在拨杆从非运行档拨到运行档时加 1；上电时拨杆已经在运行档的话没有这个"拨"的动作，
 * 安全门不会放开，要把拨杆拨回去再拨上（故意的）。
 * 失控保护帧（接收机失联时一般改发带 failsafe 位的帧）不更新拨杆档位，只让"运行"变假：信号恢复后
 * 拨杆还在运行档不算一次拨上，要像上电时那样拨回去再拨上；与超时那条路的行为一致。
 *
 * 只做这一件事：摇杆控制云台、底盘要经过控制权仲裁，以后单独做。仲裁做好后本能力改发布 `remote.input`，
 * `safety.stop` 由仲裁发布。
 *
 * @note 上电后按固定周期（如 5 ms）调用 step()。能力零 RTOS，UART 句柄由组合根在构造时注入。
 */
class Remote final
{
  public:
    /** 来自 profile 的参数（params.hpp）。 */
    struct Parameters
    {
        std::uint8_t channel;                   // 拨杆所在通道，0–15
        std::uint16_t run_minimum_raw;          // 运行档的原始值下限（含）
        std::uint16_t run_maximum_raw;          // 运行档的原始值上限（含）
        hal::os::Microseconds frame_timeout_us; // 这么久没有正常帧就当失控
    };

    /** 统计，调试器可读（只有运行 step() 的任务写）。 */
    struct Report
    {
        std::uint32_t bytes;       // 读到的字节
        std::uint32_t frames;      // 解析出的帧
        std::uint32_t rejected;    // 格式不对的帧
        std::uint32_t failsafe;    // 接收机报告失控保护的帧
        std::uint16_t channel_raw; // 最近一帧里拨杆通道的原始值
        bool fresh;                // 最近一帧还在超时以内
        bool running;              // 拨杆在运行档、帧新鲜、没有 failsafe
        bool stop;                 // 发布的 safety.stop.stop
        std::uint32_t arm_sequence;
        bool port_ready;
    };

    constexpr explicit Remote(hal::Uart port) noexcept : port_{port} {}

    /** 检查参数并取得 safety.stop 的发布句柄。 */
    [[nodiscard]] bool init(const Parameters &parameters) noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    hal::Uart port_;
    Parameters parameters_{};
    protocol::SbusParser parser_{};
    topic::Publisher<topics::safety::Stop> publisher_{};
    Report report_{};
    hal::os::Microseconds last_frame_us_{0U};
    bool has_frame_{false};
    bool has_previous_frame_{false}; // 见过一帧正常帧以后才能判"拨"的动作
    bool last_run_position_{false};
    bool failsafe_{false}; // 最近一帧是不是失控保护（这一帧的拨杆值不可信）
};

} // namespace remote

#endif
