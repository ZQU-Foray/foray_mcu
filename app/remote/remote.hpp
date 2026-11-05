#ifndef REMOTE_REMOTE_HPP
#define REMOTE_REMOTE_HPP

#include "driver/protocol/sbus/sbus.hpp"
#include "hal/os/clock.hpp"
#include "hal/uart.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/operator_input.hpp"
#include "topics/remote.hpp"

#include <array>
#include <cstdint>

namespace remote
{

/** 参数里表示"没有这个通道"。 */
inline constexpr std::uint8_t NO_CHANNEL{0xFFU};

/**
 * @brief 遥控：SBUS 字节流 → 统一的操作输入 `remote.input`。
 *
 * 本能力只做翻译：摇杆按标定归一化到 [−1, 1]（扣中位、按方向、按死区），拨杆按门限换成档位，
 * 读不到的通道填 NONE / 0。拨杆代表什么功能、能不能动，由仲裁能力判断。
 *
 * 每个 step() 都发布一次 `remote.input`：仲裁按自己记下的收到时刻判断这个来源还活着没有。
 * 失控保护帧或超过 `frame_timeout_us` 没有正常帧时 `connected` 为假，摇杆与拨杆保持上一份正常值
 * （失控帧里的值是接收机预设的，不可信）。
 *
 * 上电后按固定周期（如 5 ms）调用 step()。能力零 RTOS，UART 句柄由组合根在构造时注入。
 */
class Remote final
{
  public:
    /** 一根摇杆的标定：中位与两端的原始值、方向、归一化之后的死区。 */
    struct Stick
    {
        std::uint8_t channel{NO_CHANNEL}; // 0–15
        std::uint16_t minimum_raw{0U};
        std::uint16_t center_raw{0U};
        std::uint16_t maximum_raw{0U};
        bool inverted{false}; // 反装、或想让摇杆方向与默认相反
        float deadband{0.0F}; // 归一化之后的死区，0 表示不过滤
    };

    /**
     * @brief 一个拨杆的标定：低于 lower_raw 是 low_position，高于 upper_raw 是 high_position，
     *        两者之间是 MIDDLE。两档开关把两个门限填成同一个值。
     */
    struct Switch
    {
        std::uint8_t channel{NO_CHANNEL};
        std::uint16_t lower_raw{0U};
        std::uint16_t upper_raw{0U};
        topics::SwitchPosition low_position{topics::SwitchPosition::NONE};
        topics::SwitchPosition high_position{topics::SwitchPosition::NONE};
    };

    /** 来自 profile 的参数（params.hpp）。 */
    struct Parameters
    {
        std::array<Stick, topics::STICK_COUNT> sticks{};
        std::array<Switch, topics::SWITCH_COUNT> switches{};
        hal::os::Microseconds frame_timeout_us{100000U}; // 这么久没有正常帧就当链路断了
    };

    /** 统计，调试器可读（只有运行 step() 的任务写）。 */
    struct Report
    {
        std::uint32_t bytes;    // 读到的字节
        std::uint32_t frames;   // 解析出的帧
        std::uint32_t rejected; // 格式不对的帧
        std::uint32_t failsafe; // 接收机报告失控保护的帧
        std::array<float, topics::STICK_COUNT> sticks;
        std::array<topics::SwitchPosition, topics::SWITCH_COUNT> switches;
        bool connected; // 链路活着：最近一帧正常、没有失控保护
        bool fresh;     // 最近一帧在超时以内
        bool port_ready;
    };

    constexpr explicit Remote(hal::Uart port) noexcept : port_{port} {}

    /** 检查参数并取得 remote.input 的发布句柄。 */
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
    topic::Publisher<topics::OperatorInput> input_publisher_{};
    Report report_{};
    hal::os::Microseconds last_frame_us_{0U};
    hal::os::Microseconds last_normal_frame_us_{0U};
    bool has_frame_{false};
    bool failsafe_{false};
};

} // namespace remote

#endif
