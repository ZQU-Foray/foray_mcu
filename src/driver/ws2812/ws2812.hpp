#ifndef LIBRARIES_DEVICE_WS2812_WS2812_HPP
#define LIBRARIES_DEVICE_WS2812_WS2812_HPP

#include "Platform/Interface/Spi.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

struct RgbColor final
{
    std::uint8_t red{0U};
    std::uint8_t green{0U};
    std::uint8_t blue{0U};
};

[[nodiscard]] constexpr bool operator==(RgbColor lhs, RgbColor rhs) noexcept
{
    return lhs.red == rhs.red && lhs.green == rhs.green && lhs.blue == rhs.blue;
}

[[nodiscard]] constexpr bool operator!=(RgbColor lhs, RgbColor rhs) noexcept
{
    return !(lhs == rhs);
}

namespace ws2812_color
{

inline constexpr RgbColor BLACK{0U, 0U, 0U};
inline constexpr RgbColor WHITE{255U, 255U, 255U};
inline constexpr RgbColor RED{255U, 0U, 0U};
inline constexpr RgbColor GREEN{0U, 255U, 0U};
inline constexpr RgbColor BLUE{0U, 0U, 255U};
inline constexpr RgbColor YELLOW{255U, 255U, 0U};
inline constexpr RgbColor CYAN{0U, 255U, 255U};
inline constexpr RgbColor MAGENTA{255U, 0U, 255U};

} // namespace ws2812_color

class Ws2812 final
{
  public:
    static constexpr std::size_t ENCODED_COLOR_SIZE{24U};
    static constexpr std::size_t RESET_SIZE{64U};
    static constexpr std::size_t FRAME_SIZE{ENCODED_COLOR_SIZE + RESET_SIZE};

    enum class State : std::uint8_t
    {
        IDLE,
        TRANSMITTING,
        BUSY,
        NOT_READY,
        ERROR
    };

    Ws2812() noexcept = default;

    /**
   * @brief 设置下一次刷新使用的 RGB 颜色。
   * @param color 红、绿、蓝三个八位颜色分量。
   * @note 本函数不启动传输，实际刷新由 Process 推进。
   */
    void set_color(RgbColor color) noexcept;

    /**
   * @brief 设置全局亮度缩放值。
   * @param brightness 亮度，0 表示全灭，255 表示不缩放。
   * @note 本函数不改变保存的原始颜色。
   */
    void set_brightness(std::uint8_t brightness) noexcept;

    /**
   * @brief 将目标颜色设置为黑色，由后续 Process 刷新灯珠。
   */
    void off() noexcept;

    /**
   * @brief 推进异步发送完成检查并在需要时启动下一次 SPI DMA 刷新。
   * @note 应由单一任务周期调用，不应在中断上下文调用。
   */
    void process() noexcept;

    /**
   * @brief 获取尚未进行亮度缩放的目标颜色。
   * @return 当前保存的 RGB 颜色。
   */
    [[nodiscard]] RgbColor get_color() const noexcept
    {
        return color_;
    }

    /**
   * @brief 获取当前全局亮度值。
   * @return 0 到 255 的亮度值。
   */
    [[nodiscard]] std::uint8_t get_brightness() const noexcept
    {
        return brightness_;
    }

    /**
   * @brief 获取最近一次 Process 更新后的驱动状态。
   * @return 空闲、发送中、忙、未就绪或错误状态。
   */
    [[nodiscard]] State get_state() const noexcept
    {
        return state_;
    }

  private:
    using Frame = std::array<std::uint8_t, FRAME_SIZE>;

    static constexpr std::uint8_t LEVEL_0{0x60U};
    static constexpr std::uint8_t LEVEL_1{0x78U};

    [[nodiscard]] static std::uint8_t scale(std::uint8_t value, std::uint8_t brightness) noexcept;
    static void encode_byte(std::uint8_t value, Frame &frame, std::size_t offset) noexcept;
    void build_frame() noexcept;
    void handle_completed_transfer() noexcept;
    void handle_start_result(platform::Spi::Result result) noexcept;

    RgbColor color_{};
    std::uint8_t brightness_{255U};
    State state_{State::IDLE};
    bool refresh_pending_{true};
    Frame frame_{};
};

static_assert(Ws2812::FRAME_SIZE <= platform::Spi::MAX_ASYNC_LENGTH,
              "WS2812 frame exceeds the SPI asynchronous transfer limit");

} // namespace device

#endif
