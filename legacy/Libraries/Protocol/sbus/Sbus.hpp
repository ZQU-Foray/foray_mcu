#ifndef LIBRARIES_PROTOCOL_SBUS_SBUS_HPP
#define LIBRARIES_PROTOCOL_SBUS_SBUS_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace protocol
{

class SbusParser final
{
  public:
    static constexpr std::size_t FRAME_SIZE{25U};
    static constexpr std::size_t CHANNEL_COUNT{16U};
    static constexpr std::uint8_t START_BYTE{0x0FU};

    struct Frame final
    {
        std::array<std::uint16_t, CHANNEL_COUNT> channels{};
        bool digital_channel_17{false};
        bool digital_channel_18{false};
        bool frame_lost{false};
        bool failsafe{false};
    };

    struct Statistics final
    {
        std::uint32_t accepted_frame_count{0U};
        std::uint32_t rejected_frame_count{0U};
        std::uint32_t discarded_byte_count{0U};
    };

    SbusParser() = default;

    void reset() noexcept;

    // 接受任意分段的连续字节流。一次调用可解析多帧，latestFrame 返回
    // 最后一帧，返回值为本次调用成功解析的帧数。
    [[nodiscard]] std::size_t input(const std::uint8_t *data, std::size_t length, Frame &latest_frame) noexcept;

    [[nodiscard]] const Statistics &get_statistics() const noexcept
    {
        return statistics_;
    }

    // 仅解析一帧，不修改流解析器状态，供协议测试和离线数据使用。
    [[nodiscard]] static bool decode_frame(const std::array<std::uint8_t, FRAME_SIZE> &raw_frame,
                                           Frame &frame) noexcept;

  private:
    [[nodiscard]] static bool is_supported_end_byte(std::uint8_t byte) noexcept;
    void recover_after_rejected_frame() noexcept;

    std::array<std::uint8_t, FRAME_SIZE> buffer_{};
    std::size_t buffered_byte_count_{0U};
    Statistics statistics_{};
};

} // namespace protocol

#endif
