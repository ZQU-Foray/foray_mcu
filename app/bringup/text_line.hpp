#ifndef BRINGUP_TEXT_LINE_HPP
#define BRINGUP_TEXT_LINE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace bringup
{

/**
 * @brief 定长的文本行：拼接字符串、整数与定点小数，不用 printf（没有堆，也不链接浮点格式化）。
 *        写满后多余的内容丢弃，overflowed() 为 true。
 */
class TextLine final
{
  public:
    static constexpr std::size_t CAPACITY{192U};

    void clear() noexcept;
    void append(const char *text) noexcept;
    void append_unsigned(std::uint32_t value) noexcept;
    void append_signed(std::int32_t value) noexcept;
    /** 按 decimals 位小数四舍五入输出，如 append_fixed(-1.2345F, 3) 得 "-1.235"；超出 ±2^31 的输出 "inf"。 */
    void append_fixed(float value, std::uint8_t decimals) noexcept;

    [[nodiscard]] const std::uint8_t *data() const noexcept
    {
        return buffer_.data();
    }
    [[nodiscard]] std::size_t size() const noexcept
    {
        return size_;
    }
    [[nodiscard]] bool overflowed() const noexcept
    {
        return overflowed_;
    }

  private:
    void put(char character) noexcept;

    std::array<std::uint8_t, CAPACITY> buffer_{};
    std::size_t size_{0U};
    bool overflowed_{false};
};

} // namespace bringup

#endif // BRINGUP_TEXT_LINE_HPP
