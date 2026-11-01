#include "bringup/text_line.hpp"

namespace bringup
{

void TextLine::clear() noexcept
{
    size_ = 0U;
    overflowed_ = false;
}

void TextLine::put(char character) noexcept
{
    if (size_ < buffer_.size())
    {
        buffer_[size_++] = static_cast<std::uint8_t>(character);
    }
    else
    {
        overflowed_ = true;
    }
}

void TextLine::append(const char *text) noexcept
{
    for (; text != nullptr && *text != '\0'; ++text)
    {
        put(*text);
    }
}

void TextLine::append_unsigned(std::uint32_t value) noexcept
{
    std::array<char, 10> digits{};
    std::size_t count = 0U;
    do
    {
        digits[count++] = static_cast<char>('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);
    while (count > 0U)
    {
        put(digits[--count]);
    }
}

void TextLine::append_signed(std::int32_t value) noexcept
{
    if (value < 0)
    {
        put('-');
        // 先转无符号再取负：INT32_MIN 也正确
        append_unsigned(0U - static_cast<std::uint32_t>(value));
        return;
    }
    append_unsigned(static_cast<std::uint32_t>(value));
}

void TextLine::append_fixed(float value, std::uint8_t decimals) noexcept
{
    std::uint32_t scale = 1U;
    for (std::uint8_t index = 0U; index < decimals; ++index)
    {
        scale *= 10U;
    }
    const float scaled = value * static_cast<float>(scale);
    // NaN 的比较都为假，也走这里
    if (!(scaled > -2147483520.0F && scaled < 2147483520.0F))
    {
        append("inf");
        return;
    }
    const bool negative = scaled < 0.0F;
    const auto magnitude = static_cast<std::uint32_t>((negative ? -scaled : scaled) + 0.5F);
    const std::uint32_t whole = magnitude / scale;
    const std::uint32_t fraction = magnitude % scale;
    if (negative && magnitude != 0U)
    {
        put('-');
    }
    append_unsigned(whole);
    if (decimals == 0U)
    {
        return;
    }
    put('.');
    for (std::uint32_t divisor = scale / 10U; divisor > 0U; divisor /= 10U)
    {
        put(static_cast<char>('0' + (fraction / divisor) % 10U));
    }
}

} // namespace bringup
