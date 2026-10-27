#ifndef LIB_ALG_MATH_MATRIX_HPP
#define LIB_ALG_MATH_MATRIX_HPP

#include <array>
#include <cmath>
#include <cstddef>

namespace alg_math
{

/**
 * @brief 固定尺寸行优先浮点矩阵。
 * @tparam Rows 行数，必须大于零。
 * @tparam Columns 列数，必须大于零。
 * @note 容量在编译期确定，不使用动态分配，也不做边界检查：越界由调用方保证。
 * @note 所有运算都是纯函数（返回新矩阵），因此不存在输入输出重叠的问题；
 *       需要就地修改时显式赋值。序列化时按行优先展平访问 Data()。
 */
template <std::size_t Rows, std::size_t Columns> struct Matrix final
{
    static_assert(Rows > 0U, "矩阵行数必须大于零");
    static_assert(Columns > 0U, "矩阵列数必须大于零");

    using Value = float;

    std::array<float, Rows * Columns> values{};

    /** @brief 访问元素，行列均从零开始。 */
    [[nodiscard]] float &at(std::size_t row, std::size_t column) noexcept
    {
        return values[row * Columns + column];
    }

    [[nodiscard]] float at(std::size_t row, std::size_t column) const noexcept
    {
        return values[row * Columns + column];
    }

    /** @brief 行优先数据首地址，供需要连续内存的场合使用。 */
    [[nodiscard]] float *data() noexcept
    {
        return values.data();
    }
    [[nodiscard]] const float *data() const noexcept
    {
        return values.data();
    }

    /** @brief 全零矩阵。 */
    [[nodiscard]] static Matrix zero() noexcept
    {
        return Matrix{};
    }

    /** @brief 单位矩阵，仅对正方形矩阵可用。 */
    [[nodiscard]] static Matrix identity() noexcept
    {
        static_assert(Rows == Columns, "单位矩阵只对正方形矩阵有意义");
        Matrix result{};
        for (std::size_t index = 0U; index < Rows; ++index)
        {
            result.at(index, index) = 1.0f;
        }
        return result;
    }

    /** @brief 矩阵乘法，内维由模板参数推导。 */
    template <std::size_t Inner>
    [[nodiscard]] static Matrix<Rows, Columns> multiply(const Matrix<Rows, Inner> &first,
                                                        const Matrix<Inner, Columns> &second) noexcept
    {
        Matrix<Rows, Columns> result{};
        for (std::size_t row = 0U; row < Rows; ++row)
        {
            for (std::size_t column = 0U; column < Columns; ++column)
            {
                float sum = 0.0f;
                for (std::size_t index = 0U; index < Inner; ++index)
                {
                    sum += first.at(row, index) * second.at(index, column);
                }
                result.at(row, column) = sum;
            }
        }
        return result;
    }

    /** @brief 转置矩阵。 */
    [[nodiscard]] Matrix<Columns, Rows> transposed() const noexcept
    {
        Matrix<Columns, Rows> result{};
        for (std::size_t row = 0U; row < Rows; ++row)
        {
            for (std::size_t column = 0U; column < Columns; ++column)
            {
                result.at(column, row) = at(row, column);
            }
        }
        return result;
    }

    /** @brief 逐元素相加。 */
    [[nodiscard]] Matrix added(const Matrix &other) const noexcept
    {
        Matrix result{};
        for (std::size_t index = 0U; index < values.size(); ++index)
        {
            result.values[index] = values[index] + other.values[index];
        }
        return result;
    }

    /** @brief 逐元素相减。 */
    [[nodiscard]] Matrix subtracted(const Matrix &other) const noexcept
    {
        Matrix result{};
        for (std::size_t index = 0U; index < values.size(); ++index)
        {
            result.values[index] = values[index] - other.values[index];
        }
        return result;
    }

    /** @brief 逐元素乘以标量。 */
    [[nodiscard]] Matrix scaled(float factor) const noexcept
    {
        Matrix result{};
        for (std::size_t index = 0U; index < values.size(); ++index)
        {
            result.values[index] = values[index] * factor;
        }
        return result;
    }

    /** @brief 对称化 (M + M^T) / 2，仅对正方形矩阵可用；用于协方差矩阵的数值修复。 */
    [[nodiscard]] Matrix symmetrized() const noexcept
    {
        static_assert(Rows == Columns, "对称化只对正方形矩阵有意义");
        Matrix result{};
        for (std::size_t row = 0U; row < Rows; ++row)
        {
            for (std::size_t column = 0U; column < Columns; ++column)
            {
                result.at(row, column) = 0.5f * (at(row, column) + at(column, row));
            }
        }
        return result;
    }

    /**
     * @brief 求逆，使用列主元高斯-约当消元。
     * @param input 待求逆矩阵，必须可逆。
     * @param output 求逆结果；失败时被清零。
     * @return 主元全部非零且结果有限时返回 true。
     * @note 单精度下"接近奇异"的矩阵可能仍然返回成功，调用方需要自行判断结果是否可用。
     */
    [[nodiscard]] static bool inverted(const Matrix &input, Matrix &output) noexcept
    {
        static_assert(Rows == Columns, "求逆只对正方形矩阵有意义");
        Matrix work = input;
        output = Matrix::identity();
        for (std::size_t column = 0U; column < Rows; ++column)
        {
            std::size_t pivot = column;
            float pivot_magnitude = std::fabs(work.at(column, column));
            for (std::size_t row = column + 1U; row < Rows; ++row)
            {
                const float candidate = std::fabs(work.at(row, column));
                if (candidate > pivot_magnitude)
                {
                    pivot_magnitude = candidate;
                    pivot = row;
                }
            }
            if (!std::isfinite(pivot_magnitude) || !(pivot_magnitude > 0.0f))
            {
                output = Matrix{};
                return false;
            }
            if (pivot != column)
            {
                for (std::size_t index = 0U; index < Rows; ++index)
                {
                    const float temporary = work.at(column, index);
                    work.at(column, index) = work.at(pivot, index);
                    work.at(pivot, index) = temporary;
                    const float output_temporary = output.at(column, index);
                    output.at(column, index) = output.at(pivot, index);
                    output.at(pivot, index) = output_temporary;
                }
            }
            const float inverse_pivot = 1.0f / work.at(column, column);
            for (std::size_t index = 0U; index < Rows; ++index)
            {
                work.at(column, index) *= inverse_pivot;
                output.at(column, index) *= inverse_pivot;
            }
            for (std::size_t row = 0U; row < Rows; ++row)
            {
                if (row == column)
                {
                    continue;
                }
                const float factor = work.at(row, column);
                if (factor == 0.0f)
                {
                    continue;
                }
                for (std::size_t index = 0U; index < Rows; ++index)
                {
                    work.at(row, index) -= factor * work.at(column, index);
                    output.at(row, index) -= factor * output.at(column, index);
                }
            }
        }
        return output.is_finite();
    }

    /** @brief 所有元素都是有限值时返回 true。 */
    [[nodiscard]] bool is_finite() const noexcept
    {
        for (const float value : values)
        {
            if (!std::isfinite(value))
            {
                return false;
            }
        }
        return true;
    }

    /** @brief 与另一矩阵逐元素最大绝对差，用于测试与数值比较。 */
    [[nodiscard]] float max_abs_difference(const Matrix &other) const noexcept
    {
        float maximum = 0.0f;
        for (std::size_t index = 0U; index < values.size(); ++index)
        {
            const float difference = std::fabs(values[index] - other.values[index]);
            if (difference > maximum)
            {
                maximum = difference;
            }
        }
        return maximum;
    }
};

} // namespace alg_math

#endif
