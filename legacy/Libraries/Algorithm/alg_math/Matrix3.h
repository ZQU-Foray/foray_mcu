#ifndef LIBRARIES_ALGORITHM_ALG_MATH_MATRIX3_H
#define LIBRARIES_ALGORITHM_ALG_MATH_MATRIX3_H

namespace alg_math
{

/**
 * @brief 3×3 双精度矩阵，行主序扁平存储。
 *
 * @note 为什么与 `Matrix<Rows, Columns>` 并存：`Matrix<>` 是 float、逐元素经 `At()`
 *       访问的通用模板，适合"可读性优先、调用频次低"的场合（例如 EKF 的协方差）；
 *       本类型面向**每个样本都要跑**的热路径——扁平数组直接索引、双精度、无模板
 *       实例化，在 `-O0` 下按元素访问不产生函数调用。两者都是纯值语义、无动态分配。
 * @note 采用公开的 `value[9]` 而不是访问器，是为了让 `-O0` 下的元素访问成本与裸数组
 *       完全相同；与本工程 `Vector3` / `Quaternion` 的公开成员风格一致。
 * @note 存储约定：`value[3*row + column]`。
 */
struct Matrix3 final
{
  double value[9]{};
};

/** @brief 对角为 scale 的对角矩阵。 */
[[nodiscard]] Matrix3 ScaledIdentity(double scale) noexcept;

/** @brief 矩阵乘：first · second。 */
[[nodiscard]] Matrix3 Multiply(const Matrix3 &first, const Matrix3 &second) noexcept;

/** @brief first 转置后乘 second：firstᵀ · second。 */
[[nodiscard]] Matrix3 MultiplyTransposedFirst(const Matrix3 &first, const Matrix3 &second) noexcept;

/** @brief first 乘 second 转置：first · secondᵀ。 */
[[nodiscard]] Matrix3 MultiplyTransposedSecond(const Matrix3 &first, const Matrix3 &second) noexcept;

/**
 * @brief 求逆（伴随矩阵 / 行列式）。
 * @param input 待求逆矩阵。
 * @param output 逆矩阵输出。
 * @return 行列式绝对值大于 float 机器精度时写入结果并返回 true；否则输出清零并返回 false。
 * @note 奇异判据与上游参考实现一致（用 float 的 epsilon 而不是 double 的），以便跨实现对照。
 */
[[nodiscard]] bool Inverted(const Matrix3 &input, Matrix3 &output) noexcept;

/**
 * @brief 逐元素限幅（就地）。
 * @param value 输入输出矩阵。
 * @param minimum 下限。
 * @param maximum 上限。
 */
void LimitInPlace(Matrix3 &value, double minimum, double maximum) noexcept;

} // namespace alg_math

#endif // LIBRARIES_ALGORITHM_ALG_MATH_MATRIX3_H
