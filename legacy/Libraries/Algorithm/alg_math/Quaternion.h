#ifndef LIBRARIES_ALGORITHM_ALG_MATH_QUATERNION_H
#define LIBRARIES_ALGORITHM_ALG_MATH_QUATERNION_H

#include "Libraries/Algorithm/alg_math/Vector3.h"

namespace alg_math
{

/**
 * @brief 四元数，标量在前，采用 Hamilton 乘法约定。
 * @note 旋转约定：q 表示把 A 系分量旋转到 B 系分量，即 v_B = q ⊗ v_A ⊗ q*。
 *       姿态估计中 A 取机体系、B 取世界系，因此 Rotate 得到世界系分量，
 *       RotateInverse 得到机体系分量。
 * @note 本类型不强制单位化：只有 Normalize 会改变模长，其余函数按输入模长直接计算。
 *       参与旋转的 q 应当已经单位化。
 */
struct Quaternion final
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/** @brief 四元数模长。 */
[[nodiscard]] float Norm(const Quaternion &value) noexcept;

/**
 * @brief 就地单位化。
 * @param value 输入输出四元数。
 * @return 各分量有限且模长大于零时返回 true；否则置为单位四元数并返回 false。
 */
[[nodiscard]] bool Normalize(Quaternion &value) noexcept;

/** @brief 共轭四元数，单位四元数下等价于逆旋转。 */
[[nodiscard]] Quaternion Conjugate(const Quaternion &value) noexcept;

/** @brief Hamilton 乘积，表示先作用 second 再作用 first 的复合旋转。 */
[[nodiscard]] Quaternion Multiply(const Quaternion &first, const Quaternion &second) noexcept;

/** @brief 把 A 系向量旋转到 B 系：v_B = q ⊗ v_A ⊗ q*。 */
[[nodiscard]] Vector3 Rotate(const Quaternion &rotation, const Vector3 &value) noexcept;

/** @brief 把 B 系向量旋转回 A 系：v_A = q* ⊗ v_B ⊗ q。 */
[[nodiscard]] Vector3 RotateInverse(const Quaternion &rotation, const Vector3 &value) noexcept;

/**
 * @brief 提取 ZYX 欧拉角。
 * @param value 单位四元数。
 * @return {yaw, pitch, roll}，单位 rad；旋转矩阵约定为 R = Rz(yaw)·Ry(pitch)·Rx(roll)。
 * @note yaw 与 roll 由 atan2 给出，pitch 由反正弦给出因而取值范围是 [-π/2, π/2]；
 *       pitch 在 ±π/2 附近时 roll 与 yaw 不再独立，此时函数仍连续返回有限值：
 *       反正弦参数被钳位到 [-1, 1]。
 * @note 注意与部分第三方实现的命名差异：本函数严格遵守“pitch 为绕 y 轴、roll 为绕 x 轴”
 *       的 ZYX 定义，而某些工程把两者的提取公式与标签互换。使用四元数而非欧拉角
 *       与外部对接时不受该差异影响。
 */
[[nodiscard]] Vector3 ToEulerZyxRadians(const Quaternion &value) noexcept;

/**
 * @brief 由 ZYX 欧拉角构造四元数。
 * @param euler {yaw, pitch, roll}，单位 rad。
 * @return 对应的单位四元数。
 */
[[nodiscard]] Quaternion FromEulerZyxRadians(const Vector3 &euler) noexcept;

/** @brief 四个分量都是有限值时返回 true。 */
[[nodiscard]] bool IsFinite(const Quaternion &value) noexcept;

} // namespace alg_math

#endif
