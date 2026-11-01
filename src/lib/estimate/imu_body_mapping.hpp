#ifndef LIB_ESTIMATE_IMU_BODY_MAPPING_HPP
#define LIB_ESTIMATE_IMU_BODY_MAPPING_HPP

#include <array>
#include <cstdint>

// 传感器坐标系 → 机体坐标系的固定轴映射：有符号的轴置换，行列式为 +1（不含镜像）。
// 只处理三维向量，与具体传感器无关：IMU 能力把驱动的原始计数换算成 SI 单位后再交给它（F3 去掉了对
// BMI088 驱动类型的依赖，lib → driver 的反向依赖消除，Decision-20261101-02）。

namespace estimate
{

struct AxisMapping
{
    std::array<std::uint8_t, 3> source{};
    std::array<std::int8_t, 3> sign{};
};

[[nodiscard]] constexpr bool is_valid_axis_mapping(const AxisMapping &mapping) noexcept
{
    std::int32_t inversions = 0;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (mapping.source[axis] > 2U)
        {
            return false;
        }
        if (mapping.sign[axis] != 1 && mapping.sign[axis] != -1)
        {
            return false;
        }
        for (std::size_t other = axis + 1U; other < 3U; ++other)
        {
            if (mapping.source[axis] == mapping.source[other])
            {
                return false;
            }
            if (mapping.source[axis] > mapping.source[other])
            {
                ++inversions;
            }
        }
    }

    std::int32_t determinant = static_cast<std::int32_t>(mapping.sign[0]) * static_cast<std::int32_t>(mapping.sign[1]) *
                               static_cast<std::int32_t>(mapping.sign[2]);
    if ((inversions % 2) != 0)
    {
        determinant = -determinant;
    }
    return determinant == 1;
}

/** 不旋转：传感器坐标系与机体坐标系重合。实际安装方向是 profile 的参数。 */
inline constexpr AxisMapping IDENTITY_AXIS_MAPPING{{0U, 1U, 2U}, {1, 1, 1}};
static_assert(is_valid_axis_mapping(IDENTITY_AXIS_MAPPING));

/**
 * @brief 按给定映射把传感器坐标系三轴向量搬到机体坐标系。
 * @param mapping 固定轴映射。
 * @param sensor 传感器坐标系输入，顺序为 x、y、z。
 * @param body 机体坐标系输出，顺序为 x、y、z。
 * @note 允许 body 与 sensor 为同一数组，内部先复制输入再写出结果。
 */
void map_sensor_vector_to_body(const AxisMapping &mapping, const float (&sensor)[3], float (&body)[3]) noexcept;

} // namespace estimate

#endif
