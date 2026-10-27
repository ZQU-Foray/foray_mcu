#ifndef LIB_ALG_ESTIMATE_IMU_BODY_MAPPING_HPP
#define LIB_ALG_ESTIMATE_IMU_BODY_MAPPING_HPP

#include "driver/bmi088/bmi088_accel.hpp"
#include "driver/bmi088/bmi088_gyro.hpp"
#include "driver/bmi088/bmi088_sample_queue.hpp"
#include <array>
#include <cstdint>

namespace alg_estimate
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

inline constexpr AxisMapping BMI088_SENSOR_TO_BODY{{0U, 1U, 2U}, {1, 1, 1}};
static_assert(is_valid_axis_mapping(BMI088_SENSOR_TO_BODY), "BMI088 安装映射必须是有符号轴置换且行列式为 +1");

static_assert(device::Bmi088Gyro::RANGE_REGISTER_VALUE == device::Bmi088Gyro::GYRO_RANGE_2000_DPS,
              "BMI088 陀螺量程不再是 ±2000 °/s，陀螺换算与 EKF 标度假设需要同步复核");
static_assert(device::Bmi088Accel::RANGE_REGISTER_VALUE == device::Bmi088Accel::ACCEL_RANGE_3G,
              "BMI088 加速度计量程不再是 ±3 g，加速度换算与 EKF 标度假设需要同步复核");

/**
 * @brief 陀螺原始计数换算为 SI 角速度。
 * @param raw 陀螺仪原始计数值。
 * @return 角速度，单位 rad/s。
 * @note 实现为“设备层 °/s 换算后再乘 DEG_TO_RAD”，保证与本工程既有的 to_dps 结果
 *       逐位一致，不引入第二套标度常量。
 */
[[nodiscard]] float gyro_raw_to_radians_per_second(std::int16_t raw) noexcept;

/**
 * @brief 按给定映射把传感器坐标系三轴向量搬到机体坐标系。
 * @param mapping 固定轴映射。
 * @param sensor 传感器坐标系输入，顺序为 x、y、z。
 * @param body 机体坐标系输出，顺序为 x、y、z。
 * @note 允许 body 与 sensor 为同一数组，内部先复制输入再写出结果。
 */
void map_sensor_vector_to_body(const AxisMapping &mapping, const float (&sensor)[3], float (&body)[3]) noexcept;

/**
 * @brief 陀螺样本换算并映射为机体系角速度。
 * @param record 驱动收割到的原始样本记录，只读。
 * @param body 机体坐标系输出，单位 rad/s，顺序为 x、y、z。
 * @param mapping 固定轴映射。
 */
void gyro_record_to_body(const device::Bmi088SampleRecord &record,
                         float (&body)[3],
                         const AxisMapping &mapping = BMI088_SENSOR_TO_BODY) noexcept;

/**
 * @brief 加速度计样本换算并映射为机体系加速度。
 * @param record 驱动收割到的原始样本记录，只读。
 * @param body 机体坐标系输出，单位 m/s^2，顺序为 x、y、z。
 * @param mapping 固定轴映射。
 */
void accel_record_to_body(const device::Bmi088SampleRecord &record,
                          float (&body)[3],
                          const AxisMapping &mapping = BMI088_SENSOR_TO_BODY) noexcept;

} // namespace alg_estimate

#endif
