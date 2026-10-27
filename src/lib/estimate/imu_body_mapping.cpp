#include "lib/estimate/imu_body_mapping.hpp"
#include "utils/math/basic_math.hpp"

namespace estimate
{

float gyro_raw_to_radians_per_second(std::int16_t raw) noexcept
{
    return device::Bmi088Gyro::to_dps(raw) * math::DEG_TO_RAD;
}

void map_sensor_vector_to_body(const AxisMapping &mapping, const float (&sensor)[3], float (&body)[3]) noexcept
{
    // 先复制，允许 body 与 sensor 为同一数组。
    const float source[3]{sensor[0], sensor[1], sensor[2]};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        body[axis] = static_cast<float>(mapping.sign[axis]) * source[mapping.source[axis]];
    }
}

void gyro_record_to_body(const device::Bmi088SampleRecord &record,
                         float (&body)[3],
                         const AxisMapping &mapping) noexcept
{
    const float sensor[3]{gyro_raw_to_radians_per_second(record.xyz[0]),
                          gyro_raw_to_radians_per_second(record.xyz[1]),
                          gyro_raw_to_radians_per_second(record.xyz[2])};
    map_sensor_vector_to_body(mapping, sensor, body);
}

void accel_record_to_body(const device::Bmi088SampleRecord &record,
                          float (&body)[3],
                          const AxisMapping &mapping) noexcept
{
    const float sensor[3]{device::Bmi088Accel::to_mps2(record.xyz[0]),
                          device::Bmi088Accel::to_mps2(record.xyz[1]),
                          device::Bmi088Accel::to_mps2(record.xyz[2])};
    map_sensor_vector_to_body(mapping, sensor, body);
}

} // namespace estimate
