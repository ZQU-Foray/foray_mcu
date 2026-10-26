#include "Libraries/Algorithm/alg_estimate/ImuBodyMapping.hpp"
#include "Libraries/Algorithm/alg_math/BasicMath.h"

namespace alg_estimate
{

float GyroRawToRadiansPerSecond(std::int16_t raw) noexcept
{
    return device::Bmi088Gyro::ToDps(raw) * alg_math::degToRad;
}

void MapSensorVectorToBody(const AxisMapping &mapping, const float (&sensor)[3], float (&body)[3]) noexcept
{
    // 先复制，允许 body 与 sensor 为同一数组。
    const float source[3]{sensor[0], sensor[1], sensor[2]};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        body[axis] = static_cast<float>(mapping.sign[axis]) * source[mapping.source[axis]];
    }
}

void GyroRecordToBody(const device::Bmi088SampleRecord &record, float (&body)[3], const AxisMapping &mapping) noexcept
{
    const float sensor[3]{GyroRawToRadiansPerSecond(record.xyz[0]),
                          GyroRawToRadiansPerSecond(record.xyz[1]),
                          GyroRawToRadiansPerSecond(record.xyz[2])};
    MapSensorVectorToBody(mapping, sensor, body);
}

void AccelRecordToBody(const device::Bmi088SampleRecord &record, float (&body)[3], const AxisMapping &mapping) noexcept
{
    const float sensor[3]{device::Bmi088Accel::ToMps2(record.xyz[0]),
                          device::Bmi088Accel::ToMps2(record.xyz[1]),
                          device::Bmi088Accel::ToMps2(record.xyz[2])};
    MapSensorVectorToBody(mapping, sensor, body);
}

} // namespace alg_estimate
