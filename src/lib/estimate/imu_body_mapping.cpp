#include "lib/estimate/imu_body_mapping.hpp"

namespace estimate
{

void map_sensor_vector_to_body(const AxisMapping &mapping, const float (&sensor)[3], float (&body)[3]) noexcept
{
    // 先复制，允许 body 与 sensor 为同一数组。
    const float source[3]{sensor[0], sensor[1], sensor[2]};
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        body[axis] = static_cast<float>(mapping.sign[axis]) * source[mapping.source[axis]];
    }
}

} // namespace estimate
