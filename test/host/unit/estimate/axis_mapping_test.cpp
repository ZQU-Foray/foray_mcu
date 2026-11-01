// 传感器 → 机体的轴映射（estimate::AxisMapping）：只接受行列式为 +1 的有符号轴置换，映射可以原地进行。
#include "lib/estimate/imu_body_mapping.hpp"

#include <cassert>

int main()
{
    using estimate::AxisMapping;
    using estimate::is_valid_axis_mapping;

    // 合法：恒等、绕 z 转 90°（x ← y、y ← −x）、绕 x 转 180°
    static_assert(is_valid_axis_mapping(estimate::IDENTITY_AXIS_MAPPING));
    constexpr AxisMapping YAW_90{{1U, 0U, 2U}, {1, -1, 1}};
    static_assert(is_valid_axis_mapping(YAW_90));
    static_assert(is_valid_axis_mapping(AxisMapping{{0U, 1U, 2U}, {1, -1, -1}}));

    // 不合法：镜像（行列式 −1）、重复的源轴、源轴越界、符号不是 ±1
    static_assert(!is_valid_axis_mapping(AxisMapping{{0U, 1U, 2U}, {-1, 1, 1}}));
    static_assert(!is_valid_axis_mapping(AxisMapping{{1U, 0U, 2U}, {1, 1, 1}}));
    static_assert(!is_valid_axis_mapping(AxisMapping{{0U, 0U, 2U}, {1, 1, 1}}));
    static_assert(!is_valid_axis_mapping(AxisMapping{{0U, 1U, 3U}, {1, 1, 1}}));
    static_assert(!is_valid_axis_mapping(AxisMapping{{0U, 1U, 2U}, {1, 0, 1}}));

    const float sensor[3]{1.0F, 2.0F, 3.0F};
    float body[3]{};
    estimate::map_sensor_vector_to_body(YAW_90, sensor, body);
    assert(body[0] == 2.0F && body[1] == -1.0F && body[2] == 3.0F);

    // 原地映射：输入与输出是同一个数组
    float in_place[3]{1.0F, 2.0F, 3.0F};
    estimate::map_sensor_vector_to_body(YAW_90, in_place, in_place);
    assert(in_place[0] == 2.0F && in_place[1] == -1.0F && in_place[2] == 3.0F);
    return 0;
}
