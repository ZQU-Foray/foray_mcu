#ifndef LIBRARIES_DEVICE_MOTOR_MOTOR_HPP
#define LIBRARIES_DEVICE_MOTOR_MOTOR_HPP

#include "Platform/Interface/Time.hpp"
#include <cstddef>
#include <cstdint>

namespace device
{

// 物理量均指配置的输出轴与正方向。
// 位置以首次反馈归零；离线期间的运动不做推断。
struct MotorState final
{
  float angleDegrees{0.0F};
  float speedRpm{0.0F};
  bool feedbackValid{false};
  bool online{false};
  platform::Time::Tick lastUpdateTick{0U};
};

// 型号选择只用于装配；控制器不依赖本枚举。
enum class MotorModel : std::uint8_t
{
  M2006,
  M3508,
  GM6020Current
};
struct MotorConnection final
{
  std::uint8_t deviceId{0U};
  int direction{1};
};

} // namespace device
#endif
