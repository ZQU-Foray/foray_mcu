#ifndef APPLICATION_IMU_DIAGNOSTICS_HPP
#define APPLICATION_IMU_DIAGNOSTICS_HPP

#include "Libraries/Device/bmi088/Bmi088.hpp"

namespace application
{
// 仅 Debug 构建：被动观察任务属主已收割的样本。
// 不写传感器、不做滤波、零速率钳位或零偏补偿。
class ImuDiagnostics final
{
public:
  static void Observe(const device::Bmi088 &imu, const device::Bmi088SampleRecord &record, bool gyroscope) noexcept;
};
} // namespace application
#endif
