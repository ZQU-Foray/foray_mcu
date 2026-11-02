#ifndef DRIVER_MOTOR_DM_DM_MODELS_HPP
#define DRIVER_MOTOR_DM_DM_MODELS_HPP

#include "driver/motor/dm/dm_motor.hpp"

// 达妙型号（Decision-20261102-03：型号做成数据，按品牌分文件）。电机里的控制模式必须是 MIT、量程与通讯超时
// 必须与这里一致，否则核对不通过、关节不能使能；电机里的设置用达妙调试助手改。

namespace device::dm
{

/** DM-J4310-2EC V1.2：量程 12.5 rad、30 rad/s、10 N·m，峰值扭矩 12.5 N·m，通讯超时 50 ms。 */
inline constexpr DmMotor::Profile DM_J4310{DmMotor::Profile::j4310()};

} // namespace device::dm

#endif // DRIVER_MOTOR_DM_DM_MODELS_HPP
