#ifndef DRIVER_MOTOR_DJI_DJI_MODELS_HPP
#define DRIVER_MOTOR_DJI_DJI_MODELS_HPP

#include "driver/motor/dji/dji_motor.hpp"

// 大疆型号（Decision-20261102-03：型号做成数据，按品牌分文件）。减速比是转子到电机输出轴的原生减速箱，
// 外接传动写在关节的 JointMechanics 里。GM6020 必须在电调里设成电流模式，软件选型不会改电调。

namespace device::dji
{

/** M2006 + C610：36:1，Kt 0.18 N·m/A（输出轴），电流满量程 10 A，无温度反馈，位置从第一帧起算。 */
inline constexpr DjiMotor::Model M2006{DjiMotor::Profile::c610(), 36.0F, false};

/** M3508 + C620：19.2:1，Kt 0.3 N·m/A（输出轴），电流满量程 20 A，位置从第一帧起算。 */
inline constexpr DjiMotor::Model M3508{DjiMotor::Profile::c620(), 19.2F, false};

/** GM6020 电流模式：直驱，Kt 0.741 N·m/A，电流满量程 3 A，位置上电即绝对。 */
inline constexpr DjiMotor::Model GM6020_CURRENT{DjiMotor::Profile::gm6020_current(), 1.0F, true};

} // namespace device::dji

#endif // DRIVER_MOTOR_DJI_DJI_MODELS_HPP
