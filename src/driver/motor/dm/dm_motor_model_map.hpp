#ifndef DRIVER_MOTOR_DM_DM_MOTOR_MODEL_MAP_HPP
#define DRIVER_MOTOR_DM_DM_MOTOR_MODEL_MAP_HPP

#include "driver/motor/dm/dm_motor_adapter.hpp"
#include "driver/motor/motor_group.hpp"

namespace device
{
namespace dm
{

// 共用装配：型号档案注入适配器配置；MotorConnection 的 device_id 是 ESC_ID，master_id 是 MST_ID。
template <std::size_t N>
[[nodiscard]] DmMotorAdapter::Config
make_config(const DmMotor::Profile &profile, hal::Can bus, const std::array<MotorConnection, N> &connections) noexcept
{
    static_assert(N > 0U && N <= DmMotorAdapter::MAXIMUM_MOTORS, "电机数量超出达妙适配器能力");
    DmMotorAdapter::Config config{bus, profile, {}, N};
    for (std::size_t index = 0U; index < N; ++index)
    {
        config.mappings[index] = {
            connections[index].device_id, connections[index].master_id, connections[index].direction};
    }
    return config;
}

} // namespace dm

// 达妙型号映射：任务包含本头即选定达妙品牌（Decision-20261102-01）。
// 电机里的控制模式必须是 MIT、量程必须与档案一致，否则核对不通过、不能使能。
template <>
struct MotorAdapterFor<MotorModel::DM_J4310>
{
    using Adapter = DmMotorAdapter;
    static constexpr std::size_t MAXIMUM_MOTORS{DmMotor::MAXIMUM_MOTORS};
    template <std::size_t N>
    [[nodiscard]] static DmMotorAdapter::Config make_config(hal::Can bus,
                                                            const std::array<MotorConnection, N> &connections) noexcept
    {
        return dm::make_config(DmMotor::Profile::j4310(), bus, connections);
    }
};

} // namespace device
#endif // DRIVER_MOTOR_DM_DM_MOTOR_MODEL_MAP_HPP
