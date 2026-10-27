#ifndef DRIVER_MOTOR_DJI_DJI_MOTOR_MODEL_MAP_HPP
#define DRIVER_MOTOR_DJI_DJI_MOTOR_MODEL_MAP_HPP

#include "driver/motor/motor_group.hpp"
#include "driver/motor/dji/dji_motor_adapter.hpp"

namespace device
{
namespace dji
{

// 共用装配：品牌默认参数（电调档案与型号原生减速比）注入适配器配置。
template <std::size_t N>
[[nodiscard]] DjiMotorAdapter::Config make_config(const DjiMotor::Profile &profile,
                                                  float gear_ratio,
                                                  platform::Can::Channel channel,
                                                  const std::array<MotorConnection, N> &connections) noexcept
{
    static_assert(N > 0U && N <= DjiMotorAdapter::MAXIMUM_MOTORS, "电机数量超出 DJI 适配器能力");
    DjiMotorAdapter::Config config{channel, profile, {}, N};
    for (std::size_t index = 0U; index < N; ++index)
    {
        config.mappings[index] = {connections[index].device_id, gear_ratio, connections[index].direction};
    }
    return config;
}

} // namespace dji

// DJI 型号映射：任务包含本头即选定 DJI 品牌；更换品牌只需包含对应品牌的
// 映射头并改用其型号。默认减速比为型号原生减速箱参数。
template <> struct MotorAdapterFor<MotorModel::M2006>
{
    using Adapter = DjiMotorAdapter;
    static constexpr std::size_t MAXIMUM_MOTORS{DjiMotor::Profile::c610().dialect.maximum_device_count};
    template <std::size_t N>
    [[nodiscard]] static DjiMotorAdapter::Config make_config(platform::Can::Channel channel,
                                                             const std::array<MotorConnection, N> &connections) noexcept
    {
        return dji::make_config(DjiMotor::Profile::c610(), 36.0F, channel, connections);
    }
};

template <> struct MotorAdapterFor<MotorModel::M3508>
{
    using Adapter = DjiMotorAdapter;
    static constexpr std::size_t MAXIMUM_MOTORS{DjiMotor::Profile::c620().dialect.maximum_device_count};
    template <std::size_t N>
    [[nodiscard]] static DjiMotorAdapter::Config make_config(platform::Can::Channel channel,
                                                             const std::array<MotorConnection, N> &connections) noexcept
    {
        return dji::make_config(DjiMotor::Profile::c620(), 19.2F, channel, connections);
    }
};

template <> struct MotorAdapterFor<MotorModel::GM6020_CURRENT>
{
    using Adapter = DjiMotorAdapter;
    static constexpr std::size_t MAXIMUM_MOTORS{DjiMotor::Profile::gm6020_current().dialect.maximum_device_count};
    template <std::size_t N>
    [[nodiscard]] static DjiMotorAdapter::Config make_config(platform::Can::Channel channel,
                                                             const std::array<MotorConnection, N> &connections) noexcept
    {
        return dji::make_config(DjiMotor::Profile::gm6020_current(), 1.0F, channel, connections);
    }
};

} // namespace device
#endif
