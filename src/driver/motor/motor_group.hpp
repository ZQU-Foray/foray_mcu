#ifndef DRIVER_MOTOR_MOTOR_GROUP_HPP
#define DRIVER_MOTOR_MOTOR_GROUP_HPP
#include "driver/motor/motor.hpp"
#include "hal/can.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief 型号→适配器映射的编译期定制点：主模板仅声明。各品牌目录对所需
 *        型号提供特化后电机组才可实例化，未映射型号在编译期拒绝；电机组
 *        与控制器不含任何品牌依赖。
 * @note 特化契约（无虚函数、无运行时型号分派）：
 *       - using Adapter：适配器类型，可由 Config 直接构造，并提供
 *         init/process/enable/disable/read_state/set_torque/clear_commands/maximum_torque_newton_meter 操作契约；
 *       - static constexpr MAXIMUM_MOTORS：该型号允许的最大电机数量；
 *       - template <N> make_config(bus, connections)：把品牌默认参数
 *         （电调档案、型号原生减速比等）装配为 Adapter::Config。
 */
template <MotorModel Model>
struct MotorAdapterFor;

/**
 * @brief 一个任务持有的同型号电机组，直接持有编译期选定的品牌适配器。
 * @note Init 先于调度执行，成功后实例须保持存活至固件结束；所有收发和
 *       读写由同一个任务调用。每组独占一条 CAN 总线的接收：适配器在 Init 里用
 *       hal::Can::claim_receiver() 占住总线，同一总线上的第二个电机组（含同型号）
 *       Init 失败。输出为型号原生输出轴单位；外接传动不包含在型号档案内。
 * @note 出力要先 enable()（Decision-20261102-01）：init() 之后、disable() 之后都不出力，
 *       set_torque() 返回 false。达妙电机在 init() 之后还要回读核对，核对完成前 enable() 返回 false。
 */
template <MotorModel Model, std::size_t N>
class MotorGroup final
{
    static_assert(N > 0U, "电机组数量必须为正");
    using Mapping = MotorAdapterFor<Model>;
    using Adapter = typename Mapping::Adapter;
    static_assert(N <= Mapping::MAXIMUM_MOTORS, "电机组数量超过型号能力");

  public:
    MotorGroup(hal::Can bus, const std::array<MotorConnection, N> &connections) noexcept
        : adapter_{Mapping::template make_config<N>(bus, connections)}
    {
    }
    MotorGroup(const MotorGroup &) = delete;
    MotorGroup &operator=(const MotorGroup &) = delete;
    MotorGroup(MotorGroup &&) = delete;
    MotorGroup &operator=(MotorGroup &&) = delete;

    [[nodiscard]] bool init() noexcept
    {
        return adapter_.init();
    }
    [[nodiscard]] static constexpr std::size_t count() noexcept
    {
        return N;
    }
    // 沿用“收、老化、发送、发布”，暂存的新指令在下次 Process 发出。
    void process() noexcept
    {
        adapter_.process();
    }
    // 全组请求出力。任何一台还不能使能（未初始化、达妙还在核对或核对不符）就一台都不使能，返回 false。
    // 已经在出力的电机不受影响。电机出故障后撤销请求，要再调用一次才会重试（达妙会先清除错误）。
    [[nodiscard]] bool enable() noexcept
    {
        return adapter_.enable();
    }
    // 全组撤销出力请求并清零暂存指令；达妙电机随即失能，DJI 电调改发全零保活帧。
    void disable() noexcept
    {
        adapter_.disable();
    }
    // 编号从零开始；失败清空状态，无反馈返回有效读取但 feedback_valid=false。
    [[nodiscard]] bool read_state(std::size_t id, MotorState &state) const noexcept
    {
        return adapter_.read_state(id, state);
    }
    // 原生输出轴 N·m；没有请求出力时返回 false；非有限输入清零有效槽位并返回 false。
    [[nodiscard]] bool set_torque(std::size_t id, float value) noexcept
    {
        return adapter_.set_torque(id, value);
    }
    // 输出轴最大力矩能力（N·m）：型号档案决定，供应用层裁剪控制器限幅。
    [[nodiscard]] float maximum_torque_newton_meter() const noexcept
    {
        return adapter_.maximum_torque_newton_meter();
    }
    // 只清暂存指令；不撤销出力请求，不停止保活。
    void clear_commands() noexcept
    {
        adapter_.clear_commands();
    }

  private:
    Adapter adapter_;
};
} // namespace device
#endif
