#ifndef LIBRARIES_DEVICE_MOTOR_MOTOR_GROUP_HPP
#define LIBRARIES_DEVICE_MOTOR_MOTOR_GROUP_HPP
#include "Libraries/Device/motor/Motor.hpp"
#include "Platform/Interface/Can.hpp"
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
 *         Init/Process/ReadState/SetTorque/ClearCommands/MaximumTorqueNewtonMeter 操作契约；
 *       - static constexpr MaximumMotors：该型号允许的最大电机数量；
 *       - template <N> MakeConfig(channel, connections)：把品牌默认参数
 *         （电调档案、型号原生减速比等）装配为 Adapter::Config。
 */
template <MotorModel Model> struct MotorAdapterFor;

namespace detail
{
// 全部电机组共享的通道占用表；仅调度前 Init 访问，成功占用后不提供释放。
inline std::array<const void *, CAN_PORT_CHANNEL_COUNT> motorChannelOwners{};
} // namespace detail

/**
 * @brief 一个任务持有的同型号电机组，直接持有编译期选定的品牌适配器。
 * @note Init 先于调度执行，成功后实例须保持存活至固件结束；所有收发和
 *       读写由同一个任务调用。每组独占一路 CAN：同通道被第二个电机组
 *       （含同型号）占用时 Init 失败，失败实例不占用通道。
 *       输出为型号原生输出轴单位；外接传动不包含在型号档案内。
 */
template <MotorModel Model, std::size_t N> class MotorGroup final
{
  static_assert(N > 0U, "电机组数量必须为正");
  using Mapping = MotorAdapterFor<Model>;
  using Adapter = typename Mapping::Adapter;
  static_assert(N <= Mapping::MaximumMotors, "电机组数量超过型号能力");

public:
  MotorGroup(platform::Can::Channel channel, const std::array<MotorConnection, N> &connections) noexcept
      : channel_{channel}, adapter_{Mapping::template MakeConfig<N>(channel, connections)}
  {
  }
  MotorGroup(const MotorGroup &) = delete;
  MotorGroup &operator=(const MotorGroup &) = delete;
  MotorGroup(MotorGroup &&) = delete;
  MotorGroup &operator=(MotorGroup &&) = delete;

  [[nodiscard]] bool Init() noexcept
  {
    const std::size_t channel = static_cast<std::size_t>(channel_);
    if (channel >= detail::motorChannelOwners.size())
    {
      return false;
    }
    if (detail::motorChannelOwners[channel] != nullptr && detail::motorChannelOwners[channel] != this)
    {
      return false;
    }
    if (!adapter_.Init())
    {
      return false;
    }
    detail::motorChannelOwners[channel] = this;
    return true;
  }
  [[nodiscard]] static constexpr std::size_t Count() noexcept
  {
    return N;
  }
  // 沿用“收、老化、发送、发布”，暂存的新指令在下次 Process 发出。
  void Process() noexcept
  {
    adapter_.Process();
  }
  // 编号从零开始；失败清空状态，无反馈返回有效读取但 feedbackValid=false。
  [[nodiscard]] bool ReadState(std::size_t id, MotorState &state) const noexcept
  {
    return adapter_.ReadState(id, state);
  }
  // 原生输出轴 N·m；非有限输入清零有效槽位并返回 false。
  [[nodiscard]] bool SetTorque(std::size_t id, float value) noexcept
  {
    return adapter_.SetTorque(id, value);
  }
  // 输出轴最大力矩能力（N·m）：型号档案决定，供应用层裁剪控制器限幅。
  [[nodiscard]] float MaximumTorqueNewtonMeter() const noexcept
  {
    return adapter_.MaximumTorqueNewtonMeter();
  }
  // 只清暂存指令；不代表硬件失能，不停止保活。
  void ClearCommands() noexcept
  {
    adapter_.ClearCommands();
  }

private:
  platform::Can::Channel channel_;
  Adapter adapter_;
};
} // namespace device
#endif
