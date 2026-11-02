#ifndef DRIVER_MOTOR_DM_DM_MOTOR_ADAPTER_HPP
#define DRIVER_MOTOR_DM_DM_MOTOR_ADAPTER_HPP

#include "driver/motor/dm/dm_motor.hpp"
#include "driver/motor/motor.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief 达妙电机接入电机组契约（MotorGroup）：逻辑编号 → 一台电机（ESC_ID、MST_ID、方向），只给力矩
 *        （MIT，Kp = Kd = 0），角度从核对后的第一帧反馈起算（Decision-20261102-01）。
 * @note 一个适配器独占一条 CAN 总线的接收；全部接口在同一个任务里调用。init() 之后电机还要回读核对，
 *       核对完成前 enable() 返回 false，能力按周期重试即可。
 */
class DmMotorAdapter final
{
  public:
    struct Mapping final
    {
        std::uint16_t esc_id{0U};
        std::uint16_t master_id{0U};
        int direction{1}; // +1 或 -1；作用于角度、转速与指令力矩
    };
    struct Config final
    {
        hal::Can bus; // 组合根注入的总线句柄
        DmMotor::Profile profile;
        std::array<Mapping, DmMotor::MAXIMUM_MOTORS> mappings{};
        std::size_t count{0U};
    };

    static constexpr std::size_t MAXIMUM_MOTORS{DmMotor::MAXIMUM_MOTORS};

    explicit DmMotorAdapter(const Config &config) noexcept;

    /** @brief 检查方向并初始化驱动；成功后再调用直接返回 true，不重新核对。 */
    [[nodiscard]] bool init() noexcept;
    [[nodiscard]] std::size_t count() const noexcept
    {
        return config_.count;
    }
    void process() noexcept;
    /** @brief 全组请求出力：任何一台不是 READY 就一台都不使能，返回 false。 */
    [[nodiscard]] bool enable() noexcept;
    /** @brief 全组撤销出力请求并清零指令，电机随即失能。 */
    void disable() noexcept;
    [[nodiscard]] bool read_state(std::size_t id, MotorState &state) const noexcept;
    [[nodiscard]] bool set_torque(std::size_t id, float value) noexcept;
    void clear_commands() noexcept;
    /** @brief 峰值扭矩与 TMAX 的较小者（J4310 为 10 N·m）；未初始化时为 0。 */
    [[nodiscard]] float maximum_torque_newton_meter() const noexcept;

    /** @brief 达妙电机的完整快照（核对读到的值、温度、阶段），排查与 bringup 检查用。 */
    [[nodiscard]] bool get_snapshot(std::size_t id, DmMotor::Snapshot &snapshot) const noexcept
    {
        return ready_ && motors_.get_snapshot(id, snapshot);
    }

  private:
    [[nodiscard]] static DmMotor::Config motor_config(const Config &config) noexcept;

    Config config_;
    DmMotor motors_;
    bool ready_{false};
};

} // namespace device

#endif // DRIVER_MOTOR_DM_DM_MOTOR_ADAPTER_HPP
