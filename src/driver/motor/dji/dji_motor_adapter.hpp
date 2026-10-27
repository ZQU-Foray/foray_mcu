#ifndef DRIVER_MOTOR_DJI_DJI_MOTOR_ADAPTER_HPP
#define DRIVER_MOTOR_DJI_DJI_MOTOR_ADAPTER_HPP

#include "driver/motor/motor.hpp"
#include "driver/motor/dji/dji_motor.hpp"

namespace device
{

// 一个适配器独占一路 CAN 会话；严禁同通道挂多个消费者。
class DjiMotorAdapter final
{
  public:
    struct Mapping final
    {
        std::uint8_t device_id{0U};
        float gear_ratio{1.0F};
        int direction{1}; // +1 或 -1；作用于角度、转速与指令力矩
    };
    struct Config final
    {
        platform::Can::Channel channel;
        DjiMotor::Profile profile;
        std::array<Mapping, DjiMotor::MAXIMUM_MOTORS> mappings{};
        std::size_t count{0U};
    };

    static constexpr std::size_t MAXIMUM_MOTORS{DjiMotor::MAXIMUM_MOTORS};

    explicit DjiMotorAdapter(const Config &config) noexcept;
    [[nodiscard]] bool init() noexcept;
    [[nodiscard]] std::size_t count() const noexcept
    {
        return config_.count;
    }
    void process() noexcept;
    [[nodiscard]] bool read_state(std::size_t id, MotorState &state) const noexcept;
    [[nodiscard]] bool set_torque(std::size_t id, float value) noexcept;
    void clear_commands() noexcept;

    /**
   * @brief 输出轴最大力矩能力（N·m）＝转矩常数 × 电流满量程。
   * @note 档案的 Kt 已是输出轴口径，因此不再乘减速比；与 set_torque/set_current_ampere
   *       的换算一致（命令比率会被钳到满量程，超出部分本就不可实现）。
   * @return Init 成功后返回能力值；未就绪时返回 0（表示能力不可用）。
   * @note 用途：应用层用它校验控制器限幅，避免"控制器以为能给出的力矩"超过执行器能力。
   */
    [[nodiscard]] float maximum_torque_newton_meter() const noexcept
    {
        if (!ready_)
        {
            return 0.0F;
        }
        return config_.profile.torque_constant_newton_meter_per_ampere * config_.profile.current_full_scale_ampere;
    }

  private:
    Config config_;
    DjiMotor bus_;
    bool ready_{false};
};

} // namespace device
#endif
