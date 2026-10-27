#ifndef PLATFORM_INTERFACE_PWM_HPP
#define PLATFORM_INTERFACE_PWM_HPP

#include "Detail/Pwm.h"
#include <cstdint>

namespace platform
{

class Pwm final
{
  public:
    enum class Channel : std::uint8_t
    {
        BUZZER = PWM_PORT_CHANNEL_BUZZER,
        IMU_HEATER = PWM_PORT_CHANNEL_IMU_HEATER
    };

    enum class Result : std::uint8_t
    {
        COMPLETED = PWM_PORT_RESULT_COMPLETED,
        NOT_READY = PWM_PORT_RESULT_NOT_READY,
        INVALID_ARGUMENT = PWM_PORT_RESULT_INVALID_ARGUMENT,
        ERROR = PWM_PORT_RESULT_ERROR
    };

    static constexpr std::uint16_t MAX_DUTY_PERMILLE{PWM_PORT_DUTY_PERMILLE_MAX};

    /**
   * @brief 禁止创建 Pwm 实例，所有能力均通过静态方法访问。
   */
    Pwm() = delete;

    /**
   * @brief 查询指定 PWM 逻辑通道是否可以设置输出。
   * @param channel 要查询的逻辑通道。
   * @return 通道已初始化并可用时返回 true。
   */
    [[nodiscard]] static bool is_ready(Channel channel) noexcept
    {
        return PwmPort_IsReady(to_port_channel(channel));
    }

    /**
   * @brief 设置指定 PWM 逻辑通道的频率和占空比。
   * @param channel 目标逻辑通道。
   * @param frequency_hz 输出频率，单位 Hz，必须大于 0。
   * @param duty_permille 占空比千分值，范围为 0 到 1000。
   * @return 返回完成、未就绪、参数错误或底层错误。
   */
    [[nodiscard]] static Result set(Channel channel, std::uint32_t frequency_hz, std::uint16_t duty_permille) noexcept
    {
        return to_result(PwmPort_Set(to_port_channel(channel), frequency_hz, duty_permille));
    }

    /**
   * @brief 将指定 PWM 逻辑通道的占空比设置为 0。
   * @param channel 目标逻辑通道。
   * @return 返回完成、未就绪或参数错误。
   */
    [[nodiscard]] static Result silence(Channel channel) noexcept
    {
        return to_result(PwmPort_Silence(to_port_channel(channel)));
    }

  private:
    /**
   * @brief 将 C++ 逻辑通道转换为 Detail C ABI 使用的通道值。
   * @param channel C++ 逻辑通道。
   * @return 对应的 C ABI 通道值。
   */
    [[nodiscard]] static constexpr PwmPort_Channel to_port_channel(Channel channel) noexcept
    {
        return static_cast<PwmPort_Channel>(channel);
    }

    /**
   * @brief 将 Detail C ABI 的 PWM 结果转换为 C++ 结果枚举。
   * @param result C ABI PWM 结果。
   * @return 对应的 C++ 结果，未知值统一转换为 Error。
   */
    [[nodiscard]] static constexpr Result to_result(PwmPort_Result result) noexcept
    {
        return result <= PWM_PORT_RESULT_ERROR ? static_cast<Result>(result) : Result::ERROR;
    }
};

} // namespace platform

#endif
