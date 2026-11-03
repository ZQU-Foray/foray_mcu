#ifndef HAL_PWM_HPP
#define HAL_PWM_HPP

#include <cstdint>

// 硬件契约：PWM 输出通道。句柄由各板的 board.hpp 给出，实现由 port 提供，
// 宿主测试由 platform/mock 提供。

namespace hal
{

/**
 * @brief 一个 PWM 输出通道的句柄：频率与占空比都可以改。
 *
 * 可设的频率范围由板决定（见 board.hpp）。句柄只含一个编号，可以拷贝，按值存进驱动的配置。
 */
class Pwm final
{
  public:
    enum class Result : std::uint8_t
    {
        COMPLETED = 0U,
        NOT_READY = 1U,        // hal::init() 还没有成功启动这个通道
        INVALID_ARGUMENT = 2U, // 编号无效、频率为 0 或超出本板范围、占空比超过 MAX_DUTY_PERMILLE
        ERROR = 3U
    };

    /** 占空比的单位是千分之一，1000 为一直有效。 */
    static constexpr std::uint16_t MAX_DUTY_PERMILLE{1000U};

    constexpr explicit Pwm(std::uint8_t index) noexcept : index_{index} {}

    /**
     * @brief 通道是否已经启动、可以设置输出。
     * @note 任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] bool is_ready() const noexcept;

    /**
     * @brief 设置频率与占空比，立即生效。
     * @param frequency_hz 输出频率（Hz），大于 0，上下限由板决定。
     * @param duty_permille 占空比（‰），0 到 MAX_DUTY_PERMILLE。
     * @note 在任务里调用，不阻塞。一个通道只应有一个写入方。
     */
    [[nodiscard]] Result set(std::uint32_t frequency_hz, std::uint16_t duty_permille) const noexcept;

    /**
     * @brief 关闭输出：占空比设为 0，频率不变。
     * @note 同 set()。
     */
    [[nodiscard]] Result off() const noexcept;

  private:
    std::uint8_t index_; // 由 port 或 mock 解释
};

} // namespace hal

#endif // HAL_PWM_HPP
