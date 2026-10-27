#ifndef DRIVER_BMI088_BMI088_TEMPERATURE_HPP
#define DRIVER_BMI088_BMI088_TEMPERATURE_HPP

#include "driver/bmi088/bmi088_transfer.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief BMI088 温度读取子设备，使用加速度计接口域的温度寄存器。
 * @note 温度寄存器位于加速度计片选域内，须在加速度计完成初始化后由
 *       协调器调用 Enable 启用；数据读取全部通过 DMA 非阻塞接口完成。
 */
class Bmi088Temperature final
{
  public:
    static constexpr float CELSIUS_PER_LSB{0.125F};
    static constexpr float CELSIUS_OFFSET{23.0F};

    enum class State : std::uint8_t
    {
        DISABLED,
        RUNNING,
        ERROR
    };

    struct Sample final
    {
        std::int16_t raw{0};
    };

    Bmi088Temperature() noexcept = default;

    /**
   * @brief 重置状态机并回到禁用状态，等待协调器在加速度计就绪后启用。
   */
    void init() noexcept;

    /**
   * @brief 启用温度读取，仅在加速度计初始化完成后调用。
   */
    void enable() noexcept;

    /**
   * @brief 查询并收割在途异步事务的终态，完成原始温度解析。
   */
    void harvest() noexcept;

    /**
   * @brief 在无在途事务时启动一次温度读取。
   * @note 总线被共享方占用时收到 Busy 并直接返回，下一周期重试。
   */
    // 仅当本次调用启动了新传输（而非既有传输）时为 true。
    bool try_start() noexcept;

    void set_completion_notification(platform::Spi::CompletionNotification callback, void *context) noexcept
    {
        completion_.set_notification(callback, context);
    }
    [[nodiscard]] std::uint32_t get_completion_tick() const noexcept
    {
        return completion_.completed_tick();
    }

    /**
   * @brief 查询温度读取是否已被启用。
   * @return 已通过 Enable 启用时返回 true。
   */
    [[nodiscard]] bool is_enabled() const noexcept
    {
        return state_ != State::DISABLED;
    }

    /**
   * @brief 查询本子设备是否有尚未收割终态的异步事务。
   * @return 存在在途事务时返回 true，供协调器进行总线避让。
   */
    [[nodiscard]] bool has_pending_transfer() const noexcept
    {
        return transfer_active_;
    }

    [[nodiscard]] State get_state() const noexcept
    {
        return state_;
    }

    /**
   * @brief 获取最近一次成功读取的 11 位符号扩展原始温度计数。
   * @param sample 用于接收原始样本的输出对象。
   * @return 至少完成一次数据读取时返回 true。
   */
    [[nodiscard]] bool try_get_sample(Sample &sample) const noexcept;

    [[nodiscard]] std::uint32_t get_read_count() const noexcept
    {
        return read_count_;
    }

    [[nodiscard]] std::uint32_t get_error_count() const noexcept
    {
        return error_count_;
    }

    /**
   * @brief 将原始温度计数换算为摄氏温度。
   * @param raw 符号扩展后的 11 位原始温度计数值。
   * @return 按 0.125 °C/LSB 与 23 °C 偏移换算的温度值。
   */
    [[nodiscard]] static float to_celsius(std::int16_t raw) noexcept
    {
        return static_cast<float>(raw) * CELSIUS_PER_LSB + CELSIUS_OFFSET;
    }

  private:
    static constexpr std::size_t MAXIMUM_TRANSFER_LENGTH{4U};

    void fail() noexcept;

    State state_{State::DISABLED};
    Bmi088Transfer completion_{};
    bool transfer_active_{false};
    bool sample_valid_{false};
    Sample sample_{};
    std::uint32_t read_count_{0U};
    std::uint32_t error_count_{0U};
    std::array<std::uint8_t, MAXIMUM_TRANSFER_LENGTH> transmit_buffer_{};
    std::array<std::uint8_t, MAXIMUM_TRANSFER_LENGTH> receive_buffer_{};
};

} // namespace device

#endif
