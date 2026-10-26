#ifndef LIBRARIES_DEVICE_BMI088_BMI088_GYRO_HPP
#define LIBRARIES_DEVICE_BMI088_BMI088_GYRO_HPP

#include "Libraries/Device/bmi088/Bmi088Transfer.hpp"
#include "Platform/Interface/Time.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief BMI088 陀螺仪子设备驱动。
 * @note 全部 SPI 事务均通过 DMA 非阻塞接口完成，初始化配置链与数据读取
 *       由单一任务周期调用 Harvest 与 try_start 推进，任何路径都不阻塞。
 * @note 默认使用 2000 Hz ODR / 532 Hz 带宽；构建选项可选择 1000 Hz / 116 Hz
 * 对照测试。 （0x00，2000 Hz ODR、532 Hz 滤波带宽），量程 GYRO_RANGE_2000_DPS
 *       （±2000 °/s），并启用高有效推挽 INT3 DRDY；配置链完成后
 *       逐项读回校验（带宽保留位按掩码忽略），失败重试后进入 Error。
 */
class Bmi088Gyro final
{
  public:
    static constexpr std::uint8_t EXPECTED_CHIP_ID{0x0FU};

    // GYRO_RANGE(0x0F)：量程 gyro_range[1:0]
    static constexpr std::uint8_t GYRO_RANGE_2000_DPS{0x00U};
    static constexpr std::uint8_t GYRO_RANGE_1000_DPS{0x01U};
    static constexpr std::uint8_t GYRO_RANGE_500_DPS{0x02U};
    static constexpr std::uint8_t GYRO_RANGE_250_DPS{0x03U};

    // GYRO_BANDWIDTH(0x10)：gyro_bw[3:2] | gyro_odr[1:0]，bit7 保留。
    static constexpr std::uint8_t GYRO_BANDWIDTH_2000_HZ_532_HZ{0x00U};
    static constexpr std::uint8_t GYRO_BANDWIDTH_2000_HZ_230_HZ{0x01U};
    static constexpr std::uint8_t GYRO_BANDWIDTH_1000_HZ_116_HZ{0x02U};
    static constexpr std::uint8_t GYRO_BANDWIDTH_1000_HZ_47_HZ{0x03U};
    static constexpr std::uint8_t GYRO_BANDWIDTH_400_HZ_47_HZ{0x04U};
    static constexpr std::uint8_t GYRO_BANDWIDTH_400_HZ_23_HZ{0x05U};
    static constexpr std::uint8_t GYRO_BANDWIDTH_100_HZ_12_HZ{0x06U};
    static constexpr std::uint8_t GYRO_BANDWIDTH_100_HZ_6_HZ{0x07U};

    static constexpr std::uint8_t RANGE_REGISTER_VALUE{GYRO_RANGE_2000_DPS};
#ifndef BMI088_GYRO_BANDWIDTH_REGISTER
#define BMI088_GYRO_BANDWIDTH_REGISTER 0x00U
#endif
    static constexpr std::uint8_t BANDWIDTH_REGISTER_VALUE{BMI088_GYRO_BANDWIDTH_REGISTER};
    static_assert(BANDWIDTH_REGISTER_VALUE == GYRO_BANDWIDTH_2000_HZ_532_HZ ||
                  BANDWIDTH_REGISTER_VALUE == GYRO_BANDWIDTH_1000_HZ_116_HZ);
    static constexpr float DPS_PER_LSB{
        (RANGE_REGISTER_VALUE == GYRO_RANGE_1000_DPS
             ? 1000.0F
             : (RANGE_REGISTER_VALUE == GYRO_RANGE_500_DPS ? 500.0F
                                                      : (RANGE_REGISTER_VALUE == GYRO_RANGE_250_DPS ? 250.0F : 2000.0F))) /
        32768.0F};

    enum class State : std::uint8_t
    {
        SHUTDOWN,
        INITIALIZING,
        READY,
        ERROR
    };

    struct Sample final
    {
        std::int16_t x_axis{0};
        std::int16_t y_axis{0};
        std::int16_t z_axis{0};
    };

    Bmi088Gyro() noexcept = default;

    /**
   * @brief 重置状态机并进入初始化序列，本函数不访问硬件。
   * @note 首个 ID 读取前等待 POWER_UP_SETTLE_MS；ID 与配置读回校验失败时
   *       按有限次数重试，超过上限才进入 Error。
   */
    void init() noexcept;

    /**
   * @brief 查询并收割在途异步事务的终态，完成配置步校验或样本解析。
   */
    void harvest() noexcept;

    /**
   * @brief 在无在途事务且步进等待窗已过时启动下一步 SPI 事务。
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
   * @brief 查询初始化与数据链路是否已完成并可提供样本。
   * @return 配置链验证通过且处于周期读取状态时返回 true。
   */
    [[nodiscard]] bool is_ready() const noexcept
    {
        return state_ == State::READY;
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
   * @brief 获取最近一次成功读取的原始三轴角速度计数。
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
   * @brief 将原始计数换算为物理角速度，单位 °/s。
   * @param raw 陀螺仪原始计数值。
   * @return 按 RANGE_REGISTER_VALUE 所选量程换算的角速度值。
   */
    [[nodiscard]] static float to_dps(std::int16_t raw) noexcept
    {
        return static_cast<float>(raw) * DPS_PER_LSB;
    }

  private:
    enum class Step : std::uint8_t
    {
        CHECK_CHIP_ID,
        WRITE_RANGE,
        WRITE_BANDWIDTH,
        WRITE_INTERRUPT_CONTROL,
        WRITE_INTERRUPT_IO,
        WRITE_INTERRUPT_MAP,
        VERIFY_CORE_CONFIGURATION,
        VERIFY_INTERRUPT_CONTROL_AND_IO,
        VERIFY_INTERRUPT_MAP,
        READ_DATA
    };

    static constexpr std::size_t MAXIMUM_TRANSFER_LENGTH{7U};
    static constexpr std::uint32_t POWER_UP_SETTLE_MS{100U};
    static constexpr std::uint32_t CHIP_ID_RETRY_WAIT_MS{100U};
    static constexpr std::uint32_t CHIP_ID_MAX_ATTEMPTS{3U};
    static constexpr std::uint32_t CONFIGURATION_VERIFY_RETRY_WAIT_MS{100U};
    static constexpr std::uint32_t CONFIGURATION_VERIFY_MAX_ATTEMPTS{3U};
    static constexpr std::uint8_t RANGE_REGISTER_READ_MASK{0x03U};
    static constexpr std::uint8_t BANDWIDTH_REGISTER_READ_MASK{0x0FU};

    void begin_step() noexcept;
    void complete_step() noexcept;
    void advance(Step next_step, std::uint32_t wait_ms) noexcept;
    void fail() noexcept;
    [[nodiscard]] bool wait_elapsed() const noexcept;

    State state_{State::SHUTDOWN};
    Step step_{Step::CHECK_CHIP_ID};
    platform::Time::Tick step_tick_{0U};
    std::uint32_t step_wait_ms_{0U};
    Bmi088Transfer completion_{};
    bool transfer_active_{false};
    bool sample_valid_{false};
    Sample sample_{};
    std::uint32_t read_count_{0U};
    std::uint32_t error_count_{0U};
    std::uint32_t chip_id_attempts_{0U};
    std::uint32_t configuration_verify_attempts_{0U};
    std::array<std::uint8_t, MAXIMUM_TRANSFER_LENGTH> transmit_buffer_{};
    std::array<std::uint8_t, MAXIMUM_TRANSFER_LENGTH> receive_buffer_{};
};

} // namespace device

#endif
