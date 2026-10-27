#ifndef IMU_ATTITUDE_ESTIMATOR_HPP
#define IMU_ATTITUDE_ESTIMATOR_HPP

#include "imu/gyro_bias_calibration.hpp"
#include "driver/bmi088/bmi088_sample_queue.hpp"
#include <cstdint>

namespace application
{

/**
 * @brief 姿态解算的数据通路与所有者。
 *   把已排序的原始样本映射到机体系，按样本类型分别送入 VQF；
 *   保留时序与质量检查，首个陀螺样本只建立时基，断流时不虚构缺失运动；
 *   保留温度门控的上电静止标定，把三轴原始零偏一次性注入 VQF，
 *    不在陀螺输入侧再次扣除零偏；
 *   加速度样本只在首次收到时更新一次，最近值仅供上电标定门控使用。
 */
class AttitudeEstimator final
{
  public:
    /** @brief 结果存档版本，结构变化时递增。 */
    static constexpr std::uint32_t OUTPUT_VERSION{3U};

    /**
   * @brief 对外状态存档，布局固定，供调试器读取。
   */
    struct Output final
    {
        std::uint32_t version{OUTPUT_VERSION};
        std::uint32_t update_tick{0U};               // 最近一次更新的 drdy_tick
        float quaternion[4]{1.0F, 0.0F, 0.0F, 0.0F}; // 机体系到世界系，标量在前
        float euler_degrees[3]{};                    // ZYX：yaw、pitch、roll
        float gyro_bias_rad_per_sec[3]{};            // VQF 估计的三轴原始陀螺零偏
        float yaw_total_degrees{0.0F};               // 跨 ±180° 累计的连续 yaw
        float bias_uncertainty_rad_per_sec{0.0F};    // 三轴零偏不确定度上界
        std::uint32_t update_count{0U};              // 有效陀螺更新次数
        float gyroscope_deviation_ratio{0.0F};       // 静止判据的陀螺偏离与阈值之比
        std::uint32_t valid{0U};                     // 至少一次陀螺更新且姿态有限
        std::uint32_t rest_detected{0U};             // VQF 连续静止判定结果
        float last_accel_correction_radians{0.0F};   // 最近一次倾角修正角
        float accelerometer_deviation_ratio{0.0F};   // 静止判据的加速度偏离与阈值之比
        // 版本 2 起的上电静止标定结果与状态，版本 3 保持字段位置不变。
        float calibration_bias_rad_per_sec[3]{};   // 静止窗口估计的三轴零偏
        float calibration_spread_rad_per_sec[3]{}; // 窗口内每轴极差
        std::uint32_t calibration_state{0U};       // 见 GyroBiasCalibration::State
        std::uint32_t calibration_samples{0U};     // 当前窗口样本数
        float calibration_temperature_celsius{0.0F};
        std::uint32_t calibration_elapsed_ms{0U}; // 累计处于采样状态的时间
    };
    static_assert(sizeof(Output) == 120U, "调试器存档布局必须稳定");

    /**
   * @brief 时序与质量统计，用于判断估计结果是否可信。
   */
    struct Statistics final
    {
        std::uint32_t gyro_received{0U};
        std::uint32_t accel_received{0U};
        std::uint32_t gyro_consumed{0U};
        std::uint32_t accel_consumed{0U};
        std::uint32_t rejected_flags{0U};
        std::uint32_t rejected_sequence{0U};
        std::uint32_t gyro_gaps{0U};
        std::uint32_t accel_stale{0U}; // 陀螺更新时最近加速度样本超过诊断年龄上限
        std::uint32_t reserved[3]{};   // 原加速度复用与预测计数槽位，不再使用
        std::uint32_t calibration_restarts{0U};
    };
    static_assert(sizeof(Statistics) == 48U, "统计布局必须稳定");

    /** @brief 陀螺间隔上限（秒），超过视为断流并重新建立时间基准。 */
    static constexpr float GYRO_GAP_LIMIT_SECONDS{0.01F};

    /** @brief 最近加速度样本的诊断年龄上限（秒），不控制 VQF 是否积分陀螺。 */
    static constexpr float ACCELEROMETER_STALE_LIMIT_SECONDS{0.005F};

    AttitudeEstimator() = delete;

    /**
   * @brief 初始化滤波器与上电零偏标定。必须在 IMU 任务开始消费样本前调用。
   * @return 滤波器参数有效时返回 true。
   */
    [[nodiscard]] static bool init() noexcept;

    /** @brief 查询是否已完成初始化。 */
    [[nodiscard]] static bool is_ready() noexcept;

    /**
   * @brief 按时间顺序喂入一条原始样本。常数时间、不阻塞。
   * @param record 驱动收割到的原始样本。
   * @param gyroscope true 表示陀螺仪样本，false 表示加速度计样本。
   * @param temperature_celsius 最近一次温度读数，用于上电零偏标定的温度门控；
   *                           无有效读数时传 NaN。
   */
    static void process(const device::Bmi088SampleRecord &record, bool gyroscope, float temperature_celsius) noexcept;

    /** @brief 读取结果存档。 */
    [[nodiscard]] static Output get_output() noexcept;

    /** @brief 读取时序与质量统计。 */
    [[nodiscard]] static Statistics get_statistics() noexcept;
};

} // namespace application

#endif
