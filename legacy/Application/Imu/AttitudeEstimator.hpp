#ifndef APPLICATION_IMU_ATTITUDE_ESTIMATOR_HPP
#define APPLICATION_IMU_ATTITUDE_ESTIMATOR_HPP

#include "Application/Imu/GyroBiasCalibration.hpp"
#include "Libraries/Device/bmi088/Bmi088SampleQueue.hpp"
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
  static constexpr std::uint32_t OutputVersion{3U};

  /**
   * @brief 对外状态存档，布局固定，供调试器读取。
   */
  struct Output final
  {
    std::uint32_t version{OutputVersion};
    std::uint32_t updateTick{0U};                // 最近一次更新的 drdyTick
    float quaternion[4]{1.0F, 0.0F, 0.0F, 0.0F}; // 机体系到世界系，标量在前
    float eulerDegrees[3]{};                     // ZYX：yaw、pitch、roll
    float gyroBiasRadPerSec[3]{};                // VQF 估计的三轴原始陀螺零偏
    float yawTotalDegrees{0.0F};                 // 跨 ±180° 累计的连续 yaw
    float biasUncertaintyRadPerSec{0.0F};        // 三轴零偏不确定度上界
    std::uint32_t updateCount{0U};               // 有效陀螺更新次数
    float gyroscopeDeviationRatio{0.0F};         // 静止判据的陀螺偏离与阈值之比
    std::uint32_t valid{0U};                     // 至少一次陀螺更新且姿态有限
    std::uint32_t restDetected{0U};              // VQF 连续静止判定结果
    float lastAccelCorrectionRadians{0.0F};      // 最近一次倾角修正角
    float accelerometerDeviationRatio{0.0F};     // 静止判据的加速度偏离与阈值之比
    // 版本 2 起的上电静止标定结果与状态，版本 3 保持字段位置不变。
    float calibrationBiasRadPerSec[3]{};   // 静止窗口估计的三轴零偏
    float calibrationSpreadRadPerSec[3]{}; // 窗口内每轴极差
    std::uint32_t calibrationState{0U};    // 见 GyroBiasCalibration::State
    std::uint32_t calibrationSamples{0U};  // 当前窗口样本数
    float calibrationTemperatureCelsius{0.0F};
    std::uint32_t calibrationElapsedMs{0U}; // 累计处于采样状态的时间
  };
  static_assert(sizeof(Output) == 120U, "调试器存档布局必须稳定");

  /**
   * @brief 时序与质量统计，用于判断估计结果是否可信。
   */
  struct Statistics final
  {
    std::uint32_t gyroReceived{0U};
    std::uint32_t accelReceived{0U};
    std::uint32_t gyroConsumed{0U};
    std::uint32_t accelConsumed{0U};
    std::uint32_t rejectedFlags{0U};
    std::uint32_t rejectedSequence{0U};
    std::uint32_t gyroGaps{0U};
    std::uint32_t accelStale{0U}; // 陀螺更新时最近加速度样本超过诊断年龄上限
    std::uint32_t reserved[3]{};  // 原加速度复用与预测计数槽位，不再使用
    std::uint32_t calibrationRestarts{0U};
  };
  static_assert(sizeof(Statistics) == 48U, "统计布局必须稳定");

  /** @brief 陀螺间隔上限（秒），超过视为断流并重新建立时间基准。 */
  static constexpr float GyroGapLimitSeconds{0.01F};

  /** @brief 最近加速度样本的诊断年龄上限（秒），不控制 VQF 是否积分陀螺。 */
  static constexpr float AccelerometerStaleLimitSeconds{0.005F};

  AttitudeEstimator() = delete;

  /**
   * @brief 初始化滤波器与上电零偏标定。必须在 IMU 任务开始消费样本前调用。
   * @return 滤波器参数有效时返回 true。
   */
  [[nodiscard]] static bool Init() noexcept;

  /** @brief 查询是否已完成初始化。 */
  [[nodiscard]] static bool IsReady() noexcept;

  /**
   * @brief 按时间顺序喂入一条原始样本。常数时间、不阻塞。
   * @param record 驱动收割到的原始样本。
   * @param gyroscope true 表示陀螺仪样本，false 表示加速度计样本。
   * @param temperatureCelsius 最近一次温度读数，用于上电零偏标定的温度门控；
   *                           无有效读数时传 NaN。
   */
  static void Process(const device::Bmi088SampleRecord &record, bool gyroscope, float temperatureCelsius) noexcept;

  /** @brief 读取结果存档。 */
  [[nodiscard]] static Output GetOutput() noexcept;

  /** @brief 读取时序与质量统计。 */
  [[nodiscard]] static Statistics GetStatistics() noexcept;
};

} // namespace application

#endif
