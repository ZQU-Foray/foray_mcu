#ifndef LIBRARIES_ALGORITHM_ALG_ESTIMATE_VQF_H
#define LIBRARIES_ALGORITHM_ALG_ESTIMATE_VQF_H

#include "Libraries/Algorithm/alg_filter/ButterworthLowPass.hpp"
#include "Libraries/Algorithm/alg_math/Matrix3.h"
#include "Libraries/Algorithm/alg_math/Quaternion.h"
#include "Libraries/Algorithm/alg_math/Vector3.h"
#include <cstdint>

namespace alg_estimate
{

/**
 * @brief VQF 姿态滤波器（6 维，无磁力计）：陀螺积分 + 加速度计倾角修正
 *        + 静止判定 + 三轴陀螺零偏估计。
 *
 * 算法来源：
 *   D. Laidig, T. Seel. "VQF: Highly Accurate IMU Orientation Estimation with Bias
 *   Estimation and Magnetic Disturbance Rejection." Information Fusion 2023, 91, 187-204.
 *   doi:10.1016/j.inffus.2022.10.014（预印本 arXiv:2203.17024）。
 *   参考实现：https://github.com/dlaidig/vqf
 *
 * 采样率：陀螺与加速度计允许不同（本工程 2000 Hz / 1600 Hz），因此两者的周期在
 * `Init` 里分别给出，调用方按记录类型分别调用 `UpdateGyroscope` /
 * `UpdateAccelerometer`。采样周期的小幅抖动
 * 由滤波器时间常数吸收
 *
 * 本类不访问硬件、不读时间、不动态分配。
 */
class Vqf final
{
public:
  /**
   * @brief 调参。 的默认值，角度量统一换算为 rad/s 表示。
   * @note 在此以 rad/s 给出并在括注里标出等价值，避免调用方
   *       在两套单位之间来回换算。
   */
  struct Config final
  {
    float tauAccSeconds{3.0F};                      // τ_acc：加速度计低通时间常数，越大越信陀螺
    bool restBiasEstEnabled{true};                  // 静止窗口内更新零偏
    bool motionBiasEstEnabled{true};                // 运动过程中更新零偏
    float biasSigmaInitDegPerSec{0.5F};             // 零偏初值标准差（deg/s）
    float biasForgettingTimeSeconds{100.0F};        // 零偏方差从 0 涨到 (0.1 deg/s)^2 所需时间
    float biasClipRadPerSec{0.0349065851F};         // 零偏限幅，等价 2.0 deg/s
    float biasSigmaMotionDegPerSec{0.1F};           // 运动更新的量测噪声（deg/s）
    float biasVerticalForgettingFactor{1.0e-4F};    // 运动更新中重力方向分量的遗忘因子
    float biasSigmaRestDegPerSec{0.03F};            // 静止更新的量测噪声（deg/s）
    float restMinSeconds{1.5F};                     // 方差持续低于阈值多久才算静止
    float restFilterTauSeconds{0.5F};               // 静止判定用的低通时间常数
    float restThresholdGyrRadPerSec{0.0349065851F}; // 陀螺方差阈值，等价 2.0 deg/s
    float restThresholdAccMetersPerSec2{0.5F};      // 加速度计方差阈值
  };

  /** @brief 对外状态快照。 */
  struct State final
  {
    alg_math::Quaternion quaternion{};      // body → earth，earth 重力沿 +z
    alg_math::Vector3 gyroBiasRadPerSec{};  // 机体系零偏估计
    float biasUncertaintyRadPerSec{0.0F};   // 零偏估计不确定度
    alg_math::Vector3 eulerRadians{};       // ZYX：yaw、pitch、roll
    alg_math::Vector3 eulerDegrees{};       // 同上，单位 degree
    float lastAccelCorrectionRadians{0.0F}; // 最近一次倾角修正的转角（诊断）
    bool restDetected{false};               // 最近一次加速度计更新时的静止判定
    bool valid{false};                      // 至少完成一次陀螺更新
    std::uint32_t updateCount{0U};          // 有效陀螺更新次数
  };

  /**
   * @brief 静止判据的可观测量。
   * 用途：把 `restThresholdGyrRadPerSec` / `restThresholdAccMetersPerSec2` 标定到本机
   * 实际噪声水平。两个比值都按“实测偏离 / 阈值”归一化，**1.0 就是判据边界**：
   * 静止时应远小于 1，运动时应大于 1；若静止时比值已经接近 1，就说明阈值开得太小。
   */
  struct RestObservables final
  {
    float gyroscopeDeviationRatio{0.0F};     // sqrt(陀螺侧方差)/阈值，≥1 判为运动
    float accelerometerDeviationRatio{0.0F}; // sqrt(加速度计侧方差)/阈值，≥1 判为运动
    float accumulatedRestSeconds{0.0F};      // 加速度计侧累计满足静止条件的时长
    bool restDetected{false};                // 当前静止判定结果
  };

  /**
   * @brief 滤波器完整内部状态（调试/离线回放逐段续算/检查点）。
   * @note 对应 `VQF::getState/setState`。
   *       低通子状态用显式字段（状态数组 + 初始化平均累加 + 是否处于
   *       初始化阶段），而不是那种“状态槽里塞 NaN 哨兵 + 槽位复用”。
   *       快照记录完整调参与采样周期；恢复时同时校验这些值与低通系数，
   *       防止把异配置快照恢复到当前滤波器。
   */
  struct InternalState final
  {
    // 恢复兼容性
    Config config{};
    double gyroPeriodSeconds{0.0};
    double accelPeriodSeconds{0.0};
    // 姿态
    alg_math::Quaternion gyroQuaternion{};
    alg_math::Quaternion accelQuaternion{};
    alg_math::Vector3 lastAccelLp{};
    float lastAccelCorrectionRadians{0.0F};
    alg_filter::ButterworthLowPass<3U>::InternalState accelLowPass{};
    // 零偏估计
    alg_math::Vector3 gyroBiasRadPerSec{};
    alg_math::Matrix3 biasCovariance{};
    alg_filter::ButterworthLowPass<9U>::InternalState motionBiasRotationLowPass{};
    alg_filter::ButterworthLowPass<2U>::InternalState motionBiasProjectionLowPass{};
    // 静止判定
    bool restDetected{false};
    double restSeconds{0.0};
    double restSquaredDeviations[2]{};
    alg_math::Vector3 restLastGyroLp{};
    alg_math::Vector3 restLastAccelLp{};
    alg_filter::ButterworthLowPass<3U>::InternalState restGyroLowPass{};
    alg_filter::ButterworthLowPass<3U>::InternalState restAccelLowPass{};
    // 计数
    std::uint32_t updateCount{0U};
    bool valid{false};
  };

  Vqf() noexcept = default;

  /**
   * @brief 用给定参数与采样周期初始化滤波器并复位状态。
   * @param config 调参。
   * @param gyroPeriodSeconds 陀螺采样周期，单位 s（本工程 2000 Hz → 5e-4）。
   * @param accelPeriodSeconds 加速度计采样周期，单位 s（本工程 1600 Hz → 6.25e-4）。
   * @return 参数与周期全部有效时返回 true 并进入就绪；否则保持未就绪并返回 false。
   * @note 校验项：两个周期有限且为正；`tauAccSeconds` 与 `restFilterTauSeconds` 有限且为正；
   *       `restMinSeconds`、`biasForgettingTimeSeconds` 有限且为正；其余噪声/限幅/阈值
   *       为有限值（阈值允许为 0，表示永判为运动）。
   */
  [[nodiscard]] bool Init(const Config &config, float gyroPeriodSeconds, float accelPeriodSeconds) noexcept;

  /** @brief 复位滤波器状态（保留已设置的参数与周期）。 */
  void Reset() noexcept;

  /**
   * @brief 陀螺更新步：静止判定的陀螺侧、去零偏、四元数积分。
   * @param gyroRadPerSec 机体系角速度，单位 rad/s。
   * @note 输入非有限时整步被忽略（不做部分更新），避免污染状态。
   */
  void UpdateGyroscope(const alg_math::Vector3 &gyroRadPerSec) noexcept;

  /**
   * @brief 加速度计更新步：静止判定的加速度侧、惯性系低通、倾角修正、零偏估计。
   * @param accelMetersPerSec2 机体系比力，单位 m/s^2。
   * @note 全零样本按"本拍无加速度计数据"处理（本工程正常数据不会全零，
   *       除自由落体极端情形）；输入非有限时整步被忽略。
   */
  void UpdateAccelerometer(const alg_math::Vector3 &accelMetersPerSec2) noexcept;

  /**
   * @brief 便利入口：陀螺与加速度计同周期时一次喂入两路。
   * @note 内部依次执行 `UpdateGyroscope` 与 `UpdateAccelerometer`；两路周期不同时
   *       必须改用单路入口。
   */
  void Update(const alg_math::Vector3 &gyroRadPerSec, const alg_math::Vector3 &accelMetersPerSec2) noexcept;

  /** @brief 读取状态快照（含四元数、零偏、零偏不确定度与 ZYX 欧拉角）。 */
  [[nodiscard]] State GetState() const noexcept;

  /**
   * @brief 读取静止判据的可观测量（用于阈值标定与调试）。
   */
  [[nodiscard]] RestObservables GetRestObservables() const noexcept;

  /**
   * @brief 注入零偏估计，可选同时设定其不确定度。
   * @param biasRadPerSec 机体系零偏，单位 rad/s。
   * @param uncertaintyRadPerSec 零偏标准差，单位 rad/s；≤0 或非有限表示只改零偏、不动协方差。
   * @note `VQF::setBiasEstimate`。本工程的上电静止零偏标定
   *       （`GyroBiasCalibration`）就是它的典型调用方。
   * @note 协方差按 `P = (σ·180·100/π)²·I` 设置，
   *       即"注入的零偏有 σ 的不确定度"，随后由滤波器继续跟踪残余零偏。
   *       零偏非有限时整次调用被忽略。
   */
  void SetBiasEstimate(const alg_math::Vector3 &biasRadPerSec, float uncertaintyRadPerSec = -1.0F) noexcept;

  /** @brief 读取完整内部状态（检查点/离线逐段续算/调试）。 */
  [[nodiscard]] InternalState GetInternalState() const noexcept;

  /**
   * @brief 恢复完整内部状态。
   * @param state 先前由 `GetInternalState` 读出的快照。
   * @return 滤波器已就绪、且快照与当前配置同源时返回 true；否则不改动任何状态并返回 false。
   * @note 与 `SetBiasEstimate` 的区别：本函数恢复**全部**状态（含低通内部状态与静止判定），
   *       用于逐位再现后续演进；`SetBiasEstimate` 只动零偏与协方差。
   */
  [[nodiscard]] bool SetInternalState(const InternalState &state) noexcept;

  /** @brief 查询六维姿态四元数（body → earth）。 */
  [[nodiscard]] alg_math::Quaternion GetQuaternion() const noexcept;

  [[nodiscard]] bool IsInitialized() const noexcept
  {
    return initialized_;
  }

private:
  // 周期与零偏估计标定（Init 时算好，运行期不变）
  Config config_{};
  double gyroPeriod_{0.0};
  double accelPeriod_{0.0};
  double biasP0_{0.0};
  double biasV_{0.0};
  double biasMotionW_{0.0};
  double biasVerticalW_{0.0};
  double biasRestW_{0.0};
  double restThresholdGyr_{0.0};
  double restThresholdAcc_{0.0};
  double restMin_{1.5};
  double biasClip_{0.0};
  bool restBiasEstEnabled_{true};
  bool motionBiasEstEnabled_{true};

  // 姿态状态
  alg_math::Quaternion gyroQuat_{};  // 角速度积分四元数
  alg_math::Quaternion accelQuat_{}; // 倾角修正四元数
  alg_math::Vector3 lastAccelLp_{};  // 最近一次惯性系下低通后的加速度
  float lastAccelCorrectionRadians_{0.0F};
  alg_filter::ButterworthLowPass<3U> accelLowPass_{}; // 惯性系加速度低通（τ_acc / 加速度计周期）

  // 零偏估计状态
  alg_math::Vector3 gyroBias_{};
  alg_math::Matrix3 biasCovariance_{};
  alg_filter::ButterworthLowPass<9U> motionBiasRotationLowPass_{};   // R 低通
  alg_filter::ButterworthLowPass<2U> motionBiasProjectionLowPass_{}; // R·b 的 x/y 低通

  // 静止判定状态
  bool restDetected_{false};
  double restT_{0.0};
  double restSquaredDeviations_[2]{};
  alg_math::Vector3 restLastGyroLp_{};
  alg_math::Vector3 restLastAccelLp_{};
  alg_filter::ButterworthLowPass<3U> restGyroLowPass_{};  // τ_rest / 陀螺周期
  alg_filter::ButterworthLowPass<3U> restAccelLowPass_{}; // τ_rest / 加速度计周期

  std::uint32_t updateCount_{0U};
  bool initialized_{false};
  bool valid_{false};
};

} // namespace alg_estimate

#endif // LIBRARIES_ALGORITHM_ALG_ESTIMATE_VQF_H
