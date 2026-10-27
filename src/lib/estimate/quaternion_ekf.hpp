#ifndef LIB_ESTIMATE_QUATERNION_EKF_HPP
#define LIB_ESTIMATE_QUATERNION_EKF_HPP

#include "lib/estimate/kalman_filter.hpp"
#include "lib/math/quaternion.hpp"
#include "lib/math/vector3.hpp"
#include <cstdint>

namespace estimate
{

/**
 * @brief 六状态加性四元数 EKF：四元数 4 维 + 陀螺零偏 x/y 两轴。
 *
 * 状态向量布局：xᴴ = [q0, q1, q2, q3, b_x, b_y]，其中四元数为机体系到世界系的旋转
 * （标量在前），b 为陀螺零偏，单位 rad/s。零偏 z 轴不参与估计，因此无外部航向参考时
 * yaw 不可观，这是算法本身的物理边界，不是实现缺陷。
 *
 * 状态转移按一阶线性化：四元数分块为 I + 0.5·dt·Ω(ω−b)，零偏分块为单位阵；零偏对
 * 四元数的耦合项只在协方差传播中使用（先验状态预测时该分块为零），与算法前身一致。
 * 过程噪声为 diag(Q1·dt ×4, Q2·dt ×2)，量测噪声为 diag(R ×3)，量测为归一化后的
 * 机体加速度方向，观测量为预测的机体重力方向。
 *
 * 量测更新包含完整的前身后处理：卡方检验三分支（收敛判定、发散计数与仅预测、
 * 自适应增益）、零偏行按方向余弦缩放、收敛后零偏增量限幅、四元数 z 分量不修正
 * （重力观测不约束偏航），以及静止判据。
 *
 * @note 输入输出统一 SI：角速度 rad/s、加速度 m/s^2、时间 s；欧拉角对外同时给出
 *       rad 与 degree，degree 仅用于显示与既有调试习惯。
 * @note 无动态分配，单次更新运算量固定；本类不是线程安全的，属于单一所有者。
 */
class QuaternionEkf final
{
  public:
    /**
     * @brief 算法参数，默认值与前身的运行配置一致。
     */
    struct Config final
    {
        float process_noise_quaternion = 10.0F;     // Q1：四元数过程噪声方差率
        float process_noise_bias = 0.001F;          // Q2：零偏过程噪声方差率
        float measure_noise = 1.0e7F;               // R：加速度量测噪声方差
        float fading_factor = 1.0F;                 // 零偏方差渐消因子，大于 1 时按 1 处理
        float initial_quaternion_variance = 1.0e5F; // 四元数初值方差
        float initial_bias_variance = 100.0F;       // 零偏初值方差
        float initial_cross_variance = 0.1F;        // 初值协方差非对角项
    };

    /**
     * @brief 对外状态快照。
     */
    struct State final
    {
        math::Quaternion quaternion{};         // 机体系到世界系
        math::Vector3 gyro_bias_rad_per_sec{}; // 仅 x/y 被估计，z 恒为零
        math::Vector3 euler_radians{};         // ZYX：yaw、pitch、roll
        math::Vector3 euler_degrees{};         // 同上，单位 degree
        float yaw_total_degrees = 0.0F;        // 跨 ±180° 累计后的连续 yaw
        float chi_square = 0.0F;               // 最近一次卡方检验统计量
        float adaptive_gain_scale = 1.0F;      // 最近一次自适应增益缩放
        std::uint32_t update_count = 0U;       // 有效更新次数
        std::uint32_t divergence_count = 0U;   // 被判定为发散的次数
        bool converged = false;                // 卡方检验判定已收敛
        bool stable = false;                   // 最近一次静止判据结果
        bool valid = false;                    // 至少完成一次更新且状态有限
    };

    QuaternionEkf() noexcept = default;

    /**
     * @brief 用给定参数初始化滤波器并复位状态。
     * @return 参数有限且为正时返回 true；否则保持未初始化并返回 false。
     */
    [[nodiscard]] bool init(const Config &config) noexcept;

    /**
     * @brief 复位滤波器状态与统计，保留已设置的参数。
     */
    void reset() noexcept;

    /**
     * @brief 一次完整的预测 + 重力观测更新。
     * @param gyro_rad_per_sec 机体系角速度，单位 rad/s。
     * @param accel_meters_per_sec2 机体系加速度，单位 m/s^2。
     * @param dt_seconds 距上一次陀螺样本的时间间隔，单位 s。
     */
    void update(const math::Vector3 &gyro_rad_per_sec,
                const math::Vector3 &accel_meters_per_sec2,
                float dt_seconds) noexcept;

    /**
     * @brief 用外部静止标定结果注入零偏估计初值（仅 x/y 两轴）。
     * @param bias_rad_per_sec 机体系零偏，单位 rad/s。
     * @note 上电静止标定完成后一次性调用：滤波器随后只跟踪残余零偏，不会与外部
     *       修正重复抵消。四元数、协方差与收敛状态保持不变；z 轴没有对应状态，
     *       由调用方在输入侧扣除。非有限输入被忽略。
     */
    void seed_gyro_bias(const math::Vector3 &bias_rad_per_sec) noexcept;

    /**
     * @brief 仅预测，不进行重力观测更新。
     * @note 用于没有可信加速度样本的场合；协方差只做预测，不用旧增益修正。
     */
    void predict(const math::Vector3 &gyro_rad_per_sec, float dt_seconds) noexcept;

    [[nodiscard]] State get_state() const noexcept
    {
        return state_;
    }

    [[nodiscard]] bool is_initialized() const noexcept
    {
        return initialized_;
    }

    [[nodiscard]] const Config &get_config() const noexcept
    {
        return config_;
    }

    /** @brief 状态协方差对角线，顺序与状态向量一致，供诊断使用。 */
    void get_covariance_diagonal(float diagonal[6]) const noexcept;

  private:
    static constexpr std::size_t STATE_QUATERNION_INDEX{0U};
    static constexpr std::size_t STATE_BIAS_X_INDEX{4U};
    static constexpr std::size_t STATE_BIAS_Y_INDEX{5U};
    static constexpr std::size_t BIAS_STATE_COUNT{2U};

    using Filter = KalmanFilter<6U, 3U, 0U>;

    // 用户步骤：1 为状态转移线性化与协方差渐消，2 为观测矩阵，3 为量测校正。
    static void linearization_step(Filter &filter, void *context) noexcept;
    static void observation_step(Filter &filter, void *context) noexcept;
    static void correction_step(Filter &filter, void *context) noexcept;

    void advance(const math::Vector3 &gyro_rad_per_sec, float dt_seconds) noexcept;
    void prepare_prediction(const math::Vector3 &gyro_rad_per_sec, float dt_seconds) noexcept;
    void publish_state() noexcept;

    Filter filter_{};
    Config config_{};
    bool initialized_{false};
    bool correcting_{false};
    bool converged_{false};
    bool stable_{false};
    float chi_square_{};
    float adaptive_gain_scale_{1.0F};
    float step_seconds_{0.0F}; // 本轮步长，供线性化步骤使用
    float orientation_cosine_[3]{};
    std::uint32_t error_count_{0U};
    std::uint32_t divergence_count_{0U};
    std::uint32_t update_count_{0U};
    std::int32_t yaw_round_count_{0};
    float yaw_angle_last_{};
    State state_{};
};

} // namespace estimate

#endif
