#ifndef LIB_COMPONENT_AXIS_CONTROLLER_HPP
#define LIB_COMPONENT_AXIS_CONTROLLER_HPP

#include "lib/alg_controller/pid.hpp"
#include "lib/alg_math/basic_math.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace component
{

/**
 * @brief 轴级控制模式：力矩开环 / 转速环 / 角度-转速串级。
 * @note 底盘与云台共用同一枚举，避免两份同内容的模式定义各自漂移。
 */
enum class AxisControlMode : std::uint8_t
{
    TORQUE = 0U,
    SPEED,
    ANGLE
};

/** @brief 控制器级状态：回答"本拍这个控制器整体在做什么"。 */
enum class ControllerStatus : std::uint8_t
{
    NOT_READY = 0U, // 未完成 Init
    DISABLED,       // 未使能
    INVALID_DT,     // 本拍步长非法（非有限、≤0 或 ≥1 s）
    ACTIVE          // 已使能且步长合法，各轴状态见 AxisStatus
};

/**
 * @brief 单轴状态：回答"这一轴的输出为什么是 0（或为什么是开环）"。
 * @note 取代原先"输出 0 但外部不知原因"的静默降级：上层可以把该状态上抛到遥测
 *       与安全监督，而不必反推代码路径。
 */
enum class AxisStatus : std::uint8_t
{
    TORQUE_OPEN_LOOP = 0U, // Torque 模式：目标钳位后直通（非故障）
    CLOSED_LOOP,           // 闭环正常输出
    DISABLED,              // 控制器级未就绪/未使能/步长非法
    FEEDBACK_INVALID,      // 反馈无效、离线或量测非有限
    TARGET_INVALID,        // 目标非有限
    LOOP_FAULT             // PID 拒绝计算（内部数值溢出，状态已自动回溯）
};

/**
 * @brief 角度目标预处理策略（把"底盘不钳位、云台按机械范围钳位"变成数据）。
 * @note 只作用于 Angle 模式的角度目标；Speed 模式目标一律按 ±max_speed_rpm 钳位。
 */
enum class TargetClamp : std::uint8_t
{
    NONE = 0U,     // 目标直通（无机械限位的执行器，例如车轮）
    ABSOLUTE_RANGE // 钳到 [min_target, max_target]（有机械限位的执行器，例如云台）
};

/**
 * @brief 命令失败（下发失败、总线异常）时的目标策略。
 * @note RoboLink 底盘目标立即归零、云台保持最后目标。差异必须由
 *       配置显式表达，不能靠复制粘贴继承。
 */
enum class FailurePolicy : std::uint8_t
{
    CLEAR_TARGETS = 0U, // 复位环并清空目标（底盘）
    HOLD_TARGETS        // 仅复位环，保留目标（云台）
};

/**
 * @brief 单轴量测输入。
 * @note 刻意不直接使用 device::MotorState：本组件位于 Libraries/Algorithm，若依赖
 *       Libraries/Device 会形成算法层→设备层的斜向依赖，也会让宿主机测试必须带上
 *       设备头。应用层做一次 4 个字段的拷贝即可。
 */
struct AxisMeasurement final
{
    float angle_degrees{0.0F}; // 输出轴累计角度
    float speed_rpm{0.0F};     // 输出轴转速
    bool feedback_valid{false};
    bool online{false};
};

/**
 * @brief 通用轴级闭环控制器：N 轴共用一份级联结构、限幅链、模式与诊断。
 *
 * 控制结构（全部为输出轴单位：角度 deg、转速 rpm、力矩 N·m）：
 *   Torque：目标 → 总限幅钳位 → 输出（开环直通，不经过 PID）；
 *   Speed ：输出轴转速环（rpm → N·m），目标钳 ±max_speed_rpm；
 *   Angle ：角度环（deg → rpm）串接转速环（rpm → N·m），角度目标按 TargetClamp 预处理。
 *
 * 限幅链（两级，语义不同，实际输出取两者中更严格者）：
 *   - `AxisConfig::speed_loop.Maxout`：电调/热限幅（N·m），转速环输出的饱和值；
 *     通常是更紧的那一道（硬件约束，例如 C610 按 3 A 折算的 0.54 N·m）；
 *   - `Config::max_torque_newton_meter`：策略总限幅（N·m），外部兜底。
 *   另有最终输出钳位仍按 max_torque_newton_meter（与既有实现一致）。
 *   `AxisConfig::angle_loop.Maxout` 会被 `Config::max_speed_rpm` 覆盖：外环输出限幅
 *   即内环目标上限，不给两处独立设定的机会。
 * @note 两级限幅都必须不超过执行器实际能力，否则控制器以为能给出的力矩是假的
 *       （GM6020 电流档满量程 3 A × 0.741 N·m/A = 2.223 N·m）。该能力校验由应用层
 *       用 `MotorGroup::maximum_torque_newton_meter()` 完成，本组件不依赖设备层。
 *
 * 失败与降级：未就绪 / 未使能 / 步长非法 / 反馈无效 / 离线 / 量测非有限 / 目标非有限
 * 时该轴输出 0 并复位该轴 PID（不保留会导致重放旧输出的积分），状态见 get_axis_status。
 * 上层据此把"输出 0"的原因上抛，而不是只知道结果是 0。
 *
 * @note 纯策略对象：不访问硬件、不读取时间、不依赖 RTOS；步长由调用方提供，因此
 *       可在宿主机上闭环仿真（与 PID 的既有约定一致）。
 * @note 目标值、使能位与模式为原子量，允许控制任务之外的单写者设定（遥控/决策任务）。
 * @note 已知限制：模式与目标是两个独立原子量，跨任务"先设目标再切模式"存在撕裂窗口，
 *       打包成单原子快照留待控制权裁决层统一处理。
 * @note 角度目标必须与量测处于同一累计角系（多圈累计角）；跨圈归算与最短路径属于
 *       目标解算层的职责，本组件不做隐式 wrap。
 *
 * @tparam AxisCount 轴数（底盘 4、云台 2），编译期固定，无动态分配。
 */
template <std::size_t AxisCount> class AxisController final
{
    static_assert(AxisCount > 0U, "轴数必须大于零");
    static_assert(std::atomic<float>::is_always_lock_free, "轴目标需要使用无锁的 float 原子变量");
    static_assert(std::atomic<AxisControlMode>::is_always_lock_free, "控制模式需要使用无锁的枚举原子变量");

  public:
    using Mode = AxisControlMode;

    /** @brief 单轴配置。 */
    struct AxisConfig final
    {
        alg_controller::PID::Config speed_loop; // 输出轴 rpm → N·m（Maxout＝电调/热限幅）
        alg_controller::PID::Config angle_loop; // 输出轴 deg → rpm（Maxout 被 max_speed_rpm 覆盖）
        TargetClamp clamp{TargetClamp::NONE};   // 角度目标预处理策略
        float min_target{0.0F};                 // 仅 clamp == ABSOLUTE_RANGE 时校验与使用
        float max_target{0.0F};                 // 同上
    };

    /** @brief 控制器配置。 */
    struct Config final
    {
        AxisConfig axis[AxisCount]{};        // 每轴一份，互不影响
        float max_torque_newton_meter{0.0F}; // 策略总限幅（输出轴 N·m）
        float max_speed_rpm{0.0F};           // 速度目标与外环输出限幅（输出轴 rpm）
        Mode initial_mode{Mode::SPEED};      // 上电默认模式
        FailurePolicy failure_policy{FailurePolicy::CLEAR_TARGETS}; // 命令失败时的目标策略
    };

    AxisController() noexcept = default;

    /**
   * @brief 校验配置并初始化各轴 PID。
   * @return 配置全部有效时返回 true；否则返回 false 且控制器保持未就绪。
   * @note 校验项：总限幅与速度限幅为有限正数；`clamp == ABSOLUTE_RANGE` 的轴必须
   *       有限且 `min_target < max_target`（防止忘配置导致所有目标被静默钳到零）；
   *       各轴 PID 配置经 alg_controller::PID::Init 校验。两级限幅与执行器能力的比较不在此处，
   *       由应用层完成（见类注释）。
   * @note 失败时可能已部分初始化 PID（本组件只在启动期调用，失败即不 ready），
   *       为避免事务式初始化的复杂度不做回滚。
   */
    [[nodiscard]] bool init(const Config &config) noexcept
    {
        reset();
        mode_.store(config.initial_mode, std::memory_order_relaxed);
        ready_ = false;
        controller_status_ = ControllerStatus::NOT_READY;
        for (std::size_t axis = 0U; axis < AxisCount; ++axis)
        {
            axis_status_[axis] = AxisStatus::DISABLED;
        }
        loop_fault_count_ = 0U;

        if (!std::isfinite(config.max_torque_newton_meter) || !std::isfinite(config.max_speed_rpm) ||
            config.max_torque_newton_meter <= 0.0F || config.max_speed_rpm <= 0.0F)
        {
            return false;
        }

        for (std::size_t axis = 0U; axis < AxisCount; ++axis)
        {
            const AxisConfig &axis_config = config.axis[axis];
            if (axis_config.clamp == TargetClamp::ABSOLUTE_RANGE &&
                (!std::isfinite(axis_config.min_target) || !std::isfinite(axis_config.max_target) ||
                 axis_config.min_target >= axis_config.max_target))
            {
                return false;
            }
            const float angle_target_magnitude =
                axis_config.clamp == TargetClamp::ABSOLUTE_RANGE
                    ? std::max(std::fabs(axis_config.min_target), std::fabs(axis_config.max_target))
                    : 0.0F;
            if (!feed_forward_in_linear_range(axis_config.speed_loop, config.max_speed_rpm) ||
                !feed_forward_in_linear_range(axis_config.angle_loop, angle_target_magnitude))
            {
                return false;
            }
        }

        config_ = config;
        for (std::size_t axis = 0U; axis < AxisCount; ++axis)
        {
            config_.axis[axis].angle_loop.maxout = config.max_speed_rpm;
            if (!speed_pids_[axis].init(config_.axis[axis].speed_loop) ||
                !angle_pids_[axis].init(config_.axis[axis].angle_loop))
            {
                return false;
            }
        }

        ready_ = true;
        return true;
    }

    /** @brief 复位全部轴 PID 并清空目标（含模式切换）；保留配置、使能与模式。 */
    void reset() noexcept
    {
        reset_loops();
        clear_targets();
    }

    /** @brief 只复位各轴 PID 状态（含积分），保留目标。 */
    void reset_loops() noexcept
    {
        for (std::size_t axis = 0U; axis < AxisCount; ++axis)
        {
            speed_pids_[axis].reset();
            angle_pids_[axis].reset();
        }
    }

    /** @brief 只清空各轴目标，保留 PID 状态。 */
    void clear_targets() noexcept
    {
        for (std::size_t axis = 0U; axis < AxisCount; ++axis)
        {
            targets_[axis].store(0.0F, std::memory_order_relaxed);
        }
    }

    /**
   * @brief 命令下发失败时的统一处理：按 failure_policy 复位环与目标。
   * @note `ClearTargets`（底盘）：复位环 + 清目标，上层需重新下发；`HOLD_TARGETS`
   *       （云台）：仅复位环，保留目标
   */
    void on_command_failure() noexcept
    {
        reset_loops();
        if (config_.failure_policy == FailurePolicy::CLEAR_TARGETS)
        {
            clear_targets();
        }
    }

    void set_enabled(bool enabled) noexcept
    {
        enabled_.store(enabled, std::memory_order_release);
    }

    /** @brief 切换控制模式（含切回原模式）：复位全部 PID 并清空目标。 */
    void set_mode(Mode mode) noexcept
    {
        reset();
        mode_.store(mode, std::memory_order_release);
    }

    /** @brief 设定单轴目标（语义随模式：Torque→N·m，Speed→输出轴 rpm，Angle→deg）。 */
    void set_target(std::size_t axis, float value) noexcept
    {
        if (axis >= AxisCount)
        {
            return;
        }
        targets_[axis].store(value, std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_ready() const noexcept
    {
        return ready_;
    }

    [[nodiscard]] bool is_enabled() const noexcept
    {
        return enabled_.load(std::memory_order_acquire);
    }

    [[nodiscard]] Mode get_mode() const noexcept
    {
        return mode_.load(std::memory_order_acquire);
    }

    [[nodiscard]] float get_target(std::size_t axis) const noexcept
    {
        if (axis >= AxisCount)
        {
            return 0.0F;
        }
        return targets_[axis].load(std::memory_order_relaxed);
    }

    /** @brief 最近一次 Update 的控制器级状态。 */
    [[nodiscard]] ControllerStatus get_status() const noexcept
    {
        return controller_status_;
    }

    /**
   * @brief 最近一次 Update 的单轴状态。
   * @return 下标合法返回 true 并写出状态；越界返回 false。
   */
    [[nodiscard]] bool get_axis_status(std::size_t axis, AxisStatus &status) const noexcept
    {
        if (axis >= AxisCount)
        {
            return false;
        }
        status = axis_status_[axis];
        return true;
    }

    /** @brief 累计 PID 拒绝计算（LOOP_FAULT）次数，供上层区分瞬时扰动与持续故障。 */
    [[nodiscard]] std::uint32_t get_loop_fault_count() const noexcept
    {
        return loop_fault_count_;
    }

    /**
   * @brief 推进一个控制周期。
   * @param measurements 各轴量测（下标即轴序）。
   * @param dt 本周期时长（秒），须满足 0 < dt < 1。
   * @param out_torque_newton_meter 输出力矩指令，恒为有限值。
   */
    void update(const AxisMeasurement (&measurements)[AxisCount],
                float dt,
                float (&out_torque_newton_meter)[AxisCount]) noexcept
    {
        const bool dt_valid = std::isfinite(dt) && dt > 0.0F && dt < 1.0F;
        controller_status_ =
            !ready_ ? ControllerStatus::NOT_READY
                    : (!is_enabled() ? ControllerStatus::DISABLED
                                     : (dt_valid ? ControllerStatus::ACTIVE : ControllerStatus::INVALID_DT));
        const bool active = (controller_status_ == ControllerStatus::ACTIVE);
        const Mode mode = get_mode();

        for (std::size_t axis = 0U; axis < AxisCount; ++axis)
        {
            out_torque_newton_meter[axis] = 0.0F;
            const auto reset_axis = [&]() noexcept
            {
                speed_pids_[axis].reset();
                angle_pids_[axis].reset();
            };

            if (!active)
            {
                axis_status_[axis] = AxisStatus::DISABLED;
                reset_axis();
                continue;
            }

            const AxisMeasurement &measurement = measurements[axis];
            if (!measurement.feedback_valid || !measurement.online || !std::isfinite(measurement.angle_degrees) ||
                !std::isfinite(measurement.speed_rpm))
            {
                axis_status_[axis] = AxisStatus::FEEDBACK_INVALID;
                reset_axis();
                continue;
            }

            const float target = targets_[axis].load(std::memory_order_relaxed);
            if (!std::isfinite(target))
            {
                axis_status_[axis] = AxisStatus::TARGET_INVALID;
                reset_axis();
                continue;
            }

            if (mode == Mode::TORQUE)
            {
                reset_axis();
                out_torque_newton_meter[axis] =
                    alg_math::limit(target, -config_.max_torque_newton_meter, config_.max_torque_newton_meter);
                axis_status_[axis] = AxisStatus::TORQUE_OPEN_LOOP;
                continue;
            }

            float target_rpm = 0.0F;
            if (mode == Mode::SPEED)
            {
                angle_pids_[axis].reset();
                target_rpm = alg_math::limit(target, -config_.max_speed_rpm, config_.max_speed_rpm);
            }
            else
            {
                const float target_angle = clamp_angle_target(axis, target);
                if (!angle_pids_[axis].calculate_loop(measurement.angle_degrees, target_angle, dt))
                {
                    axis_status_[axis] = AxisStatus::LOOP_FAULT;
                    ++loop_fault_count_;
                    reset_axis();
                    continue;
                }
                target_rpm =
                    alg_math::limit(angle_pids_[axis].get_out(), -config_.max_speed_rpm, config_.max_speed_rpm);
            }

            if (!speed_pids_[axis].calculate_loop(measurement.speed_rpm, target_rpm, dt))
            {
                axis_status_[axis] = AxisStatus::LOOP_FAULT;
                ++loop_fault_count_;
                reset_axis();
                continue;
            }
            out_torque_newton_meter[axis] = alg_math::limit(
                speed_pids_[axis].get_out(), -config_.max_torque_newton_meter, config_.max_torque_newton_meter);
            axis_status_[axis] = AxisStatus::CLOSED_LOOP;
        }
    }

    /** @brief 读取单轴 PID 输出，供诊断与等价性测试使用（不改变任何状态）。 */
    [[nodiscard]] bool
    get_loop_output(std::size_t axis, float &speed_loop_output, float &angle_loop_output) const noexcept
    {
        if (axis >= AxisCount)
        {
            return false;
        }
        speed_loop_output = speed_pids_[axis].get_out();
        angle_loop_output = angle_pids_[axis].get_out();
        return true;
    }

  private:
    /**
   * @brief 校验前馈增益在整个目标范围内都工作在线性区，避免它退化为恒定偏置。
   * @param loop_config 待校验环路的 PID 配置。
   * @param target_magnitude 该环路目标绝对值上限；为 0 表示目标范围无界，此时只允许 Kf 为 0。
   * @return Kf 为 0，或 `|Kf| × target_magnitude` 不超过前馈独立限幅时返回 true。
   */
    [[nodiscard]] static bool feed_forward_in_linear_range(const alg_controller::PID::Config &loop_config,
                                                           float target_magnitude) noexcept
    {
        if (loop_config.kf == 0.0F)
        {
            return true;
        }
        return target_magnitude > 0.0F &&
               std::fabs(loop_config.kf) * target_magnitude <= loop_config.feed_forward_limit;
    }

    /** @brief 按该轴策略预处理角度目标（Angle 模式专用）。 */
    [[nodiscard]] float clamp_angle_target(std::size_t axis, float target) const noexcept
    {
        if (config_.axis[axis].clamp != TargetClamp::ABSOLUTE_RANGE)
        {
            return target;
        }
        return alg_math::limit(target, config_.axis[axis].min_target, config_.axis[axis].max_target);
    }

    bool ready_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<Mode> mode_{Mode::TORQUE};
    std::array<std::atomic<float>, AxisCount> targets_{};
    std::array<alg_controller::PID, AxisCount> speed_pids_{};
    std::array<alg_controller::PID, AxisCount> angle_pids_{};
    Config config_{};
    ControllerStatus controller_status_{ControllerStatus::NOT_READY};
    std::array<AxisStatus, AxisCount> axis_status_{};
    std::uint32_t loop_fault_count_{0U};
};

} // namespace component

#endif // LIB_COMPONENT_AXIS_CONTROLLER_HPP
