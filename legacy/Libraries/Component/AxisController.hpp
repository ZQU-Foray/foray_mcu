#ifndef LIBRARIES_COMPONENT_AXIS_CONTROLLER_HPP
#define LIBRARIES_COMPONENT_AXIS_CONTROLLER_HPP

#include "Libraries/Algorithm/alg_controller/Pid.h"
#include "Libraries/Algorithm/alg_math/BasicMath.h"
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
  Torque = 0U,
  Speed,
  Angle
};

/** @brief 控制器级状态：回答"本拍这个控制器整体在做什么"。 */
enum class ControllerStatus : std::uint8_t
{
  NotReady = 0U, // 未完成 Init
  Disabled,      // 未使能
  InvalidDt,     // 本拍步长非法（非有限、≤0 或 ≥1 s）
  Active         // 已使能且步长合法，各轴状态见 AxisStatus
};

/**
 * @brief 单轴状态：回答"这一轴的输出为什么是 0（或为什么是开环）"。
 * @note 取代原先"输出 0 但外部不知原因"的静默降级：上层可以把该状态上抛到遥测
 *       与安全监督，而不必反推代码路径。
 */
enum class AxisStatus : std::uint8_t
{
  TorqueOpenLoop = 0U, // Torque 模式：目标钳位后直通（非故障）
  ClosedLoop,          // 闭环正常输出
  Disabled,            // 控制器级未就绪/未使能/步长非法
  FeedbackInvalid,     // 反馈无效、离线或量测非有限
  TargetInvalid,       // 目标非有限
  LoopFault            // PID 拒绝计算（内部数值溢出，状态已自动回溯）
};

/**
 * @brief 角度目标预处理策略（把"底盘不钳位、云台按机械范围钳位"变成数据）。
 * @note 只作用于 Angle 模式的角度目标；Speed 模式目标一律按 ±maxSpeedRpm 钳位。
 */
enum class TargetClamp : std::uint8_t
{
  None = 0U,    // 目标直通（无机械限位的执行器，例如车轮）
  AbsoluteRange // 钳到 [minTarget, maxTarget]（有机械限位的执行器，例如云台）
};

/**
 * @brief 命令失败（下发失败、总线异常）时的目标策略。
 * @note RoboLink 底盘目标立即归零、云台保持最后目标。差异必须由
 *       配置显式表达，不能靠复制粘贴继承。
 */
enum class FailurePolicy : std::uint8_t
{
  ClearTargets = 0U, // 复位环并清空目标（底盘）
  HoldTargets        // 仅复位环，保留目标（云台）
};

/**
 * @brief 单轴量测输入。
 * @note 刻意不直接使用 device::MotorState：本组件位于 Libraries/Algorithm，若依赖
 *       Libraries/Device 会形成算法层→设备层的斜向依赖，也会让宿主机测试必须带上
 *       设备头。应用层做一次 4 个字段的拷贝即可。
 */
struct AxisMeasurement final
{
  float angleDegrees{0.0F}; // 输出轴累计角度
  float speedRpm{0.0F};     // 输出轴转速
  bool feedbackValid{false};
  bool online{false};
};

/**
 * @brief 通用轴级闭环控制器：N 轴共用一份级联结构、限幅链、模式与诊断。
 *
 * 控制结构（全部为输出轴单位：角度 deg、转速 rpm、力矩 N·m）：
 *   Torque：目标 → 总限幅钳位 → 输出（开环直通，不经过 PID）；
 *   Speed ：输出轴转速环（rpm → N·m），目标钳 ±maxSpeedRpm；
 *   Angle ：角度环（deg → rpm）串接转速环（rpm → N·m），角度目标按 TargetClamp 预处理。
 *
 * 限幅链（两级，语义不同，实际输出取两者中更严格者）：
 *   - `AxisConfig::speedLoop.Maxout`：电调/热限幅（N·m），转速环输出的饱和值；
 *     通常是更紧的那一道（硬件约束，例如 C610 按 3 A 折算的 0.54 N·m）；
 *   - `Config::maxTorqueNewtonMeter`：策略总限幅（N·m），外部兜底。
 *   另有最终输出钳位仍按 maxTorqueNewtonMeter（与既有实现一致）。
 *   `AxisConfig::angleLoop.Maxout` 会被 `Config::maxSpeedRpm` 覆盖：外环输出限幅
 *   即内环目标上限，不给两处独立设定的机会。
 * @note 两级限幅都必须不超过执行器实际能力，否则控制器以为能给出的力矩是假的
 *       （GM6020 电流档满量程 3 A × 0.741 N·m/A = 2.223 N·m）。该能力校验由应用层
 *       用 `MotorGroup::MaximumTorqueNewtonMeter()` 完成，本组件不依赖设备层。
 *
 * 失败与降级：未就绪 / 未使能 / 步长非法 / 反馈无效 / 离线 / 量测非有限 / 目标非有限
 * 时该轴输出 0 并复位该轴 PID（不保留会导致重放旧输出的积分），状态见 GetAxisStatus。
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
    alg_controller::PID::Config speedLoop; // 输出轴 rpm → N·m（Maxout＝电调/热限幅）
    alg_controller::PID::Config angleLoop; // 输出轴 deg → rpm（Maxout 被 maxSpeedRpm 覆盖）
    TargetClamp clamp{TargetClamp::None};  // 角度目标预处理策略
    float minTarget{0.0F};                 // 仅 clamp == AbsoluteRange 时校验与使用
    float maxTarget{0.0F};                 // 同上
  };

  /** @brief 控制器配置。 */
  struct Config final
  {
    AxisConfig axis[AxisCount]{};                             // 每轴一份，互不影响
    float maxTorqueNewtonMeter{0.0F};                         // 策略总限幅（输出轴 N·m）
    float maxSpeedRpm{0.0F};                                  // 速度目标与外环输出限幅（输出轴 rpm）
    Mode initialMode{Mode::Speed};                            // 上电默认模式
    FailurePolicy failurePolicy{FailurePolicy::ClearTargets}; // 命令失败时的目标策略
  };

  AxisController() noexcept = default;

  /**
   * @brief 校验配置并初始化各轴 PID。
   * @return 配置全部有效时返回 true；否则返回 false 且控制器保持未就绪。
   * @note 校验项：总限幅与速度限幅为有限正数；`clamp == AbsoluteRange` 的轴必须
   *       有限且 `minTarget < maxTarget`（防止忘配置导致所有目标被静默钳到零）；
   *       各轴 PID 配置经 alg_controller::PID::Init 校验。两级限幅与执行器能力的比较不在此处，
   *       由应用层完成（见类注释）。
   * @note 失败时可能已部分初始化 PID（本组件只在启动期调用，失败即不 ready），
   *       为避免事务式初始化的复杂度不做回滚。
   */
  [[nodiscard]] bool Init(const Config &config) noexcept
  {
    Reset();
    mode_.store(config.initialMode, std::memory_order_relaxed);
    ready_ = false;
    controllerStatus_ = ControllerStatus::NotReady;
    for (std::size_t axis = 0U; axis < AxisCount; ++axis)
    {
      axisStatus_[axis] = AxisStatus::Disabled;
    }
    loopFaultCount_ = 0U;

    if (!std::isfinite(config.maxTorqueNewtonMeter) || !std::isfinite(config.maxSpeedRpm) ||
        config.maxTorqueNewtonMeter <= 0.0F || config.maxSpeedRpm <= 0.0F)
    {
      return false;
    }

    for (std::size_t axis = 0U; axis < AxisCount; ++axis)
    {
      const AxisConfig &axisConfig = config.axis[axis];
      if (axisConfig.clamp == TargetClamp::AbsoluteRange &&
          (!std::isfinite(axisConfig.minTarget) || !std::isfinite(axisConfig.maxTarget) ||
           axisConfig.minTarget >= axisConfig.maxTarget))
      {
        return false;
      }
      const float angleTargetMagnitude =
          axisConfig.clamp == TargetClamp::AbsoluteRange
              ? std::max(std::fabs(axisConfig.minTarget), std::fabs(axisConfig.maxTarget))
              : 0.0F;
      if (!FeedForwardInLinearRange(axisConfig.speedLoop, config.maxSpeedRpm) ||
          !FeedForwardInLinearRange(axisConfig.angleLoop, angleTargetMagnitude))
      {
        return false;
      }
    }

    config_ = config;
    for (std::size_t axis = 0U; axis < AxisCount; ++axis)
    {
      config_.axis[axis].angleLoop.Maxout = config.maxSpeedRpm;
      if (!speedPids_[axis].Init(config_.axis[axis].speedLoop) || !anglePids_[axis].Init(config_.axis[axis].angleLoop))
      {
        return false;
      }
    }

    ready_ = true;
    return true;
  }

  /** @brief 复位全部轴 PID 并清空目标（含模式切换）；保留配置、使能与模式。 */
  void Reset() noexcept
  {
    ResetLoops();
    ClearTargets();
  }

  /** @brief 只复位各轴 PID 状态（含积分），保留目标。 */
  void ResetLoops() noexcept
  {
    for (std::size_t axis = 0U; axis < AxisCount; ++axis)
    {
      speedPids_[axis].Reset();
      anglePids_[axis].Reset();
    }
  }

  /** @brief 只清空各轴目标，保留 PID 状态。 */
  void ClearTargets() noexcept
  {
    for (std::size_t axis = 0U; axis < AxisCount; ++axis)
    {
      targets_[axis].store(0.0F, std::memory_order_relaxed);
    }
  }

  /**
   * @brief 命令下发失败时的统一处理：按 failurePolicy 复位环与目标。
   * @note `ClearTargets`（底盘）：复位环 + 清目标，上层需重新下发；`HoldTargets`
   *       （云台）：仅复位环，保留目标
   */
  void OnCommandFailure() noexcept
  {
    ResetLoops();
    if (config_.failurePolicy == FailurePolicy::ClearTargets)
    {
      ClearTargets();
    }
  }

  void SetEnabled(bool enabled) noexcept
  {
    enabled_.store(enabled, std::memory_order_release);
  }

  /** @brief 切换控制模式（含切回原模式）：复位全部 PID 并清空目标。 */
  void SetMode(Mode mode) noexcept
  {
    Reset();
    mode_.store(mode, std::memory_order_release);
  }

  /** @brief 设定单轴目标（语义随模式：Torque→N·m，Speed→输出轴 rpm，Angle→deg）。 */
  void SetTarget(std::size_t axis, float value) noexcept
  {
    if (axis >= AxisCount)
    {
      return;
    }
    targets_[axis].store(value, std::memory_order_relaxed);
  }

  [[nodiscard]] bool IsReady() const noexcept
  {
    return ready_;
  }

  [[nodiscard]] bool IsEnabled() const noexcept
  {
    return enabled_.load(std::memory_order_acquire);
  }

  [[nodiscard]] Mode GetMode() const noexcept
  {
    return mode_.load(std::memory_order_acquire);
  }

  [[nodiscard]] float GetTarget(std::size_t axis) const noexcept
  {
    if (axis >= AxisCount)
    {
      return 0.0F;
    }
    return targets_[axis].load(std::memory_order_relaxed);
  }

  /** @brief 最近一次 Update 的控制器级状态。 */
  [[nodiscard]] ControllerStatus GetStatus() const noexcept
  {
    return controllerStatus_;
  }

  /**
   * @brief 最近一次 Update 的单轴状态。
   * @return 下标合法返回 true 并写出状态；越界返回 false。
   */
  [[nodiscard]] bool GetAxisStatus(std::size_t axis, AxisStatus &status) const noexcept
  {
    if (axis >= AxisCount)
    {
      return false;
    }
    status = axisStatus_[axis];
    return true;
  }

  /** @brief 累计 PID 拒绝计算（LoopFault）次数，供上层区分瞬时扰动与持续故障。 */
  [[nodiscard]] std::uint32_t GetLoopFaultCount() const noexcept
  {
    return loopFaultCount_;
  }

  /**
   * @brief 推进一个控制周期。
   * @param measurements 各轴量测（下标即轴序）。
   * @param dt 本周期时长（秒），须满足 0 < dt < 1。
   * @param outTorqueNewtonMeter 输出力矩指令，恒为有限值。
   */
  void
  Update(const AxisMeasurement (&measurements)[AxisCount], float dt, float (&outTorqueNewtonMeter)[AxisCount]) noexcept
  {
    const bool dtValid = std::isfinite(dt) && dt > 0.0F && dt < 1.0F;
    controllerStatus_ = !ready_ ? ControllerStatus::NotReady
                                : (!IsEnabled() ? ControllerStatus::Disabled
                                                : (dtValid ? ControllerStatus::Active : ControllerStatus::InvalidDt));
    const bool active = (controllerStatus_ == ControllerStatus::Active);
    const Mode mode = GetMode();

    for (std::size_t axis = 0U; axis < AxisCount; ++axis)
    {
      outTorqueNewtonMeter[axis] = 0.0F;
      const auto resetAxis = [&]() noexcept
      {
        speedPids_[axis].Reset();
        anglePids_[axis].Reset();
      };

      if (!active)
      {
        axisStatus_[axis] = AxisStatus::Disabled;
        resetAxis();
        continue;
      }

      const AxisMeasurement &measurement = measurements[axis];
      if (!measurement.feedbackValid || !measurement.online || !std::isfinite(measurement.angleDegrees) ||
          !std::isfinite(measurement.speedRpm))
      {
        axisStatus_[axis] = AxisStatus::FeedbackInvalid;
        resetAxis();
        continue;
      }

      const float target = targets_[axis].load(std::memory_order_relaxed);
      if (!std::isfinite(target))
      {
        axisStatus_[axis] = AxisStatus::TargetInvalid;
        resetAxis();
        continue;
      }

      if (mode == Mode::Torque)
      {
        resetAxis();
        outTorqueNewtonMeter[axis] =
            alg_math::Limit(target, -config_.maxTorqueNewtonMeter, config_.maxTorqueNewtonMeter);
        axisStatus_[axis] = AxisStatus::TorqueOpenLoop;
        continue;
      }

      float targetRpm = 0.0F;
      if (mode == Mode::Speed)
      {
        anglePids_[axis].Reset();
        targetRpm = alg_math::Limit(target, -config_.maxSpeedRpm, config_.maxSpeedRpm);
      }
      else
      {
        const float targetAngle = ClampAngleTarget(axis, target);
        if (!anglePids_[axis].CalculateLoop(measurement.angleDegrees, targetAngle, dt))
        {
          axisStatus_[axis] = AxisStatus::LoopFault;
          ++loopFaultCount_;
          resetAxis();
          continue;
        }
        targetRpm = alg_math::Limit(anglePids_[axis].GetOut(), -config_.maxSpeedRpm, config_.maxSpeedRpm);
      }

      if (!speedPids_[axis].CalculateLoop(measurement.speedRpm, targetRpm, dt))
      {
        axisStatus_[axis] = AxisStatus::LoopFault;
        ++loopFaultCount_;
        resetAxis();
        continue;
      }
      outTorqueNewtonMeter[axis] =
          alg_math::Limit(speedPids_[axis].GetOut(), -config_.maxTorqueNewtonMeter, config_.maxTorqueNewtonMeter);
      axisStatus_[axis] = AxisStatus::ClosedLoop;
    }
  }

  /** @brief 读取单轴 PID 输出，供诊断与等价性测试使用（不改变任何状态）。 */
  [[nodiscard]] bool GetLoopOutput(std::size_t axis, float &speedLoopOutput, float &angleLoopOutput) const noexcept
  {
    if (axis >= AxisCount)
    {
      return false;
    }
    speedLoopOutput = speedPids_[axis].GetOut();
    angleLoopOutput = anglePids_[axis].GetOut();
    return true;
  }

private:
  /**
   * @brief 校验前馈增益在整个目标范围内都工作在线性区，避免它退化为恒定偏置。
   * @param loopConfig 待校验环路的 PID 配置。
   * @param targetMagnitude 该环路目标绝对值上限；为 0 表示目标范围无界，此时只允许 Kf 为 0。
   * @return Kf 为 0，或 `|Kf| × targetMagnitude` 不超过前馈独立限幅时返回 true。
   */
  [[nodiscard]] static bool FeedForwardInLinearRange(const alg_controller::PID::Config &loopConfig,
                                                     float targetMagnitude) noexcept
  {
    if (loopConfig.Kf == 0.0F)
    {
      return true;
    }
    return targetMagnitude > 0.0F && std::fabs(loopConfig.Kf) * targetMagnitude <= loopConfig.FeedForwardLimit;
  }

  /** @brief 按该轴策略预处理角度目标（Angle 模式专用）。 */
  [[nodiscard]] float ClampAngleTarget(std::size_t axis, float target) const noexcept
  {
    if (config_.axis[axis].clamp != TargetClamp::AbsoluteRange)
    {
      return target;
    }
    return alg_math::Limit(target, config_.axis[axis].minTarget, config_.axis[axis].maxTarget);
  }

  bool ready_{false};
  std::atomic<bool> enabled_{false};
  std::atomic<Mode> mode_{Mode::Torque};
  std::array<std::atomic<float>, AxisCount> targets_{};
  std::array<alg_controller::PID, AxisCount> speedPids_{};
  std::array<alg_controller::PID, AxisCount> anglePids_{};
  Config config_{};
  ControllerStatus controllerStatus_{ControllerStatus::NotReady};
  std::array<AxisStatus, AxisCount> axisStatus_{};
  std::uint32_t loopFaultCount_{0U};
};

} // namespace component

#endif // LIBRARIES_COMPONENT_AXIS_CONTROLLER_HPP
