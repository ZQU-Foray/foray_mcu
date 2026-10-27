#ifndef APPLICATION_TASK_TASK_MANAGER_HPP
#define APPLICATION_TASK_TASK_MANAGER_HPP

#include <cstdint>

namespace application::task
{

/**
 * @brief 应用任务生命周期管理器：把"创建哪些任务、什么时机创建、失败在哪一步"
 *        集中到一处，取代散落在启动路径里的三组属性字面量与三连 osThreadNew。
 *
 * 启动口径：
 *   Application_Init     → TaskManager::Init      核前：校验平台前置条件并初始化各任务的
 *                                                 非 RTOS 部分，不创建任何 RTOS 对象；
 *   Application_InitRtos → TaskManager::init_rtos  osKernelInitialize 之后、osKernelStart
 *                                                 之前：按任务表创建全部应用线程；
 *   Application_Start    → TaskManager::Start     内核运行后：确认任务表就绪并推进到 Running。
 *
 * @note 三个入口按序执行且幂等：重复调用不会重复创建线程，也不会重复初始化外设会话。
 * @note 容错：Start 首次调用时若发现线程尚未创建，会补做 init_rtos（创建时机记为
 *       POST_KERNEL）——目的是避免漏接启动路径一行就导致整机静默不启动。
 * @note 失败环节经 get_diagnostics 暴露，不再由 `||` 短路吞掉；Error 状态不自动恢复，
 *       需复位后重新走启动路径。
 * @note Init/init_rtos 属于单线程启动路径，不是线程安全的；Start 允许被默认任务周期
 *       调用（幂等、无副作用、不阻塞）。
 */
class TaskManager final
{
  public:
    /** @brief 任务栈余量巡检的采样能力上界，与任务表登记数一致。 */
    static constexpr std::uint32_t MAXIMUM_SAMPLED_TASKS{3U};

    /** @brief StackHealth::low_margin_task_index 的"余量充足"取值。 */
    static constexpr std::uint32_t MARGIN_OK_INDEX{0xFFFFFFFFU};

    /**
   * @brief 任务栈余量巡检快照：回答"哪个任务离栈溢出还有多远"。
   * @note 数值是 FreeRTOS 高水位统计的**剩余栈**，单位字（32 位机上一字为 4 字节）。
   *       本工程 `configCHECK_FOR_STACK_OVERFLOW=2` 的溢出钩子位于 CubeMX 生成文件
   *       （P0 禁止修改）且为空实现，溢出不会停机，因此必须靠本快照在溢出前发现。
   */
    struct StackHealth final
    {
        std::uint32_t sample_count{0U};                       // 已完成的采样轮数（约 1 Hz）
        std::uint32_t sampled_task_count{0U};                 // 本轮实际采样的任务数
        std::uint32_t minimum_free_words{0U};                 // 全部任务中最小的剩余栈（字）
        std::uint32_t low_margin_task_index{MARGIN_OK_INDEX}; // 余量不足的任务下标，充足时为 MARGIN_OK_INDEX
        std::uint32_t free_words[MAXIMUM_SAMPLED_TASKS]{};    // 与任务表同序的剩余栈（字）
    };
    // 调试器与脚本按字节偏移解析本快照，布局被锁定。
    static_assert(sizeof(StackHealth) == 28U, "栈余量快照布局必须稳定");

    /** @brief 启动状态机：任一环节失败即进入 Error。 */
    enum class State : std::uint8_t
    {
        UNINITIALIZED = 0U, // 尚未调用 Init
        PRE_KERNEL_READY,   // Init 完成：平台前置条件通过，未创建 RTOS 对象
        RTOS_READY,         // init_rtos 完成：全部应用线程已创建，等待内核接管
        RUNNING,            // Start 完成：内核已在运行且任务表就绪
        ERROR               // 初始化或线程创建失败，需复位后重试
    };

    /**
   * @brief 应用线程的创建时机，用于确认启动口径是否按设计接线。
   * @note POST_KERNEL 表示 init_rtos 是在内核启动之后补做的（例如启动路径上只有默认
   *       任务一个接入点时）：功能等价，但不满足"osKernelStart 之前创建"的口径，
   *       应作为接线缺陷处理。
   */
    enum class RtosInitTiming : std::uint8_t
    {
        NONE = 0U,
        PRE_KERNEL,
        POST_KERNEL
    };

    /** @brief 首个失败环节：取代 `||` 短路导致的"只知失败、不知何处失败"。 */
    enum class FailureStage : std::uint8_t
    {
        NONE = 0U,
        PLATFORM, // 平台前置条件（时基等）
        IMU_TASK,
        CHASSIS_TASK,
        GIMBAL_TASK,
        THREAD_CREATION
    };

    /** @brief 启动诊断快照，供启动日志、调试器与验收脚本读取。 */
    struct Diagnostics final
    {
        State state{State::UNINITIALIZED};
        RtosInitTiming rtos_init_timing{RtosInitTiming::NONE};
        FailureStage failure{FailureStage::NONE};
        std::uint32_t expected_task_count{0U}; // 任务表登记的应用任务数
        std::uint32_t created_task_count{0U};  // 已成功创建的线程数
        std::uint32_t start_refused_count{0U}; // Start 在任务表未就绪时被调用的次数（接线错误可观测）
    };

    TaskManager() = delete;

    /**
   * @brief 核前初始化：校验平台前置条件，逐个初始化各任务的非 RTOS 部分。
   * @return 全部通过返回 true；否则置 Error 并记录失败环节。
   * @note 本函数禁止创建线程或依赖内核状态的对象。
   */
    [[nodiscard]] static bool init() noexcept;

    /**
   * @brief 按任务表创建全部应用线程，只允许调用一次。
   * @return 全部线程创建成功返回 true；否则置 Error 并记录失败环节。
   * @note 未完成核前初始化时先补做 Init（幂等），避免"漏调 Init"退化成零任务运行。
   */
    [[nodiscard]] static bool init_rtos() noexcept;

    /**
   * @brief 内核运行后确认启动：任务表就绪时推进到 Running。
   * @return 已处于 Running 时返回 true（幂等）；任务表未就绪时返回 false 并计数。
   */
    [[nodiscard]] static bool start() noexcept;

    [[nodiscard]] static State get_state() noexcept;

    [[nodiscard]] static Diagnostics get_diagnostics() noexcept;

    /**
   * @brief 采样各任务栈的高水位余量并写入调试器可见存档，内部按 1 Hz 限频。
   * @note 允许每个任务节拍调用（限频后绝大多数调用立即返回）。任务表就绪前不做任何事。
   * @note 之所以由应用层自采样：栈溢出钩子在 CubeMX 生成文件里且为空实现，P0 禁止修改；
   *       本接口用 FreeRTOS 的高水位统计把余量暴露给调试器与验收脚本，使溢出可见。
   */
    static void sample_stack_health() noexcept;

    /**
   * @brief 读取最近一次栈余量巡检快照。
   * @return 尚未采样时返回全零快照（sample_count 为 0），调用方据此区分"未采样"与"余量 0"。
   */
    [[nodiscard]] static StackHealth get_stack_health() noexcept;
};

} // namespace application::task

#endif // APPLICATION_TASK_TASK_MANAGER_HPP
