#ifndef HAL_INIT_HPP
#define HAL_INIT_HPP

// 硬件契约：板级初始化（架构 v2 §3.1）。实现由 platform 的 port 提供。

namespace hal
{

/**
 * @brief 初始化本板的外设与 hal/os 的时基。
 * @return 全部成功返回 true；返回 false 时 boot_init() 失败。
 * @note 由组合根在 boot_init() 里调用一次：外设初始化之后、内核启动之前。
 *       返回之后 hal::os::now_us() 开始单调递增。
 */
[[nodiscard]] bool init() noexcept;

} // namespace hal

#endif // HAL_INIT_HPP
