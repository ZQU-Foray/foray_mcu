#ifndef HAL_GPIO_HPP
#define HAL_GPIO_HPP

#include "hal/callback.hpp"

#include <cstdint>

// 硬件契约：GPIO 引脚（Decision-20261101-01）。句柄由各板的 board.hpp 给出，实现由 port 提供，
// 宿主测试由 platform/mock 提供。

namespace hal
{

/**
 * @brief 一个 GPIO 引脚的句柄。
 *
 * 能力按引脚区分，各板的 board.hpp 写明每个引脚支持什么：
 * - 必选：read()；
 * - 只有输出引脚支持 write()、toggle()，其他引脚返回 false；
 * - 只有接了外部中断的引脚支持 set_interrupt()，其他引脚返回 false。触发边沿由板级配置决定。
 *
 * 句柄只含一个编号，可以拷贝，按值存进驱动的配置。编号只有 port 与 mock 能读。
 */
class Gpio final
{
  public:
    enum class Level : std::uint8_t
    {
        LOW = 0U,
        HIGH = 1U
    };

    constexpr explicit Gpio(std::uint8_t index) noexcept : index_{index} {}

    /**
     * @brief 读引脚电平（输出引脚读到的是正在输出的电平）。
     * @return 编号无效时返回 LOW。
     * @note 任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] Level read() const noexcept;

    /**
     * @brief 设置输出电平。
     * @return 不是输出引脚时返回 false，电平不变。
     * @note 任务与框架中断里都能调用，不阻塞。一个引脚只应有一个写入方。
     */
    [[nodiscard]] bool write(Level level) const noexcept;

    /**
     * @brief 翻转输出电平。
     * @return 不是输出引脚时返回 false。
     * @note 同 write()。
     */
    [[nodiscard]] bool toggle() const noexcept;

    /**
     * @brief 登记外部中断回调：引脚出现触发边沿时，在框架中断里调用 callback(context)。
     * @param callback 传 nullptr 取消登记。
     * @return 引脚没有接外部中断时返回 false。
     * @note 在任务里调用，一般在初始化阶段；登记与中断并发时，中断看到的是登记前或登记后的完整状态。
     */
    [[nodiscard]] bool set_interrupt(Callback callback, void *context) const noexcept;

  private:
    std::uint8_t index_; // 由 port 或 mock 解释
};

} // namespace hal

#endif // HAL_GPIO_HPP
