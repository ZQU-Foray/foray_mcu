#ifndef BRINGUP_POWER_OUTPUTS_HPP
#define BRINGUP_POWER_OUTPUTS_HPP

#include "hal/gpio.hpp"

#include <cstddef>

namespace bringup
{

/**
 * @brief 板上可控电源输出：上电时按 profile 打开或关闭，接在这些输出上的电机随之上电。
 *        只在 init() 里写一次，没有 step()。
 * @note 达妙电机上电默认失能，大疆电调没有控制帧时不出力，bringup 只收不发：打开电源不会让电机动。
 */
class PowerOutputs final
{
  public:
    /** 来自 profile 的参数（params.hpp）。 */
    struct Parameters
    {
        bool on; // 上电时打开
    };

    /** 组合根注入：输出引脚与打开时的电平（板级约定）。 */
    struct Config
    {
        const hal::Gpio *outputs; // 指向组合根里的引脚数组
        std::size_t count;
        hal::Gpio::Level on_level; // 打开时的电平，DM-MC02 为 HIGH
        Parameters parameters;
    };

    /** 把每一路写成打开或关闭；有一路写失败（不是输出引脚）就返回 false。 */
    [[nodiscard]] bool init(const Config &config) noexcept;

    [[nodiscard]] bool is_on() const noexcept
    {
        return on_;
    }

  private:
    bool on_{false};
};

} // namespace bringup

#endif // BRINGUP_POWER_OUTPUTS_HPP
