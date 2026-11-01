#ifndef PLATFORM_DM_MC02_PORT_PORT_HPP
#define PLATFORM_DM_MC02_PORT_PORT_HPP

#include "hal/callback.hpp"
#include "hal/os/critical_section.hpp"

// DM-MC02 port 私有的内容（Decision-20261101-01）：各外设的初始化与中断通知。只给 port 自己的 .cpp 用。

namespace port
{

// ---- 各外设的初始化：hal::init() 依次调用一次，在 CubeMX 初始化外设之后、内核启动之前 ----

/** 检查 PWM 定时器的计数频率，以占空比 0 启动全部通道。 */
[[nodiscard]] bool init_pwm() noexcept;

/** 配置接收过滤器（收下全部标准帧与扩展帧），启动 FDCAN1–3 并打开接收与 Bus-Off 中断。 */
[[nodiscard]] bool init_can() noexcept;

/**
 * @brief 中断通知：任务登记回调，中断取出并调用（代替旧 PortKit 的 Notifier）。
 *
 * 登记在临界区内写入回调与上下文，屏蔽了框架中断，中断里读到的总是一对完整的值。
 */
class Notifier final
{
  public:
    /** 登记或取消（callback 为 nullptr）。任务里调用。 */
    void set(hal::Callback callback, void *context) noexcept
    {
        const hal::os::CriticalSection guard;
        callback_ = callback;
        context_ = context;
    }

    /** 有登记就调用。在框架中断里调用。 */
    void fire() const noexcept
    {
        if (callback_ != nullptr)
        {
            callback_(context_);
        }
    }

  private:
    hal::Callback callback_{nullptr};
    void *context_{nullptr};
};

} // namespace port

#endif // PLATFORM_DM_MC02_PORT_PORT_HPP
