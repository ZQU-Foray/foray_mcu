#ifndef HAL_CALLBACK_HPP
#define HAL_CALLBACK_HPP

// 硬件契约：中断回调（Decision-20261101-01）。

namespace hal
{

/**
 * @brief 外设在中断里调用的回调，context 是登记时给的指针，原样传回。
 *
 * 在框架中断里执行（优先级不高于框架线，架构 §3.2）：只做取数据、发布 topic、Signal::notify() 这类短操作，
 * 不阻塞，不调用只能在任务里用的接口。各外设的文档写明什么时候调用它。
 */
using Callback = void (*)(void *context) noexcept;

} // namespace hal

#endif // HAL_CALLBACK_HPP
