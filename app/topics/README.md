# app/topics：产品的 topic 清单

本产品的全部 topic 都定义在这里（规则 17）。能力之间只经 topic 通信，共享的头文件只有这里的。机制见 `src/middleware/topic/topic.hpp` 与[架构 v2 §3.4](../../docs/ai/architecture.md)，设计记录见 [decision.md](../../decision.md) Decision-20261031-05。

现有的 topic（`bringup.hpp`，框架验证用）：`bringup.heartbeat`（F2，Decision-20261031-12）；`bringup.imu_event`（在中断里发布）、`bringup.imu`、`bringup.sbus`（Decision-20261101-03）；`safety.stop`（`safety.hpp`，急停与重新确认，安全门订阅；发布者是阶段 A 的遥控、裁判系统能力，Decision-20261102-03）。阶段 A：`imu.event`（在中断里发布）、`imu.attitude`（云台姿态、零偏标定状态，`imu.hpp`）；`pc_link.control`（PC 发来的控制快照，第一版只记账，`pc_link.hpp`）（Decision-20261103-01）。能力的 topic 在阶段 A 重建能力时加入。

## 约定

**文件**：按发布者所属的能力分文件，`app/topics/<capability>.hpp`，命名空间 `topics::<capability>`。一个能力可以发布多个 topic，也可以订阅任何 topic。

**消息**：

- `struct`，可平凡拷贝；只用标准整数、浮点、`bool` 与定长数组，不用 `lib` 的类型（如 `math::Vector3`）；
- 不超过 `topic::MAX_MESSAGE_SIZE` 字节（暂定 64，F2 在板上实测后再定）。发布时在临界区内拷贝消息，消息越大，关中断越久；
- 需要采样时刻的，放一个 `timestamp_us` 字段（`hal::os::Microseconds`）。topic 自己只记最后发布时刻，用于观测。

上面前两条由 `topic::Topic` 的 `static_assert` 检查。

**实例**：`inline` 全局变量，名字字符串为 `"<capability>.<topic>"`，与命名空间一致：

```cpp
// app/topics/imu.hpp（示意）
#ifndef TOPICS_IMU_HPP
#define TOPICS_IMU_HPP

#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"

#include <array>

namespace topics::imu
{

struct Attitude
{
    std::array<float, 4> quaternion;
    std::array<float, 3> euler_rad;
    hal::os::Microseconds timestamp_us;
};

inline topic::Topic<Attitude> attitude{"imu.attitude"};

} // namespace topics::imu

#endif // TOPICS_IMU_HPP
```

**总表**：`app/topics/all.hpp` 列出全部 topic 的观测接口，给调试通道用。新增 topic 时同步加入：

```cpp
inline constexpr std::array<const topic::TopicBase *, 1> ALL{&imu::attitude};
```

## 能力怎么用

```cpp
// 发布：每个 topic 只有一个能力发布，init() 里取得句柄，失败则 init() 返回 false
publisher_ = topics::imu::attitude.claim_publisher();
if (!publisher_)
{
    return false;
}
publisher_.publish(message); // 任务与框架中断里都能调用

// 订阅：订阅者是能力的成员，一个上下文一个
topic::Subscriber<topics::imu::Attitude> attitude_{topics::imu::attitude};
attitude_.update(message);                 // 有没读过的新数据才拷贝；任务与框架中断里都能用
attitude_.enable_wait();                   // 要阻塞等待的，init() 里先占一个等待槽，失败则 init() 返回 false
attitude_.wait(message, timeout);          // 只在任务里用：有没读过的数据就立即返回，否则等到下一次发布或超时
```

## 构建与检查（Decision-20261031-12）

- INTERFACE target `app_topics`，只依赖 `middleware_topic` 与 `hal_os`；新增头文件时列进 `app/CMakeLists.txt`。
- `tool/ci/arch_check.py` 的 `topics` 层：只能依赖 middleware 与契约；能力、任务运行器、组合根都可以依赖它（规则 18 不把它算作能力）。
- 命名：`.clang-tidy` 要求全局变量加 `g_`，清单里的 topic 实例不加（架构 §9.2），由 `tool/ci/naming.py` 对 `app/topics/` 的全局变量放行。所以这里的全局变量只能是 topic 实例；`ALL` 是编译期常量，按常量规则大写。
- 任务运行器按 topic 触发任务时，也占 topic 的一个等待槽（共 `topic::WAIT_SLOTS` 个）。
- 以后：框架验证好用之后，清单改为从描述文件生成，生成器放 `tool/`。
