#ifndef TOPICS_OPERATOR_INPUT_HPP
#define TOPICS_OPERATOR_INPUT_HPP

#include "hal/os/clock.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

// 操作输入的统一消息类型：遥控、以后的键鼠都翻译成这一种形状，仲裁只认它。
// 这个头只有类型、没有 topic 实例：每个来源各一个 topic（remote.input，以后 referee.operator_input）。
// `operator` 是 C++ 关键字，不能当命名空间名，所以类型直接放在 topics 命名空间里。

namespace topics
{

/** 拨杆档位。回弹开关按两档处理，只会出现 UP 与 DOWN。 */
enum class SwitchPosition : std::uint8_t
{
    NONE = 0U, // 没有这个拨杆，或者还没读到
    UP = 1U,
    MIDDLE = 2U,
    DOWN = 3U
};

// 键盘位，顺序同裁判系统协议（0x0304 与图传 21 字节帧相同）
inline constexpr std::uint16_t KEY_W{1U << 0U};
inline constexpr std::uint16_t KEY_S{1U << 1U};
inline constexpr std::uint16_t KEY_A{1U << 2U};
inline constexpr std::uint16_t KEY_D{1U << 3U};
inline constexpr std::uint16_t KEY_SHIFT{1U << 4U};
inline constexpr std::uint16_t KEY_CTRL{1U << 5U};
inline constexpr std::uint16_t KEY_Q{1U << 6U};
inline constexpr std::uint16_t KEY_E{1U << 7U};
inline constexpr std::uint16_t KEY_R{1U << 8U};
inline constexpr std::uint16_t KEY_F{1U << 9U};
inline constexpr std::uint16_t KEY_G{1U << 10U};
inline constexpr std::uint16_t KEY_Z{1U << 11U};
inline constexpr std::uint16_t KEY_X{1U << 12U};
inline constexpr std::uint16_t KEY_C{1U << 13U};
inline constexpr std::uint16_t KEY_V{1U << 14U};
inline constexpr std::uint16_t KEY_B{1U << 15U};

inline constexpr std::uint8_t MOUSE_LEFT{1U << 0U};
inline constexpr std::uint8_t MOUSE_RIGHT{1U << 1U};
inline constexpr std::uint8_t MOUSE_MIDDLE{1U << 2U};

// 摇杆在 sticks 里的下标。方向：向右、向上为正（归一化之后）
inline constexpr std::size_t STICK_RIGHT_X{0U};
inline constexpr std::size_t STICK_RIGHT_Y{1U};
inline constexpr std::size_t STICK_LEFT_X{2U};
inline constexpr std::size_t STICK_LEFT_Y{3U};
inline constexpr std::size_t STICK_COUNT{4U};
inline constexpr std::size_t SWITCH_COUNT{8U}; // 遥控上的开关最多取 8 个

/**
 * @brief 一个操作输入来源的最新状态（SBUS 遥控、以后的键鼠共用）。
 *
 * 消息必须可平凡拷贝（topic 的约束），所以不放默认成员初始化：发布者用 OperatorInput{} 清零。
 * 源里没有的那部分保持默认：SBUS 遥控填 has_keyboard_mouse = false，键鼠填 has_remote = false。
 * 失控保护或超时只把 connected 置假，摇杆与拨杆保持上一份正常值（失控帧里的值不可信）。
 */
struct OperatorInput
{
    std::array<float, STICK_COUNT> sticks;             // [-1, 1]，已扣中位、已按方向与死区处理
    std::array<SwitchPosition, SWITCH_COUNT> switches; // 没有的填 NONE
    std::uint8_t mouse_buttons;                        // MOUSE_*
    std::int16_t mouse_x;                              // 协议原值：移动速度，向左为负
    std::int16_t mouse_y;                              // 协议原值：向下（向后）为负
    std::int16_t mouse_z;                              // 协议原值：滚轮
    std::uint16_t keys;                                // KEY_*
    bool has_remote;                                   // 有摇杆、拨杆这部分
    bool has_keyboard_mouse;                           // 有键鼠这部分
    bool connected;                                    // 链路活着，没有失控保护
    hal::os::Microseconds timestamp_us;                // 最近一帧正常帧的接收时刻，只给人看
};

} // namespace topics

#endif
