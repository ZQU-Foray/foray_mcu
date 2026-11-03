// hal::Gpio、hal::Pwm 契约 v1 在 platform/mock 上的语义。
// 替身的行为就是契约写给驱动的约定：驱动的宿主测试都以它为准，所以先把它测清楚。
#include "hal/gpio.hpp"
#include "hal/pwm.hpp"
#include "mock_hal.hpp"

#include <cassert>
#include <cstdint>

namespace
{

struct Counter
{
    std::uint32_t calls{0U};
};

void count(void *context) noexcept
{
    ++static_cast<Counter *>(context)->calls;
}

// 句柄是一个字节、可以常量初始化，驱动按值保存
constexpr hal::Gpio OUTPUT_PIN{0U};
static_assert(sizeof(hal::Gpio) == 1U && sizeof(hal::Pwm) == 1U);

void test_gpio()
{
    mock::gpio::reset();
    const hal::Gpio input{1U};
    const hal::Gpio invalid{mock::gpio::PIN_COUNT};

    // 输出：write、toggle 改变电平，read 读到正在输出的电平
    assert(OUTPUT_PIN.read() == hal::Gpio::Level::LOW);
    assert(OUTPUT_PIN.write(hal::Gpio::Level::HIGH));
    assert(OUTPUT_PIN.read() == hal::Gpio::Level::HIGH);
    assert(OUTPUT_PIN.toggle());
    assert(mock::gpio::level(0U) == hal::Gpio::Level::LOW);
    assert(mock::gpio::write_count(0U) == 2U);

    // 只能输入的引脚：write、toggle 返回 false，电平由外部决定
    mock::gpio::set_capabilities(1U, false, false);
    assert(!input.write(hal::Gpio::Level::HIGH));
    assert(!input.toggle());
    mock::gpio::set_level(1U, hal::Gpio::Level::HIGH);
    assert(input.read() == hal::Gpio::Level::HIGH);

    // 外部中断：没有中断线的引脚登记失败；登记后每次触发调用一次回调；传 nullptr 取消
    Counter counter;
    assert(!input.set_interrupt(count, &counter));
    const hal::Gpio interrupt_pin{2U};
    assert(!mock::gpio::trigger_interrupt(2U));
    assert(interrupt_pin.set_interrupt(count, &counter));
    assert(mock::gpio::trigger_interrupt(2U));
    assert(mock::gpio::trigger_interrupt(2U));
    assert(counter.calls == 2U);
    assert(interrupt_pin.set_interrupt(nullptr, nullptr));
    assert(!mock::gpio::trigger_interrupt(2U));
    assert(counter.calls == 2U);

    // 无效编号：读到 LOW，其余失败
    assert(invalid.read() == hal::Gpio::Level::LOW);
    assert(!invalid.write(hal::Gpio::Level::HIGH));
    assert(!invalid.set_interrupt(count, &counter));
}

void test_pwm()
{
    mock::pwm::reset();
    const hal::Pwm channel{3U};
    assert(channel.is_ready());
    assert(channel.set(1000U, 250U) == hal::Pwm::Result::COMPLETED);
    mock::pwm::Output output = mock::pwm::output(3U);
    assert(output.frequency_hz == 1000U && output.duty_permille == 250U && output.set_count == 1U);

    // off()：占空比归零，频率不变
    assert(channel.off() == hal::Pwm::Result::COMPLETED);
    output = mock::pwm::output(3U);
    assert(output.frequency_hz == 1000U && output.duty_permille == 0U && output.set_count == 2U);

    // 参数检查先于就绪检查
    assert(channel.set(0U, 100U) == hal::Pwm::Result::INVALID_ARGUMENT);
    assert(channel.set(1000U, hal::Pwm::MAX_DUTY_PERMILLE + 1U) == hal::Pwm::Result::INVALID_ARGUMENT);
    assert(channel.set(1000U, hal::Pwm::MAX_DUTY_PERMILLE) == hal::Pwm::Result::COMPLETED);
    mock::pwm::set_ready(3U, false);
    assert(!channel.is_ready());
    assert(channel.set(1000U, 100U) == hal::Pwm::Result::NOT_READY);
    assert(channel.off() == hal::Pwm::Result::NOT_READY);

    const hal::Pwm invalid{mock::pwm::CHANNEL_COUNT};
    assert(!invalid.is_ready());
    assert(invalid.set(1000U, 100U) == hal::Pwm::Result::INVALID_ARGUMENT);
    assert(invalid.off() == hal::Pwm::Result::INVALID_ARGUMENT);
}

} // namespace

int main()
{
    test_gpio();
    test_pwm();
    return 0;
}
