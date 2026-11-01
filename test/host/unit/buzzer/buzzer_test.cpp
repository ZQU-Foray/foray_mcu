// 蜂鸣器驱动（device::Buzzer）在 platform/mock 上：频率范围、音量到占空比的换算、停止与失败时的状态。
#include "driver/buzzer/buzzer.hpp"
#include "mock_hal.hpp"

#include <cassert>

int main()
{
    mock::pwm::reset();
    device::Buzzer buzzer{hal::Pwm{1U}};
    assert(buzzer.is_ready());
    assert(buzzer.get_state() == device::Buzzer::State::STOPPED);

    // 音量 255 对应 50% 占空比，128 约为 25%（四舍五入到 ‰）
    assert(buzzer.play(1000U, 255U) == device::Buzzer::Result::COMPLETED);
    assert(mock::pwm::output(1U).frequency_hz == 1000U);
    assert(mock::pwm::output(1U).duty_permille == 500U);
    assert(buzzer.get_state() == device::Buzzer::State::PLAYING);
    assert(buzzer.get_frequency_hz() == 1000U && buzzer.get_volume() == 255U);
    assert(buzzer.play(2000U, 128U) == device::Buzzer::Result::COMPLETED);
    assert(mock::pwm::output(1U).duty_permille == 251U);

    // 频率超出 196–4000 Hz：拒绝，不改输出与状态
    assert(buzzer.play(100U, 200U) == device::Buzzer::Result::INVALID_ARGUMENT);
    assert(buzzer.play(5000U, 200U) == device::Buzzer::Result::INVALID_ARGUMENT);
    assert(mock::pwm::output(1U).frequency_hz == 2000U);
    assert(buzzer.get_state() == device::Buzzer::State::PLAYING);

    // 音量 0 等于停止
    assert(buzzer.play(1000U, 0U) == device::Buzzer::Result::COMPLETED);
    assert(mock::pwm::output(1U).duty_permille == 0U);
    assert(buzzer.get_state() == device::Buzzer::State::STOPPED);
    assert(buzzer.get_frequency_hz() == 0U && buzzer.get_volume() == 0U);

    // PWM 未就绪：返回 NOT_READY，状态不变
    mock::pwm::set_ready(1U, false);
    assert(!buzzer.is_ready());
    assert(buzzer.play(1000U, 100U) == device::Buzzer::Result::NOT_READY);
    assert(buzzer.get_state() == device::Buzzer::State::STOPPED);
    assert(buzzer.get_last_result() == device::Buzzer::Result::NOT_READY);
    return 0;
}
