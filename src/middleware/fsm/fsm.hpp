#ifndef MIDDLEWARE_FSM_FSM_HPP
#define MIDDLEWARE_FSM_FSM_HPP

#include <cstdint>
#include <limits>

namespace fsm
{

enum class StatusStage : uint8_t
{
    DISABLE = 0,
    ENABLE,
};

struct StatusInfo
{
    StatusStage stage;
    uint32_t count_time;
};

template <uint8_t StatusMax = 10>
class Fsm
{
    static_assert(StatusMax > 0, "StatusMax must be greater than zero");

  public:
    StatusInfo status[StatusMax] = {};

    bool init(uint8_t now_status_serial = 0);

    inline uint8_t get_now_status_serial() const;

    inline bool is_initialized() const;

    inline bool set_status(uint8_t next_status_serial);

    void calculate_loop();

  protected:
    uint8_t now_status_serial_ = 0;
    bool initialized_ = false;
};

template <uint8_t StatusMax>
bool Fsm<StatusMax>::init(uint8_t now_status_serial)
{
    if (now_status_serial >= StatusMax)
    {
        return false;
    }

    for (uint8_t i = 0; i < StatusMax; i++)
    {
        status[i].stage = StatusStage::DISABLE;
        status[i].count_time = 0;
    }

    now_status_serial_ = now_status_serial;
    status[now_status_serial].stage = StatusStage::ENABLE;
    initialized_ = true;

    return true;
}

template <uint8_t StatusMax>
void Fsm<StatusMax>::calculate_loop()
{
    if (!initialized_ || now_status_serial_ >= StatusMax)
    {
        return;
    }

    if (status[now_status_serial_].count_time < std::numeric_limits<uint32_t>::max())
    {
        status[now_status_serial_].count_time++;
    }

    // 自行编写状态转移
}

template <uint8_t StatusMax>
inline uint8_t Fsm<StatusMax>::get_now_status_serial() const
{
    return (now_status_serial_);
}

template <uint8_t StatusMax>
inline bool Fsm<StatusMax>::is_initialized() const
{
    return (initialized_);
}

template <uint8_t StatusMax>
inline bool Fsm<StatusMax>::set_status(uint8_t next_status_serial)
{
    if (!initialized_ || now_status_serial_ >= StatusMax || next_status_serial >= StatusMax)
    {
        return false;
    }

    status[now_status_serial_].stage = StatusStage::DISABLE;
    status[now_status_serial_].count_time = 0;

    status[next_status_serial].stage = StatusStage::ENABLE;
    status[next_status_serial].count_time = 0;
    now_status_serial_ = next_status_serial;

    return true;
}

} // namespace fsm

#endif
