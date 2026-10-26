#ifndef FSM_H
#define FSM_H

#include <cstdint>
#include <limits>

namespace alg_fsm
{

enum class StatusStage : uint8_t
{
    Disable = 0,
    Enable,
};

struct StatusInfo
{
    StatusStage Stage;
    uint32_t CountTime;
};

template <uint8_t StatusMax = 10> class Fsm
{
    static_assert(StatusMax > 0, "StatusMax must be greater than zero");

  public:
    StatusInfo Status[StatusMax] = {};

    bool Init(uint8_t nowStatusSerial = 0);

    inline uint8_t GetNowStatusSerial() const;

    inline bool IsInitialized() const;

    inline bool SetStatus(uint8_t nextStatusSerial);

    void CalculateLoop();

  protected:
    uint8_t NowStatusSerial = 0;
    bool Initialized = false;
};

template <uint8_t StatusMax> bool Fsm<StatusMax>::Init(uint8_t nowStatusSerial)
{
    if (nowStatusSerial >= StatusMax)
    {
        return false;
    }

    for (uint8_t i = 0; i < StatusMax; i++)
    {
        Status[i].Stage = StatusStage::Disable;
        Status[i].CountTime = 0;
    }

    NowStatusSerial = nowStatusSerial;
    Status[nowStatusSerial].Stage = StatusStage::Enable;
    Initialized = true;

    return true;
}

template <uint8_t StatusMax> void Fsm<StatusMax>::CalculateLoop()
{
    if (!Initialized || NowStatusSerial >= StatusMax)
    {
        return;
    }

    if (Status[NowStatusSerial].CountTime < std::numeric_limits<uint32_t>::max())
    {
        Status[NowStatusSerial].CountTime++;
    }

    // 自行编写状态转移
}

template <uint8_t StatusMax> inline uint8_t Fsm<StatusMax>::GetNowStatusSerial() const
{
    return (NowStatusSerial);
}

template <uint8_t StatusMax> inline bool Fsm<StatusMax>::IsInitialized() const
{
    return (Initialized);
}

template <uint8_t StatusMax> inline bool Fsm<StatusMax>::SetStatus(uint8_t nextStatusSerial)
{
    if (!Initialized || NowStatusSerial >= StatusMax || nextStatusSerial >= StatusMax)
    {
        return false;
    }

    Status[NowStatusSerial].Stage = StatusStage::Disable;
    Status[NowStatusSerial].CountTime = 0;

    Status[nextStatusSerial].Stage = StatusStage::Enable;
    Status[nextStatusSerial].CountTime = 0;
    NowStatusSerial = nextStatusSerial;

    return true;
}

} // namespace alg_fsm

#endif
