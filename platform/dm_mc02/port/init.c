#include "Init.h"
#include "hal/detail/time.h"

bool Port_Init(void)
{
    if (!TimePort_Init())
    {
        return false;
    }

    return true;
}
