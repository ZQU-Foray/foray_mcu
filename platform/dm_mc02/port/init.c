#include "Init.h"
#include "hal/detail/time.h"
#include "hal/detail/uart.h"

bool Port_Init(void)
{
    if (!TimePort_Init())
    {
        return false;
    }

    if (!UartPort_Init())
    {
        return false;
    }

    return true;
}
