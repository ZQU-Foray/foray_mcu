#include "Init.h"
#include "hal/detail/can.h"
#include "hal/detail/pwm.h"
#include "hal/detail/spi.h"
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

    if (!SpiPort_Init())
    {
        return false;
    }

    if (!PwmPort_Init())
    {
        return false;
    }

    if (!CanPort_Init())
    {
        return false;
    }

    return true;
}
