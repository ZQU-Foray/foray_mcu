#include "Init.h"
#include "Detail/Can.h"
#include "Detail/Pwm.h"
#include "Detail/Spi.h"
#include "Detail/Time.h"
#include "Detail/Uart.h"

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
