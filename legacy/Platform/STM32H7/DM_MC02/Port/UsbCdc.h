#ifndef PLATFORM_STM32H7_DM_MC02_PORT_USB_CDC_H
#define PLATFORM_STM32H7_DM_MC02_PORT_USB_CDC_H

#include "Detail/UsbCdc.h"

#ifdef __cplusplus
extern "C"
{
#endif

  // 以下入口仅供 CubeMX USB CDC USER CODE 回调调用。
  bool UsbCdcPort_HandleConfigured(uint8_t *receive_buffer);
  void UsbCdcPort_HandleDisconnected(void);
  bool UsbCdcPort_HandleReceive(uint8_t *data, uint32_t length);
  void UsbCdcPort_HandleTransmitComplete(void);

#ifdef __cplusplus
}
#endif

#endif
