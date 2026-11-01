#ifndef PLATFORM_ST_DM_MC02_PORT_USBCDC_H
#define PLATFORM_ST_DM_MC02_PORT_USBCDC_H

// USB 生成代码（usb_device.c、usbd_cdc_if.c 的 USER CODE 区）调用 port 的接口（例外 X-01），按名字 include，
// 所以保持原文件名。实现在 usb_cdc.cpp（C++，extern "C"）。链路能力重建时按 Q8 改为回调注册后删除。

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** MX_USB_DEVICE_Init 开头调用：配置 PA11/PA12、清零 USB DMA 缓冲区、复位状态。 */
    bool UsbCdcPort_Init(void);

    // 以下由 CDC 类回调调用（USB 中断里）
    bool UsbCdcPort_HandleConfigured(uint8_t *receive_buffer);
    void UsbCdcPort_HandleDisconnected(void);
    bool UsbCdcPort_HandleReceive(uint8_t *data, uint32_t length);
    void UsbCdcPort_HandleTransmitComplete(void);

#ifdef __cplusplus
}
#endif

#endif
