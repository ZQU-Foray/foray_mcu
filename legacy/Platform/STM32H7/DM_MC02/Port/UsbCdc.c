#include "PortKit.h"
#include "UsbCdc.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include <stddef.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t __usb_dma_buffer_start__;
extern uint8_t __usb_dma_buffer_end__;

enum
{
  USB_CDC_PORT_RX_QUEUE_CAPACITY = 4096U,
  USB_CDC_PORT_ENDPOINT_COUNT = 9U,
  USB_CDC_PORT_DMA_ALIGNMENT = 4U
};

PORTKIT_STATIC_ASSERT_POWER_OF_TWO(USB_CDC_PORT_RX_QUEUE_CAPACITY);

typedef struct
{
  PortKit_Queue receive_queue;
  uint8_t receive_queue_buffer[USB_CDC_PORT_RX_QUEUE_CAPACITY];
  volatile uint32_t received_byte_count;
  volatile uint32_t dropped_byte_count;
  volatile uint32_t receive_error_count;
  volatile uint32_t transmitted_byte_count;
  volatile uint32_t transmit_failure_count;
  volatile uint32_t connection_count;
  volatile uint32_t disconnection_count;
  volatile uint32_t transmit_length;
  PortKit_Notifier receive_notifier;
  PortKit_Notifier transmit_notifier;
  volatile bool transmit_active;
  volatile bool connected;
  volatile bool initialized;
} UsbCdcPort_State;

static UsbCdcPort_State usb_cdc_port_state;

__attribute__((section(".usb_dma_buffer"),
               aligned(32))) static uint8_t usb_cdc_port_transmit_buffer[USB_CDC_PORT_MAX_TRANSMIT_SIZE];

__attribute__((section(".usb_dma_buffer"), aligned(32))) static uint8_t
    usb_cdc_port_transmit_bounce[USB_CDC_PORT_ENDPOINT_COUNT][USB_CDC_PORT_MAX_TRANSMIT_SIZE];

static void UsbCdcPort_ConfigurePins(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &gpio_init);
}

HAL_StatusTypeDef __real_HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *data, uint32_t length);

static bool UsbCdcPort_IsDmaAccessible(const uint8_t *data, uint32_t length)
{
  const uintptr_t start = (uintptr_t)data;
  const uintptr_t end = start + length;

  if (data == NULL || end < start)
  {
    return false;
  }

  return (start >= 0x24000000UL && end <= 0x24050000UL) || (start >= 0x30000000UL && end <= 0x30008000UL) ||
         (start >= 0x38000000UL && end <= 0x38004000UL);
}

HAL_StatusTypeDef __wrap_HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *data, uint32_t length)
{
  const uint32_t endpoint = ep_addr & EP_ADDR_MSK;
  uint8_t *dma_data = data;

  if (length == 0U)
  {
    __DSB();
    return __real_HAL_PCD_EP_Transmit(hpcd, ep_addr, NULL, 0U);
  }

  if (endpoint >= USB_CDC_PORT_ENDPOINT_COUNT || length > USB_CDC_PORT_MAX_TRANSMIT_SIZE || data == NULL)
  {
    return HAL_ERROR;
  }

  if (!UsbCdcPort_IsDmaAccessible(data, length) || ((uintptr_t)data & (USB_CDC_PORT_DMA_ALIGNMENT - 1U)) != 0U ||
      (length & (USB_CDC_PORT_DMA_ALIGNMENT - 1U)) != 0U)
  {
    dma_data = usb_cdc_port_transmit_bounce[endpoint];
    memcpy(dma_data, data, length);
    memset(&dma_data[length],
           0,
           (USB_CDC_PORT_DMA_ALIGNMENT - (length & (USB_CDC_PORT_DMA_ALIGNMENT - 1U))) &
               (USB_CDC_PORT_DMA_ALIGNMENT - 1U));
  }

  __DSB();
  return __real_HAL_PCD_EP_Transmit(hpcd, ep_addr, dma_data, length);
}

static bool UsbCdcPort_IsStackConfigured(void)
{
  return hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceHS.pClassData != NULL;
}

static uint32_t UsbCdcPort_Enqueue(const uint8_t *data, uint32_t length)
{
  UsbCdcPort_State *state = &usb_cdc_port_state;
  const uint32_t accepted_length = PortKit_Queue_Push(&state->receive_queue, data, length);

  state->received_byte_count += length;
  state->dropped_byte_count += length - accepted_length;
  return accepted_length;
}

bool UsbCdcPort_Init(void)
{
  uint32_t interrupt_mask;

  UsbCdcPort_ConfigurePins();
  interrupt_mask = PortKit_Critical_Enter();
  memset(&__usb_dma_buffer_start__,
         0,
         (size_t)((uintptr_t)&__usb_dma_buffer_end__ - (uintptr_t)&__usb_dma_buffer_start__));
  memset(&usb_cdc_port_state, 0, sizeof(usb_cdc_port_state));
  PortKit_Queue_Init(
      &usb_cdc_port_state.receive_queue, usb_cdc_port_state.receive_queue_buffer, USB_CDC_PORT_RX_QUEUE_CAPACITY);
  usb_cdc_port_state.initialized = true;
  __DMB();
  PortKit_Critical_Exit(interrupt_mask);
  return true;
}

bool UsbCdcPort_IsReady(void)
{
  return usb_cdc_port_state.initialized;
}

bool UsbCdcPort_IsConnected(void)
{
  return usb_cdc_port_state.initialized && usb_cdc_port_state.connected && UsbCdcPort_IsStackConfigured();
}

bool UsbCdcPort_SetReceiveNotification(UsbCdcPort_ReceiveNotification notification, void *context)
{
  if (!usb_cdc_port_state.initialized)
  {
    return false;
  }

  PortKit_Notifier_Set(&usb_cdc_port_state.receive_notifier, notification, context);
  return true;
}

bool UsbCdcPort_SetTransmitNotification(UsbCdcPort_TransmitNotification notification, void *context)
{
  if (!usb_cdc_port_state.initialized)
  {
    return false;
  }

  PortKit_Notifier_Set(&usb_cdc_port_state.transmit_notifier, notification, context);
  return true;
}

UsbCdcPort_ReceiveResult UsbCdcPort_TryRead(uint8_t *data, uint32_t data_capacity, uint32_t *length)
{
  UsbCdcPort_State *state = &usb_cdc_port_state;
  uint32_t read_length;

  if (length == NULL)
  {
    return USB_CDC_PORT_RECEIVE_INVALID_ARGUMENT;
  }

  *length = 0U;
  if (data == NULL || data_capacity == 0U)
  {
    return USB_CDC_PORT_RECEIVE_INVALID_ARGUMENT;
  }

  if (!state->initialized)
  {
    return USB_CDC_PORT_RECEIVE_NOT_READY;
  }

  read_length = PortKit_Queue_Pop(&state->receive_queue, data, data_capacity);
  if (read_length == 0U)
  {
    return USB_CDC_PORT_RECEIVE_EMPTY;
  }

  *length = read_length;
  return USB_CDC_PORT_RECEIVE_RECEIVED;
}

UsbCdcPort_TransmitResult UsbCdcPort_TryWrite(const uint8_t *data, uint32_t length)
{
  UsbCdcPort_State *state = &usb_cdc_port_state;
  USBD_CDC_HandleTypeDef *class_state;
  USBD_StatusTypeDef status;
  uint32_t interrupt_mask;

  if (data == NULL || length == 0U || length > USB_CDC_PORT_MAX_TRANSMIT_SIZE)
  {
    return USB_CDC_PORT_TRANSMIT_INVALID_ARGUMENT;
  }

  if (!UsbCdcPort_IsConnected())
  {
    return USB_CDC_PORT_TRANSMIT_NOT_READY;
  }

  interrupt_mask = PortKit_Critical_Enter();
  if (!state->connected || !UsbCdcPort_IsStackConfigured())
  {
    PortKit_Critical_Exit(interrupt_mask);
    return USB_CDC_PORT_TRANSMIT_NOT_READY;
  }

  class_state = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
  if (state->transmit_active || class_state == NULL || class_state->TxState != 0U)
  {
    PortKit_Critical_Exit(interrupt_mask);
    return USB_CDC_PORT_TRANSMIT_BUSY;
  }

  memcpy(usb_cdc_port_transmit_buffer, data, length);
  state->transmit_length = length;
  state->transmit_active = true;
  __DMB();

  status = (USBD_StatusTypeDef)USBD_CDC_SetTxBuffer(&hUsbDeviceHS, usb_cdc_port_transmit_buffer, length);
  if (status == USBD_OK)
  {
    status = (USBD_StatusTypeDef)USBD_CDC_TransmitPacket(&hUsbDeviceHS);
  }

  if (status != USBD_OK)
  {
    state->transmit_length = 0U;
    state->transmit_active = false;
    if (status == USBD_FAIL)
    {
      ++state->transmit_failure_count;
    }
    __DMB();
    PortKit_Critical_Exit(interrupt_mask);
    return status == USBD_BUSY ? USB_CDC_PORT_TRANSMIT_BUSY : USB_CDC_PORT_TRANSMIT_ERROR;
  }

  PortKit_Critical_Exit(interrupt_mask);
  return USB_CDC_PORT_TRANSMIT_STARTED;
}

bool UsbCdcPort_GetStatistics(UsbCdcPort_Statistics *statistics)
{
  uint32_t interrupt_mask;

  if (statistics == NULL || !usb_cdc_port_state.initialized)
  {
    return false;
  }

  interrupt_mask = PortKit_Critical_Enter();
  statistics->received_byte_count = usb_cdc_port_state.received_byte_count;
  statistics->dropped_byte_count = usb_cdc_port_state.dropped_byte_count;
  statistics->receive_error_count = usb_cdc_port_state.receive_error_count;
  statistics->transmitted_byte_count = usb_cdc_port_state.transmitted_byte_count;
  statistics->transmit_failure_count = usb_cdc_port_state.transmit_failure_count;
  statistics->connection_count = usb_cdc_port_state.connection_count;
  statistics->disconnection_count = usb_cdc_port_state.disconnection_count;
  PortKit_Critical_Exit(interrupt_mask);
  return true;
}

bool UsbCdcPort_HandleConfigured(uint8_t *receive_buffer)
{
  UsbCdcPort_State *state = &usb_cdc_port_state;
  USBD_StatusTypeDef status;

  if (!state->initialized || receive_buffer == NULL)
  {
    return false;
  }

  status = (USBD_StatusTypeDef)USBD_CDC_SetRxBuffer(&hUsbDeviceHS, receive_buffer);
  if (status != USBD_OK)
  {
    ++state->receive_error_count;
    return false;
  }

  PortKit_Queue_Reset(&state->receive_queue);
  state->transmit_length = 0U;
  state->transmit_active = false;
  state->connected = true;
  ++state->connection_count;
  __DMB();
  return true;
}

void UsbCdcPort_HandleDisconnected(void)
{
  UsbCdcPort_State *state = &usb_cdc_port_state;
  bool transmit_completed = false;

  if (!state->initialized || !state->connected)
  {
    return;
  }

  state->connected = false;
  PortKit_Queue_Reset(&state->receive_queue);
  ++state->disconnection_count;
  if (state->transmit_active)
  {
    state->transmit_length = 0U;
    state->transmit_active = false;
    ++state->transmit_failure_count;
    transmit_completed = true;
  }
  __DMB();

  if (transmit_completed)
  {
    (void)PortKit_Notifier_Fire(&state->transmit_notifier);
  }
}

bool UsbCdcPort_HandleReceive(uint8_t *data, uint32_t length)
{
  UsbCdcPort_State *state = &usb_cdc_port_state;
  USBD_StatusTypeDef status;
  uint32_t accepted_length = 0U;

  if (!state->initialized || !state->connected || data == NULL)
  {
    if (state->initialized)
    {
      ++state->receive_error_count;
    }
    return false;
  }

  if (length > 0U)
  {
    accepted_length = UsbCdcPort_Enqueue(data, length);
  }

  status = (USBD_StatusTypeDef)USBD_CDC_SetRxBuffer(&hUsbDeviceHS, data);
  if (status == USBD_OK)
  {
    status = (USBD_StatusTypeDef)USBD_CDC_ReceivePacket(&hUsbDeviceHS);
  }
  if (status != USBD_OK)
  {
    ++state->receive_error_count;
    return false;
  }

  if (accepted_length > 0U)
  {
    (void)PortKit_Notifier_Fire(&state->receive_notifier);
  }
  return true;
}

void UsbCdcPort_HandleTransmitComplete(void)
{
  UsbCdcPort_State *state = &usb_cdc_port_state;

  if (!state->initialized || !state->transmit_active)
  {
    return;
  }

  state->transmitted_byte_count += state->transmit_length;
  state->transmit_length = 0U;
  state->transmit_active = false;
  __DMB();

  (void)PortKit_Notifier_Fire(&state->transmit_notifier);
}