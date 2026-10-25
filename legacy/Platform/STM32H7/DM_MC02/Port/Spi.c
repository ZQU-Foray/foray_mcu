#include "Detail/Spi.h"
#include "PortKit.h"
#include "main.h"
#include "spi.h"
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum
{
  SPI_PORT_CACHE_LINE_SIZE = 32U
};

typedef struct
{
  SPI_HandleTypeDef *handle;
  uint8_t *transmit_dma_buffer;
  uint8_t *receive_dma_buffer;
  uint32_t capacity;
  volatile bool active;
  volatile bool spi_completed;
  volatile bool transmit_dma_completed;
  bool transfer;
  SpiPort_Device device;
  uint8_t *receive_data;
  uint32_t length;
  SpiPort_CompletionNotification notification;
  void *notification_context;
} SpiPort_AsyncState;

typedef struct
{
  SPI_HandleTypeDef *handle;
  GPIO_TypeDef *chip_select_port;
  uint16_t chip_select_pin;
  volatile uint8_t *busy;
  SpiPort_AsyncState *async_state;
  bool supports_transfer;
} SpiPort_Mapping;

static bool SpiPort_IsDmaHandleReady(DMA_HandleTypeDef *handle);
static bool SpiPort_IsAsyncStateReady(const SpiPort_AsyncState *state);

static uint8_t spi2_transmit_dma_buffer[SPI_PORT_MAX_ASYNC_LENGTH]
    __attribute__((section(".dma_buffer"), aligned(SPI_PORT_CACHE_LINE_SIZE)));
static uint8_t spi2_receive_dma_buffer[SPI_PORT_MAX_ASYNC_LENGTH]
    __attribute__((section(".dma_buffer"), aligned(SPI_PORT_CACHE_LINE_SIZE)));
static uint8_t spi6_transmit_dma_buffer[SPI_PORT_MAX_ASYNC_LENGTH]
    __attribute__((section(".bdma_buffer"), aligned(SPI_PORT_CACHE_LINE_SIZE)));

static volatile uint8_t spi2_busy = 0U;
static volatile uint8_t spi6_busy = 0U;
static bool spi_port_initialized = false;
static volatile SpiPort_Result spi_port_async_results[SPI_PORT_DEVICE_COUNT];

static SpiPort_AsyncState spi2_async_state = {.handle = &hspi2,
                                              .transmit_dma_buffer = spi2_transmit_dma_buffer,
                                              .receive_dma_buffer = spi2_receive_dma_buffer,
                                              .capacity = SPI_PORT_MAX_ASYNC_LENGTH};

static SpiPort_AsyncState spi6_async_state = {.handle = &hspi6,
                                              .transmit_dma_buffer = spi6_transmit_dma_buffer,
                                              .receive_dma_buffer = NULL,
                                              .capacity = SPI_PORT_MAX_ASYNC_LENGTH};

static const SpiPort_Mapping spi_port_mappings[SPI_PORT_DEVICE_COUNT] = {
    [SPI_PORT_DEVICE_IMU_ACCELEROMETER] =
        {&hspi2, BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, &spi2_busy, &spi2_async_state, true},
    [SPI_PORT_DEVICE_IMU_GYROSCOPE] =
        {&hspi2, BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, &spi2_busy, &spi2_async_state, true},
    [SPI_PORT_DEVICE_ADDRESSABLE_LED] = {&hspi6, NULL, 0U, &spi6_busy, &spi6_async_state, false}};

_Static_assert(sizeof(spi_port_mappings) / sizeof(spi_port_mappings[0]) == SPI_PORT_DEVICE_COUNT,
               "SPI mapping count mismatch");
_Static_assert((SPI_PORT_MAX_ASYNC_LENGTH % SPI_PORT_CACHE_LINE_SIZE) == 0U,
               "SPI asynchronous buffer size must be cache-line aligned");

static SpiPort_AsyncState *SpiPort_GetAsyncStateFromHandle(SPI_HandleTypeDef *handle)
{
  if (handle == spi2_async_state.handle)
  {
    return &spi2_async_state;
  }

  if (handle == spi6_async_state.handle)
  {
    return &spi6_async_state;
  }

  return NULL;
}

static bool SpiPort_TryAcquire(const SpiPort_Mapping *mapping)
{
  const uint32_t interrupt_mask = PortKit_Critical_Enter();
  bool acquired;

  acquired = *mapping->busy == 0U;
  if (acquired)
  {
    *mapping->busy = 1U;
    __DMB();
  }
  PortKit_Critical_Exit(interrupt_mask);

  return acquired;
}

static void SpiPort_Release(const SpiPort_Mapping *mapping)
{
  const uint32_t interrupt_mask = PortKit_Critical_Enter();

  __DMB();
  *mapping->busy = 0U;
  __DMB();
  PortKit_Critical_Exit(interrupt_mask);
}

static void SpiPort_Select(const SpiPort_Mapping *mapping)
{
  if (mapping->chip_select_port != NULL)
  {
    HAL_GPIO_WritePin(mapping->chip_select_port, mapping->chip_select_pin, GPIO_PIN_RESET);
  }
}

static void SpiPort_Deselect(const SpiPort_Mapping *mapping)
{
  if (mapping->chip_select_port != NULL)
  {
    HAL_GPIO_WritePin(mapping->chip_select_port, mapping->chip_select_pin, GPIO_PIN_SET);
  }
}

static SpiPort_Result SpiPort_MapStatus(HAL_StatusTypeDef status)
{
  if (status == HAL_OK)
  {
    return SPI_PORT_RESULT_COMPLETED;
  }

  if (status == HAL_BUSY)
  {
    return SPI_PORT_RESULT_BUSY;
  }

  if (status == HAL_TIMEOUT)
  {
    return SPI_PORT_RESULT_TIMEOUT;
  }

  return SPI_PORT_RESULT_ERROR;
}

static SpiPort_Result SpiPort_Prepare(SpiPort_Device device, const SpiPort_Mapping **mapping)
{
  if (device >= SPI_PORT_DEVICE_COUNT)
  {
    return SPI_PORT_RESULT_INVALID_ARGUMENT;
  }

  *mapping = &spi_port_mappings[device];
  if (!spi_port_initialized)
  {
    return SPI_PORT_RESULT_NOT_READY;
  }

  if (!SpiPort_TryAcquire(*mapping))
  {
    return SPI_PORT_RESULT_BUSY;
  }

  if (HAL_SPI_GetState((*mapping)->handle) != HAL_SPI_STATE_READY ||
      !SpiPort_IsAsyncStateReady((*mapping)->async_state))
  {
    const HAL_SPI_StateTypeDef state = HAL_SPI_GetState((*mapping)->handle);
    const bool dma_not_ready = !SpiPort_IsAsyncStateReady((*mapping)->async_state);
    SpiPort_Release(*mapping);
    return dma_not_ready || state == HAL_SPI_STATE_BUSY || state == HAL_SPI_STATE_BUSY_TX ||
                   state == HAL_SPI_STATE_BUSY_RX || state == HAL_SPI_STATE_BUSY_TX_RX || state == HAL_SPI_STATE_ABORT
               ? SPI_PORT_RESULT_BUSY
               : SPI_PORT_RESULT_NOT_READY;
  }

  return SPI_PORT_RESULT_COMPLETED;
}

static bool SpiPort_IsDmaHandleReady(DMA_HandleTypeDef *handle)
{
  return handle != NULL && HAL_DMA_GetState(handle) == HAL_DMA_STATE_READY && handle->Init.Mode == DMA_NORMAL &&
         handle->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE &&
         handle->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE;
}

static bool SpiPort_IsAsyncStateReady(const SpiPort_AsyncState *state)
{
  if (state == NULL || state->handle == NULL || !SpiPort_IsDmaHandleReady(state->handle->hdmatx))
  {
    return false;
  }

  return state->receive_dma_buffer == NULL || SpiPort_IsDmaHandleReady(state->handle->hdmarx);
}

static void SpiPort_ResetAsyncState(SpiPort_AsyncState *state)
{
  state->active = false;
  state->spi_completed = false;
  state->transmit_dma_completed = false;
  state->transfer = false;
  state->device = 0U;
  state->receive_data = NULL;
  state->length = 0U;
  state->notification = NULL;
  state->notification_context = NULL;
}

static uint32_t SpiPort_GetCacheMaintenanceLength(uint32_t length)
{
  return (length + SPI_PORT_CACHE_LINE_SIZE - 1U) & ~(SPI_PORT_CACHE_LINE_SIZE - 1U);
}

static bool SpiPort_IsDataCacheEnabled(void)
{
  return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
}

static void SpiPort_PrepareTransmitBuffer(uint8_t *buffer, uint32_t length)
{
  if (SpiPort_IsDataCacheEnabled())
  {
    SCB_CleanDCache_by_Addr(buffer, (int32_t)SpiPort_GetCacheMaintenanceLength(length));
  }
}

static void SpiPort_PrepareReceiveBuffer(uint8_t *buffer, uint32_t length)
{
  if (SpiPort_IsDataCacheEnabled())
  {
    SCB_CleanInvalidateDCache_by_Addr(buffer, (int32_t)SpiPort_GetCacheMaintenanceLength(length));
  }
}

static void SpiPort_CompleteReceiveBuffer(uint8_t *buffer, uint32_t length)
{
  if (SpiPort_IsDataCacheEnabled())
  {
    SCB_InvalidateDCache_by_Addr(buffer, (int32_t)SpiPort_GetCacheMaintenanceLength(length));
  }
}

static bool SpiPort_IsDmaSettled(DMA_HandleTypeDef *handle)
{
  HAL_DMA_StateTypeDef state;

  if (handle == NULL)
  {
    return true;
  }

  state = HAL_DMA_GetState(handle);
  return state != HAL_DMA_STATE_BUSY && state != HAL_DMA_STATE_ABORT;
}

static bool SpiPort_IsErrorSettled(const SpiPort_AsyncState *state)
{
  return HAL_SPI_GetState(state->handle) == HAL_SPI_STATE_READY && SpiPort_IsDmaSettled(state->handle->hdmatx) &&
         SpiPort_IsDmaSettled(state->handle->hdmarx);
}

static void SpiPort_FinalizeAsync(SpiPort_AsyncState *state, SpiPort_Result result);

static void SpiPort_TryFinalizeSuccessfulTransfer(SpiPort_AsyncState *state)
{
  if (state != NULL && state->active && state->transfer && state->spi_completed && state->transmit_dma_completed)
  {
    SpiPort_FinalizeAsync(state, SPI_PORT_RESULT_COMPLETED);
  }
}

static void SpiPort_TransmitDmaComplete(DMA_HandleTypeDef *dma_handle)
{
  SPI_HandleTypeDef *spi_handle;
  SpiPort_AsyncState *state;

  if (dma_handle == NULL)
  {
    return;
  }

  spi_handle = (SPI_HandleTypeDef *)dma_handle->Parent;
  state = SpiPort_GetAsyncStateFromHandle(spi_handle);
  if (state == NULL || !state->active || !state->transfer)
  {
    return;
  }

  state->transmit_dma_completed = true;
  __DMB();
  SpiPort_TryFinalizeSuccessfulTransfer(state);
}

static void SpiPort_CompleteErrorIfSettled(SpiPort_AsyncState *state)
{
  if (state == NULL || !state->active || !SpiPort_IsErrorSettled(state))
  {
    return;
  }

  SpiPort_FinalizeAsync(state, SPI_PORT_RESULT_ERROR);
}

static void SpiPort_DmaAbortAfterError(DMA_HandleTypeDef *dma_handle)
{
  SPI_HandleTypeDef *spi_handle;
  SpiPort_AsyncState *state;

  if (dma_handle == NULL)
  {
    return;
  }

  dma_handle->XferAbortCallback = NULL;
  spi_handle = (SPI_HandleTypeDef *)dma_handle->Parent;
  state = SpiPort_GetAsyncStateFromHandle(spi_handle);
  SpiPort_CompleteErrorIfSettled(state);
}

static void SpiPort_AbortBusyDmaAfterError(DMA_HandleTypeDef *dma_handle)
{
  if (dma_handle == NULL || HAL_DMA_GetState(dma_handle) != HAL_DMA_STATE_BUSY)
  {
    return;
  }

  dma_handle->XferAbortCallback = SpiPort_DmaAbortAfterError;
  if (HAL_DMA_Abort_IT(dma_handle) != HAL_OK)
  {
    dma_handle->XferAbortCallback = NULL;
    if (HAL_DMA_GetState(dma_handle) == HAL_DMA_STATE_BUSY)
    {
      (void)HAL_DMA_Abort(dma_handle);
    }
  }
}

static void SpiPort_RecoverDmaAfterError(SpiPort_AsyncState *state)
{
  if (state == NULL || !state->active)
  {
    return;
  }

  SpiPort_AbortBusyDmaAfterError(state->handle->hdmatx);
  if (state->active)
  {
    SpiPort_AbortBusyDmaAfterError(state->handle->hdmarx);
  }
  SpiPort_CompleteErrorIfSettled(state);
}

static void SpiPort_FinalizeAsync(SpiPort_AsyncState *state, SpiPort_Result result)
{
  const SpiPort_Mapping *mapping;
  SpiPort_CompletionNotification notification;
  void *notification_context;
  uint8_t *receive_data;
  uint32_t length;
  SpiPort_Device device;
  bool transfer;
  uint32_t interrupt_mask;

  if (state == NULL)
  {
    return;
  }

  interrupt_mask = PortKit_Critical_Enter();
  if (!state->active)
  {
    PortKit_Critical_Exit(interrupt_mask);
    return;
  }

  device = state->device;
  notification = state->notification;
  notification_context = state->notification_context;
  receive_data = state->receive_data;
  length = state->length;
  transfer = state->transfer;
  state->active = false;
  state->spi_completed = false;
  state->transmit_dma_completed = false;
  state->transfer = false;
  state->receive_data = NULL;
  state->length = 0U;
  state->notification = NULL;
  state->notification_context = NULL;
  __DMB();
  PortKit_Critical_Exit(interrupt_mask);

  mapping = &spi_port_mappings[device];
  SpiPort_Deselect(mapping);

  if (result == SPI_PORT_RESULT_COMPLETED && transfer && receive_data != NULL)
  {
    SpiPort_CompleteReceiveBuffer(state->receive_dma_buffer, length);
    memcpy(receive_data, state->receive_dma_buffer, length);
  }

  __DMB();
  spi_port_async_results[device] = result;
  __DMB();
  SpiPort_Release(mapping);

  if (notification != NULL)
  {
    notification(notification_context);
  }
}

static SpiPort_Result SpiPort_StartAsync(SpiPort_Device device,
                                         const uint8_t *transmit_data,
                                         uint8_t *receive_data,
                                         uint32_t length,
                                         bool transfer,
                                         SpiPort_CompletionNotification notification,
                                         void *context)
{
  const SpiPort_Mapping *mapping;
  SpiPort_AsyncState *state;
  SpiPort_Result result;
  HAL_StatusTypeDef status;
  uint32_t interrupt_mask;

  result = SpiPort_Prepare(device, &mapping);
  if (result != SPI_PORT_RESULT_COMPLETED)
  {
    return result;
  }

  state = mapping->async_state;
  if (length > state->capacity)
  {
    SpiPort_Release(mapping);
    return SPI_PORT_RESULT_INVALID_ARGUMENT;
  }

  if (transfer && !mapping->supports_transfer)
  {
    SpiPort_Release(mapping);
    return SPI_PORT_RESULT_UNSUPPORTED;
  }

  memcpy(state->transmit_dma_buffer, transmit_data, length);
  SpiPort_PrepareTransmitBuffer(state->transmit_dma_buffer, length);
  if (transfer)
  {
    SpiPort_PrepareReceiveBuffer(state->receive_dma_buffer, length);
  }

  interrupt_mask = PortKit_Critical_Enter();
  state->spi_completed = false;
  state->transmit_dma_completed = false;
  state->transfer = transfer;
  state->device = device;
  state->receive_data = receive_data;
  state->length = length;
  state->notification = notification;
  state->notification_context = context;
  spi_port_async_results[device] = SPI_PORT_RESULT_BUSY;
  __DMB();
  state->active = true;
  __DMB();

  SpiPort_Select(mapping);
  if (transfer)
  {
    status = HAL_SPI_TransmitReceive_DMA(
        mapping->handle, state->transmit_dma_buffer, state->receive_dma_buffer, (uint16_t)length);
    if (status == HAL_OK)
    {
      mapping->handle->hdmatx->XferCpltCallback = SpiPort_TransmitDmaComplete;
    }
  }
  else
  {
    status = HAL_SPI_Transmit_DMA(mapping->handle, state->transmit_dma_buffer, (uint16_t)length);
  }
  if (status != HAL_OK)
  {
    result = SpiPort_MapStatus(status);
    SpiPort_ResetAsyncState(state);
    spi_port_async_results[device] = result;
  }
  __DMB();
  PortKit_Critical_Exit(interrupt_mask);

  if (status == HAL_OK)
  {
    return SPI_PORT_RESULT_STARTED;
  }

  SpiPort_Deselect(mapping);
  SpiPort_Release(mapping);
  return result;
}

bool SpiPort_Init(void)
{
  spi_port_initialized = false;
  spi2_busy = 0U;
  spi6_busy = 0U;
  SpiPort_ResetAsyncState(&spi2_async_state);
  SpiPort_ResetAsyncState(&spi6_async_state);

  for (SpiPort_Device device = 0U; device < SPI_PORT_DEVICE_COUNT; ++device)
  {
    spi_port_async_results[device] = SPI_PORT_RESULT_NOT_READY;
  }

  if (!SpiPort_IsAsyncStateReady(&spi2_async_state) || !SpiPort_IsAsyncStateReady(&spi6_async_state))
  {
    return false;
  }

  for (SpiPort_Device device = 0U; device < SPI_PORT_DEVICE_COUNT; ++device)
  {
    const SpiPort_Mapping *const mapping = &spi_port_mappings[device];

    if (HAL_SPI_GetState(mapping->handle) != HAL_SPI_STATE_READY)
    {
      return false;
    }

    SpiPort_Deselect(mapping);
  }

  spi_port_initialized = true;
  return true;
}

bool SpiPort_IsReady(SpiPort_Device device)
{
  if (!spi_port_initialized || device >= SPI_PORT_DEVICE_COUNT)
  {
    return false;
  }

  const SpiPort_Mapping *const mapping = &spi_port_mappings[device];
  return *mapping->busy == 0U && HAL_SPI_GetState(mapping->handle) == HAL_SPI_STATE_READY &&
         SpiPort_IsAsyncStateReady(mapping->async_state);
}

SpiPort_Result SpiPort_Transmit(SpiPort_Device device, const uint8_t *data, uint32_t length, uint32_t timeout_ms)
{
  const SpiPort_Mapping *mapping;
  SpiPort_Result result;

  if (data == NULL || length == 0U || length > UINT16_MAX)
  {
    return SPI_PORT_RESULT_INVALID_ARGUMENT;
  }

  result = SpiPort_Prepare(device, &mapping);
  if (result != SPI_PORT_RESULT_COMPLETED)
  {
    return result;
  }

  SpiPort_Select(mapping);
  result =
      SpiPort_MapStatus(HAL_SPI_Transmit(mapping->handle, (uint8_t *)(uintptr_t)data, (uint16_t)length, timeout_ms));
  SpiPort_Deselect(mapping);
  SpiPort_Release(mapping);
  return result;
}

SpiPort_Result SpiPort_Transfer(
    SpiPort_Device device, const uint8_t *transmit_data, uint8_t *receive_data, uint32_t length, uint32_t timeout_ms)
{
  const SpiPort_Mapping *mapping;
  SpiPort_Result result;

  if (transmit_data == NULL || receive_data == NULL || length == 0U || length > UINT16_MAX)
  {
    return SPI_PORT_RESULT_INVALID_ARGUMENT;
  }

  result = SpiPort_Prepare(device, &mapping);
  if (result != SPI_PORT_RESULT_COMPLETED)
  {
    return result;
  }

  if (!mapping->supports_transfer)
  {
    SpiPort_Release(mapping);
    return SPI_PORT_RESULT_UNSUPPORTED;
  }

  SpiPort_Select(mapping);
  result = SpiPort_MapStatus(HAL_SPI_TransmitReceive(
      mapping->handle, (uint8_t *)(uintptr_t)transmit_data, receive_data, (uint16_t)length, timeout_ms));
  SpiPort_Deselect(mapping);
  SpiPort_Release(mapping);
  return result;
}

SpiPort_Result SpiPort_StartTransmitAsync(SpiPort_Device device,
                                          const uint8_t *data,
                                          uint32_t length,
                                          SpiPort_CompletionNotification notification,
                                          void *context)
{
  if (data == NULL || length == 0U || length > SPI_PORT_MAX_ASYNC_LENGTH)
  {
    return SPI_PORT_RESULT_INVALID_ARGUMENT;
  }

  return SpiPort_StartAsync(device, data, NULL, length, false, notification, context);
}

SpiPort_Result SpiPort_StartTransferAsync(SpiPort_Device device,
                                          const uint8_t *transmit_data,
                                          uint8_t *receive_data,
                                          uint32_t length,
                                          SpiPort_CompletionNotification notification,
                                          void *context)
{
  if (transmit_data == NULL || receive_data == NULL || length == 0U || length > SPI_PORT_MAX_ASYNC_LENGTH)
  {
    return SPI_PORT_RESULT_INVALID_ARGUMENT;
  }

  return SpiPort_StartAsync(device, transmit_data, receive_data, length, true, notification, context);
}

SpiPort_Result SpiPort_GetAsyncResult(SpiPort_Device device)
{
  SpiPort_Result result;

  if (device >= SPI_PORT_DEVICE_COUNT)
  {
    return SPI_PORT_RESULT_INVALID_ARGUMENT;
  }

  if (!spi_port_initialized)
  {
    return SPI_PORT_RESULT_NOT_READY;
  }

  result = spi_port_async_results[device];
  __DMB();
  return result;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *handle)
{
  SpiPort_AsyncState *const state = SpiPort_GetAsyncStateFromHandle(handle);

  if (state == NULL || !state->active)
  {
    return;
  }

  SpiPort_FinalizeAsync(state, state->transfer ? SPI_PORT_RESULT_ERROR : SPI_PORT_RESULT_COMPLETED);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *handle)
{
  SpiPort_AsyncState *const state = SpiPort_GetAsyncStateFromHandle(handle);

  if (state == NULL || !state->active)
  {
    return;
  }

  if (!state->transfer)
  {
    SpiPort_FinalizeAsync(state, SPI_PORT_RESULT_ERROR);
    return;
  }

  state->spi_completed = true;
  __DMB();
  SpiPort_TryFinalizeSuccessfulTransfer(state);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle)
{
  SpiPort_AsyncState *const state = SpiPort_GetAsyncStateFromHandle(handle);

  SpiPort_RecoverDmaAfterError(state);
}