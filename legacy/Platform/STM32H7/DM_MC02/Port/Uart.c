#include "Detail/Uart.h"
#include "PortKit.h"
#include "main.h"
#include "usart.h"
#include <stddef.h>
#include <stdint.h>

enum
{
  UART_PORT_CACHE_LINE_SIZE = 32U,
  UART_PORT_RX_DMA_CAPACITY = 64U,
  UART_PORT_RX_QUEUE_CAPACITY = 256U,
  UART_PORT_TX_DMA_CAPACITY = UART_PORT_MAX_TRANSMIT_SIZE
};

PORTKIT_STATIC_ASSERT_POWER_OF_TWO(UART_PORT_RX_QUEUE_CAPACITY);

typedef struct
{
  UART_HandleTypeDef *handle;
  uint8_t *receive_dma_buffer;
  uint16_t receive_dma_capacity;
  uint8_t *transmit_dma_buffer;
  uint16_t transmit_dma_capacity;
} UartPort_Mapping;

typedef struct
{
  PortKit_Queue receive_queue;
  uint8_t receive_queue_buffer[UART_PORT_RX_QUEUE_CAPACITY];
  volatile uint32_t received_byte_count;
  volatile uint32_t dropped_byte_count;
  volatile uint32_t error_event_count;
  volatile uint32_t restart_failure_count;
  volatile uint32_t transmitted_byte_count;
  volatile uint32_t transmit_failure_count;
  volatile uint32_t transmit_length;
  PortKit_Notifier receive_notifier;
  PortKit_Notifier transmit_notifier;
  volatile bool transmit_active;
  volatile bool initialized;
} UartPort_State;

static uint8_t uart5_receive_dma_buffer[UART_PORT_RX_DMA_CAPACITY]
    __attribute__((section(".dma_buffer"), aligned(UART_PORT_CACHE_LINE_SIZE)));
static uint8_t uart10_receive_dma_buffer[UART_PORT_RX_DMA_CAPACITY]
    __attribute__((section(".dma_buffer"), aligned(UART_PORT_CACHE_LINE_SIZE)));
static uint8_t uart10_transmit_dma_buffer[UART_PORT_TX_DMA_CAPACITY]
    __attribute__((section(".dma_buffer"), aligned(UART_PORT_CACHE_LINE_SIZE)));

static const UartPort_Mapping uart_port_mappings[UART_PORT_ENDPOINT_COUNT] = {
    [UART_PORT_ENDPOINT_REMOTE_RECEIVER] = {.handle = &huart5,
                                            .receive_dma_buffer = uart5_receive_dma_buffer,
                                            .receive_dma_capacity = UART_PORT_RX_DMA_CAPACITY,
                                            .transmit_dma_buffer = NULL,
                                            .transmit_dma_capacity = 0U},
    [UART_PORT_ENDPOINT_DEBUG_CONSOLE] = {.handle = &huart10,
                                          .receive_dma_buffer = uart10_receive_dma_buffer,
                                          .receive_dma_capacity = UART_PORT_RX_DMA_CAPACITY,
                                          .transmit_dma_buffer = uart10_transmit_dma_buffer,
                                          .transmit_dma_capacity = UART_PORT_TX_DMA_CAPACITY}};

static UartPort_State uart_port_states[UART_PORT_ENDPOINT_COUNT];

_Static_assert(sizeof(uart_port_mappings) / sizeof(uart_port_mappings[0]) == UART_PORT_ENDPOINT_COUNT,
               "UART endpoint count mismatch");
_Static_assert((UART_PORT_RX_DMA_CAPACITY % UART_PORT_CACHE_LINE_SIZE) == 0U,
               "UART DMA buffer capacity must be cache-line aligned");
_Static_assert((UART_PORT_TX_DMA_CAPACITY % UART_PORT_CACHE_LINE_SIZE) == 0U,
               "UART transmit DMA capacity must be cache-line aligned");

static bool UartPort_GetEndpoint(UART_HandleTypeDef *handle, UartPort_Endpoint *endpoint)
{
  if (handle == uart_port_mappings[UART_PORT_ENDPOINT_REMOTE_RECEIVER].handle)
  {
    *endpoint = UART_PORT_ENDPOINT_REMOTE_RECEIVER;
    return true;
  }

  if (handle == uart_port_mappings[UART_PORT_ENDPOINT_DEBUG_CONSOLE].handle)
  {
    *endpoint = UART_PORT_ENDPOINT_DEBUG_CONSOLE;
    return true;
  }

  return false;
}

static bool UartPort_IsDataCacheEnabled(void)
{
  return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
}

static void UartPort_PrepareReceiveBuffer(const UartPort_Mapping *mapping)
{
  if (UartPort_IsDataCacheEnabled())
  {
    SCB_CleanInvalidateDCache_by_Addr(mapping->receive_dma_buffer, mapping->receive_dma_capacity);
  }
}

static void UartPort_CompleteReceiveBuffer(const UartPort_Mapping *mapping)
{
  if (UartPort_IsDataCacheEnabled())
  {
    SCB_InvalidateDCache_by_Addr(mapping->receive_dma_buffer, mapping->receive_dma_capacity);
  }
}

static void UartPort_PrepareTransmitBuffer(const UartPort_Mapping *mapping, uint32_t length)
{
  if (UartPort_IsDataCacheEnabled())
  {
    const uint32_t aligned_length = (length + UART_PORT_CACHE_LINE_SIZE - 1U) & ~(UART_PORT_CACHE_LINE_SIZE - 1U);
    SCB_CleanDCache_by_Addr(mapping->transmit_dma_buffer, aligned_length);
  }
}

static bool UartPort_IsReceiveActive(const UartPort_Mapping *mapping)
{
  return mapping->handle->hdmarx != NULL && mapping->handle->RxState == HAL_UART_STATE_BUSY_RX &&
         mapping->handle->ReceptionType == HAL_UART_RECEPTION_TOIDLE &&
         HAL_DMA_GetState(mapping->handle->hdmarx) == HAL_DMA_STATE_BUSY &&
         (mapping->handle->Instance->CR3 & USART_CR3_DMAR) != 0U;
}

static bool UartPort_StartReceive(const UartPort_Mapping *mapping)
{
  if (mapping->handle->hdmarx == NULL || mapping->handle->RxState != HAL_UART_STATE_READY ||
      HAL_DMA_GetState(mapping->handle->hdmarx) != HAL_DMA_STATE_READY)
  {
    return false;
  }

  UartPort_PrepareReceiveBuffer(mapping);
  if (HAL_UARTEx_ReceiveToIdle_DMA(mapping->handle, mapping->receive_dma_buffer, mapping->receive_dma_capacity) !=
      HAL_OK)
  {
    return false;
  }

  __HAL_DMA_DISABLE_IT(mapping->handle->hdmarx, DMA_IT_HT);
  return UartPort_IsReceiveActive(mapping);
}

static void UartPort_ResetState(UartPort_State *state)
{
  PortKit_Queue_Init(&state->receive_queue, state->receive_queue_buffer, UART_PORT_RX_QUEUE_CAPACITY);
  state->received_byte_count = 0U;
  state->dropped_byte_count = 0U;
  state->error_event_count = 0U;
  state->restart_failure_count = 0U;
  state->transmitted_byte_count = 0U;
  state->transmit_failure_count = 0U;
  state->transmit_length = 0U;
  PortKit_Notifier_Set(&state->receive_notifier, NULL, NULL);
  PortKit_Notifier_Set(&state->transmit_notifier, NULL, NULL);
  state->transmit_active = false;
  state->initialized = false;
  __DMB();
}

static void UartPort_Enqueue(UartPort_State *state, const uint8_t *data, uint32_t length)
{
  const uint32_t accepted = PortKit_Queue_Push(&state->receive_queue, data, length);

  state->received_byte_count += length;
  state->dropped_byte_count += length - accepted;
}

bool UartPort_Init(void)
{
  for (UartPort_Endpoint endpoint = 0U; endpoint < UART_PORT_ENDPOINT_COUNT; ++endpoint)
  {
    const UartPort_Mapping *const mapping = &uart_port_mappings[endpoint];
    UartPort_State *state;

    state = &uart_port_states[endpoint];
    if (state->initialized)
    {
      if (!UartPort_IsReceiveActive(mapping))
      {
        return false;
      }
      continue;
    }

    UartPort_ResetState(state);
    state->initialized = true;
    __DMB();
    if (!UartPort_StartReceive(mapping))
    {
      state->initialized = false;
      __DMB();
      return false;
    }
  }

  return true;
}

bool UartPort_IsReady(UartPort_Endpoint endpoint)
{
  if (endpoint >= UART_PORT_ENDPOINT_COUNT)
  {
    return false;
  }

  const UartPort_Mapping *const mapping = &uart_port_mappings[endpoint];
  return uart_port_states[endpoint].initialized && UartPort_IsReceiveActive(mapping);
}

bool UartPort_SetReceiveNotification(UartPort_Endpoint endpoint,
                                     UartPort_ReceiveNotification notification,
                                     void *context)
{
  if (endpoint >= UART_PORT_ENDPOINT_COUNT)
  {
    return false;
  }

  PortKit_Notifier_Set(&uart_port_states[endpoint].receive_notifier, notification, context);
  return true;
}

bool UartPort_SetTransmitNotification(UartPort_Endpoint endpoint,
                                      UartPort_TransmitNotification notification,
                                      void *context)
{
  if (endpoint >= UART_PORT_ENDPOINT_COUNT)
  {
    return false;
  }

  PortKit_Notifier_Set(&uart_port_states[endpoint].transmit_notifier, notification, context);
  return true;
}

UartPort_ReceiveResult
UartPort_TryRead(UartPort_Endpoint endpoint, uint8_t *data, uint32_t data_capacity, uint32_t *length)
{
  UartPort_State *state;
  uint32_t read_length;

  if (length == NULL)
  {
    return UART_PORT_RECEIVE_INVALID_ARGUMENT;
  }

  *length = 0U;
  if (endpoint >= UART_PORT_ENDPOINT_COUNT || data == NULL || data_capacity == 0U)
  {
    return UART_PORT_RECEIVE_INVALID_ARGUMENT;
  }

  state = &uart_port_states[endpoint];
  read_length = PortKit_Queue_Pop(&state->receive_queue, data, data_capacity);
  if (read_length == 0U)
  {
    return UartPort_IsReady(endpoint) ? UART_PORT_RECEIVE_EMPTY : UART_PORT_RECEIVE_NOT_READY;
  }

  *length = read_length;
  return UART_PORT_RECEIVE_RECEIVED;
}

UartPort_TransmitResult UartPort_TryWrite(UartPort_Endpoint endpoint, const uint8_t *data, uint32_t length)
{
  const UartPort_Mapping *mapping;
  UartPort_State *state;
  uint32_t interrupt_mask;

  if (endpoint >= UART_PORT_ENDPOINT_COUNT || data == NULL || length == 0U)
  {
    return UART_PORT_TRANSMIT_INVALID_ARGUMENT;
  }

  mapping = &uart_port_mappings[endpoint];
  if (mapping->transmit_dma_buffer == NULL || mapping->transmit_dma_capacity == 0U || mapping->handle->hdmatx == NULL)
  {
    return UART_PORT_TRANSMIT_NOT_SUPPORTED;
  }

  if (length > mapping->transmit_dma_capacity)
  {
    return UART_PORT_TRANSMIT_INVALID_ARGUMENT;
  }

  state = &uart_port_states[endpoint];
  if (!state->initialized)
  {
    return UART_PORT_TRANSMIT_NOT_READY;
  }

  interrupt_mask = PortKit_Critical_Enter();
  if (state->transmit_active || mapping->handle->gState != HAL_UART_STATE_READY ||
      HAL_DMA_GetState(mapping->handle->hdmatx) != HAL_DMA_STATE_READY)
  {
    PortKit_Critical_Exit(interrupt_mask);
    return UART_PORT_TRANSMIT_BUSY;
  }

  for (uint32_t index = 0U; index < length; ++index)
  {
    mapping->transmit_dma_buffer[index] = data[index];
  }

  UartPort_PrepareTransmitBuffer(mapping, length);
  state->transmit_length = length;
  state->transmit_active = true;
  __DMB();
  if (HAL_UART_Transmit_DMA(mapping->handle, mapping->transmit_dma_buffer, (uint16_t)length) != HAL_OK)
  {
    state->transmit_active = false;
    state->transmit_length = 0U;
    ++state->transmit_failure_count;
    __DMB();
    PortKit_Critical_Exit(interrupt_mask);
    return UART_PORT_TRANSMIT_ERROR;
  }

  PortKit_Critical_Exit(interrupt_mask);
  return UART_PORT_TRANSMIT_STARTED;
}

bool UartPort_GetStatistics(UartPort_Endpoint endpoint, UartPort_Statistics *statistics)
{
  const UartPort_State *state;
  uint32_t interrupt_mask;

  if (endpoint >= UART_PORT_ENDPOINT_COUNT || statistics == NULL)
  {
    return false;
  }

  state = &uart_port_states[endpoint];
  interrupt_mask = PortKit_Critical_Enter();
  statistics->received_byte_count = state->received_byte_count;
  statistics->dropped_byte_count = state->dropped_byte_count;
  statistics->error_event_count = state->error_event_count;
  statistics->restart_failure_count = state->restart_failure_count;
  statistics->transmitted_byte_count = state->transmitted_byte_count;
  statistics->transmit_failure_count = state->transmit_failure_count;
  PortKit_Critical_Exit(interrupt_mask);
  return true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle)
{
  UartPort_State *state;
  UartPort_Endpoint endpoint;

  if (!UartPort_GetEndpoint(handle, &endpoint))
  {
    return;
  }

  state = &uart_port_states[endpoint];
  if (!state->transmit_active)
  {
    return;
  }

  state->transmitted_byte_count += state->transmit_length;
  state->transmit_length = 0U;
  state->transmit_active = false;
  __DMB();
  (void)PortKit_Notifier_Fire(&state->transmit_notifier);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, uint16_t size)
{
  const UartPort_Mapping *mapping;
  UartPort_State *state;
  UartPort_Endpoint endpoint;
  bool received = false;

  if (!UartPort_GetEndpoint(handle, &endpoint))
  {
    return;
  }

  mapping = &uart_port_mappings[endpoint];
  state = &uart_port_states[endpoint];
  if (!state->initialized)
  {
    return;
  }

  if (HAL_UARTEx_GetRxEventType(handle) == HAL_UART_RXEVENT_HT)
  {
    return;
  }

  if (size > 0U && size <= mapping->receive_dma_capacity)
  {
    UartPort_CompleteReceiveBuffer(mapping);
    UartPort_Enqueue(state, mapping->receive_dma_buffer, size);
    received = true;
  }
  else
  {
    ++state->error_event_count;
  }

  if (!UartPort_StartReceive(mapping))
  {
    ++state->restart_failure_count;
    state->initialized = false;
    __DMB();
  }

  if (received)
  {
    (void)PortKit_Notifier_Fire(&state->receive_notifier);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
  const UartPort_Mapping *mapping;
  UartPort_State *state;
  UartPort_Endpoint endpoint;
  bool transmit_completed = false;

  if (!UartPort_GetEndpoint(handle, &endpoint))
  {
    return;
  }

  mapping = &uart_port_mappings[endpoint];
  state = &uart_port_states[endpoint];
  if (!state->initialized)
  {
    return;
  }

  ++state->error_event_count;
  if (state->transmit_active)
  {
    state->transmit_length = 0U;
    state->transmit_active = false;
    ++state->transmit_failure_count;
    transmit_completed = true;
    __DMB();
  }

  if (!UartPort_IsReceiveActive(mapping) && !UartPort_StartReceive(mapping))
  {
    ++state->restart_failure_count;
    state->initialized = false;
    __DMB();
  }

  if (transmit_completed)
  {
    (void)PortKit_Notifier_Fire(&state->transmit_notifier);
  }
}