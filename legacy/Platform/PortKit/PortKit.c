#include "PortKit.h"
#include "cmsis_compiler.h"
#include <stddef.h>

void PortKit_Queue_Init(PortKit_Queue *queue, uint8_t *buffer, uint32_t capacity)
{
  queue->buffer = buffer;
  queue->capacity = capacity;
  queue->mask = capacity - 1U;
  queue->head = 0U;
  queue->tail = 0U;
}

uint32_t PortKit_Queue_Used(const PortKit_Queue *queue)
{
  return queue->head - queue->tail;
}

uint32_t PortKit_Queue_Free(const PortKit_Queue *queue)
{
  return queue->capacity - PortKit_Queue_Used(queue);
}

uint32_t PortKit_Queue_Push(PortKit_Queue *queue, const uint8_t *data, uint32_t length)
{
  const uint32_t head = queue->head;
  const uint32_t free_capacity = queue->capacity - (head - queue->tail);
  const uint32_t accepted = length < free_capacity ? length : free_capacity;

  if (data == NULL)
  {
    return 0U;
  }

  for (uint32_t index = 0U; index < accepted; ++index)
  {
    queue->buffer[(head + index) & queue->mask] = data[index];
  }

  __DMB();
  queue->head = head + accepted;
  return accepted;
}

uint32_t PortKit_Queue_Pop(PortKit_Queue *queue, uint8_t *data, uint32_t data_capacity)
{
  const uint32_t tail = queue->tail;
  const uint32_t available = queue->head - tail;
  const uint32_t read_length = available < data_capacity ? available : data_capacity;

  if (data == NULL || data_capacity == 0U)
  {
    return 0U;
  }

  __DMB();
  for (uint32_t index = 0U; index < read_length; ++index)
  {
    data[index] = queue->buffer[(tail + index) & queue->mask];
  }

  __DMB();
  queue->tail = tail + read_length;
  return read_length;
}

void PortKit_Queue_Reset(PortKit_Queue *queue)
{
  __DMB();
  queue->tail = queue->head;
  __DMB();
}

void PortKit_Notifier_Set(PortKit_Notifier *notifier, PortKit_Notification function, void *context)
{
  const uint32_t interrupt_mask = PortKit_Critical_Enter();

  notifier->function = NULL;
  __DMB();
  notifier->context = context;
  __DMB();
  notifier->function = function;
  __DMB();
  PortKit_Critical_Exit(interrupt_mask);
}

bool PortKit_Notifier_Fire(const PortKit_Notifier *notifier)
{
  PortKit_Notification function;
  void *context;

  __DMB();
  function = notifier->function;
  context = notifier->context;
  __DMB();
  if (function == NULL)
  {
    return false;
  }

  function(context);
  return true;
}

uint32_t PortKit_Critical_Enter(void)
{
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  return interrupt_mask;
}

void PortKit_Critical_Exit(uint32_t interrupt_mask)
{
  __set_PRIMASK(interrupt_mask);
}