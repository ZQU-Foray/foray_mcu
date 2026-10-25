#ifndef PORTKIT_H
#define PORTKIT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
 * @brief 单生产者-单消费者字节环形队列。
 * @note 生产者（通常是 ISR）只能 Push，消费者（通常是任务）只能 Pop/Reset；
 *       capacity 必须为 2 的幂。
 */
  typedef struct
  {
    uint8_t *buffer;
    uint32_t capacity;
    uint32_t mask;
    volatile uint32_t head; // 生产者独占写
    volatile uint32_t tail; // 消费者独占写
  } PortKit_Queue;

  void PortKit_Queue_Init(PortKit_Queue *queue, uint8_t *buffer, uint32_t capacity);
  uint32_t PortKit_Queue_Used(const PortKit_Queue *queue);
  uint32_t PortKit_Queue_Free(const PortKit_Queue *queue);

  /**
 * @brief 入队，返回实际入队字节数（剩余空间不足时截断，不报错）。
 */
  uint32_t PortKit_Queue_Push(PortKit_Queue *queue, const uint8_t *data, uint32_t length);

  /**
 * @brief 出队，返回实际出队字节数（数据不足时返回现有全部）。
 */
  uint32_t PortKit_Queue_Pop(PortKit_Queue *queue, uint8_t *data, uint32_t data_capacity);

  /**
 * @brief 清空队列（tail 追平 head），可在 ISR 调用。
 */
  void PortKit_Queue_Reset(PortKit_Queue *queue);

  /**
 * @brief 中断上下文回调通知：任务侧注册，ISR 侧读取并调用。
 */
  typedef void (*PortKit_Notification)(void *context);

  typedef struct
  {
    PortKit_Notification function;
    void *context;
  } PortKit_Notifier;

  /**
 * @brief 任务上下文注册通知（NULL 取消）。内部用临界区保证 ISR
 * 不会看到半更新状态。
 */
  void PortKit_Notifier_Set(PortKit_Notifier *notifier, PortKit_Notification function, void *context);

  /**
 * @brief ISR 上下文读取并调用通知；未注册时返回 false。
 */
  bool PortKit_Notifier_Fire(const PortKit_Notifier *notifier);

  /**
 * @brief 短临界区：保存并屏蔽 PRIMASK。仅用于微秒级、无阻塞代码段。
 */
  uint32_t PortKit_Critical_Enter(void);
  void PortKit_Critical_Exit(uint32_t interrupt_mask);

#define PORTKIT_STATIC_ASSERT_POWER_OF_TWO(capacity)                                                                   \
  _Static_assert(((capacity) != 0U) && (((capacity) & ((capacity) - 1U)) == 0U),                                       \
                 "PortKit queue capacity must be a power of two")

#ifdef __cplusplus
}
#endif

#endif