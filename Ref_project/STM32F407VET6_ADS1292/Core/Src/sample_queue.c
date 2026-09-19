#include "sample_queue.h"
#include "stm32f4xx.h"

void sample_queue_init(sample_queue_t *queue, emg_sample_t *storage, uint16_t capacity)
{
    queue->storage = storage;
    queue->capacity = capacity;
    queue->head = 0U;
    queue->tail = 0U;
    queue->dropped = 0U;
}

void sample_queue_clear(sample_queue_t *queue)
{
    queue->tail = queue->head;
    queue->dropped = 0U;
}

bool sample_queue_push_isr(sample_queue_t *queue, const emg_sample_t *sample)
{
    uint16_t next = (uint16_t)(queue->head + 1U);
    if (next >= queue->capacity)
    {
        next = 0U;
    }
    if (next == queue->tail)
    {
        queue->dropped++;
        return false;
    }
    queue->storage[queue->head] = *sample;
    __DMB();
    queue->head = next;
    return true;
}

bool sample_queue_pop(sample_queue_t *queue, emg_sample_t *sample)
{
    uint16_t tail = queue->tail;
    if (tail == queue->head)
    {
        return false;
    }
    *sample = queue->storage[tail];
    __DMB();
    tail++;
    if (tail >= queue->capacity)
    {
        tail = 0U;
    }
    queue->tail = tail;
    return true;
}

uint16_t sample_queue_count(const sample_queue_t *queue)
{
    uint16_t head = queue->head;
    uint16_t tail = queue->tail;
    return (head >= tail) ? (uint16_t)(head - tail)
                          : (uint16_t)(queue->capacity - tail + head);
}
