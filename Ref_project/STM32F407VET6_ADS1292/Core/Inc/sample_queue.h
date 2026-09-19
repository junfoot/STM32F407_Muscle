#ifndef SAMPLE_QUEUE_H
#define SAMPLE_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t sequence;
    uint32_t tick_ms;
    float channel[2];
} emg_sample_t;

typedef struct
{
    emg_sample_t *storage;
    uint16_t capacity;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t dropped;
} sample_queue_t;

void sample_queue_init(sample_queue_t *queue, emg_sample_t *storage, uint16_t capacity);
void sample_queue_clear(sample_queue_t *queue);
bool sample_queue_push_isr(sample_queue_t *queue, const emg_sample_t *sample);
bool sample_queue_pop(sample_queue_t *queue, emg_sample_t *sample);
uint16_t sample_queue_count(const sample_queue_t *queue);

#endif
