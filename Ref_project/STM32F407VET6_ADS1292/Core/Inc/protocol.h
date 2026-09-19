#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include "imu_parser.h"
#include "sample_queue.h"

void protocol_init(void);
void protocol_service(void);
bool protocol_push_isr(const emg_sample_t *sample);
bool protocol_push_imu(const imu_sample_t *sample);
bool protocol_is_streaming(void);
uint32_t protocol_dropped(void);
uint32_t protocol_imu_dropped(void);

#endif
