#ifndef SD_RECORDER_H
#define SD_RECORDER_H

#include <stdbool.h>
#include <stdint.h>
#include "imu_parser.h"
#include "sample_queue.h"

typedef enum
{
    SD_RECORDER_IDLE = 0,
    SD_RECORDER_ACTIVE,
    SD_RECORDER_STOPPING,
    SD_RECORDER_ERROR
} sd_recorder_state_t;

void sd_recorder_init(uint32_t sample_rate, uint8_t pga, uint8_t channel_mask, uint8_t rld_enabled);
bool sd_recorder_start(void);
void sd_recorder_stop(void);
void sd_recorder_service(void);
bool sd_recorder_push_isr(const emg_sample_t *sample);
bool sd_recorder_push_imu(const imu_sample_t *sample);
bool sd_recorder_is_accepting_isr(void);
sd_recorder_state_t sd_recorder_state(void);
uint32_t sd_recorder_dropped(void);
uint32_t sd_recorder_records_written(void);
uint32_t sd_recorder_imu_dropped(void);
uint32_t sd_recorder_imu_records_written(void);
uint32_t sd_recorder_error_detail(void);
const char *sd_recorder_filename(void);
const char *sd_recorder_imu_filename(void);

#endif
