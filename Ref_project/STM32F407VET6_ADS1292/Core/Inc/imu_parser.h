#ifndef IMU_PARSER_H
#define IMU_PARSER_H

#include <stdint.h>

#define IMU_MAX_SLAVES          32U
#define IMU_ONLINE_TIMEOUT_MS   1000U

typedef struct
{
    uint8_t device_id;
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];
    int16_t angle[3];
    int16_t battery_raw;
    float accel_g[3];
    float gyro_dps[3];
    float angle_deg[3];
    float battery_voltage;
} imu_data_t;

/* Fixed-size record shared by the SD and UART paths. */
typedef struct
{
    uint32_t sequence;
    uint32_t tick_ms;
    uint8_t device_id;
    uint8_t reserved0[3];
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];
    int16_t angle[3];
    int16_t battery_raw;
    uint16_t reserved1;
} imu_sample_t;

void imu_parser_init(void);
void imu_parse_stream(const uint8_t *data, uint16_t length);
const imu_data_t *imu_get_slave_data(uint8_t device_id);
uint8_t imu_is_slave_online(uint8_t device_id);
uint32_t imu_online_mask(void);
uint32_t imu_frames_received(void);

/* Application hook called once for every complete native-rate IMU frame. */
void imu_frame_received_callback(const imu_sample_t *sample);

#endif
