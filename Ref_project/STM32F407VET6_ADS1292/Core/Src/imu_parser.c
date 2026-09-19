#include "imu_parser.h"

#include <string.h>
#include "stm32f4xx_hal.h"

#define IMU_HEADER          0x55U
#define IMU_FLAG_DATA       0x61U
#define IMU_FRAME_DATA_LEN  26U

typedef enum
{
    IMU_WAIT_HEADER = 0,
    IMU_WAIT_FLAG,
    IMU_READ_DATA
} imu_parse_state_t;

static imu_parse_state_t parse_state;
static uint8_t previous_byte;
static uint8_t frame_data[IMU_FRAME_DATA_LEN];
static uint8_t frame_index;
static uint32_t frame_sequence;
static imu_data_t slave_data[IMU_MAX_SLAVES];
static uint32_t slave_last_rx_tick[IMU_MAX_SLAVES];
static uint32_t slave_seen_mask;

static int16_t bytes_to_i16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8U) | low);
}

static void parse_complete_frame(uint8_t device_id)
{
    uint8_t index = device_id & (IMU_MAX_SLAVES - 1U);
    imu_data_t *data = &slave_data[index];
    imu_sample_t sample;
    uint8_t axis;

    data->device_id = device_id;
    for (axis = 0U; axis < 3U; axis++)
    {
        data->accel[axis] = bytes_to_i16(frame_data[axis * 2U], frame_data[axis * 2U + 1U]);
        data->gyro[axis] = bytes_to_i16(frame_data[6U + axis * 2U], frame_data[7U + axis * 2U]);
        data->mag[axis] = bytes_to_i16(frame_data[12U + axis * 2U], frame_data[13U + axis * 2U]);
        data->angle[axis] = bytes_to_i16(frame_data[18U + axis * 2U], frame_data[19U + axis * 2U]);
        data->accel_g[axis] = (float)data->accel[axis] * (16.0f / 32768.0f);
        data->gyro_dps[axis] = (float)data->gyro[axis] * (2000.0f / 32768.0f);
        data->angle_deg[axis] = (float)data->angle[axis] * (180.0f / 32768.0f);
    }
    data->battery_raw = bytes_to_i16(frame_data[24], frame_data[25]);
    data->battery_voltage = (float)data->battery_raw / 100.0f;
    slave_last_rx_tick[index] = HAL_GetTick();
    slave_seen_mask |= (1UL << index);

    memset(&sample, 0, sizeof(sample));
    sample.sequence = frame_sequence++;
    sample.tick_ms = slave_last_rx_tick[index];
    sample.device_id = device_id;
    memcpy(sample.accel, data->accel, sizeof(sample.accel));
    memcpy(sample.gyro, data->gyro, sizeof(sample.gyro));
    memcpy(sample.mag, data->mag, sizeof(sample.mag));
    memcpy(sample.angle, data->angle, sizeof(sample.angle));
    sample.battery_raw = data->battery_raw;
    imu_frame_received_callback(&sample);
}

void imu_parser_init(void)
{
    parse_state = IMU_WAIT_HEADER;
    previous_byte = 0U;
    frame_index = 0U;
    frame_sequence = 0U;
    memset(slave_data, 0, sizeof(slave_data));
    memset(slave_last_rx_tick, 0, sizeof(slave_last_rx_tick));
    slave_seen_mask = 0U;
}

void imu_parse_stream(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    for (index = 0U; index < length; index++)
    {
        uint8_t value = data[index];
        switch (parse_state)
        {
            case IMU_WAIT_HEADER:
                if (value == IMU_HEADER)
                {
                    parse_state = IMU_WAIT_FLAG;
                }
                else
                {
                    previous_byte = value;
                }
                break;

            case IMU_WAIT_FLAG:
                if (value == IMU_FLAG_DATA)
                {
                    frame_index = 0U;
                    parse_state = IMU_READ_DATA;
                }
                else if (value == IMU_HEADER)
                {
                    previous_byte = IMU_HEADER;
                }
                else
                {
                    previous_byte = value;
                    parse_state = IMU_WAIT_HEADER;
                }
                break;

            case IMU_READ_DATA:
                frame_data[frame_index++] = value;
                if (frame_index >= IMU_FRAME_DATA_LEN)
                {
                    parse_complete_frame(previous_byte);
                    parse_state = IMU_WAIT_HEADER;
                }
                break;

            default:
                parse_state = IMU_WAIT_HEADER;
                break;
        }
    }
}

const imu_data_t *imu_get_slave_data(uint8_t device_id)
{
    return &slave_data[device_id & (IMU_MAX_SLAVES - 1U)];
}

uint8_t imu_is_slave_online(uint8_t device_id)
{
    uint8_t index = device_id & (IMU_MAX_SLAVES - 1U);
    return ((slave_seen_mask & (1UL << index)) != 0U
            && (HAL_GetTick() - slave_last_rx_tick[index]) < IMU_ONLINE_TIMEOUT_MS) ? 1U : 0U;
}

uint32_t imu_online_mask(void)
{
    uint32_t mask = 0U;
    uint8_t index;
    for (index = 0U; index < IMU_MAX_SLAVES; index++)
    {
        if (imu_is_slave_online(index)) mask |= (1UL << index);
    }
    return mask;
}

uint32_t imu_frames_received(void)
{
    return frame_sequence;
}

__weak void imu_frame_received_callback(const imu_sample_t *sample)
{
    (void)sample;
}
