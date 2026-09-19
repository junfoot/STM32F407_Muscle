#ifndef IMU_RECEIVER_H
#define IMU_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

void imu_receiver_init(void);
void imu_receiver_service(void);
bool imu_receiver_connected(void);
uint8_t imu_receiver_usb_state(void);
uint8_t imu_receiver_cdc_state(void);
uint32_t imu_receiver_bytes_received(void);

#endif
