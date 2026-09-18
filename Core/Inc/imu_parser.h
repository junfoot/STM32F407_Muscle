/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    imu_parser.h
  * @brief   WT9011DCL-RF default IMU frame parser.
  *          Parses the 0x61 data stream from the CH340 receiver and converts
  *          raw 16-bit values to physical units.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __IMU_PARSER_H
#define __IMU_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef struct
{
  uint8_t  device_id;       /* Slave device ID */
  int16_t  accel[3];        /* Accelerometer raw (X,Y,Z) */
  int16_t  gyro[3];         /* Gyroscope raw (X,Y,Z) */
  int16_t  mag[3];          /* Magnetometer raw (X,Y,Z) */
  int16_t  angle[3];        /* Angle raw (Roll,Pitch,Yaw) */
  int16_t  battery_raw;     /* Battery register raw value */

  float    accel_g[3];      /* Acceleration in g */
  float    gyro_dps[3];     /* Angular velocity in °/s */
  float    angle_deg[3];    /* Angle in ° */
  float    battery_voltage; /* Battery voltage in V */
} IMU_Data_t;

/* Exported constants --------------------------------------------------------*/
#define IMU_MAX_SLAVES       32U
#define IMU_ONLINE_TIMEOUT_MS 1000U  /* No frame within this time => slave considered offline */

/* Exported functions -------------------------------------------------------*/

/**
  * @brief  Feed the raw byte stream received from the CH340 into the parser.
  *         The parser is a byte-level state machine that recognizes the 0x55 0x61 frame header.
  */
void IMU_ParseStream(const uint8_t *data, uint16_t len);

/**
  * @brief  Get the latest IMU data for the specified slave ID.
  * @retval Pointer to IMU_Data_t, always valid. Use IMU_SlaveNewData() before calling to check for a new frame.
  */
IMU_Data_t *IMU_GetSlaveData(uint8_t device_id);

/**
  * @brief  Check whether the specified slave has a new frame.
  */
uint8_t IMU_SlaveNewData(uint8_t device_id);

/**
  * @brief  Clear the new-data flag of the specified slave.
  */
void IMU_ClearSlaveNewData(uint8_t device_id);

/**
  * @brief  Check whether the specified slave is currently online.
  *         A slave is online if a frame has been received within IMU_ONLINE_TIMEOUT_MS.
  */
uint8_t IMU_IsSlaveOnline(uint8_t device_id);

/**
  * @brief  Hook invoked with the raw payload of every parsed frame
  *         (weak default does nothing; recorder.c overrides it).
  */
void IMU_RawFrameHook(uint8_t device_id, const uint8_t *payload, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_PARSER_H */
